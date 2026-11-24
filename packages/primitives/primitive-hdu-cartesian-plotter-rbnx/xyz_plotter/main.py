# SPDX-License-Identifier: MIT
"""xyz_plotter — Robonix primitive for an XYZ Cartesian pen plotter.

Owns ``robonix/primitive/plotter/*``:

  * ``driver``     shared lifecycle contract (CMD_INIT connects the PLC).
  * ``trajectory`` ordered absolute waypoints, one PLC ACK per point.
  * ``home``       PLC ``MC_GroupHome`` routine (gRPC + MCP).
  * ``set_params`` global motion profile + pen heights (SET_PARAM, gRPC + MCP).
  * ``status``     topic_out (gRPC server stream) at ``status_rate_hz``.

All PLC-specific bytes live in the vendored ``robonix_sdk`` (mirrors
``SDK/robonix_sdk``). This module only maps Robonix contracts to SDK
calls and enforces the single-trajectory mutex.

Wire facts that shape this module:

  * The PLC exposes PING / MOVE_LINEAR / HOME / SET_PARAM / CLOSE only.
    There is no STOP, no STATUS query and no per-point velocity.
  * Motion parameters are global until the next SET_PARAM.
  * HOME always goes to the PLC's fixed ``Home = (0, 0, 0)``.
  * Closing the session must send CLOSE(0x0A); a bare socket close can
    wedge the PLC's connection state machine.
"""
from __future__ import annotations

import logging
import math
import threading
import time

from robonix_api import Err, Ok, Primitive

xyz_plotter = Primitive(id="xyz_plotter", namespace="robonix/primitive/plotter")

import plotter_pb2  # type: ignore  # noqa: E402  (rbnx codegen)
from plotter_mcp import (  # type: ignore  # noqa: E402  (rbnx codegen)
    Home_Request as MCPHome_Request,
    Home_Response as MCPHome_Response,
    SetParams_Request as MCPSetParams_Request,
    SetParams_Response as MCPSetParams_Response,
)

from robonix_sdk import ProtocolError, RobonixClient  # noqa: E402

log = logging.getLogger("xyz_plotter")
logging.basicConfig(
    level=logging.INFO,
    format="[xyz_plotter] %(levelname)s %(message)s",
)


_DEFAULTS = {
    # SDK defaults: MatrixBox MX4001 at 192.168.33.130:2000.
    "host": "192.168.33.130",
    "port": 2000,
    # Connect timeout and the floor for the per-point ACK timeout.
    "timeout_s": 30.0,
    "connect_on_init": True,
    "units": "mm",
    # Pen heights in the paper frame: paper surface z=-50, pen up z=-46.
    "z_up": -46.0,
    "z_down": -50.0,
    # Drawable area in the paper frame: origin (0,0), +X right, +Y up.
    "area_x_max": 180.0,
    "area_y_max": 150.0,
    # Z travel limits used for validation; other machines override these.
    "z_min": -50.0,
    "z_max": 0.0,
    # Global motion profile (PLC SET_PARAM values).
    "velocity": 20.0,
    "acceleration": 300.0,
    "deceleration": 300.0,
    "jerk": 3000.0,
    "status_rate_hz": 2.0,
    "max_points": 100000,
}

_state = {
    "cfg": dict(_DEFAULTS),
    "client": None,
    "active": False,
    "busy": False,
    "homing": False,
    "last_error": "",
    "position": None,
    "progress": 0,
    "total": 0,
    "status_seq": 0,
    "stop": threading.Event(),
}
_lock = threading.RLock()


# ── config helpers ───────────────────────────────────────────────────
def _as_float(cfg: dict, key: str, default: float, *, minimum=None, maximum=None) -> float:
    raw = cfg.get(key, default)
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{key} must be a number, got {raw!r}") from exc
    if not math.isfinite(value):
        raise ValueError(f"{key} must be finite, got {value}")
    if minimum is not None and value < minimum:
        raise ValueError(f"{key} must be >= {minimum}, got {value}")
    if maximum is not None and value > maximum:
        raise ValueError(f"{key} must be <= {maximum}, got {value}")
    return value


def _as_bool(cfg: dict, key: str, default: bool) -> bool:
    raw = cfg.get(key, default)
    if isinstance(raw, bool):
        return raw
    if isinstance(raw, str):
        return raw.strip().lower() in ("1", "true", "yes", "on")
    return bool(raw)


def _merge_config(raw: dict) -> dict:
    cfg = dict(_DEFAULTS)

    cfg["host"] = str(raw.get("host", cfg["host"]))
    try:
        cfg["port"] = int(raw.get("port", cfg["port"]))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"port must be an integer, got {raw.get('port')!r}") from exc
    if not 1 <= cfg["port"] <= 65535:
        raise ValueError(f"port out of range: {cfg['port']}")

    cfg["units"] = str(raw.get("units", cfg["units"])).strip().lower()
    if cfg["units"] != "mm":
        raise ValueError(f"unsupported units {cfg['units']!r}; only 'mm' is supported")

    cfg["timeout_s"] = _as_float(raw, "timeout_s", cfg["timeout_s"], minimum=0.1)
    cfg["status_rate_hz"] = _as_float(
        raw, "status_rate_hz", cfg["status_rate_hz"], minimum=0.1
    )

    cfg["z_up"] = _as_float(raw, "z_up", cfg["z_up"])
    cfg["z_down"] = _as_float(raw, "z_down", cfg["z_down"])
    if cfg["z_up"] <= cfg["z_down"]:
        raise ValueError(f"z_up ({cfg['z_up']}) must be greater than z_down ({cfg['z_down']})")

    # Z travel limits — machine-specific; override via config for other rigs.
    cfg["z_min"] = _as_float(raw, "z_min", cfg["z_min"])
    cfg["z_max"] = _as_float(raw, "z_max", cfg["z_max"])
    if cfg["z_max"] <= cfg["z_min"]:
        raise ValueError(f"z_max ({cfg['z_max']}) must be greater than z_min ({cfg['z_min']})")
    if not (cfg["z_min"] <= cfg["z_down"] < cfg["z_up"] <= cfg["z_max"]):
        raise ValueError(
            "pen heights must satisfy z_min <= z_down < z_up <= z_max, got "
            f"z_min={cfg['z_min']}, z_down={cfg['z_down']}, "
            f"z_up={cfg['z_up']}, z_max={cfg['z_max']}"
        )

    # Drawable area in the paper frame; x/y minima are fixed at 0.
    cfg["area_x_max"] = _as_float(raw, "area_x_max", cfg["area_x_max"], minimum=0.0)
    cfg["area_y_max"] = _as_float(raw, "area_y_max", cfg["area_y_max"], minimum=0.0)

    for key in ("velocity", "acceleration", "deceleration", "jerk"):
        cfg[key] = _as_float(raw, key, cfg[key], minimum=0.1)

    try:
        cfg["max_points"] = int(raw.get("max_points", cfg["max_points"]))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"max_points must be an integer, got {raw.get('max_points')!r}") from exc
    if cfg["max_points"] < 1:
        raise ValueError(f"max_points must be >= 1, got {cfg['max_points']}")

    cfg["connect_on_init"] = _as_bool(raw, "connect_on_init", cfg["connect_on_init"])
    return cfg


def _make_client(cfg: dict) -> RobonixClient:
    return RobonixClient(host=cfg["host"], port=cfg["port"], timeout=cfg["timeout_s"])


def _connect(cfg: dict) -> RobonixClient:
    """Open a session, verify it with PING, and push the global params."""
    client = _make_client(cfg)
    client.connect()
    try:
        if not client.ping():
            raise ConnectionError("PLC did not answer PING")
        client.set_param(
            cfg["velocity"],
            cfg["acceleration"],
            cfg["deceleration"],
            cfg["jerk"],
            cfg["z_up"],
            cfg["z_down"],
        )
    except Exception:
        client.close()
        raise
    return client


def _close_client(client: RobonixClient | None, *, graceful: bool = True) -> None:
    """Close a session, preferring CMD_CLOSE so the PLC can accept again."""
    if client is None:
        return
    if graceful:
        try:
            client.close_session()
            return
        except Exception as exc:  # noqa: BLE001 - always fall back to close()
            log.debug("graceful close failed: %s", exc)
    try:
        client.close()
    except Exception:  # noqa: BLE001 - teardown must not raise
        log.exception("client.close failed")


def _ensure_client() -> RobonixClient:
    """Return a connected client, reconnecting and re-sending params if needed."""
    cfg = _state["cfg"]
    client = _state["client"]
    if client is not None and client.connected:
        return client
    if client is not None:
        _close_client(client, graceful=False)
    client = _connect(cfg)
    _state["client"] = client
    return client


def _drop_client() -> None:
    """Forget the current session after a failure; next call reconnects.

    Try CMD_CLOSE even after a failed command: a bare socket close can
    leave the PLC looping between states 20 and 60 and refusing new
    connections. ``close_session`` falls back to a plain close itself
    after its 3 s budget.
    """
    client, _state["client"] = _state["client"], None
    _close_client(client, graceful=True)


def _begin() -> str | None:
    """Take the single-trajectory mutex. Returns an error string if busy."""
    with _lock:
        if _state["busy"]:
            return "plotter is busy executing another request"
        _state["busy"] = True
        _state["progress"] = 0
        _state["total"] = 0
        return None


def _end() -> None:
    with _lock:
        _state["busy"] = False
        _state["homing"] = False


def _remember_error(exc: BaseException) -> str:
    message = str(exc) or exc.__class__.__name__
    _state["last_error"] = message
    return message


def _set_position(x: float, y: float, z: float) -> None:
    _state["position"] = (float(x), float(y), float(z))


# ── lifecycle ────────────────────────────────────────────────────────
@xyz_plotter.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE. Parse config and optionally connect."""
    try:
        merged = _merge_config(cfg or {})
    except (TypeError, ValueError) as exc:
        return Err(f"invalid plotter config: {exc}")

    _state["cfg"] = merged
    if merged["connect_on_init"]:
        try:
            _state["client"] = _connect(merged)
        except Exception as exc:  # noqa: BLE001 - boot boundary
            _state["last_error"] = str(exc)
            return Err(
                f"cannot connect to PLC {merged['host']}:{merged['port']}: {exc}"
            )

    log.info(
        "init ok: PLC %s:%d connect_on_init=%s z_up=%.2f z_down=%.2f",
        merged["host"], merged["port"], merged["connect_on_init"],
        merged["z_up"], merged["z_down"],
    )
    return Ok()


@xyz_plotter.on_activate
def activate():
    """INACTIVE → ACTIVE. Reconnect if the PLC dropped while inactive."""
    _state["active"] = True
    _state["stop"].clear()
    cfg = _state["cfg"]
    client = _state["client"]
    if cfg["connect_on_init"] and (client is None or not client.connected):
        try:
            _state["client"] = _connect(cfg)
        except Exception as exc:  # noqa: BLE001 - activation should still succeed
            _state["last_error"] = str(exc)
            log.warning("activate: PLC still unreachable: %s", exc)
    return Ok()


@xyz_plotter.on_deactivate
def deactivate():
    """ACTIVE → INACTIVE. Stops the status stream.

    The wire protocol has no STOP command, so an in-flight trajectory
    cannot be aborted from Robonix. It keeps running to completion; the
    connection stays up so the PLC is not left half-closed.
    """
    _state["active"] = False
    _state["stop"].set()
    if _state["busy"]:
        log.warning(
            "deactivate while busy: the PLC protocol has no STOP; motion continues"
        )
    return Ok()


@xyz_plotter.on_shutdown
def shutdown():
    """any → TERMINATED. Best-effort CMD_CLOSE, then drop the socket."""
    _state["stop"].set()
    client, _state["client"] = _state["client"], None
    _close_client(client, graceful=True)
    log.info("shutdown")
    return Ok()


# ── trajectory ───────────────────────────────────────────────────────
@xyz_plotter.grpc("robonix/primitive/plotter/trajectory")
def trajectory(req, ctx):
    points = [(float(p.x), float(p.y), float(p.z)) for p in req.waypoints]
    total = len(points)
    if total == 0:
        return plotter_pb2.Trajectory_Response(
            success=False, message="empty trajectory", points_accepted=0
        )

    max_points = int(_state["cfg"]["max_points"])
    if total > max_points:
        return plotter_pb2.Trajectory_Response(
            success=False,
            message=f"too many waypoints: {total} > max_points {max_points}",
            points_accepted=0,
        )
    for x, y, z in points:
        if not all(math.isfinite(v) for v in (x, y, z)):
            return plotter_pb2.Trajectory_Response(
                success=False, message=f"non-finite waypoint: ({x}, {y}, {z})",
                points_accepted=0,
            )

    busy = _begin()
    if busy is not None:
        return plotter_pb2.Trajectory_Response(
            success=False, message=busy, points_accepted=0
        )

    _state["total"] = total
    completed = 0
    started = time.monotonic()
    try:
        client = _ensure_client()
        cfg = _state["cfg"]

        # Lift-first protection: if the last known pose has the pen down,
        # raise it before travelling to the first waypoint. The inserted
        # move is not counted in points_completed.
        send = list(points)
        offset = 0
        position = _state["position"]
        if position is not None:
            px, py, pz = position
            if pz < cfg["z_up"] - 1e-9:
                send.insert(0, (px, py, cfg["z_up"]))
                offset = 1

        def _on_progress(index: int, _n: int, point) -> None:
            nonlocal completed
            _set_position(*point)
            completed = max(0, index - offset)
            _state["progress"] = completed

        client.draw_waypoints(send, on_progress=_on_progress)
        _state["last_error"] = ""
        return plotter_pb2.Trajectory_Response(
            success=True,
            message="ok",
            points_accepted=total,
            points_completed=completed,
            elapsed_s=time.monotonic() - started,
        )
    except Exception as exc:  # noqa: BLE001 - service boundary
        message = _remember_error(exc)
        _drop_client()
        log.warning("trajectory failed after %d/%d points: %s", completed, total, message)
        return plotter_pb2.Trajectory_Response(
            success=False,
            message=message,
            points_accepted=total,
            points_completed=completed,
            elapsed_s=time.monotonic() - started,
        )
    finally:
        _end()


# ── home ─────────────────────────────────────────────────────────────
def _home_impl() -> tuple[bool, str, float]:
    busy = _begin()
    if busy is not None:
        return False, busy, 0.0

    _state["homing"] = True
    started = time.monotonic()
    try:
        client = _ensure_client()
        client.home()
        # MC_GroupHome targets the PLC's fixed Home array (0, 0, 0).
        _set_position(0.0, 0.0, 0.0)
        _state["last_error"] = ""
        return True, "ok", time.monotonic() - started
    except Exception as exc:  # noqa: BLE001 - service boundary
        message = _remember_error(exc)
        _drop_client()
        log.warning("home failed: %s", message)
        return False, message, time.monotonic() - started
    finally:
        _end()


@xyz_plotter.grpc("robonix/primitive/plotter/home")
def home(req, ctx):
    success, message, elapsed = _home_impl()
    return plotter_pb2.Home_Response(
        success=success, message=message, elapsed_s=elapsed
    )


@xyz_plotter.mcp("robonix/primitive/plotter/home")
def home_tool(req: MCPHome_Request) -> MCPHome_Response:
    """Send the gantry to the PLC home position and wait for completion.

    Use this before a new job, after drawing, or when the user asks to
    home, park, or reset the machine. The PLC always homes to (0, 0, 0).
    """
    success, message, elapsed = _home_impl()
    return MCPHome_Response(
        success=success, message=message, elapsed_s=elapsed
    )


# ── set_params ───────────────────────────────────────────────────────
_PARAM_KEYS = (
    "velocity",
    "acceleration",
    "deceleration",
    "jerk",
    "z_up",
    "z_down",
)


def _set_params_impl(req) -> tuple[bool, str]:
    """Merge a partial profile into the global config and push it.

    A field value of 0 means "keep the current value", so an MCP caller
    can change only velocity (or only the pen heights) without resetting
    the rest of the global profile. The PLC has no GET_PARAM; the cached
    config here is the source of truth until the next SET_PARAM.
    """
    cfg = _state["cfg"]
    values = {}
    for key in _PARAM_KEYS:
        raw = float(getattr(req, key))
        if not math.isfinite(raw):
            return False, "params must be finite"
        values[key] = float(cfg[key]) if raw == 0.0 else raw

    if not (cfg["z_min"] <= values["z_down"] < values["z_up"] <= cfg["z_max"]):
        return False, (
            "pen heights must satisfy "
            f"z_min <= z_down < z_up <= z_max, got z_down={values['z_down']}, "
            f"z_up={values['z_up']} (z_min={cfg['z_min']}, z_max={cfg['z_max']})"
        )
    if any(
        values[key] <= 0.0
        for key in ("velocity", "acceleration", "deceleration", "jerk")
    ):
        return False, (
            "velocity/acceleration/deceleration/jerk must be > 0 "
            "(use 0 to keep the current value)"
        )

    busy = _begin()
    if busy is not None:
        return False, busy

    try:
        client = _state["client"]
        if client is None or not client.connected:
            cfg.update(values)
            _state["last_error"] = ""
            return True, "params stored; PLC is offline, will apply on reconnect"
        client.set_param(
            values["velocity"],
            values["acceleration"],
            values["deceleration"],
            values["jerk"],
            values["z_up"],
            values["z_down"],
        )
        cfg.update(values)
        _state["last_error"] = ""
        return True, "ok"
    except Exception as exc:  # noqa: BLE001 - service boundary
        message = _remember_error(exc)
        _drop_client()
        log.warning("set_params failed: %s", message)
        return False, message
    finally:
        _end()


@xyz_plotter.grpc("robonix/primitive/plotter/set_params")
def set_params(req, ctx):
    success, message = _set_params_impl(req)
    return plotter_pb2.SetParams_Response(success=success, message=message)


@xyz_plotter.mcp("robonix/primitive/plotter/set_params")
def set_params_tool(req: MCPSetParams_Request) -> MCPSetParams_Response:
    """Set the global motion profile used by subsequent trajectories.

    All six values are millimetres or millimetres/second; pass 0 for any
    field that should keep its current value. SET_PARAM does not move the
    machine. The profile stays in effect until the next set_params call
    (or a primitive reconnect, which reapplies the cached profile).

    velocity is mm/s; acceleration/deceleration are mm/s^2; jerk is mm/s^3.
    Pen heights must satisfy z_min <= z_down < z_up <= z_max (the machine's
    configured travel); both are usually -46 / -50 on this rig.
    """
    success, message = _set_params_impl(req)
    return MCPSetParams_Response(success=success, message=message)


# ── geometry ─────────────────────────────────────────────────────
@xyz_plotter.grpc("robonix/primitive/plotter/geometry")
def geometry(req, ctx):
    """Report the static machine geometry from the deployment config.

    Skills call this once on activation. Area and pen heights are config,
    not feedback: the PLC has no readback.
    """
    cfg = _state["cfg"]
    return plotter_pb2.Geometry_Response(
        success=True,
        message="ok",
        x_min=0.0,
        x_max=float(cfg["area_x_max"]),
        y_min=0.0,
        y_max=float(cfg["area_y_max"]),
        z_up=float(cfg["z_up"]),
        z_down=float(cfg["z_down"]),
    )


# ── status stream ────────────────────────────────────────────────────
def _status_message():
    cfg = _state["cfg"]
    client = _state["client"]
    connected = bool(client is not None and client.connected)
    busy = bool(_state["busy"])
    last_error = _state["last_error"]

    position = _state["position"]
    if position is None:
        x = y = z = 0.0
    else:
        x, y, z = position

    if busy and _state["homing"]:
        state = "HOMING"
    elif busy:
        state = "MOVING"
    elif not connected:
        state = "ERROR" if last_error else "DISCONNECTED"
    elif last_error:
        state = "ERROR"
    else:
        state = "IDLE"

    _state["status_seq"] += 1
    return plotter_pb2.PlotterStatus(
        connected=connected,
        busy=busy,
        last_error=last_error,
        x=float(x),
        y=float(y),
        z=float(z),
        z_up=float(cfg["z_up"]),
        z_down=float(cfg["z_down"]),
        area_x_max=float(cfg["area_x_max"]),
        area_y_max=float(cfg["area_y_max"]),
        state=state,
        seq=int(_state["status_seq"]),
    )


@xyz_plotter.grpc("robonix/primitive/plotter/status")
def status(req, ctx):
    """Server-streaming handler for the topic_out contract."""
    period = 1.0 / max(0.1, float(_state["cfg"]["status_rate_hz"]))
    while not _state["stop"].is_set():
        if ctx is not None and hasattr(ctx, "is_active") and not ctx.is_active():
            return
        yield _status_message()
        _state["stop"].wait(period)


def main() -> int:
    xyz_plotter.run()
    return 0


if __name__ == "__main__":
    import sys

    sys.exit(main())