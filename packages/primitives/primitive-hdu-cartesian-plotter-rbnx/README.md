# primitive-hdu-cartesian-plotter-rbnx

Robonix **primitive** for the XYZ Cartesian pen plotter. It is the only
package that speaks TCP to the PLC and owns the
`robonix/primitive/plotter/*` namespace: framing, CRC16-Modbus, point-by-point
ACKs, homing, global motion parameters, and the synthesized status stream.

Catalog name: `robonix.primitive.xyz.plotter`.

The wire protocol, coordinate conventions, and reference client come from the
repository [`SDK/`](../../../SDK). Skills never import the SDK transport; they
call the contracts below over gRPC.

## Capability surface

| Contract | Mode | Transport | Source / handler |
| --- | --- | --- | --- |
| `robonix/primitive/plotter/driver` | rpc | gRPC | `Driver(CMD_INIT, config_json)` — lifecycle gate; connects and PINGs the PLC |
| `robonix/primitive/plotter/trajectory` | rpc | gRPC | `Trajectory(waypoints)` — one `MOVE_LINEAR` per point, blocks for the ACK |
| `robonix/primitive/plotter/home` | rpc + MCP | gRPC / MCP | `Home(Empty)` → PLC `MC_GroupHome` (fixed `(0,0,0)`); MCP tool `plotter_home` |
| `robonix/primitive/plotter/set_params` | rpc + MCP | gRPC / MCP | `SetParams(vel, acc, dec, jerk, z_up, z_down)`; MCP tool `plotter_set_params`; `0` keeps the current value |
| `robonix/primitive/plotter/geometry` | rpc | gRPC | `Geometry(Empty)` → drawable area + pen heights from config |
| `robonix/primitive/plotter/status` | topic_out | gRPC stream | `Status(Empty) → stream PlotterStatus` |

`status` is a msg-backed topic. The current PLC firmware has **no STATUS
query**, so the stream is synthesized from local state: socket connectivity,
busy flag, last commanded position, and the most recent error.

`home` and `set_params` are registered as both gRPC and MCP so Pilot/LLM can
call them directly without going through a skill. The PLC cannot read
parameters back; the primitive caches the last successfully sent profile.

## Boot ordering

This primitive must boot **before** any text or draw skill in the deployment
manifest. `rbnx boot` has no defer/retry, and each skill resolves
`primitive/plotter/{trajectory,home}` during activation.

## Driver-init lifecycle

`scripts/start.sh` execs `python3 -m xyz_plotter.main`; it does not spawn ROS
or a PLC subprocess.

1. `CMD_INIT`: parse and validate the manifest `config:` block. With
   `connect_on_init: true` (default), connect to the PLC, `PING` it, and push
   the default `SET_PARAM` profile. Failure returns `Err`, so a missing PLC
   fails boot immediately.
2. `CMD_ACTIVATE`: mark the primitive active; reconnect if the PLC dropped
   while inactive.
3. `CMD_DEACTIVATE`: stop the status stream. The protocol has no `STOP`, so a
   motion already in flight is not aborted.
4. `CMD_SHUTDOWN`: best-effort `CLOSE(0x0A)`, wait for its ACK, then close the
   socket. Skipping `CLOSE` can leave the PLC stuck between its `20`/`60`
   states.

## Layout

```text
primitive-hdu-cartesian-plotter-rbnx/
├── package_manifest.yaml
├── capabilities/
│   ├── plotter/{driver,trajectory,home,set_params,status}.v1.toml
│   └── lib/plotter/{srv,msg}/...       # package-local IDL
├── xyz_plotter/
│   ├── __init__.py
│   └── main.py                         # lifecycle, mutex, status stream
├── robonix_sdk/                        # vendored SDK transport
│   ├── protocol.py                     # SOF / CRC16-Modbus / f32 / f64
│   ├── client.py                       # per-point ACK, SET_PARAM, HOME, CLOSE
│   └── errors.py
├── scripts/
│   ├── build.sh                        # rbnx codegen
│   └── start.sh                        # exec main
├── tests/protocol_test.py
├── config.spec
└── CAPABILITY.md
```

`xyz_plotter/main.py` maps Robonix contracts onto the SDK client. The
vendored modules under `robonix_sdk/` mirror `SDK/robonix_sdk/{protocol,
client}.py` and exist only for this primitive; skills never import them.

## Config (passed via `Driver(CMD_INIT, config_json)`)

```yaml
host:              192.168.33.130   # PLC IPv4 / hostname
port:              2000             # PLC TCP port
timeout_s:         30.0             # connect timeout and per-point ACK floor
connect_on_init:   true             # false -> first motion call connects
units:             mm               # only "mm" is supported
z_up:              -46.0            # pen-up height
z_down:            -50.0            # pen-down height; calibrate by hand
velocity:          20.0             # mm/s
acceleration:      300.0            # mm/s^2
deceleration:      300.0            # mm/s^2
jerk:              3000.0           # mm/s^3
status_rate_hz:    2.0              # synthesized status stream rate
max_points:        100000           # max waypoints in one trajectory
```

Pen heights must satisfy `-50 <= z_down < z_up <= 0` in the paper frame.
Machine zero `(0,0,0)` is 50 mm above the paper's lower-left corner, so the
paper surface and `z_down` are `-50`.

The global profile is sent with the PLC's `SET_PARAM` (six `float32` values:
velocity, acceleration, deceleration, jerk, `z_up`, `z_down`). It affects
every subsequent `MOVE_LINEAR` until changed or the PLC restarts.

## Protocol limits

- **No STOP command.** Once `MOVE_LINEAR` is sent, Robonix cannot abort it;
  emergency stop is PLC-side only.
- **No position readback.** `x/y/z` are the last commanded/successful points,
  not encoder or limit-switch feedback.
- **No batched paths.** One motion command is in flight at a time;
  `BufferMode` has no effect.
- **Global parameters only.** There is no per-point speed or acceleration.
- **Home after power-up.** The axis group needs a valid reference position;
  run `plotter_home` before moving.
- **Pen-up guard.** If the last known position is below `z_up`, the primitive
  lifts the pen before starting a trajectory.

This primitive knows coordinates, frames, CRC, and ACKs. It does not know
what a glyph or an image is; that belongs to the skills.

## Build / run standalone

```bash
bash scripts/build.sh          # uv sync + rbnx codegen
uv run --project ../../.. pytest tests -q
ROBONIX_ATLAS=127.0.0.1:50051 \
    bash scripts/start.sh      # registers with Atlas and awaits Init
```

`scripts/build.sh` runs the shared root `uv_sync.sh` before codegen, and
`scripts/start.sh` uses the root `.venv` interpreter. Normally `rbnx boot`
launches the primitive from the manifest.

## Verification

```bash
rbnx caps | grep primitive/plotter
# Expected: xyz_plotter provider with
#   robonix/primitive/plotter/{driver,trajectory,home,set_params,status}
```

Then ask Pilot to run `plotter_home` and confirm the status stream reports
`connected` and `IDLE`. A successful home followed by a one-point dry run is
the minimum end-to-end check before drawing.

## License

MIT. See [LICENSE](../../../LICENSE).