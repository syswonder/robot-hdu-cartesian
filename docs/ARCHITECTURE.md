# Architecture

> **Scope.** This document describes the deployment's process topology,
> contract boundaries, lifecycle, execution semantics, and failure modes. It
> does not restate per-tool arguments or algorithms; those live in the package
> READMEs. Wire framing is in
> [PLC_PROTOCOL.md](PLC_PROTOCOL.md).

## 1. Design constraints

The deployment is deliberately small and deterministic:

- Robonix sends validated waypoints; it never executes model-generated code.
- Only one package owns the PLC socket. No skill knows the PLC address, frame
  format, CRC, or ACK encoding.
- Geometry is a pure function of the request. A request either produces a
  fully validated plan or no motion at all.
- The machine has no perception and no real-time feedback; correctness comes
  from planning-time validation and PLC ACKs, not closed-loop control.

## 2. Process topology

```text
Pilot / LLM
    │ MCP tool calls
    ├───────────────────────────┐
    ▼                           ▼
skill-text-rbnx         skill-draw-rbnx
robonix/skill/text/*        robonix/skill/plot/*
text → strokes → waypoints  scene / raster / prompt → waypoints
    │                           │
    └─────────────┬─────────────┘
                  │ gRPC: trajectory, home
                  ▼
primitive-hdu-cartesian-plotter-rbnx
robonix/primitive/plotter/*
lifecycle · busy mutex · pen-up guard · status synthesis
                  │ TCP: framed binary commands
                  ▼
external PLC (MatrixBox MX4001)
motion interpolation · limits · emergency stop
```

System services (`atlas`, `executor`, `soma`, `pilot`, `liaison`) are provided
by the Robonix CLI and started by `rbnx boot` before user packages. Atlas is
the discovery plane; Executor activates skills on their first MCP call.

## 3. Contract surface

### Primitive

| Contract | Mode | Transport |
| --- | --- | --- |
| `robonix/primitive/plotter/driver` | rpc | gRPC lifecycle |
| `robonix/primitive/plotter/trajectory` | rpc | gRPC |
| `robonix/primitive/plotter/home` | rpc + MCP | gRPC / MCP `plotter_home` |
| `robonix/primitive/plotter/set_params` | rpc + MCP | gRPC / MCP `plotter_set_params` |
| `robonix/primitive/plotter/geometry` | rpc | gRPC |
| `robonix/primitive/plotter/status` | topic_out | gRPC server stream |

### Skills

| Contract | Provider | Mode |
| --- | --- | --- |
| `robonix/skill/text/driver` | text | rpc |
| `robonix/skill/text/write_text` | text | rpc + MCP |
| `robonix/skill/plot/driver` | draw | rpc |
| `robonix/skill/plot/config` | draw | rpc + MCP |
| `robonix/skill/plot/draw_scene` | draw | rpc + MCP |
| `robonix/skill/plot/draw_prompt` | draw | rpc + MCP |
| `robonix/skill/plot/render_preview` | draw | rpc + MCP |
| `robonix/skill/plot/draw_image` | draw | rpc + MCP |
| `robonix/skill/plot/list_images` | draw | rpc + MCP |

The skills consume only `trajectory`, `home`, and `geometry`. `set_params`, `home`, and
`status` are also exposed directly to Pilot/LLM so motion parameters and
homing are not hidden behind a skill.

## 4. Package boundaries

### Why there is no service layer

A service layer would add an Atlas lookup and a process hop without adding
semantics: there is no perception, planning, or multi-device coordination.
Text-to-waypoints and raster-to-waypoints are pure conversions and belong in
skills. If perception is added later, it can be introduced as a service and
discovered through Atlas without changing the primitive contract.

### Why text and drawing are separate skills

| | Text | Drawing |
| --- | --- | --- |
| Input | String | JSON scene, raster file, or prompt |
| Configuration | Font size, spacing, origin | Directories, renderer, tracing, imagegen |
| File I/O | None | Controlled `image_dir` / `output_dir` |
| Typical failures | Missing glyphs, layout overflow | Path escape, invalid scene, blank image |

Separate packages keep each `config.spec` and MCP description small and
prevent fields such as `font_size_mm` from leaking into the drawing surface.

### Why the primitive is declared first

`rbnx boot` starts packages in manifest order and performs no retry of the
whole stack. Skills resolve `trajectory` and `home` when they are first
activated, so the primitive must already be registered in Atlas. Declaring
the primitive first avoids the activation retry delay.

## 5. Runtime lifecycle

### Primitive

- `CMD_INIT`: parse and validate config. With `connect_on_init: true`,
  connect to the PLC, `PING`, and push the default `SET_PARAM` profile. A
  connection or PING failure returns `Err`, failing boot early.
- `CMD_ACTIVATE`: mark active and reconnect if the PLC dropped while
  inactive.
- `CMD_DEACTIVATE`: stop the status stream. There is no `STOP` command, so an
  in-flight trajectory is not interrupted.
- `CMD_SHUTDOWN`: send `CMD_CLOSE`, wait for its ACK, then close the socket.

### Skills

- `CMD_INIT`: parse config and validate directories. No upstream discovery
  and no PLC connection.
- `CMD_ACTIVATE`: triggered by Executor on the first MCP call. Resolve
  `trajectory` and `home` through Atlas. Missing endpoints return `Deferred`;
  the next MCP call retries.
- Tool calls are synchronous: they return only after the primitive responds
  or the configured timeout expires.

Details per package:
[primitive](../packages/primitives/primitive-hdu-cartesian-plotter-rbnx/README.md),
[text](../packages/skills/skill-text-rbnx/README.md),
[draw](../packages/skills/skill-draw-rbnx/README.md).

## 6. Data flow

All motion paths share the same contract:

```text
request
  → validate config, paths, and input domain
  → build waypoints
  → validate workspace, pen heights, and point budget
  → [HOME before motion]
  → [debug preview, drawing only]
  → trajectory: one ACK per point
  → [auto_home after success]
  → response with plan summary or explicit failure
```

Geometry (drawable area, pen heights) is read once on activation through
`robonix/primitive/plotter/geometry`. `status` is diagnostic only and is never
used as a geometry source: it is a `topic_out` stream with no consumable
request/response stub.

Tool-specific transformations:

| Tool | Transformation | Motion |
| --- | --- | --- |
| `write_text` | Text → SDK single-stroke glyphs → scaled, spaced, wrapped polylines | Yes, unless `dry_run` |
| `draw_scene` | JSON scene → analytic polylines; no rasterisation | Yes, unless `dry_run` |
| `render_preview` | JSON scene → supersampled monochrome PNG with orientation metadata | No |
| `draw_prompt` | Prompt → external image service → PNG → trace → polylines | Yes, unless `dry_run` |
| `draw_image` | Raster → threshold/edge extraction → trace → polylines | Yes, unless `dry_run` |
| `list_images` | Enumerate allowed files under the read roots | No |
| `plot_config` | Return runtime policy flags | No |
| `plotter_home` | PLC homing routine to `(0,0,0)` | Yes |
| `plotter_set_params` | Update the global motion profile | No |

`dry_run` performs every validation and planning step but sends no
trajectory and never homes or previews. `draw_prompt` with `dry_run=true`
still generates and writes its PNG, because image generation precedes
planning. `render_preview`, `list_images`, and `plot_config` are
side-effect-free with respect to motion.

The `draw_prompt` failure path is a planner decision, not an automatic
fallback: the tool returns `success=false`, and Pilot may author a simple
declarative scene and call `draw_scene` next.

## 7. Execution semantics

The primitive implements a synchronous, one-job-at-a-time execution model:

1. Acquire the busy mutex. A second job returns a busy error instead of
   queueing.
2. If the last known position is below `z_up`, prepend a pen-up move at the
   current X/Y. This protection point is not counted in `points_completed`.
3. Send one `MOVE_LINEAR` per waypoint and wait for the matching `SEQ` ACK.
   The PLC ACKs after motion completion, so each point blocks.
4. Update the known position after every successful point and report
   `points_accepted` / `points_completed`.
5. On any ACK failure, stop immediately and return `success=false`. Motion
   commands are never retried automatically; retrying could duplicate a
   pen-down stroke. Connection-level failures are handled by reconnecting and
   re-sending `SET_PARAM` before the next job.

The wire-level meaning of ACKs, timeouts, and `CLOSE` is defined in
[PLC_PROTOCOL.md](PLC_PROTOCOL.md).

Drawing tools apply the capability policies in
[`config.spec`](../packages/skills/skill-draw-rbnx/config.spec) before
entering this execution model. Their order is HOME, then the debug preview, then the trajectory. A
failed HOME aborts before motion; a preview failure is non-fatal.

## 8. Codegen and contract mirroring

Skills need gRPC stubs for `Trajectory`, `Home`, and `Geometry`, but those contracts are
owned by the primitive. Each skill keeps a read-only mirror:

```text
skill-*/capabilities/
├── <own>.v1.toml                 # provided, listed in package_manifest.yaml
├── lib/...                       # own IDL
└── primitive/plotter/*.v1.toml   # mirrored, consume-only
    lib/plotter/{srv,msg}/...     # identical to the primitive IDL
```

Mirrored contracts are **not** listed under `capabilities` in
`package_manifest.yaml`. Atlas therefore does not treat the skill as a
provider; the mirrors exist only so `rbnx codegen --mcp` emits client stubs.

## 9. Runtime environment

The primitive and both skills share one uv-managed environment at the
repository root (`pyproject.toml`, `uv.lock`, `.python-version`, `.venv/`).
Each package `scripts/build.sh` calls `scripts/uv_sync.sh`, which serializes
`uv sync` with `flock` because `rbnx build` may invoke package builds in
parallel. Each `scripts/start.sh` executes the root `.venv/bin/python`.

`robonix_api` itself is not installed by uv. Start scripts prepend
`rbnx path robonix-api` to `PYTHONPATH`, while `pyproject.toml` pins its
runtime dependencies (`grpcio`, `protobuf`, `mcp`, `fastmcp`, `pyyaml`) so the
codegen stubs import against compatible versions.

## 10. Failure modes

| Condition | Behavior |
| --- | --- |
| PLC unreachable at boot (`connect_on_init: true`) | Primitive `CMD_INIT` returns `Err`; boot fails fast |
| PLC drops mid-job | Current trajectory fails; the next call reconnects and re-sends parameters |
| ACK timeout | Trajectory fails with completed-point count; `last_error` records the cause |
| HOME fails before a draw | Job aborts with `success=false`; no trajectory is sent |
| Preview render fails (`debug: true`) | Warning in `message`; drawing continues |
| Workspace or pen-height violation | Skill rejects the plan; no motion |
| Path outside `image_dir` / `output_dir` | `draw_image` / `list_images` reject before reading |
| Invalid or oversized JSON scene | `draw_scene` / `render_preview` reject the whole request |
| Raster produces no drawable strokes | `draw_image` fails with a hint to change `trace` or `invert` |
| Image generation unavailable or invalid | `draw_prompt` returns `success=false`; generated files, if any, remain in `output_dir` |
| Plan exceeds `max_points` | Rejected during planning; no partial trajectory |
| Concurrent job | Second job receives a busy error; nothing is queued |
| Emergency stop or mid-job failure | No software abort; the pen remains where it stopped and must be homed before retry |
| Missing `.venv` or codegen output | `scripts/start.sh` exits before starting the process |