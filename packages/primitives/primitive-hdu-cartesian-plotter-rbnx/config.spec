# Runtime config accepted by the XYZ plotter primitive.
#
# This file documents the mapping passed as this package's `config:` value in
# robonix_manifest.yaml. Missing keys use the defaults below.
#
# Protocol and coordinate conventions come from SDK/docs/protocol.md:
# paper origin (0,0) at the lower-left corner, +X right, +Y up, paper
# surface at z = -50, machine zero at (0,0,0) = 50 mm above the paper.

config:
  # string, default: 192.168.33.130.
  # PLC IPv4/hostname. The primitive is the only process that talks TCP.
  host: 192.168.33.130

  # int, default: 2000. PLC TCP port, 1..65535.
  port: 2000

  # float seconds, default: 30.0.
  # TCP connect timeout, and the floor for the per-point ACK timeout
  # (MOVE_LINEAR ACKs only after the move completes). Raise it when the
  # PLC velocity is very low or the travel is long.
  timeout_s: 30.0

  # bool, default: true.
  # true  -> CMD_INIT connects, PINGs the PLC and pushes the motion params;
  #          failure fails boot (fail fast).
  # false -> CMD_INIT only parses config; the first motion call connects.
  connect_on_init: true

  # string enum, default: mm. Only "mm" is supported; the wire protocol
  # and every contract value are millimetres.
  units: mm

  # float mm, default: -46.0. Pen-up height (4 mm above the paper).
  # Must satisfy z_min <= z_down < z_up <= z_max; z is encoded in every
  # waypoint, so the PLC does not use its own z_up/z_down for motion.
  z_up: -46.0

  # float mm, default: -50.0. Pen-down height (pen tip on the paper).
  # Machine-specific: verify it by hand before the first run.
  z_down: -50.0

  # float mm, default: -50.0. Lower Z travel limit used to validate pen
  # heights. Machine-specific: override for other rigs.
  z_min: -50.0

  # float mm, default: 0.0. Upper Z travel limit used to validate pen
  # heights. Machine-specific: override for other rigs.
  z_max: 0.0

  # float mm, default: 180.0. Drawable area width in the paper frame
  # (origin (0,0) at the paper corner; the x minimum is always 0).
  # Reported to skills via PlotterStatus; override for other rigs.
  area_x_max: 180.0

  # float mm, default: 150.0. Drawable area height (y minimum always 0).
  area_y_max: 150.0

  # Global motion profile, sent with CMD_SET_PARAM (6 x f32) on connect
  # and on every set_params call. The PLC has no per-point profile.
  # velocity mm/s (> 0), acceleration/deceleration mm/s^2 (> 0),
  # jerk mm/s^3 (> 0). These mirror the SDK defaults.
  # Partial set_params calls may pass 0 to keep the current value.
  velocity: 20.0
  acceleration: 300.0
  deceleration: 300.0
  jerk: 3000.0

  # float Hz, default: 2.0. Rate of the status stream. Must be > 0.
  # The status is synthesized from local state: the PLC has no STATUS
  # command in its current firmware.
  status_rate_hz: 2.0

  # int, default: 100000. Maximum waypoints accepted in one trajectory.
  max_points: 100000