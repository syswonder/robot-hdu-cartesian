# PLC TCP Protocol

> **Authority.** The authoritative protocol specification lives in the PLC-side
> repository (https://github.com/HDU-FROST/PLC-cartesian). This document is the
> Robonix-side summary used to review the primitive implementation; field-level
> PLC state machines and packet captures remain in the SDK documentation.
>
> Implementations: [`protocol.py`](../packages/primitives/primitive-hdu-cartesian-plotter-rbnx/xyz_plotter/sdk/protocol.py)
> and [`client.py`](../packages/primitives/primitive-hdu-cartesian-plotter-rbnx/xyz_plotter/sdk/client.py).

## 1. Transport

| Property | Value |
| --- | --- |
| Roles | PLC is the TCP server; the primitive is the client |
| Framing | Binary, little-endian |
| Configured endpoint | `192.168.1.135:2000` (from the deployment manifest) |
| Coordinates | IEEE 754 `float64`, millimetres |
| Motion parameters | IEEE 754 `float32` |
| Maximum payload | 512 B (maximum frame 520 B) |

## 2. Frame format

```text
Offset  Size   Field
0       2      SOF = 0xAA 0x55
2       1      CMD   (U8)
3       2      LEN   (U16, little-endian)
5       1      SEQ   (U8, echoed in the ACK)
6       LEN    PAYLOAD
6+LEN   2      CRC16-Modbus (U16, little-endian)
```

- Total frame length is `6 + LEN + 2` bytes.
- CRC16-Modbus uses the reflected polynomial `0x8005` with initial value
  `0xFFFF`. It covers `CMD + LEN + SEQ + PAYLOAD`; SOF and the CRC field are
  excluded.
- Receivers resynchronize on SOF and discard one byte at a time when the CRC
  or length is invalid.

## 3. Commands

| CMD | Name | Direction | Payload |
| --- | --- | --- | --- |
| `0x00` | `PING` | client → PLC | empty |
| `0x01` | `MOVE_LINEAR` | client → PLC | `x, y, z` as 3 × `float64` (24 B) |
| `0x03` | `HOME` | client → PLC | empty |
| `0x04` | `SET_PARAM` | client → PLC | `vel, acc, dec, jerk, z_up, z_down` as 6 × `float32` (24 B) |
| `0x0A` | `CLOSE` | client → PLC | empty |
| `0x80` | `ACK` | PLC → client | `status U8 + err U16 + busy U8` (4 B) |

`MOVE_LINEAR` is an absolute linear move in machine coordinates. Pen-up and
pen-down are the same command with different Z values; the PLC has no notion
of drawing versus travel.

## 4. ACK semantics

- Every client command must receive an ACK with the matching `SEQ`.
- `MOVE_LINEAR` and `HOME` ACK only after motion completes (`Done`) or fails
  (`Error`). They are synchronous commands, not fire-and-forget writes.

| Field | Meaning |
| --- | --- |
| `status` | `0` OK, `1` busy, `2` error, `3` unknown command |
| `err` | PLC `ErrorID`; valid when `status=2` |
| `busy` | `0` idle, `1` axis group busy |

The primitive estimates each point's timeout from travel distance, using a
conservative 5 mm/s speed:

```text
timeout = min(300, max(configured_timeout, 2 + distance / 5))  seconds
```

The first point uses a fixed 30 s budget. The configured `timeout_s` is the
floor for every point, not just the initial one.

## 5. Session lifecycle

Before closing the socket, the client must send `CLOSE(0x0A)` and wait for its
ACK (3 s fallback):

```text
client: CLOSE(seq=n)
PLC:    ACK(seq=n, status=0)
client: close(socket)
```

An abrupt socket close can leave the PLC's `SktTCPRcv` reporting errors while
`SktGetTCPStatus` still considers the connection valid, wedging the state
machine between states `20` and `60`. Recovery requires a PLC restart or
resetting `Tcp.TCP_State` to `0` in CASS.

## 6. Coordinates and parameters

- Units are millimetres; the frame origin and axis directions are defined in
  the [deployment README](../README.md#hardware).
- `MOVE_LINEAR` requires a valid reference position; home the axis group
  before sending absolute trajectories.
- `SET_PARAM` updates global values for all subsequent moves. Partial updates
  use `0` to keep the current value. The primitive caches the last
  successfully transmitted profile.
- Default SDK profile: `vel=20`, `acc=300`, `dec=300`, `jerk=3000`,
  `z_up=-46`, `z_down=-50` (mm and mm/s units). These are deployment values,
  not protocol constants; see the manifest and the
  [primitive config.spec](../packages/primitives/primitive-hdu-cartesian-plotter-rbnx/config.spec).

## 7. Protocol limitations

- **No `STOP`.** A motion already transmitted cannot be aborted by software.
  Emergency stop is PLC-side only.
- **No `STATUS`.** The primitive synthesizes its status stream from local
  connection state, the busy flag, and command progress; positions are last
  commanded values, not encoder feedback.
- **No batching.** One motion command is in flight at a time; `BufferMode`
  has no effect.
- **No per-point profile.** Speed, acceleration, deceleration, and jerk are
  global until changed or the PLC restarts.
- **No readback.** Parameters and positions cannot be queried from the PLC
  through this protocol.