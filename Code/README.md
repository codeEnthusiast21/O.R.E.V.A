# OREVA LiDAR Core

Hardware-independent YDLIDAR X2 packet decoding and scan assembly for OREVA.
This is the first software module in the repository. It validates the
receive-side data path before any ROS 2 or mapping code is introduced.

## Scope

This module currently provides:

- Incremental framing of an X2 serial byte stream.
- X2 packet validation, checksum verification, and resynchronisation.
- Decoding of 2-byte X2 samples into angle and distance points.
- Scan assembly across packets using start-of-revolution packets, with a
  conservative angle-wrap fallback.
- Explicit scan end reasons and completeness metadata.
- Bounded-memory safeguards for malformed or missing revolution boundaries.

It intentionally does **not** yet provide:

- Distance-range filtering or scan-usability decisions.
- Sensor-health monitoring or fault reporting.
- Timestamped scan logs, replay, or raw capture files.
- Serial-port integration.
- A command-line interface.
- ROS 2 nodes, `sensor_msgs/LaserScan`, TF, SLAM, or navigation.
- Hardware validation of any kind.

## Protocol basis

The implementation follows the **YDLIDAR X2 Development Manual
(DOC#:01.13.000101)** and the official **YDLIDAR SDK communication protocol**
documentation. The X2 packet layout is:

```text
PH(2) | CT(1) | LSN(1) | FSA(2) | LSA(2) | CS(2) | S1(2) ... Sn(2)
```

- `PH` is `0x55AA` in little-endian form, transmitted as `AA 55`.
- `CT` bit 0 marks the first packet of a revolution.
- For a start packet, `CT >> 1 / 10` reports the previous frame's scan
  frequency.
- `LSN` is the number of 2-byte samples.
- `FSA` and `LSA` encode angles as `(raw >> 1) / 64` degrees.
- `CS` is the XOR of `PH`, `FSA`, every 16-bit sample, `(LSN << 8 | CT)`, and
  `LSA`.
- Distance is `raw / 4` millimetres, retained as `raw / 4000` metres.

The X2 uses 2-byte samples without an intensity field. Firmware variants that
use a different sample width must be treated as a separate profile; this
decoder will reject them rather than silently misinterpreting the bytes.

References:

- [YDLIDAR X2 Development Manual](https://static.generation-robots.com/media/manuel-developpement-ylidar-x2.pdf)
- [YDLIDAR SDK communication protocol](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/YDLidar-SDK-Communication-Protocol.md)

## Installation

Python 3.10 or newer is required. There are no runtime dependencies.

```bash
cd Code
python3 -m venv .venv
.venv/bin/pip install -e .
```

For linting and type-checking tools:

```bash
.venv/bin/pip install -e '.[dev]'
```

## Usage

The core is deliberately usable from byte buffers, a serial library, a replay
file, or a test fixture.

```python
from oreva_lidar import PacketFramer, ScanAssembler

framer = PacketFramer()
assembler = ScanAssembler()

# A caller that reads from serial would call feed() for every read chunk.
batch = framer.feed(
    chunk,
    received_unix_ns=unix_ns,
    received_mono_ns=mono_ns,
)

for error in batch.errors:
    print(f"LiDAR parse error at {error.stream_offset}: {error.reason}: {error.detail}")

for packet in batch.packets:
    scan = assembler.add(packet)
    if scan is not None:
        print(
            scan.sequence,
            len(scan.points),
            scan.complete,
            scan.end_reason,
            scan.angular_coverage_deg,
        )

final_scan = assembler.finish()
```

### Timestamp semantics

The X2 packet format has no hardware timestamp. Both timestamps are host-side
receive times:

- `received_unix_ns` is wall-clock time and is suitable for cross-session
  correlation.
- `received_mono_ns` is monotonic time and is used for scan duration and clock
  regression checks.

All packets completed by one `feed()` call share that call's timestamps. A
caller that needs finer timing should feed smaller chunks. The assembler uses
monotonic time for control decisions; wall-clock adjustments do not split a
scan, but a wall-clock jump can make `start_unix_ns` and `end_unix_ns`
non-monotonic in recorded data.

### Scan boundaries

The X2 start packet is the primary revolution boundary. The assembler closes
the current scan when it sees one and starts the next scan with that packet.

If start packets are missing, the assembler falls back to an angle wrap: a
packet whose start angle returns to the low degrees after the previous packet
ended above 300 degrees. The fallback is disabled for scans that began with a
start packet, because those scans already have a reliable boundary and the
fallback could otherwise split a valid scan near zero degrees.

Scans are also closed as incomplete when:

- The point limit would be exceeded by the next packet.
- The monotonic duration limit is reached.
- The monotonic clock moves backwards.
- End of stream is reached via `finish()`.

`LidarScan.complete` is true only for `START_PACKET` and `ANGLE_WRAP` endings.
It does not mean the scan has full 360-degree coverage; use
`angular_coverage_deg` and point count for that.

### Data retention

Zero-distance samples (no return) and out-of-range samples are retained with
their raw values. No filtering is applied in this module. This keeps the wire
decoder honest and lets a later validation layer make explicit decisions.

## Testing

Tests use only the Python standard library and synthetic packets. The manual's
published start packet and angle-correction example are used as reference
vectors. No test uses a physical LiDAR.

```bash
cd Code
python3 -m venv .venv
.venv/bin/pip install -e '.[dev]'
.venv/bin/python -m unittest discover -s tests -t . -v
.venv/bin/python -m compileall -q src tests
.venv/bin/ruff check src tests
.venv/bin/mypy src
```

If the package is not installed, the unit tests can also be run with:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -t . -v
```

The test suite covers normal decoding, chunk-boundary splits, garbage and
power-on messages, false headers, checksum and framing failures, angle wrap,
start-packet boundaries, incomplete scans, safety limits, clock regression,
and configuration errors.

## Assumptions and limitations

- The implementation is specific to the X2 2-byte-sample protocol. It has not
  been verified against a physical X2 in this repository.
- Host receive timestamps are not sensor acquisition timestamps, and no motion
  compensation is applied.
- The angle-wrap fallback is a heuristic for captures without start packets.
  A large data gap from a high angle to a low angle can look like a revolution
  boundary.
- Single-packet burst behaviour is bounded by the packet size: a scan can
  exceed `max_points_per_scan` by at most one packet when the configured limit
  is smaller than a packet.
- Scan quality, distance filtering, health, logging, and ROS 2 integration are
  deliberately deferred.
- Health and fault reporting, when added, must not be described as a verified
  physical emergency stop.

## Next integration steps

1. Add scan validation and health monitoring.
2. Add timestamped JSONL scan recording and replay.
3. Add a thin serial source for live X2 data.
4. Publish validated scans as `sensor_msgs/LaserScan` in a ROS 2 node on the
   `laser_link` frame, with diagnostics.
5. Validate the protocol assumptions against a real X2 capture.
