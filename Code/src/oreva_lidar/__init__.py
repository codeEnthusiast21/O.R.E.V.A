"""Core YDLIDAR X2 packet decoding and scan assembly for OREVA."""

from .laserscan import (
    DEFAULT_BIN_COUNT,
    DEFAULT_FRAME_ID,
    LaserScanFrame,
    to_laser_scan_frame,
)
from .protocol import (
    HEADER_BYTES,
    HEADER_WORD,
    MAX_PACKET_BYTES,
    MAX_SAMPLES_PER_PACKET,
    PACKET_HEADER_SIZE,
    SAMPLE_SIZE,
    X2_BAUD_RATE,
    LidarPacket,
    LidarPoint,
    PacketDecodeError,
    PacketError,
    PacketFramer,
    ParseBatch,
    ParseErrorReason,
    compute_checksum,
    decode_packet,
)
from .scan import (
    DEFAULT_MAX_POINTS_PER_SCAN,
    DEFAULT_MAX_SCAN_DURATION_S,
    DEFAULT_WRAP_NEXT_MAX_DEG,
    DEFAULT_WRAP_PREV_MIN_DEG,
    LidarScan,
    ScanAssembler,
    ScanEndReason,
)

__version__ = "0.1.0"

__all__ = [
    "DEFAULT_BIN_COUNT",
    "DEFAULT_FRAME_ID",
    "DEFAULT_MAX_POINTS_PER_SCAN",
    "DEFAULT_MAX_SCAN_DURATION_S",
    "DEFAULT_WRAP_NEXT_MAX_DEG",
    "DEFAULT_WRAP_PREV_MIN_DEG",
    "HEADER_BYTES",
    "HEADER_WORD",
    "MAX_PACKET_BYTES",
    "MAX_SAMPLES_PER_PACKET",
    "PACKET_HEADER_SIZE",
    "SAMPLE_SIZE",
    "X2_BAUD_RATE",
    "LaserScanFrame",
    "LidarPacket",
    "LidarPoint",
    "LidarScan",
    "PacketDecodeError",
    "PacketError",
    "PacketFramer",
    "ParseBatch",
    "ParseErrorReason",
    "ScanAssembler",
    "ScanEndReason",
    "compute_checksum",
    "decode_packet",
    "to_laser_scan_frame",
]
