"""Write RGB PNG images without optional video dependencies."""

from pathlib import Path
import struct
import zlib

import numpy as np


def write_png(path, pixels):
    """Write an HWC uint8 RGB image as a lossless PNG."""
    pixels = np.asarray(pixels)
    if pixels.ndim != 3 or pixels.shape[2] != 3 or pixels.dtype != np.uint8:
        raise ValueError("pixels must have shape (height, width, 3) and dtype uint8")
    height, width, _ = pixels.shape
    if not height or not width:
        raise ValueError("image dimensions must be positive")

    def chunk(kind, data):
        return (
            struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))
        )

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    rows = b"".join(b"\0" + row.tobytes() for row in pixels)
    Path(path).write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(rows))
        + chunk(b"IEND", b"")
    )
