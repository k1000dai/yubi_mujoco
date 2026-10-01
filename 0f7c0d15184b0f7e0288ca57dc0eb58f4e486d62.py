#!/usr/bin/env python3
"""Fetch or verify the one pinned source assembly used by this project."""

from pathlib import Path
import hashlib
import urllib.request

ROOT = Path(__file__).resolve().parent
NAME = "YUBI Gripper Assy_Dynamixel_ver2.STEP"
SHA256 = "0e60bf62de970e5fae5e7cff104f547a954ef8d5d7b9dde9835b381b95f4fcc8"
URL = (
    "https://raw.githubusercontent.com/Toyota/yubi-hw/"
    "dd8bd13d2fd8e5003057243576f88be333d95fc5/STEP/gripper/"
    "YUBI%20Gripper%20Assy_Dynamixel_ver2.STEP"
)


def main():
    target = ROOT / "source" / NAME
    if target.exists():
        data = target.read_bytes()
        if hashlib.sha256(data).hexdigest() != SHA256:
            raise SystemExit(f"Existing source differs from the pinned SHA-256: {target}")
        print(f"Verified pinned source: {target}")
        return
    with urllib.request.urlopen(URL, timeout=60) as response:
        data = response.read()
    if hashlib.sha256(data).hexdigest() != SHA256:
        raise SystemExit("Downloaded source differs from the pinned SHA-256; no file written")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(data)
    print(f"Downloaded and verified pinned source: {target}")


if __name__ == "__main__":
    main()
