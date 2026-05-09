#!/usr/bin/env python3
"""Download the SuperPoint ONNX model used by tbai_estim's SuperPointExtractor.

The model is the standalone SuperPoint export from the LightGlue-ONNX
release page (the same source kornia.feature.OnnxLightGlue uses for its
fused models). It accepts a grayscale [1, 1, H, W] float32 image in [0, 1]
and produces (keypoints int64 [1, N, 2], scores float32 [1, N],
descriptors float32 [1, N, 256]).

Note: the SuperPoint pretrained weights are released under MagicLeap's
non-commercial research license. See
https://github.com/magicleap/SuperPointPretrainedNetwork.

Usage:
  python tbai_estim/scripts/download_superpoint_onnx.py --output ~/.tbai/superpoint.onnx
"""

import argparse
import hashlib
import os
import sys
import urllib.request
from pathlib import Path


SUPERPOINT_URL = "https://github.com/fabio-sim/LightGlue-ONNX/releases/download/v1.0.0/superpoint.onnx"
EXPECTED_SIZE_BYTES = 5_268_736  # ~5.1 MB; sanity check, not a hash
EXPECTED_SIZE_TOLERANCE = 64 * 1024  # +/- 64 KB


def _download(url: str, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    print(f"Downloading {url}\n         -> {dst}")
    tmp = dst.with_suffix(dst.suffix + ".tmp")
    with urllib.request.urlopen(url) as resp, open(tmp, "wb") as out:
        total = 0
        while True:
            chunk = resp.read(64 * 1024)
            if not chunk:
                break
            out.write(chunk)
            total += len(chunk)
    tmp.rename(dst)
    print(f"Wrote {total} bytes")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--output", "-o", type=Path, required=True, help="Destination ONNX file path")
    ap.add_argument("--force", action="store_true", help="Re-download even if the file exists")
    ap.add_argument("--url", default=SUPERPOINT_URL, help="Override the source URL")
    args = ap.parse_args()

    if args.output.exists() and not args.force:
        print(f"{args.output} already exists. Use --force to re-download.")
        return 0

    _download(args.url, args.output)

    size = args.output.stat().st_size
    if abs(size - EXPECTED_SIZE_BYTES) > EXPECTED_SIZE_TOLERANCE:
        print(
            f"WARNING: downloaded file size {size} differs from expected "
            f"{EXPECTED_SIZE_BYTES} +/- {EXPECTED_SIZE_TOLERANCE} bytes",
            file=sys.stderr,
        )

    sha = hashlib.sha256(args.output.read_bytes()).hexdigest()
    print(f"sha256: {sha}")
    print(f"\nSet TBAI_SUPERPOINT_ONNX={args.output} to point the C++ extractor at this file.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
