from __future__ import annotations

import argparse
import os
import platform
import subprocess
from pathlib import Path


def open_path(path: Path) -> None:
    system = platform.system().lower()

    if system == "windows":
        os.startfile(str(path))
        return

    if system == "darwin":
        subprocess.run(["open", str(path)], check=False)
        return

    subprocess.run(["xdg-open", str(path)], check=False)


def main() -> None:
    parser = argparse.ArgumentParser(description="Open generated preflight visual outputs")
    parser.add_argument(
        "--output-dir",
        default="artifacts/layer_preflight",
        help="Directory created by preflight_layers.py",
    )
    parser.add_argument(
        "--maps-only",
        action="store_true",
        help="Open only *_map.png files (skip histograms)",
    )
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    if not output_dir.exists():
        raise FileNotFoundError(f"Output directory not found: {output_dir}")

    pngs = sorted(output_dir.glob("*_map.png" if args.maps_only else "*.png"))
    if not pngs:
        raise FileNotFoundError(f"No PNG outputs found in: {output_dir}")

    open_path(output_dir)
    for png in pngs:
        open_path(png)

    print(f"Opened {len(pngs)} image(s) from {output_dir}")


if __name__ == "__main__":
    main()
