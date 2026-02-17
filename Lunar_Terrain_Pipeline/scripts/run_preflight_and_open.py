from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run layer preflight validation and open generated visual outputs"
    )
    layers_group = parser.add_mutually_exclusive_group(required=True)
    layers_group.add_argument(
        "--layers-json",
        help="JSON list of layer specs used by preflight_layers.py",
    )
    layers_group.add_argument(
        "--layers-file",
        help="Path to JSON file containing layer specs",
    )
    parser.add_argument(
        "--output-dir",
        default="artifacts/layer_preflight",
        help="Directory for preflight outputs",
    )
    parser.add_argument(
        "--maps-only",
        action="store_true",
        help="Open only *_map.png outputs",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    preflight_script = repo_root / "scripts" / "preflight_layers.py"
    open_script = repo_root / "scripts" / "open_preflight_outputs.py"

    preflight_cmd = [
        sys.executable,
        str(preflight_script),
        "--output-dir",
        args.output_dir,
    ]
    if args.layers_file:
        preflight_cmd.extend(["--layers-file", args.layers_file])
    else:
        preflight_cmd.extend(["--layers-json", args.layers_json])

    subprocess.run(preflight_cmd, check=True, cwd=str(repo_root))

    open_cmd = [
        sys.executable,
        str(open_script),
        "--output-dir",
        args.output_dir,
    ]
    if args.maps_only:
        open_cmd.append("--maps-only")

    subprocess.run(open_cmd, check=True, cwd=str(repo_root))


if __name__ == "__main__":
    main()
