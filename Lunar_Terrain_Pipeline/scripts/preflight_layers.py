from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import rasterio


@dataclass(slots=True)
class LayerMeta:
    name: str
    path: Path
    width: int
    height: int
    crs: str
    transform: object
    resolution: tuple[float, float]
    nodata: float | None


def parse_layers_json(value: str) -> list[dict]:
    payload = json.loads(value)
    if not isinstance(payload, list) or len(payload) == 0:
        raise ValueError("layers_json must be a non-empty JSON list")
    return payload


def parse_layers_file(path: Path) -> list[dict]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, list) or len(payload) == 0:
        raise ValueError("layers file must contain a non-empty JSON list")
    return payload


def describe_layer(path: Path) -> LayerMeta:
    with rasterio.open(path) as src:
        return LayerMeta(
            name=path.stem,
            path=path,
            width=src.width,
            height=src.height,
            crs=str(src.crs),
            transform=src.transform,
            resolution=(float(abs(src.transform.a)), float(abs(src.transform.e))),
            nodata=src.nodata,
        )


def validate_compatibility(metas: Iterable[LayerMeta]) -> list[str]:
    metas = list(metas)
    base = metas[0]
    issues: list[str] = []
    for other in metas[1:]:
        if (other.width, other.height) != (base.width, base.height):
            issues.append(
                f"Size mismatch: {other.path.name} has {other.width}x{other.height}, expected {base.width}x{base.height}"
            )
        if other.crs != base.crs:
            issues.append(f"CRS mismatch: {other.path.name} has {other.crs}, expected {base.crs}")
        if other.transform != base.transform:
            issues.append(f"Transform mismatch: {other.path.name} differs from {base.path.name}")
    return issues


def normalize_band(arr: np.ndarray, nodata: float | None) -> tuple[np.ndarray, np.ndarray]:
    valid = np.isfinite(arr)
    if nodata is not None:
        valid &= arr != nodata

    norm = np.zeros_like(arr, dtype=np.float32)
    if np.any(valid):
        min_val = float(np.min(arr[valid]))
        max_val = float(np.max(arr[valid]))
        if max_val > min_val:
            norm[valid] = (arr[valid] - min_val) / (max_val - min_val)
    return np.clip(norm, 0.0, 1.0), valid


def load_norm(path: Path, invert: bool) -> tuple[np.ndarray, np.ndarray]:
    with rasterio.open(path) as src:
        arr = src.read(1).astype(np.float32)
        nodata = src.nodata
    norm, valid = normalize_band(arr, nodata)
    if invert:
        norm = 1.0 - norm
    return norm, valid


def save_visuals(output_dir: Path, layer_name: str, norm: np.ndarray, valid: np.ndarray) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover
        print(f"[warn] matplotlib unavailable, skipping image outputs: {exc}")
        return

    output_dir.mkdir(parents=True, exist_ok=True)

    vis = np.where(valid, norm, np.nan)

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111)
    im = ax.imshow(vis, cmap="viridis")
    ax.set_title(f"{layer_name}: normalized")
    fig.colorbar(im, ax=ax)
    fig.tight_layout()
    fig.savefig(output_dir / f"{layer_name}_map.png", dpi=150)
    plt.close(fig)

    fig = plt.figure(figsize=(8, 4))
    ax = fig.add_subplot(111)
    values = vis[np.isfinite(vis)]
    ax.hist(values, bins=50)
    ax.set_title(f"{layer_name}: value histogram")
    ax.set_xlabel("normalized value")
    ax.set_ylabel("count")
    fig.tight_layout()
    fig.savefig(output_dir / f"{layer_name}_hist.png", dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate and preview multi-layer lunar GeoTIFF inputs")
    layers_group = parser.add_mutually_exclusive_group(required=True)
    layers_group.add_argument(
        "--layers-json",
        help="JSON list: [{\"name\":...,\"path\":...,\"weight\":...,\"invert\":...}, ...]",
    )
    layers_group.add_argument(
        "--layers-file",
        help="Path to JSON file containing a list of layer specs",
    )
    parser.add_argument(
        "--output-dir",
        default="artifacts/layer_preflight",
        help="Directory for reports and PNG previews",
    )
    args = parser.parse_args()

    if args.layers_file:
        specs = parse_layers_file(Path(args.layers_file))
    else:
        specs = parse_layers_json(args.layers_json)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    paths = [Path(spec["path"]) for spec in specs]
    missing = [str(path) for path in paths if not path.exists()]
    if missing:
        raise FileNotFoundError(f"Missing layer files: {missing}")

    metas = [describe_layer(path) for path in paths]
    issues = validate_compatibility(metas)

    report = {
        "layers": [
            {
                "name": meta.name,
                "path": str(meta.path),
                "width": meta.width,
                "height": meta.height,
                "crs": meta.crs,
                "resolution": [meta.resolution[0], meta.resolution[1]],
                "nodata": meta.nodata,
            }
            for meta in metas
        ],
        "compatible": len(issues) == 0,
        "issues": issues,
    }

    fused_sum = None
    fused_weight = None

    for spec in specs:
        layer_name = str(spec["name"])
        path = Path(spec["path"])
        weight = float(spec.get("weight", 1.0))
        invert = bool(spec.get("invert", False))

        norm, valid = load_norm(path, invert=invert)
        save_visuals(output_dir, layer_name, norm, valid)

        if fused_sum is None:
            fused_sum = np.zeros_like(norm, dtype=np.float32)
            fused_weight = np.zeros_like(norm, dtype=np.float32)

        fused_sum[valid] += norm[valid] * weight
        fused_weight[valid] += weight

    fused = np.zeros_like(fused_sum, dtype=np.float32)
    valid_fused = fused_weight > 0
    fused[valid_fused] = fused_sum[valid_fused] / fused_weight[valid_fused]
    save_visuals(output_dir, "fused", fused, valid_fused)

    report_path = output_dir / "report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"Report written: {report_path}")
    print(f"Compatible: {report['compatible']}")
    if issues:
        for issue in issues:
            print(f"- {issue}")


if __name__ == "__main__":
    main()
