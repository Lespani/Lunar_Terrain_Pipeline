from __future__ import annotations

from dataclasses import dataclass
import json
from typing import List

import numpy as np
import rasterio

from .geotiff_map import GeoTiffTrafficabilityMap


@dataclass(slots=True)
class LayerSpec:
    name: str
    path: str
    weight: float
    invert: bool = False


def parse_layer_specs_json(layers_json: str) -> List[LayerSpec]:
    payload = json.loads(layers_json)
    if not isinstance(payload, list) or len(payload) == 0:
        raise ValueError("layers_json must be a non-empty JSON list.")

    specs: list[LayerSpec] = []
    for item in payload:
        if not isinstance(item, dict):
            raise ValueError("Each layer spec must be a JSON object.")
        specs.append(
            LayerSpec(
                name=str(item["name"]),
                path=str(item["path"]),
                weight=float(item.get("weight", 1.0)),
                invert=bool(item.get("invert", False)),
            )
        )

    total_weight = sum(spec.weight for spec in specs)
    if total_weight <= 0:
        raise ValueError("Sum of layer weights must be > 0.")

    return specs


def _load_normalized_layer(path: str, invert: bool):
    with rasterio.open(path) as dataset:
        band = dataset.read(1).astype(np.float32)
        transform = dataset.transform
        width = dataset.width
        height = dataset.height
        no_data = dataset.nodata

    valid_mask = np.isfinite(band)
    if no_data is not None:
        valid_mask &= band != no_data

    if np.any(valid_mask):
        min_val = float(np.min(band[valid_mask]))
        max_val = float(np.max(band[valid_mask]))
    else:
        min_val = 0.0
        max_val = 0.0

    if max_val == min_val:
        norm = np.zeros_like(band, dtype=np.float32)
    else:
        norm = (band - min_val) / (max_val - min_val)

    norm = np.clip(norm, 0.0, 1.0)
    if invert:
        norm = 1.0 - norm

    return norm, valid_mask, transform, width, height


def build_fused_trafficability_map(
    specs: list[LayerSpec],
    free_threshold: float,
    occupied_threshold: float,
    unknown_value: int,
) -> GeoTiffTrafficabilityMap:
    score_sum = None
    weight_sum = None
    base_transform = None
    base_width = None
    base_height = None

    for spec in specs:
        norm, valid_mask, transform, width, height = _load_normalized_layer(spec.path, spec.invert)

        if base_transform is None:
            base_transform = transform
            base_width = width
            base_height = height
            score_sum = np.zeros((height, width), dtype=np.float32)
            weight_sum = np.zeros((height, width), dtype=np.float32)
        else:
            if width != base_width or height != base_height:
                raise ValueError("All layer rasters must have the same width/height.")
            if transform != base_transform:
                raise ValueError("All layer rasters must use the same geotransform.")

        score_sum[valid_mask] += norm[valid_mask] * spec.weight
        weight_sum[valid_mask] += spec.weight

    fused_norm = np.zeros_like(score_sum, dtype=np.float32)
    valid_fused = weight_sum > 0
    fused_norm[valid_fused] = score_sum[valid_fused] / weight_sum[valid_fused]

    occ = np.full(fused_norm.shape, unknown_value, dtype=np.int8)
    occ[valid_fused & (fused_norm <= free_threshold)] = 0
    occ[valid_fused & (fused_norm >= occupied_threshold)] = 100

    middle_mask = valid_fused & (fused_norm > free_threshold) & (fused_norm < occupied_threshold)
    occ[middle_mask] = (
        (fused_norm[middle_mask] - free_threshold)
        / (occupied_threshold - free_threshold)
        * 100.0
    ).astype(np.int8)

    return GeoTiffTrafficabilityMap(
        data=occ,
        transform=base_transform,
        width=base_width,
        height=base_height,
        resolution_x=abs(base_transform.a),
        resolution_y=abs(base_transform.e),
        no_data=None,
    )
