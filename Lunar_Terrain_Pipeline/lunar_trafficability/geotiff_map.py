from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import rasterio
from rasterio.transform import Affine


@dataclass(slots=True)
class GeoTiffTrafficabilityMap:
    data: np.ndarray
    transform: Affine
    width: int
    height: int
    resolution_x: float
    resolution_y: float
    no_data: Optional[float]

    @classmethod
    def from_file(
        cls,
        path: str,
        free_threshold: float,
        occupied_threshold: float,
        invert: bool,
        unknown_value: int,
    ) -> "GeoTiffTrafficabilityMap":
        with rasterio.open(path) as dataset:
            band = dataset.read(1).astype(np.float32)
            transform = dataset.transform
            no_data = dataset.nodata
            width = dataset.width
            height = dataset.height

        if no_data is not None:
            band[band == no_data] = np.nan

        min_val = np.nanmin(band)
        max_val = np.nanmax(band)
        if max_val == min_val:
            norm = np.zeros_like(band)
        else:
            norm = (band - min_val) / (max_val - min_val)

        if invert:
            norm = 1.0 - norm

        occ = np.full(norm.shape, unknown_value, dtype=np.int8)
        occ[norm <= free_threshold] = 0
        occ[norm >= occupied_threshold] = 100

        middle_mask = (norm > free_threshold) & (norm < occupied_threshold)
        occ[middle_mask] = ((norm[middle_mask] - free_threshold) / (occupied_threshold - free_threshold) * 100.0).astype(np.int8)

        return cls(
            data=occ,
            transform=transform,
            width=width,
            height=height,
            resolution_x=abs(transform.a),
            resolution_y=abs(transform.e),
            no_data=no_data,
        )

    def world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        inverse = ~self.transform
        col_f, row_f = inverse * (x, y)
        return int(round(col_f)), int(round(row_f))

    def pixel_to_world(self, col: int, row: int) -> Tuple[float, float]:
        x, y = self.transform * (col, row)
        return float(x), float(y)

    def extract_patch(self, center_x: float, center_y: float, size_m: float) -> Tuple[np.ndarray, float, float]:
        center_col, center_row = self.world_to_pixel(center_x, center_y)

        half_cols = max(int(size_m / self.resolution_x / 2.0), 1)
        half_rows = max(int(size_m / self.resolution_y / 2.0), 1)

        min_col = max(center_col - half_cols, 0)
        max_col = min(center_col + half_cols, self.width - 1)
        min_row = max(center_row - half_rows, 0)
        max_row = min(center_row + half_rows, self.height - 1)

        patch = self.data[min_row : max_row + 1, min_col : max_col + 1]
        origin_x, origin_y = self.pixel_to_world(min_col, min_row)
        return patch, origin_x, origin_y

    def contains_world_point(self, x: float, y: float) -> bool:
        col, row = self.world_to_pixel(x, y)
        return 0 <= col < self.width and 0 <= row < self.height
