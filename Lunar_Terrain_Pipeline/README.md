# Lunar Trafficability ROS 2 Node (Framework Starter)

This package gives you a first working framework to:

- load LROC GeoTIFF layers,
- estimate trafficability into an `OccupancyGrid`,
- project rover localization into the LROC map frame,
- publish both a global map and rover-centered local patch.

It is designed as a clean starting point for OmniLRS integration.

## What is included

- `lunar_trafficability/trafficability_node.py`
  - ROS 2 node with subscribe/publish interfaces
- `lunar_trafficability/geotiff_map.py`
  - GeoTIFF loading, normalization, threshold-to-occupancy conversion, patch extraction
- `config/trafficability.params.yaml`
  - Runtime parameters
- `launch/trafficability.launch.py`
  - Launch file

## Runtime interfaces

### Subscribed

- `/localization/pose` (`geometry_msgs/PoseStamped`)

### Published

- `/lunar/trafficability/global_grid` (`nav_msgs/OccupancyGrid`) latched
- `/lunar/trafficability/local_patch` (`nav_msgs/OccupancyGrid`)
- `/lunar/trafficability/rover_projected` (`geometry_msgs/PointStamped`)

## How the projection works

1. The node receives rover pose.
2. If needed, it transforms pose into `map_frame` using TF2.
3. It maps rover XY onto GeoTIFF coordinates.
4. It publishes:
   - rover projected point in map frame,
   - local occupancy patch around rover,
   - full global grid.

## Windows now, Ubuntu later (recommended workflow)

Use Windows for code authoring and unit-level logic checks. Run final ROS 2 integration in Ubuntu (native or WSL2), because ROS 2 and geospatial dependencies are much smoother there.

## Ubuntu build and run

Inside your ROS 2 workspace `src/` folder, place this package and run:

1. `colcon build --packages-select lunar_trafficability`
2. `source install/setup.bash`
3. Edit `config/trafficability.params.yaml` and set `geotiff_path`
4. `ros2 launch lunar_trafficability trafficability.launch.py params_file:=/absolute/path/to/trafficability.params.yaml`

## Important assumptions

- Localization node provides `PoseStamped` with coordinates consistent with your map frame or TF2 can transform them.
- GeoTIFF coordinates and localization coordinates must share a coherent reference frame/CRS.

## Multi-layer fusion (minerals + terrain + hazards)

The node supports two modes:

- Single-layer mode: set `geotiff_path`.
- Multi-layer mode: set `layers_json` (takes precedence over `geotiff_path`).

`layers_json` is a JSON list of layer specs:

```json
[
  {"name":"slope","path":"/data/lroc/slope.tif","weight":0.45,"invert":true},
  {"name":"rock_abundance","path":"/data/lroc/rocks.tif","weight":0.35,"invert":true},
  {"name":"mineral_proxy","path":"/data/lroc/mineral.tif","weight":0.20,"invert":false}
]
```

How fusion is computed:

1. Each layer is min-max normalized to `[0, 1]`.
2. Optional per-layer inversion is applied (`invert=true` flips desirability).
3. A weighted average is computed per pixel.
4. Final score is converted to occupancy via `free_threshold` and `occupied_threshold`.

Current constraint: all layers must share the same raster dimensions and geotransform.

## Next integration steps for OmniLRS

1. Replace the simple threshold model with your real trafficability metric (slope, roughness, rock abundance, shadow risk).
2. Add a map-to-rover projection output that exactly matches OmniLRS expected topic/type.
3. Add timestamp/quality metadata so downstream planning can gate uncertain regions.
4. Add tests with a tiny synthetic GeoTIFF and known rover poses.

## Preflight validation + visual checks (no ROS required)

Use the script below to verify all layers align and to generate visual previews:

Recommended (preset file):

```bash
python scripts/preflight_layers.py --layers-file config/layers.preset.json --output-dir artifacts/layer_preflight
```

Alternative (inline JSON):

```bash
python scripts/preflight_layers.py \
  --layers-json '[{"name":"slope","path":"slope.tif","weight":0.35,"invert":true},{"name":"roughness","path":"roughness.tif","weight":0.25,"invert":true},{"name":"temp_max","path":"temp_max.tif","weight":0.15,"invert":true},{"name":"temp_min","path":"temp_min.tif","weight":0.15,"invert":true},{"name":"hydrogen","path":"hydrogen.tif","weight":0.10,"invert":false}]' \
  --output-dir artifacts/layer_preflight
```

Outputs:

- `artifacts/layer_preflight/report.json` with compatibility checks (CRS, transform, dimensions)
- Per-layer PNG map + histogram (`*_map.png`, `*_hist.png`)
- Fused map + histogram (`fused_map.png`, `fused_hist.png`)

If `matplotlib` is missing, install it and rerun:

```bash
pip install matplotlib
```

Open generated previews quickly:

```bash
python scripts/open_preflight_outputs.py --output-dir artifacts/layer_preflight
```

Open only map images:

```bash
python scripts/open_preflight_outputs.py --output-dir artifacts/layer_preflight --maps-only
```

Run validation and open visuals in one command:

Recommended (preset file):

```bash
python scripts/run_preflight_and_open.py --layers-file config/layers.preset.json --output-dir artifacts/layer_preflight
```

Alternative (inline JSON):

```bash
python scripts/run_preflight_and_open.py \
  --layers-json '[{"name":"slope","path":"slope.tif","weight":0.35,"invert":true},{"name":"roughness","path":"roughness.tif","weight":0.25,"invert":true},{"name":"temp_max","path":"temp_max.tif","weight":0.15,"invert":true},{"name":"temp_min","path":"temp_min.tif","weight":0.15,"invert":true},{"name":"hydrogen","path":"hydrogen.tif","weight":0.10,"invert":false}]' \
  --output-dir artifacts/layer_preflight
```
