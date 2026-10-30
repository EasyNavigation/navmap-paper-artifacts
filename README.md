# NavMap paper artifacts

Code to reproduce the experiments of the NavMap paper: every table and figure comes from a script
of this repository run on the dataset described in [`data/MANIFEST.md`](data/MANIFEST.md).

## Layout

| Folder | Contents |
|---|---|
| `tools/` | C++ tools built against NavMap (`meshstats`: NavMap from a cloud; `lidar_sim`: simulated lidar over a mesh; `dist_dump`: per-NavCel distances) and Python helpers (mesh and `.navmap` loaders, cloud voxelization) |
| `sq1/` | SQ1, topological fidelity: ground truth from the world's collision mesh (`mesh_truth.py`), NavMap and the baselines (2.5D elevation, multi-level surface map, voxel floor) built from the same cloud, and the evaluation (`eval_sq1.py`); diagnostics |
| `sq2/` | SQ2, costs on faces or on vertices: lethal area and narrow passages against the mesh's edge length |
| `sq3/` | SQ3, metric on the surface vs projected: geodesic, projected and 3D distances to the borders |
| `bench/` | Computational cost: NavMap against a costmap (flat maps) and against OctoMap and Bonxai (3D, the same scans); planning benchmark (`bench_plan`) |
| `sq5/` | SQ5, navigation in simulation: goal runner, goal sampling, launchers and parameters of the stacks |
| `figures/` | Figures of the paper from the results |
| `sq4/` | SQ4, memory against fidelity: simplification keeping the layers, with and without a dense layer, against coarser uniform NavMaps |
| `real/` | Real data: scans and poses from the mapping rosbag of a two-floor building (`bag_scans.py`, `track.py`, `assemble.py`), and the evaluation against the route the robot drove (`eval_real.py`) |
| `data/` | Where the dataset is unpacked (not versioned, except the manifest) |

## Setup

1. A ROS 2 workspace with NavMap and EasyNav built (versions in the manifest).
2. The dataset, <https://doi.org/10.5281/zenodo.23434550> (`navmap_paper_data.tar.gz`, 2.2 GB; see
   `data/MANIFEST.md`), unpacked into `data/`, or `NAVMAP_DATA` pointing to it.
3. `source env.sh` (it also sources the workspace), then build the tools: `tools/build.sh`.

## Reproducing

| Result | Command |
|---|---|
| Ground-truth checks on toy geometry | `python3 sq1/test_mesh_truth.py` |
| SQ1 table (parking garage, excavation; NavMap with and without viewpoints; 5 seeds) | `sq1/run_sq1.sh`, then `sq1/summarize_sq1.py` |
| Sensitivity of the MLS baseline to its parameters | `sq1/run_sq1_mls.sh`, then `sq1/summarize_sq1.py` |
| Attribution: every representation from the carved cloud, NavMap with carving only | `sq1/run_sq1_ablation.sh` |
| Sensitivity of the carving parameters (ratio, voxel), garage and excavation | `sq1/run_sq1_carving.sh`, `SCENE=excavation sq1/run_sq1_carving.sh` |
| SQ2 (faces against vertices) | `sq2/run_sq2.sh` |
| SQ3 measures | `sq3/run_sq3.sh` |
| SQ4 (memory against fidelity) | `sq4/run_sq4.sh` |
| Real building: poses, cloud, maps and the route evaluation | `real/run_real.sh` |
| Computational cost (one thread) | `bench/build.sh`, then `bench/run_bench.sh` |
| Planning: 1 000 random reachable pairs per map, the planners of the stacks (costmap and NavMap on the flat maps, NavMap on the garage and the excavation) | `bench/run_plan.sh` (table in `data/bench/plan_summary.txt`) |
| SQ5: navigation in Gazebo, house (costmap and NavMap stacks) and Summit scenes | `sq5/run_sq5.sh`, `sq5/run_sq5_summit.sh` |
| SQ5 goals: feasibility in the NavMap the stack navigates (clearance, connectivity to the start) | `sq5/check_goals.py` on `tools/dist_dump` |
| SQ5 as independent episodes (every leg in a fresh simulation, robot and localizer placed at the previous goal) | `sq5/run_sq5_episodes.sh`, `sq5/summarize_episodes.py` |
| Figures of the paper | `figures/make_figures.py <paper>/images` |
| Everything above in order, with the previous results kept aside | `run_all.sh` |

The robot model is shared by the ground truth and every representation: steps up to 0.25 m,
slopes up to 30°, 1 m of free height (`sq1/mesh_truth.py`).

## License

The code of this repository is under the Apache License 2.0 (`LICENSE`). The dataset
(`data/`, <https://doi.org/10.5281/zenodo.23434550>) is under the Creative Commons Attribution 4.0 International license
(CC-BY-4.0).
