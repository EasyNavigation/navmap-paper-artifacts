# Dataset manifest

The dataset is published under CC-BY-4.0 at Zenodo, <https://doi.org/10.5281/zenodo.23434550> (concept DOI; version 1.0.0 is 10.5281/zenodo.23434551): one file, `navmap_paper_data.tar.gz` (2.2 GB, 3.3 GB unpacked, SHA-256 `46ebfcfc9449a101194c576cfb814640eee56d48fc2113ab6bdee99415192352`), that unpacks into
`$NAVMAP_DATA` with this layout. The package leaves out the working folders that are not results: `prev_2026-10-24/` (results before the rerun), `sq5_loc2/`, `sq5/smoke/`, `sq5/before_fixes/`, `sq5/episodes/smoke1/` and `sq5/episodes/prev_excavation_drop/` (a first pass of the episodes invalidated by a harness bug: the robot was teleported 6 m above the terrain).
`data/generate_inputs.sh` regenerates every simulated input from the worlds' meshes, deterministically
(same bytes on every run with the same software). Full SHA-256 in `data/SHA256SUMS`.

History (2026-10-24): every result was rerun on the versions above. The excavation terrain of
easynav_playgrounds had its tiles stitched (#16) after the first clouds were made, so the
excavation clouds, the SQ3 inputs and every excavation result changed; the garage, the house and
the real building reproduced their numbers.

History (2026-10-16): the first excavation clouds were made with an earlier `lidar_sim` and could not
be reproduced, so the whole dataset was regenerated with the versions below and SQ1 and SQ3 rerun on
it (SQ3 gave the same numbers). The garage cloud of the previous version differed in 38 of 7.5 M points
(rays through shared mesh edges, fixed in NavMap#35).

## Inputs

| Path | Size | SHA-256 | What it is | Produced by |
|---|---|---|---|---|
| `clouds/parking_garage_lidar_full.pcd` | 90.4 MB | `ec43e55c9046baa9…` | Simulated lidar over the parking garage, map frame | `data/generate_inputs.sh` |
| `clouds/lidar_10m.pcd` | 15.3 MB | `24f918b99b4abf43…` | Simulated VLP-16 over the excavation, a scan every 10 m | `data/generate_inputs.sh` |
| `clouds/lidar_noisy.pcd` | 15.3 MB | `d5952c380f6a616c…` | As `lidar_10m`, 5 cm noise and 1 % spikes | `data/generate_inputs.sh` |
| `clouds/lidar_dense.pcd` | 171 MB | `d4a409f4fb7419b3…` | A scan every 3 m, 3 cm noise, 1 % spikes | `data/generate_inputs.sh` |
| `clouds/parking_garage_lidar_full_vp.pcd` | 181 MB | `95849254d5fce3b2…` | As `parking_garage_lidar_full`, each point with its sensor position (`vp_x vp_y vp_z`) | `data/generate_inputs.sh` |
| `clouds/lidar_10m_vp.pcd` | 30.7 MB | `c1fc831e7e0e1ed5…` | As `lidar_10m`, with sensor positions | `data/generate_inputs.sh` |
| `clouds/lidar_dense_vp.pcd` | 342 MB | `7634ca3fe5a90e6a…` | As `lidar_dense`, with sensor positions | `data/generate_inputs.sh` |
| `sq3/garage.navmap` | 0.4 MB | `76dae12606c296a5…` | Parking garage NavMap (0.4 m, from the world mesh) | easynav_playgrounds `maps/parking_garage.navmap`, copied by `data/generate_inputs.sh` |
| `sq3/excavation_0.3.navmap` | 3.3 MB | `7cac763ebd773699…` | Excavation NavMap at 0.3 m, from the world's sampled mesh | `data/generate_inputs.sh` |
| `sq3/garage.bin`, `sq3/excavation.bin` | 0.9 / 7.6 MB | `1846cc14a1bcd9a4…`, `0c6b78ebfb82a117…` | Per-NavCel dump: centroid, normal z, surface, distance to the border | `data/generate_inputs.sh` (`tools/dist_dump`) |

The worlds' collision meshes are not in the dataset: they come from easynav_playgrounds
(`easynav_playground_summit_worlds/models/{parking_garage,urjc_excavation}/meshes`).

## Real data (two-floor URJC building)

`real/run_real.sh` rebuilds everything below from the rosbag and the SLAM map. Full SHA-256 in `data/SHA256SUMS`.

| Path | Size | What it is | Produced by |
|---|---|---|---|
| `rosbags/prueba_0{0..3}/` | 27.9 GB | Rosbags of four mapping runs of the Summit XL in a two-floor URJC building (lidar `/front_laser/points`, IMU, wheel odometry, GPS, TF), same route; prueba_03 is the run of the SLAM map. Not in the Zenodo record (size): available from the authors on request; only `real/run_real.sh` needs them, and its outputs (`real/scans_03.npz`, `real/poses_03.npz`) are in the package | recorded on the robot |
| `clouds/two_floors_map_noghosts.pcd` | 26.8 MB | SLAM map of prueba_03, with the trails of people walking during the mapping removed by hand (the cleaning was not scripted); the reference for the scan registration | SLAM, then hand cleaning |
| `real/scans_03.npz` | 198.6 MB | Scans of prueba_03 at 5 Hz in base_footprint (0.1 m voxels, 1 to 60 m) and the wheel odometry at each scan | `real/bag_scans.py` |
| `real/poses_03.npz` | 0.2 MB | Pose of each scan in the map frame (scan-to-map point-to-plane ICP, seeded by the odometry) and its fit | `real/track.py` |
| `real/two_floors_03_vp.pcd` | 442.2 MB | All the scans in the map frame, each point with its sensor position | `real/assemble.py` |
| `real/two_floors_carved.pcd` | 192.6 MB | The same cloud after free-space carving only (no faces dropped) | `tools/meshstats` (`NOBELOW=1 DUMP=`) |
| `real/two_floors.navmap, real/two_floors_bonxai.pcd` | 0.9 MB + 4.2 MB | The playground maps (Summit: 0.35 m step, 40 deg): NavMap with viewpoints; occupied 0.3 m voxels (centroids) of the carved cloud | `real/run_real.sh` |

## Generated (by the scripts of this repo)

| Path | By |
|---|---|
| `sq1/garage_gt.npz`, `sq1/excavation_gt.npz` | `sq1/eval_sq1.py --truth-cache` (ground truth from the mesh) |
| `sq1/navmap_<cloud>_<res>.navmap` | `sq1/eval_sq1.py` (through `tools/meshstats`) |
| `sq1/main/*.json` | `sq1/run_sq1.sh` |
| `sq1/mls/*.json` | `sq1/run_sq1_mls.sh` |
| `sq3/*.json`, `sq3/*.npz` | `sq3/run_sq3.sh` |
| `real/eval/*.json` | `real/run_real.sh` |
| `sq2/sq2.json`, `sq4/sq4.jsonl`, `bench/*.jsonl`, `bench/plan.jsonl` | `sq2/run_sq2.sh`, `sq4/run_sq4.sh`, `bench/run_bench.sh`, `bench/run_plan.sh` |
| `sq5/*_run*.json`, `.log`, `.csv` | `sq5/run_sq5.sh`, `sq5/run_sq5_summit.sh` (table by `sq5/summarize_sq5.py`) |
| `VERSIONS.txt`, `run_all_<date>.log` | `run_all.sh` |

## Software versions

The dataset and the results were produced with these commits:

| Repository | Commit |
|---|---|
| EasyNavigation/NavMap | `3c7489e` (rolling, 0.7.0) |
| EasyNavigation/easynav_plugins | `e2a4f984` (rolling, after 0.5.1) |
| EasyNavigation/easynav_playgrounds | `e50bde2` (rolling, 0.5.1); the real building's maps on branch `two_floor_urjc`, `59de913` |
| EasyNavigation/EasyNavigation | `77736d05` (rolling, after 0.5.1) |

The independent episodes (`sq5/episodes/`, 2026-10-29/30) ran on later revisions, with the workspace built in Release (`-DCMAKE_BUILD_TYPE=Release`; the runs above were built without a build type, which is why their CPU figures are higher): EasyNavigation `86ea9a3c`, easynav_plugins `6228395d`, easynav_playgrounds `81913a6`, NavMap unchanged at `3c7489e`. Between the two: NavMap, the NavMap AMCL localizer and the planners did not change; the Kobuki's wheel collisions became spheres (playgrounds #26) and its lidar target became optional (#28); the costmap obstacle filter updates only within `update_distance` (5 m by default) of the robot (plugins #147); costmap/MH/fusion localizers changed (#145, #149, #151, #153, #155; not used here); an empty PointCloud2 is an empty perception (EasyNavigation #146).

`run_all.sh` writes the commits it ran with to `VERSIONS.txt`.

Before submission: tag the versions used for the final results and list the tags here.

## Note 2026-10-29 (review 2)
- `sq1/ablation/*_carved_s*.json` were scored on a different set of observed truth samples (the carved cloud is dumped without viewpoints); `eval_sq1.py --observe-cloud` fixes the protocol. Regenerated on 2026-10-29 with `sq1/run_sq1_ablation.sh` (the earlier files are in `sq1/ablation/prev_review2/`); the `carveonly` files were unaffected and now agree with the carved NavMap rows exactly.
- `sq5/goals_check_{garage,excavation}.json`: feasibility of the SQ5 goals in the NavMap the stack navigates (`sq5/check_goals.py` on `tools/dist_dump` of the playground maps). Excavation goals 10 and 11 are unreachable at the Summit's inscribed radius in the 1 m map.
- `sq5/summarize_sq5.py` now also pairs the house stacks by run and breaks the Summit failures down by cause.
- `sq5/episodes/{house,excavation}/<stack>_g<leg>_r<rep>.{json,csv,log}` (review 2, E11): SQ4 as independent episodes, `sq5/run_sq5_episodes.sh` + `sq5/run_goals.py --episode` (fresh launch per leg, robot teleported to the previous goal with `gz service set_pose`, localizer reset at the pose `gz model -p` reports, warm-up goal, then the measured goal); table by `sq5/summarize_episodes.py` (its test averages the repetitions of each leg and pairs the 20 legs; the per-episode test is secondary). House: 20 legs x 2 stacks x 3. Excavation: 13 legs (goals 10-11 excluded, unreachable in the 1 m map; tags `g9-12`, `g12-13`, `g13-14` carry the origin) x 3 with `sq5/summit/params_norecovery/excavation.yaml` (DummyRecoveryManager). `final_dz_m` is valid in this pass (an earlier pass, not in the package, had the robot teleported 6 m above the terrain).
