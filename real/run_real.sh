#!/bin/bash
# Real data: the two-floor URJC building, from the mapping rosbag prueba_03 ($NAVMAP_DATA/rosbags).
# 1) scans and wheel odometry from the bag (5 Hz, 0.1 m voxels); 2) poses by scan-to-map ICP
# against the SLAM map of the run; 3) the cloud with each point's sensor position; 4) the route the
# robot drove as the reference: every representation, from the raw cloud and from the cloud with
# free-space carving only. Results in $NAVMAP_DATA/real/. Minutes.
source "$(dirname "$0")/../env.sh"
set -e
R=$NAVMAP_DATA/real
mkdir -p "$R/eval"
A=$ARTIFACTS/real
if [ -n "${REAL_FROM_BAG:-}" ] || [ ! -f "$R/poses_03.npz" ]; then  # scans and poses: hours, independent of NavMap
  python3 "$A/bag_scans.py" "$NAVMAP_DATA/rosbags/prueba_03" "$R/scans_03.npz" --every 2
  python3 "$A/track.py" "$NAVMAP_DATA/clouds/two_floors_map_noghosts.pcd" "$R/scans_03.npz" "$R/poses_03.npz"
fi
python3 "$A/assemble.py" "$R/scans_03.npz" "$R/poses_03.npz" "$R/two_floors_03_vp.pcd" --min-fit 0
NOBELOW=1 DUMP="$R/two_floors_carved.pcd" "$MESHSTATS" "$R/two_floors_03_vp.pcd" 0.3 1.0 0.25 0
python3 "$A/eval_real.py" "$R/two_floors_03_vp.pcd" "$R/poses_03.npz" 0.3 \
  --reps navmap,navmap+vp,elev,mls,vox --work "$R/eval" --json "$R/eval/two_floors_03_vp_0.3.json"
python3 "$A/eval_real.py" "$R/two_floors_carved.pcd" "$R/poses_03.npz" 0.3 \
  --reps navmap,elev,mls,vox --work "$R/eval" --json "$R/eval/two_floors_carved_0.3.json"
# The playground's maps (easynav_playgrounds, branch two_floor_urjc): Summit's step and slope
SLOPE=40 HOLE=10 "$MESHSTATS" "$R/two_floors_03_vp.pcd" 0.3 1.0 0.35 4 "$R/two_floors.navmap"
python3 "$ARTIFACTS/tools/pcd_voxelize.py" "$R/two_floors_carved.pcd" "$R/two_floors_bonxai.pcd" 0.3 centroid
