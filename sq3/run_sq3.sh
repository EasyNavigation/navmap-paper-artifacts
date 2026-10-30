#!/bin/bash
# SQ3: geodesic vs projected vs 3D distance to the borders, inscribed radius 0.7 m and 2.0 m.
# Inputs from $NAVMAP_DATA/sq3 (NavMaps and their dist_dump .bin); results there too.
source "$(dirname "$0")/../env.sh"
cd "$NAVMAP_DATA/sq3"
for r in 0.7 2.0; do
  tag=$([ "$r" = 0.7 ] && echo "" || echo "_r2")
  python3 "$ARTIFACTS/sq3/sq3.py" garage.navmap garage.bin garage$tag --radius $r \
    --start 0,0,0 --goals "5,0,2.35;5,0,4.7"
  python3 "$ARTIFACTS/sq3/sq3.py" excavation_0.3.navmap excavation.bin excavation$tag --radius $r
done
