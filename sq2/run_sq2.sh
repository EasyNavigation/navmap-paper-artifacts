#!/bin/bash
# SQ2: costs on faces (NavMap) or on vertices (mesh_navigation) on the same meshes: lethal area of
# a small obstacle and the narrowest corridor passed in 95% of random placements, against the
# edge length. Results in $NAVMAP_DATA/sq2/sq2.json. Minutes.
source "$(dirname "$0")/../env.sh"
mkdir -p "$NAVMAP_DATA/sq2"
python3 "$ARTIFACTS/sq2/sq2.py" --out "$NAVMAP_DATA/sq2/sq2.json" --trials 200
