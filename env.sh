# Source this file before running anything:  source env.sh
#   NAVMAP_DATA        folder with the dataset (Zenodo 10.5281/zenodo.23434550, see data/MANIFEST.md); results go there too
#   PLAYGROUND_MODELS  easynav_playground_summit_worlds/models (the worlds' collision meshes)
#   EASYNAV_WS         ROS 2 workspace with NavMap and EasyNav built (install/ is sourced)
# Defaults are this repo's data/ folder and a workspace next to it; override them before sourcing.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ARTIFACTS="$HERE"
export NAVMAP_DATA="${NAVMAP_DATA:-$HERE/data}"
export EASYNAV_WS="${EASYNAV_WS:-$(cd "$HERE/.." && pwd)}"
export PLAYGROUND_MODELS="${PLAYGROUND_MODELS:-$EASYNAV_WS/src/easynav_playgrounds/playground_summit/easynav_playground_summit_worlds/models}"
export MESHSTATS="${MESHSTATS:-$HERE/tools/build/meshstats}"
source /opt/ros/${ROS_DISTRO:-rolling}/setup.bash
source "$EASYNAV_WS/install/setup.bash"
