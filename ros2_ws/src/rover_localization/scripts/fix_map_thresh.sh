#!/usr/bin/env bash
# Patch free_thresh in a slam_toolbox-saved map YAML so that gray (unknown)
# cells are correctly classified as -1 by nav2_map_server instead of 0 (free).
#
# slam_toolbox encodes unknown cells as pixel 205 in the PGM, which gives a
# probability of (255-205)/255 ≈ 0.196. nav2_map_server's default
# free_thresh: 0.25 sits above that value, so unknowns become 0 (free).
# Setting free_thresh: 0.10 puts the boundary below 0.196 so unknowns
# correctly become -1 in the published OccupancyGrid.
#
# Usage:
#   ./fix_map_thresh.sh                           # fix all YAMLs in <project>/maps/
#   ./fix_map_thresh.sh /path/to/map.yaml         # fix a specific file
#   ./fix_map_thresh.sh /path/to/map/directory/   # fix all YAMLs in a directory

FREE_THRESH_TARGET="0.10"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_DIR="$(realpath "$SCRIPT_DIR/../../../../maps")"

fix_yaml() {
    local yaml="$1"
    if ! grep -q "free_thresh:" "$yaml"; then
        echo "  [skip] no free_thresh key found: $yaml"
        return
    fi
    local current
    current=$(grep "free_thresh:" "$yaml" | awk '{print $2}')
    if [ "$current" = "$FREE_THRESH_TARGET" ]; then
        echo "  [ok]   already ${FREE_THRESH_TARGET}: $yaml"
        return
    fi
    sed -i "s/free_thresh: .*/free_thresh: ${FREE_THRESH_TARGET}/" "$yaml"
    echo "  [fixed] ${current} -> ${FREE_THRESH_TARGET}: $yaml"
}

if [ $# -eq 0 ]; then
    echo "Fixing all map YAMLs in ${MAP_DIR}/"
    find "$MAP_DIR" -name "*.yaml" | while read -r f; do fix_yaml "$f"; done
elif [ -d "$1" ]; then
    echo "Fixing all map YAMLs in $1/"
    find "$1" -name "*.yaml" | while read -r f; do fix_yaml "$f"; done
else
    for f in "$@"; do fix_yaml "$f"; done
fi
