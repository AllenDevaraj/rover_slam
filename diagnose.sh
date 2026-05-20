#!/bin/bash
# Diagnostic for the full localization + planning pipeline.
# Run while a launch is up. Prints PASS/FAIL for every layer.

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

pass() { echo -e "${GREEN}PASS${NC} $*"; }
fail() { echo -e "${RED}FAIL${NC} $*"; }
warn() { echo -e "${YELLOW}WARN${NC} $*"; }
hdr()  { echo -e "\n=== $* ==="; }

source ~/Advanced_Robo_Project/ros2_ws/install/setup.bash 2>/dev/null

hdr "Nodes"
for n in slam_toolbox rover_node fake_odom rplidar_node ekf_filter_node astar_planner pure_pursuit; do
    if ros2 node list 2>/dev/null | grep -q "/$n"; then pass "$n"; else fail "$n not running"; fi
done

hdr "Topics flowing"
check_hz() {
    local topic=$1; local min_hz=$2; local qos=${3:-reliable}
    local hz=$(timeout 6 ros2 topic hz "$topic" --qos-reliability "$qos" 2>/dev/null \
               | grep -oP 'average rate: \K[0-9.]+' | head -1)
    if [ -z "$hz" ]; then fail "$topic — no data ($qos)"; return; fi
    if (( $(echo "$hz < $min_hz" | bc -l) )); then warn "$topic — $hz Hz (expected ≥ $min_hz)"
    else pass "$topic — $hz Hz"; fi
}
check_hz /scan 5 best_effort
check_hz /imu 10 best_effort
check_hz /odom 10 reliable
check_hz /fake_odom/raw 10 reliable
check_hz /tf 10 reliable

hdr "TF chain"
check_tf() {
    local from=$1; local to=$2
    if timeout 3 ros2 run tf2_ros tf2_echo "$from" "$to" 2>&1 | grep -q "Translation"; then
        pass "$from -> $to"
    else
        fail "$from -> $to (frame missing or stale)"
    fi
}
check_tf map odom
check_tf odom base_link
check_tf base_link laser
check_tf map base_link

hdr "Map"
if timeout 3 ros2 topic echo /map --once --no-arr 2>/dev/null | grep -q "frame_id: map"; then
    pass "/map publishing in 'map' frame"
else
    fail "/map not publishing or wrong frame"
fi

hdr "slam_toolbox"
mode=$(ros2 param get /slam_toolbox mode 2>/dev/null | grep -oP 'String value is: \K\w+')
if [ "$mode" = "localization" ]; then pass "mode=localization"
elif [ "$mode" = "mapping" ]; then warn "mode=mapping (planning mode expects localization)"
else fail "could not read slam_toolbox mode"; fi

hdr "Plan path subscribers (planning mode)"
if ros2 topic info /plan -v 2>/dev/null | grep -q "Subscription count: [1-9]"; then
    pass "/plan has subscribers"
else
    warn "/plan has no subscribers (no pure_pursuit?)"
fi
if ros2 topic info /goal_pose -v 2>/dev/null | grep -q "Subscription count: [1-9]"; then
    pass "/goal_pose has subscribers"
else
    warn "/goal_pose has no subscribers (no astar_planner?)"
fi

hdr "Done"
