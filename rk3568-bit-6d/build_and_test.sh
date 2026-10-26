#!/usr/bin/env bash
set -euo pipefail
exec > /home/llx/6D_build.log 2>&1
cd /home/llx/6D
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --event-handlers console_direct+
colcon test-result --verbose
set +u
source install/setup.sh
set -u
rm -f /tmp/pose_demo.log /tmp/object_pose.txt
timeout 12s ros2 launch monocular_6d_pose demo.launch.py > /tmp/pose_demo.log 2>&1 &
pid=$!
sleep 5
timeout 5s ros2 topic echo /object_pose --once > /tmp/object_pose.txt
wait "$pid" || test $? -eq 124
cat /tmp/pose_demo.log
echo '--- OBJECT POSE ---'
cat /tmp/object_pose.txt
