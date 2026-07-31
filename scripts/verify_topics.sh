#!/usr/bin/env bash
# Captures the evidence the assignment asks for: `ros2 topic list` plus
# `ros2 topic hz` for every required topic. Run inside the `ros` service.
#   docker compose -f docker/docker-compose.yml exec ros /workspace/scripts/verify_topics.sh
set -o pipefail
source /opt/ros/jazzy/setup.bash
OUT=/workspace/docs/topic_evidence.md
HZ_WINDOW=8

REQUIRED=(/clock /tf /tf_static /joint_states /odom)
BONUS=(/points /camera/image_raw /camera/depth /camera/camera_info /imu)

{
  echo "# Topic evidence"
  echo
  echo "Captured $(date -u '+%Y-%m-%d %H:%M UTC') | ROS 2 Jazzy | Isaac Sim 5.1"
  echo
  echo '## ros2 topic list'
  echo '```'
  ros2 topic list
  echo '```'
  echo
  echo '## ros2 topic info'
  echo '```'
  for t in "${REQUIRED[@]}" "${BONUS[@]}" /cmd_vel; do
    ros2 topic info "$t" 2>/dev/null && echo "--- $t" || echo "$t : NOT PRESENT"
  done
  echo '```'
  echo
  echo '## ros2 topic hz'
  for t in "${REQUIRED[@]}" "${BONUS[@]}"; do
    echo "### $t"
    echo '```'
    timeout "$HZ_WINDOW" ros2 topic hz "$t" 2>&1 | tail -4 || echo "no messages within ${HZ_WINDOW}s"
    echo '```'
  done
  echo
  echo '## Joint names present in /joint_states'
  echo '```'
  timeout 5 ros2 topic echo /joint_states --once --field name 2>&1 || echo "no message"
  echo '```'
  echo
  echo '## /odom frame check (expect frame_id: odom, child_frame_id: base_link)'
  echo '```'
  timeout 5 ros2 topic echo /odom --once 2>/dev/null | head -12 || echo "no message"
  echo '```'
} | tee "$OUT"

echo
echo "[verify] written to $OUT"
