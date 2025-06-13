#!/usr/bin/env bash
#
# create_tmux_sessions.sh
#
# This script creates (or restarts) five detached tmux sessions,
# each running one of the specified commands. Adjust session names if you like.

sudo ip route add 224.0.0.0/4 dev eno1 metric 100

# 1. rosbridge WebSocket server
tmux has-session -t rosbridge 2>/dev/null
if [ $? -ne 0 ]; then
  echo "Starting rosbridge WebSocket server"
  tmux new-session -d -s rosbridge \
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
fi

# 2. relay2server_socket.py
tmux has-session -t relay2server 2>/dev/null
if [ $? -ne 0 ]; then
  echo "Starting relay2server_socket.py"
  tmux new-session -d -s relay2server \
    python3 unitree_go1_deploy/websocket/relay2server_socket.py
fi

# 3. realsense (High‐accuracy preset)
tmux has-session -t realsense_high 2>/dev/null
if [ $? -ne 0 ]; then
  echo "Starting realsense (High‐accuracy preset)"
  tmux new-session -d -s realsense_high \
    ros2 launch realsense2_camera rs_launch.py \
      depth_module.profile:=640x480x30 \
      rgb_camera.profile:=640x480x30 \
      align_depth.enable:=true \
      serial_no:="'827312072741'" \
      json_file_path:="/home/curlynuc/ros2_ws/src/realsense-ros/realsense2_camera/launch/HighAccuracyPreset.json"
fi

# 4. realsense (default/default‐profile) with second serial
tmux has-session -t realsense_default 2>/dev/null
if [ $? -ne 0 ]; then
  echo "Starting realsense (default/default‐profile)"
  tmux new-session -d -s realsense_default \
    ros2 launch realsense2_camera rs_launch.py \
      serial_no:="'146322110342'"
fi

# 5. relay2go1.py
tmux has-session -t relay2go1 2>/dev/null
if [ $? -ne 0 ]; then
  echo "Starting relay2go1.py"
  tmux new-session -d -s relay2go1 \
    zsh -c "python3 unitree_go1_deploy/websocket/relay2go1.py"
fi

echo "Five tmux sessions are now running (or have been restarted)."
tmux ls
echo "Use 'tmux ls' to list them and 'tmux attach -t <session_name>' to attach. Use 'tmux kill-server' to kill all sessions."