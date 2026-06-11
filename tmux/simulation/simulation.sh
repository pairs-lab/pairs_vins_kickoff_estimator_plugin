#!/bin/bash
# Plain-tmux session script (one window per section below; panes are split and
# their commands typed via send-keys). Edit the send-keys lines to change what
# runs in each pane; ./kill.sh stops everything.

SESSION_NAME=simulation

# absolute path to this script's directory; every pane starts here
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

# commands executed first in every pane
PRE_WINDOW='export UAV_NAME=uav1; export RUN_TYPE=simulation; export UAV_TYPE=f330; export SENSORS:garmin'
SETUP="cd $SCRIPTPATH; $PRE_WINDOW"

if [ -n "$TMUX" ]; then
  echo "Already inside tmux, detach first."
  exit 1
fi

if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
  echo "Session $SESSION_NAME already exists; attach with 'tmux a -t $SESSION_NAME' or stop it with ./kill.sh."
  exit 1
fi

# ---------------- window: roscore ----------------
read W_roscore P <<< "$(tmux new-session -d -s "$SESSION_NAME" -n "roscore" -x 250 -y 50 -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'roscore' Enter
tmux select-layout -t "$W_roscore" tiled

# ---------------- window: gazebo ----------------
read W_gazebo P <<< "$(tmux new-window -t "$SESSION_NAME" -n "gazebo" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForRos; roslaunch pairs_uav_gazebo_simulation simulation.launch world_name:=grass_plane gui:=true' Enter
P=$(tmux split-window -t "$W_gazebo" -P -F '#{pane_id}')
tmux select-layout -t "$W_gazebo" tiled
tmux send-keys -t "$P" "$SETUP; "'waitForControl; gz camera -c gzclient_camera -f $UAV_NAME; history -s gz camera -c gzclient_camera -f $UAV_NAME' Enter
tmux select-layout -t "$W_gazebo" tiled

# ---------------- window: status ----------------
read W_status P <<< "$(tmux new-window -t "$SESSION_NAME" -n "status" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForHw; roslaunch pairs_uav_status status.launch' Enter
tmux select-layout -t "$W_status" tiled

# ---------------- window: spawn ----------------
read W_spawn P <<< "$(tmux new-window -t "$SESSION_NAME" -n "spawn" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForGazebo; rosservice call /pairs_drone_spawner/spawn "1 $UAV_TYPE --enable-vio --pos 0 0 0 0"' Enter
tmux select-layout -t "$W_spawn" tiled

# ---------------- window: hw_api ----------------
read W_hw_api P <<< "$(tmux new-window -t "$SESSION_NAME" -n "hw_api" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForTime; roslaunch pairs_uav_px4_api api.launch' Enter
tmux select-layout -t "$W_hw_api" tiled

# ---------------- window: core ----------------
read W_core P <<< "$(tmux new-window -t "$SESSION_NAME" -n "core" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForHw; roslaunch pairs_uav_core core.launch world_config:=./custom_configs/world_config.yaml network_config:=./custom_configs/network_config.yaml platform_config:=`rospack find pairs_uav_gazebo_simulation`/config/pairs_uav_system/$UAV_TYPE.yaml custom_config:=./custom_configs/pairs_uav_managers.yaml' Enter
tmux select-layout -t "$W_core" tiled

# ---------------- window: open_vins ----------------
read W_open_vins P <<< "$(tmux new-window -t "$SESSION_NAME" -n "open_vins" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForHw; roslaunch pairs_open_vins_core simulation_bluefox.launch' Enter
tmux select-layout -t "$W_open_vins" tiled

# ---------------- window: vins_republisher ----------------
read W_vins_republisher P <<< "$(tmux new-window -t "$SESSION_NAME" -n "vins_republisher" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForHw; roslaunch pairs_vins_republisher vins_republisher_openvins_bluefox.launch' Enter
tmux select-layout -t "$W_vins_republisher" tiled

# ---------------- window: takeoff ----------------
read W_takeoff P <<< "$(tmux new-window -t "$SESSION_NAME" -n "takeoff" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForHw; roslaunch pairs_uav_autostart automatic_start.launch' Enter
P=$(tmux split-window -t "$W_takeoff" -P -F '#{pane_id}')
tmux select-layout -t "$W_takeoff" tiled
tmux send-keys -t "$P" "$SETUP; "'waitForControl; rosservice call /$UAV_NAME/hw_api/arming 1; sleep 2; rosservice call /$UAV_NAME/hw_api/offboard' Enter
tmux select-layout -t "$W_takeoff" tiled

# ---------------- window: goto ----------------
read W_goto P <<< "$(tmux new-window -t "$SESSION_NAME" -n "goto" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'history -s rosservice call /$UAV_NAME/control_manager/goto \"goal: \[0.0, 10.0, 1.5, 0.0\]\"' Enter
tmux select-layout -t "$W_goto" tiled

# ---------------- window: rviz ----------------
read W_rviz P <<< "$(tmux new-window -t "$SESSION_NAME" -n "rviz" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForControl; roslaunch pairs_open_vins_core rviz.launch' Enter
P=$(tmux split-window -t "$W_rviz" -P -F '#{pane_id}')
tmux select-layout -t "$W_rviz" tiled
tmux send-keys -t "$P" "$SETUP; "'waitForTime; waitForControl; roslaunch pairs_rviz_plugins rviz_interface.launch' Enter
tmux select-layout -t "$W_rviz" tiled

# ---------------- window: layout ----------------
read W_layout P <<< "$(tmux new-window -t "$SESSION_NAME" -n "layout" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForControl; sleep 3; ~/.i3/layout_manager.sh ./layout.json' Enter
tmux select-layout -t "$W_layout" tiled

# ---------------- window: kill (press enter inside to stop the session) ----------------
read W_kill P <<< "$(tmux new-window -t "$SESSION_NAME" -n "kill" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SCRIPTPATH/kill.sh"

# mouse support (select panes / scroll with the mouse)
tmux set-option -t "$SESSION_NAME" mouse on

tmux select-window -t "$W_status"
tmux -2 attach-session -t "$SESSION_NAME"
