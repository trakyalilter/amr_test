#!/bin/bash

# Source ROS 2 setup
source /opt/ros/foxy/setup.bash
source /root/ros2_ws/install/local_setup.bash

# Start a new tmux session with the first pane (Pane 0)
tmux new-session -d -s ros2_session

# Split the window into a 2x3 layout
tmux split-window -h -t ros2_session:0.0 # Split horizontally
tmux split-window -v -t ros2_session:0.0 # Split vertically (left column)
tmux split-window -v -t ros2_session:0.1 # Split vertically (right column)
tmux split-window -h -t ros2_session:0.2 # Bottom left pane
tmux split-window -h -t ros2_session:0.3 # Bottom right pane

# Rearrange panes into a tiled layout (2x3 grid)
tmux select-layout -t ros2_session tiled

# Assign commands to each pane
# Pane 0: Pull latest updates from Git
tmux send-keys -t ros2_session.0 "cd /root/ros2_ws/src/first_ros2_package && git fetch origin && git pull origin main" C-m

# Pane 1: Launch rsp.launch.py
tmux send-keys -t ros2_session.1 "sleep 1 && source /opt/ros/foxy/setup.bash && source /root/ros2_ws/install/local_setup.bash && ros2 launch amr_test rsp.launch.py" C-m

# Pane 2: Start rviz2 with a 2-second delay
tmux send-keys -t ros2_session.2 "sleep 2 && source /opt/ros/foxy/setup.bash && source /root/ros2_ws/install/local_setup.bash && rviz2 -d /root/ros2_ws/src/first_ros2_package/config/default.rviz" C-m

# Pane 3: Run joint_state_publisher_gui
tmux send-keys -t ros2_session.3 "sleep 1 && source /opt/ros/foxy/setup.bash && source /root/ros2_ws/install/local_setup.bash && ros2 run joint_state_publisher_gui joint_state_publisher_gui" C-m

# Pane 4: Launch Gazebo
tmux send-keys -t ros2_session.4 "source /opt/ros/foxy/setup.bash && source /root/ros2_ws/install/local_setup.bash && ros2 launch gazebo_ros gazebo.launch.py" C-m

# Pane 5: Spawn AMR on Gazebo (wait 2 secs)
tmux send-keys -t ros2_session.5 "sleep 2 && ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity cnc_amr" C-m

# Attach to the session
tmux attach -t ros2_session
