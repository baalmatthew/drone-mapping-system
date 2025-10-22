#!/bin/bash
SESSION="px4_sim"

tmux new-session -d -s $SESSION

tmux rename-window -t $SESSION "PX4"
tmux send-keys -t $SESSION "cd ~/PX4-Autopilot/ && PX4_GZ_WORLD=baylands make px4_sitl gz_x500_mono_cam_down" C-m

tmux new-window -t $SESSION -n "Agent"
tmux send-keys -t $SESSION:1 "MicroXRCEAgent udp4 -p 8888" C-m

tmux new-window -t $SESSION -n "ROS2 and QGC"
tmux send-keys -t $SESSION:2 "cd ~/px4_ros_ws && source install/setup.bash && ros2 topic list && cd ~ && ./QGroundControl-x86_64.AppImage" C-m

tmux new-window -t $SESSION -n "Bridge"
tmux send-keys -t $SESSION:3 "source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge /world/baylands/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image /world/baylands/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo" C-m

tmux new-window -t $SESSION -n "Publisher"
tmux send-keys -t $SESSION:4 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world camera_link"

tmux new-window -t $SESSION -n "Rviz"
tmux send-keys -t $SESSION:5 "rviz2"

tmux new-window -t $SESSION -n "Waiting to mapping"
tmux send-keys -t $SESSION:6 "cd ~/px4_ros_ws && colcon build --symlink-install --packages-select map_recorder && source ~/px4_ros_ws/install/setup.bash && echo ros2 run map_recorder image_saver"

tmux attach -t $SESSION

