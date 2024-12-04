To Run:
docker run --gpus all -e DISPLAY=host.docker.internal:0.0 -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix --net=host --privileged --runtime=nvidia -it --rm --name CNC_AMR_PROJECT_iron ros2_iron
To Build:
docker build -t ros2_iron .

ros2 launch amr_test launch_sim.launch.py
ros2 launch twist_mux twist_mux_launch.py
(ros2 run twist_mux twist_mux --ros-args --params-file ./ros2_ws/src/amr_test/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped
)
ros2 launch nav2_bringup localization_launch.py map:=./ros2_ws/src/amr_test/config/savedmap.yaml use_sim_time:=true

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

ros2 run tf2_ros tf2_echo base_footprint map [POSE of base_footprint wro map]

docker run --gpus all --it --name CNC_AMR_PROJECT_iron2 ros2_iron -e DISPLAY=host.docker.internal:0.0 -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix -v /run/desktop/mnt/host/wslg:/mnt/wslg


ros2 run nav2_amcl amcl
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
docker run -p 6080:80 --security-opt seccomp=unfonfined --shm-size=512 tiryoh/ros2desktop-vnc:humble
