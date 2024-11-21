FROM osrf/ros:foxy-desktop

# Install necessary packages
RUN apt-get update && \
    apt-get install -y \
    git \
    python3-pip \
    tmux \
    vim \
    nano \
    gedit \
    python3-colcon-common-extensions \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-xacro \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-ros2-control \
    ros-foxy-ros2-controllers \
    ros-foxy-gazebo-ros2-control \
    ros-foxy-twist-mux \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-tf-transformations
    
RUN sudo pip3 install transforms3d

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Create the workspace and clone the repository
RUN mkdir -p /root/ros2_ws/src && \
    cd /root/ros2_ws/src &&  \
    git clone https://github.com/trakyalilter/first_ros2_package.git

# Source ROS 2 and build the workspace
RUN bash -c "source /opt/ros/foxy/setup.bash && \
    cd /root/ros2_ws && \
    colcon build --symlink-install"

# Create a new ROS 2 package
RUN bash -c "source /opt/ros/foxy/setup.bash && \
    source /root/ros2_ws/install/local_setup.bash && \
    cd /root/ros2_ws/src && \
    ros2 pkg create --build-type ament_cmake my_package"

# Build the workspace again to include the new package
RUN bash -c "source /opt/ros/foxy/setup.bash && \
    cd /root/ros2_ws && \
    colcon build --symlink-install"

# Copy a custom tmux script into the container
COPY start_tmux.sh /root/start_tmux.sh
RUN chmod +x /root/start_tmux.sh

# Default command to run the tmux script
CMD ["/bin/bash", "/root/start_tmux.sh"]
