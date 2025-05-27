# filepath: /home/hulajdusza/manipulator-docker/Dockerfile
FROM ros:humble

ENV ROS_WS=/ros2_ws

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${ROS_WS}

# Build workspace
RUN mkdir -p ${ROS_WS}/src

# Autoload environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]