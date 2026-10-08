# mor_luam - the PC side, on ROS 2 Jazzy.
#
# Holds the programs that command the real robot (drive_to_xy.py,
# send_heading_speed.py, drive_sequence.py, ...). They talk to the ESP32
# through the micro-ROS agent, which runs in its own container
# (see docker-compose.yml).
#
# Build (from the repo root):  docker compose -f docker/docker-compose.yml build ros
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

# What the package and its scripts import (package.xml / CMakeLists.txt)
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-yaml \
        python3-matplotlib \
        libyaml-cpp-dev \
        ros-jazzy-nav-msgs \
        ros-jazzy-geometry-msgs \
        ros-jazzy-sensor-msgs \
        ros-jazzy-std-msgs \
        ros-jazzy-rclcpp-action \
        ros-jazzy-ament-index-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY mor_luam_ws/src/mor_luam /ws/src/mor_luam
# A Windows checkout gives the scripts CRLF line ends, and Linux then looks for
# "python3\r" in the shebang. Fix the copy inside the image only.
RUN find /ws/src -type f \( -name "*.py" -o -name "*.sh" -o -name "*.yaml" -o -name "*.yml" \) \
        -exec sed -i 's/\r$//' {} + \
    && source /opt/ros/jazzy/setup.bash \
    && colcon build --event-handlers console_direct- \
    && rm -rf build log

COPY docker/entrypoint.sh /mor_luam_entrypoint.sh
# a file saved on Windows may have CRLF line ends; bash needs LF
RUN sed -i 's/\r$//' /mor_luam_entrypoint.sh && chmod +x /mor_luam_entrypoint.sh \
    && echo 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/mor_luam_entrypoint.sh"]
CMD ["bash"]
