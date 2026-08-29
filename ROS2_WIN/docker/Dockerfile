FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      libgtest-dev \
      python3-colcon-common-extensions \
      python3-pytest \
      ros-humble-ament-cmake-gtest \
      ros-humble-launch-ros \
      ros-humble-rqt \
      ros-humble-rqt-common-plugins \
      ros-humble-rviz2 \
      ros-humble-rosbag2 \
      ros-humble-rosbag2-storage-default-plugins \
      ros-humble-ros2bag \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
