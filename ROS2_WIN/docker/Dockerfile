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
      ros-humble-diagnostic-updater \
      ros-humble-diff-drive-controller \
      ros-humble-launch-ros \
      ros-humble-launch-testing-ament-cmake \
      ros-humble-launch-testing-ros \
      ros-humble-nav2-lifecycle-manager \
      ros-humble-nav2-map-server \
      ros-humble-rqt \
      ros-humble-rqt-common-plugins \
      ros-humble-robot-localization \
      ros-humble-robot-state-publisher \
      ros-humble-rviz2 \
      ros-humble-rosbag2 \
      ros-humble-rosbag2-storage-default-plugins \
      ros-humble-ros2bag \
      ros-humble-slam-toolbox \
      ros-humble-tf2-tools \
      ros-humble-xacro \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
