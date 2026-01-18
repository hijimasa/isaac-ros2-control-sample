FROM nvcr.io/nvidia/isaac-sim:5.1.0

USER root

ENV DEBIAN_FRONTEND noninteractive

# install dependencies via apt
ENV DEBCONF_NOWARNINGS yes
RUN set -x && \
  apt-get update -y -qq && \
  apt-get upgrade -y -qq --no-install-recommends && \
  apt-get install -y -qq \
    libyaml-cpp-dev curl software-properties-common \
    vim v4l-utils exfat-* \
    openssh-server \
    terminator dbus-x11 \
    python3-pip && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null && \
    apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" && \
    apt update && \
    apt install -y -qq cmake

# for ros2
RUN ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime
RUN apt update && apt install -y software-properties-common
RUN apt --fix-broken install
RUN apt install -f
RUN apt install -y libfreetype6-dev
RUN apt -y -qq update && apt -y -qq install software-properties-common mesa-utils libglu1-mesa-dev\
  freeglut3-dev mesa-common-dev libglew-dev libglfw3-dev libglm-dev \
  libao-dev libmpg123-dev libasio-dev && \
  add-apt-repository universe && \
  apt -y -qq update && apt install curl -y && \
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
  apt -y -qq update && \
  apt -y -qq upgrade && \
  apt -y -qq install ros-jazzy-desktop \
    ros-jazzy-ament-cmake ros-jazzy-angles ros-jazzy-controller-manager \
    ros-jazzy-pluginlib ros-jazzy-urdf ros-jazzy-yaml-cpp-vendor ros-jazzy-joint-state-pub* \
    ros-jazzy-cv-bridge ros-jazzy-diagnostic-updater \
    ros-jazzy-moveit && \
  apt-get install -y -qq ros-jazzy-xacro ros-jazzy-realtime-tools ros-jazzy-control-toolbox ros-jazzy-ros2-control* ros-jazzy-ros-testing \
    ros-jazzy-teleop-twist-keyboard ros-jazzy-vision-msgs && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*
  
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-colcon-ros \
    telnet \
    && apt-get clean

# 必要なパッケージをインストール
RUN apt-get update && \
    apt-get install -y wget apt-transport-https software-properties-common

# VS CodeのGPGキーをインポート
RUN wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg && \
    install -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/ && \
    rm -f packages.microsoft.gpg

# VS Codeリポジトリを追加
RUN sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'

# VS Codeをインストール
RUN apt-get update && \
    apt-get install -y code

RUN echo '#!/bin/bash\n /usr/bin/code --user-data-dir="/root/.config/Code" --no-sandbox "$@"' > /usr/local/bin/code && \
    chmod +x /usr/local/bin/code

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ros-jazzy-rmw-cyclonedds-cpp \
    iproute2 \
    && apt-get clean

RUN pip3 install --break-system-packages fastapi uvicorn

RUN usermod -aG ubuntu isaac-sim

#RUN echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/root/colcon_ws/src/isaac_ros2_utils/isaac_ros2_scripts/config/fastdds.xml' >> ~/.bashrc
#RUN echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc

USER isaac-sim

RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
RUN echo 'ip link set lo multicast on' >> ~/.bashrc

RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
RUN echo 'source /home/isaac-sim/colcon_ws/install/setup.sh' >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]
