######################################################################
# - Base stage
#   - This stage serves as the base image for the following stages.
#   - Based on Isaac Sim official image
######################################################################

FROM nvcr.io/nvidia/isaac-sim:4.5.0 AS base

LABEL org.opencontainers.image.title="Isaac Go2 ROS2"
LABEL org.opencontainers.image.authors="scx@gapp.nthu.edu.tw"
LABEL org.opencontainers.image.licenses="MIT"

ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=1000

######################################################################
# - User setup stage
#   - Create a non-root user with default bash shell.
######################################################################

FROM base AS user-setup

# RUN groupadd --gid $USER_GID $USERNAME \
    # && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
RUN apt-get update \
    && apt-get install -y sudo gosu \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

######################################################################
# - ROS2 Installation stage
#   - Install ROS2 Humble and related packages
######################################################################

FROM user-setup AS ros2-setup

USER ubuntu
WORKDIR /home/ubuntu

# ARG variables need to be redeclared in each stage
ARG ROS_DISTRO=humble

# Install ROS2 Humble
RUN sudo apt-get update && sudo apt-get install -y \
    locales \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    && sudo locale-gen en_US en_US.UTF-8 \
    && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && sudo add-apt-repository universe \
    && sudo rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Set timezone non-interactively
ENV DEBIAN_FRONTEND=noninteractive
RUN sudo ln -fs /usr/share/zoneinfo/UTC /etc/localtime

# Setup ROS2 repository using the official ros-apt-source method
RUN sudo apt update && sudo apt install -y software-properties-common curl \
    && sudo add-apt-repository universe \
    && sudo apt update

# Install ROS2 apt source
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && sudo dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb

# Force resolve dependency conflicts and install ROS desktop + RViz
RUN sudo apt update && sudo apt upgrade -y \
    && sudo apt install -y --allow-downgrades \
    libbrotli1=1.0.9-2build6 \
    && sudo apt-mark hold libbrotli1 \
    && sudo apt install -y \
    libfreetype6-dev \
    libfontconfig1-dev \
    && sudo rm -rf /var/lib/apt/lists/*

# Install ROS 2 desktop with RViz (now that dev dependencies are resolved)
RUN sudo apt update \
    && sudo apt install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-cv-bridge \
    python3-colcon-common-extensions \
    && sudo rm -rf /var/lib/apt/lists/*

# Additional ROS2 packages (commented out to reduce build time and image size)
# Uncomment as needed for your specific use case:
# ros-${ROS_DISTRO}-nav2-msgs \
# ros-${ROS_DISTRO}-nav2-util \
# ros-${ROS_DISTRO}-navigation2 \
# ros-${ROS_DISTRO}-nav2-bringup \
# ros-${ROS_DISTRO}-turtlebot3* \
# ros-${ROS_DISTRO}-urdf-launch \
# ros-${ROS_DISTRO}-joint-state-publisher \
# ros-${ROS_DISTRO}-joint-state-publisher-gui \
# ros-${ROS_DISTRO}-behaviortree-cpp \
# ros-${ROS_DISTRO}-generate-parameter-library

######################################################################
# - Tools Installation stage
#   - Install common tools for development.
######################################################################
FROM ros2-setup AS tools

RUN sudo apt-get update && sudo apt-get install -y \
    tmux \
    htop \
    tree \
    vim \
    git \
    wget \
    curl \
    unzip \
    usbutils \
    python3-pip \
    bash-completion
    
RUN mkdir -p ~/miniconda3 \
    && wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh \
    && bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3 \
    && rm ~/miniconda3/miniconda.sh \
    && bash -c "source ~/miniconda3/bin/activate && conda init --all"

# Add conda to PATH
ENV PATH="/home/ubuntu/miniconda3/bin:$PATH"

# Configure conda to avoid TOS issues
RUN /home/ubuntu/miniconda3/bin/conda config --remove channels defaults && \
    /home/ubuntu/miniconda3/bin/conda config --add channels conda-forge && \
    /home/ubuntu/miniconda3/bin/conda config --set channel_priority strict && \
    /home/ubuntu/miniconda3/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    /home/ubuntu/miniconda3/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r

# Clone IsaacLab repository and create symbolic link
RUN git clone https://github.com/isaac-sim/IsaacLab.git /home/ubuntu/IsaacLab && \
    cd /home/ubuntu/IsaacLab && \
    ln -s /isaac-sim _isaac_sim && \
    ./isaaclab.sh --conda isaaclab

# Install IsaacLab extensions
RUN sudo chown -R ubuntu:ubuntu /isaac-sim
RUN cd /home/ubuntu/IsaacLab && \
    bash -c "source /home/ubuntu/miniconda3/bin/activate && conda activate isaaclab && ./isaaclab.sh --install"

RUN sudo apt-get update && sudo apt-get dist-upgrade -y \
    && sudo apt-get autoremove -y \
    && sudo apt-get autoclean -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

######################################################################
# - Final stage
#   - Set up the environment and entrypoint.
######################################################################

FROM tools AS final

# Set up the user environment
ENV TZ=Asia/Taipei
ENV SHELL=/bin/bash
ENV TERM=xterm-256color

# Isaac Sim environment variables
ENV ACCEPT_EULA=Y
ENV PRIVACY_CONSENT=N
ENV NVIDIA_DRIVER_CAPABILITIES=all

# ROS2 environment variables
ENV ROS_DISTRO=${ROS_DISTRO}
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

USER root
WORKDIR /home/$USERNAME

# Set up bashrc
COPY .bashrc .bashrc.conf
RUN cat .bashrc.conf >> .bashrc && rm .bashrc.conf

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
