FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        software-properties-common \
    && add-apt-repository -y universe \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        nano \
        python3-colcon-common-extensions \
        python3-pil \
        python3-pip \
        python3-scipy \
        python3-venv \
        tmux \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true \
    && rosdep update

WORKDIR /workspaces/The_RoboRacer_Project
