# syntax=docker/dockerfile:1.5

############################
# Pick the Base Image
# This Dockerfile supports both amd64 and jetson architectures.
############################
ARG TARGETARCH
ARG UBUNTU_BASE=ubuntu:22.04
ARG L4T_BASE=nvcr.io/nvidia/l4t-jetpack:r35.3.1

# Base image selection per arch
FROM ${UBUNTU_BASE} AS base_amd64
FROM ${L4T_BASE} AS base_arm64

# Select the correct base
FROM base_${TARGETARCH} AS base

############################
# Stage 1: Minimal ROS 2 Base
############################
FROM base AS ros2_humble-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3
ENV ROS_LOCALHOST_ONLY=0
ENV ROS_DOMAIN_ID=0

# Locales
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && apt-get install -y locales apt-utils \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

# Base packages
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && apt-get install -y --no-install-recommends \
    curl lsb-release gnupg2 libc6-dev software-properties-common \
    iproute2 busybox && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 base
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    python3-dev python3-pip ros-humble-ros-base && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Python packages
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

############################
# Stage 2: ROS Deps Scan
############################
FROM ros2_humble-base AS ros2_humble-rosdeps
ARG DIR=/cprt_rover_24
WORKDIR ${DIR}

# Install rosdep and dev tools for scanning
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

COPY src/**/package.xml src/**/setup.py src/**/CMakeLists.txt ./src

ARG FILE=/opt/rosdeps.sh
RUN apt-get update && \
    rosdep init && \
    rosdep update && \
    rosdep install -i -r -y --from-paths src -s > ${FILE}

############################
# Stage 3: GStreamer Build
############################
FROM base AS ros2_humble-gstreamer

RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && apt-get install -y \
        zlib1g-dev libffi-dev libssl-dev python3-dev python3-pip \
        flex bison libglib2.0-dev libmount-dev libsrt-openssl-dev \
        git ninja-build curl && \
    apt-get clean

RUN python3 -m pip install --upgrade pip && \
    pip3 install --user meson

WORKDIR /gstreamer
RUN git clone https://gitlab.freedesktop.org/gstreamer/gstreamer.git && \
    cd gstreamer && \
    git checkout 1.24 && \
    ~/.local/bin/meson setup builddir --prefix=/opt/gstreamer --libdir=lib && \
    ~/.local/bin/meson compile -C builddir && \
    ~/.local/bin/meson install -C builddir --destdir /target

WORKDIR /gst-plugins-rs
ENV LD_LIBRARY_PATH=/target/opt/gstreamer/lib:$LD_LIBRARY_PATH 
ENV LIBRARY_PATH=/target/opt/gstreamer/lib:$LIBRARY_PATH 
ENV PKG_CONFIG_PATH=/target/opt/gstreamer/lib/pkgconfig:$PKG_CONFIG_PATH 

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y && \
    . "$HOME/.cargo/env" && \
    git clone https://gitlab.freedesktop.org/gstreamer/gst-plugins-rs.git && \
    cd gst-plugins-rs && \
    cargo install cargo-c && \
    cargo cbuild -p gst-plugin-webrtc --prefix=/opt/gstreamer --release && \
    cargo cbuild -p gst-plugin-rtp --prefix=/opt/gstreamer --release && \
    cargo cinstall -p gst-plugin-webrtc --prefix=/opt/gstreamer --destdir /target --libdir=lib --release && \
    cargo cinstall -p gst-plugin-rtp --prefix=/opt/gstreamer --destdir /target --libdir=lib --release

############################
# Stage 4: Minimal Runtime Base
############################
FROM ros2_humble-base AS runtime
ARG ROSDEP_FILE=/opt/rosdeps.sh
COPY --from=ros2_humble-rosdeps ${ROSDEP_FILE} ${ROSDEP_FILE}
COPY --from=ros2_humble-gstreamer /target /

# Setup GStreamer environment
ENV PATH=/opt/gstreamer/bin:$PATH 
ENV LD_LIBRARY_PATH=/opt/gstreamer/lib:$LD_LIBRARY_PATH 
ENV PKG_CONFIG_PATH=/opt/gstreamer/lib/pkgconfig:$PKG_CONFIG_PATH 
ARG TARGETARCH
RUN if [ "$TARGETARCH" = "arm64" ]; then \
      echo 'export GST_PLUGIN_PATH=/opt/gstreamer/lib/gstreamer-1.0:/usr/lib/aarch64-linux-gnu/gstreamer-1.0' >> /etc/profile.d/gst.sh; \
    else \
      echo 'export GST_PLUGIN_PATH=/opt/gstreamer/lib/gstreamer-1.0:/usr/lib/x86_64-linux-gnu/gstreamer-1.0' >> /etc/profile.d/gst.sh; \
    fi


# Install rosdep dependencies
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && chmod +x ${ROSDEP_FILE} && ${ROSDEP_FILE} && \
    rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]

############################
# Stage 5: Full Dev Environment
############################
FROM runtime AS dev

RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && apt-get install -y --no-install-recommends \
    git x11-apps ros-humble-desktop ros-dev-tools \
    ros-humble-ament-cmake python3-colcon-common-extensions \
    python3-colcon-ros clang-format && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Entry point for ROS 2 dev
RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /entrypoint.sh && \
    echo 'if [ -f /cprt_rover_24/install/setup.bash ]; then source /cprt_rover_24/install/setup.bash; fi' >> /entrypoint.sh && \
    echo 'exec "$@"' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

RUN useradd -ms /bin/bash -u 1000 vscode && \
    echo "vscode ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

############################
# Stage 6: Build Workspace
############################
FROM dev AS builder
ARG DIR=/cprt_rover_24
WORKDIR ${DIR}

# Copy source code into the builder
COPY src/ ${DIR}/src/

# Install any additional dependencies if needed
RUN rosdep install -i -r -y --from-paths src

# Build the ROS 2 workspace
RUN --mount=type=cache,target=${DIR}/colcon-cache \
    colcon build --symlink-install

############################
# Stage 7: Runtime Application
############################
FROM runtime AS app
ARG DIR=/cprt_rover_24
WORKDIR ${DIR}

# Copy built workspace from builder
COPY --from=builder ${DIR}/install ${DIR}/install

# Set environment for ROS 2
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/profile.d/ros.sh && \
    echo "if [ -f ${DIR}/install/setup.bash ]; then source ${DIR}/install/setup.bash; fi" >> /etc/profile.d/ros.sh

CMD ["/bin/bash"]