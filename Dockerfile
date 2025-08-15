# syntax=docker/dockerfile:1.5

############################
# Stage 0: Base Image per Arch
############################
ARG BASE_IMAGE=ubuntu:22.04
ARG TARGETARCH
FROM ${BASE_IMAGE} AS base

############################
# Stage 1: Minimal ROS 2 Base
############################
FROM base AS ros2_humble-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

# Per-arch APT cache (mount to the real apt cache path; keep per-arch id)
RUN --mount=type=cache,target=/var/cache/apt,id=apt-${TARGETARCH},sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        locales apt-utils curl lsb-release gnupg2 \
        software-properties-common build-essential python3-dev python3-pip \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN --mount=type=cache,target=/var/cache/apt,id=apt-${TARGETARCH},sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base python3-setuptools python3-wheel \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies (per-arch pip cache)
COPY requirements.txt .
RUN --mount=type=cache,target=/root/.cache/pip,id=pip-${TARGETARCH},sharing=locked \
    pip3 install -r requirements.txt

############################
# Stage 2: ROS deps scan
############################
FROM ros2_humble-base AS ros2_humble-rosdeps
FROM ros2_humble-base AS ros2_humble-depsplan
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG TARGETARCH
ARG DIR=/cprt_rover_24
WORKDIR ${DIR}

# rosdep tooling only here
RUN --mount=type=cache,target=/var/cache/apt,id=apt-${TARGETARCH}-rosdeps,sharing=locked \
    apt-get update && apt-get install -y python3-rosdep && rm -rf /var/lib/apt/lists/*

# prime rosdep caches once
RUN --mount=type=cache,target=/var/cache/rosdistro,id=rosdistro \
    --mount=type=cache,target=/var/cache/rosdep,id=rosdep \
    rosdep init || true && rosdep update

# bring manifests for resolution
COPY src/**/package.xml src/**/setup.py src/**/CMakeLists.txt ./src

# Compute, don't install: write apt/pip lists that runtime can consume
# Skip unresolved keys (add more if needed)
ENV ROSDEP_SKIP_KEYS="kindr_ros"
RUN --mount=type=cache,target=/var/cache/rosdistro,id=rosdistro \
    --mount=type=cache,target=/var/cache/rosdep,id=rosdep \
    rosdep install -i -r -y --rosdistro humble --from-paths src -s --skip-keys="$ROSDEP_SKIP_KEYS" \
    | tee /opt/rosdep.plan && \
    awk '/apt(-get)? install/ {
           for (i=1;i<=NF;i++) {
             if ($i !~ /^(sudo|apt(-get)?|install|-y|--no-install-recommends)$/) print $i
           }
         }' /opt/rosdep.plan | sort -u > /opt/apt-packages.txt && \
    awk '/^pip3? install/ {
           for (i=1;i<=NF;i++) {
             if ($i !~ /^(sudo|pip3?|install|-r)$/) print $i
           }
         }' /opt/rosdep.plan | sort -u > /opt/pip-requirements.txt || true

############################
# Stage 3: GStreamer Build
############################
FROM base AS ros2_humble-gstreamer

RUN --mount=type=cache,target=/var/cache/apt,id=apt-${TARGETARCH},sharing=locked \
    apt-get update && apt-get install -y \
        zlib1g-dev libffi-dev libssl-dev python3-dev python3-pip \
        flex bison libglib2.0-dev libmount-dev libsrt-openssl-dev \
        git ninja-build curl \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade --user pip meson

WORKDIR /gstreamer
RUN --mount=type=cache,target=/root/.cache/gstreamer,id=gst-${TARGETARCH} \
    --mount=type=cache,target=/root/.cache/meson,id=meson-${TARGETARCH} \
    git clone https://gitlab.freedesktop.org/gstreamer/gstreamer.git . && \
    git checkout 1.24 && \
    ~/.local/bin/meson setup builddir --prefix=/opt/gstreamer --libdir=lib && \
    ~/.local/bin/meson compile -C builddir && \
    ~/.local/bin/meson install -C builddir --destdir /target

# Build GStreamer Rust plugins
WORKDIR /gst-plugins-rs
ENV LD_LIBRARY_PATH=/target/opt/gstreamer/lib:$LD_LIBRARY_PATH
ENV LIBRARY_PATH=/target/opt/gstreamer/lib:$LIBRARY_PATH
ENV PKG_CONFIG_PATH=/target/opt/gstreamer/lib/pkgconfig:$PKG_CONFIG_PATH

RUN --mount=type=cache,target=/root/.cargo,id=cargo-home-${TARGETARCH} \
    --mount=type=cache,target=/root/.cargo/registry,id=cargo-registry-${TARGETARCH},sharing=locked \
    --mount=type=cache,target=/root/.cargo/git,id=cargo-git-${TARGETARCH},sharing=locked \
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y && \
    . "$HOME/.cargo/env" && \
    git clone https://gitlab.freedesktop.org/gstreamer/gst-plugins-rs.git . && \
    cargo install cargo-c && \
    cargo cbuild -p gst-plugin-webrtc --prefix=/opt/gstreamer --release && \
    cargo cbuild -p gst-plugin-rtp    --prefix=/opt/gstreamer --release && \
    cargo cinstall -p gst-plugin-webrtc --prefix=/opt/gstreamer --destdir /target --libdir=lib --release && \
    cargo cinstall -p gst-plugin-rtp    --prefix=/opt/gstreamer --destdir /target --libdir=lib --release

############################
# Stage 4: Minimal Runtime Base
############################
FROM ros2_humble-base AS runtime
ARG TARGETARCH
# Copy just the lists produced above
COPY --from=ros2_humble-depsplan /opt/apt-packages.txt /opt/apt-packages.txt
COPY --from=ros2_humble-depsplan /opt/pip-requirements.txt /opt/pip-requirements.txt
COPY --from=ros2_humble-gstreamer /target /

ENV PATH=/opt/gstreamer/bin:$PATH
ENV LD_LIBRARY_PATH=/opt/gstreamer/lib:$LD_LIBRARY_PATH
ENV PKG_CONFIG_PATH=/opt/gstreamer/lib/pkgconfig:$PKG_CONFIG_PATH

# Ensure repos are visible; then install the exact packages, no rosdep needed
RUN --mount=type=cache,target=/var/cache/apt,id=apt-${TARGETARCH}-runtime,sharing=locked \
    apt-get update && \
    xargs -r -a /opt/apt-packages.txt apt-get install -y && \
    rm -rf /var/lib/apt/lists/*

# Optional: if pip requirements were found
RUN --mount=type=cache,target=/root/.cache/pip,id=pip-${TARGETARCH},sharing=locked \
    if [ -s /opt/pip-requirements.txt ]; then pip3 install -r /opt/pip-requirements.txt; fi

############################
# Stage 5: Dev Environment
############################
FROM runtime AS dev
RUN --mount=type=cache,target=/var/cache/apt,id=apt-${TARGETARCH},sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        git x11-apps ros-humble-desktop ros-dev-tools \
        ros-humble-ament-cmake python3-colcon-common-extensions \
        python3-colcon-ros clang-format ccache \
    && rm -rf /var/lib/apt/lists/*

# Enable compiler caching
ENV CCACHE_DIR=/root/.cache/ccache
ENV PATH="/usr/lib/ccache:${PATH}"

# Entrypoint
RUN echo '#!/bin/bash\nsource /opt/ros/humble/setup.bash\nif [ -f /cprt_rover_24/install/setup.bash ]; then source /cprt_rover_24/install/setup.bash; fi\nexec "$@"' > /entrypoint.sh \
    && chmod +x /entrypoint.sh

RUN useradd -ms /bin/bash -u 1000 vscode && echo "vscode ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

############################
# Stage 6: Builder
############################
FROM dev AS builder
SHELL ["/bin/bash", "-o", "pipefail", "-c"]   # <-- ensure we have 'source'
ARG TARGETARCH
ARG DIR=/cprt_rover_24
WORKDIR ${DIR}

COPY src/ ${DIR}/src/

# rosdep with ROS env loaded (uses cached rosdep/rosdistro)
RUN --mount=type=cache,target=/var/cache/rosdistro,id=rosdistro \
    --mount=type=cache,target=/var/cache/rosdep,id=rosdep \
    source /opt/ros/humble/setup.bash && \
    apt-get update && \
    rosdep init && rosdep update && \
    rosdep install -i -r -y --from-paths src

# colcon build with ROS env + ccache
RUN --mount=type=cache,target=/root/.cache/ccache,id=ccache-${TARGETARCH},sharing=locked \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

############################
# Stage 7: Runtime Application
############################
FROM runtime AS rover
ARG DIR=/cprt_rover_24
WORKDIR ${DIR}
COPY --from=builder ${DIR}/install ${DIR}/install
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/profile.d/ros.sh \
    && echo "if [ -f ${DIR}/install/setup.bash ]; then source ${DIR}/install/setup.bash; fi" >> /etc/profile.d/ros.sh

CMD ["/bin/bash"]
