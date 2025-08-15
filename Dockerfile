ARG IMAGE=cprtsoftware/cprt_rover_24-jetson:latest
FROM ${IMAGE} AS base

FROM base AS builder

WORKDIR /cprt_rover_24

RUN colcon build --continue-on-error --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) \
             --build-base build-$aarch \
             --install-base install-$(uname -m)

FROM base AS runtime

# Create entrypoint script
RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /entrypoint.sh && \
    echo 'source /cprt_rover_24/setup.bash' >> /entrypoint.sh && \
    echo 'exec "$@"' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh
COPY --from=builder /cprt_rover_24/install-$(uname -m)/* /cprt_rover_24

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]