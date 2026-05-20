ARG REPO_NAME="duckiebot-slam-opdracht2"
ARG DESCRIPTION="Duckiebot Opdracht 2 — Odometry, Vision SLAM en EKF sensorfusie"
ARG MAINTAINER="Student"
ARG ARCH=arm64v8

FROM duckietown/dt-ros-commons:ente-${ARCH}

# Python dependencies
RUN pip3 install --no-cache-dir \
        opencv-python \
        numpy \
        scipy

# Launchers
COPY ./launchers /launchers
RUN chmod +x /launchers/*.sh

WORKDIR /code/catkin_ws

ENTRYPOINT ["/launchers/default.sh"]
