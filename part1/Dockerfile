# 当前网络无法稳定访问 Docker Hub，因此使用 DaoCloud 镜像并固定官方 ROS 镜像摘要。
FROM m.daocloud.io/docker.io/library/ros:humble-ros-base-jammy@sha256:75dd3aba34a3838dadbb31a9f7bef769bdfa8713e6cec686fc868db2981b0987

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        python3-colcon-common-extensions \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /vbot_ws

COPY scripts/build/build_vbot_ws.sh /usr/local/bin/build_vbot_ws
RUN chmod +x /usr/local/bin/build_vbot_ws

CMD ["bash"]
