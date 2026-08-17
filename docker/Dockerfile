ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base-noble

ARG ROS_DISTRO=jazzy
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    git \
    curl \
    vim \
    nano \
    bash-completion \
    build-essential \
    cmake \
    pkg-config \
    fonts-noto-cjk \
    can-utils \
    iproute2 \
    i2c-tools \
    libsocketcan-dev \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-dev-tools \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --break-system-packages \
    Adafruit-Blinka \
    Adafruit-Extended-Bus \
    adafruit-circuitpython-bno08x

RUN set -eux; \
    EXISTING_GROUP="$(getent group "${USER_GID}" | cut -d: -f1 || true)"; \
    if [ -z "${EXISTING_GROUP}" ]; then \
        groupadd --gid "${USER_GID}" "${USERNAME}"; \
    elif [ "${EXISTING_GROUP}" != "${USERNAME}" ]; then \
        groupmod --new-name "${USERNAME}" "${EXISTING_GROUP}"; \
    fi; \
    EXISTING_USER="$(getent passwd "${USER_UID}" | cut -d: -f1 || true)"; \
    if [ -z "${EXISTING_USER}" ]; then \
        useradd \
            --uid "${USER_UID}" \
            --gid "${USER_GID}" \
            --create-home \
            --shell /bin/bash \
            "${USERNAME}"; \
    elif [ "${EXISTING_USER}" != "${USERNAME}" ]; then \
        usermod \
            --login "${USERNAME}" \
            --home "/home/${USERNAME}" \
            --move-home \
            --shell /bin/bash \
            --gid "${USER_GID}" \
            "${EXISTING_USER}"; \
    fi; \
    usermod \
        --gid "${USER_GID}" \
        --shell /bin/bash \
        "${USERNAME}"; \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" \
        > "/etc/sudoers.d/${USERNAME}"; \
    chmod 0440 "/etc/sudoers.d/${USERNAME}"

RUN mkdir -p /ws \
    && chown -R ${USERNAME}:${USERNAME} /ws

COPY docker/entrypoint.sh /usr/local/bin/lgh_ws_entrypoint.sh
COPY docker/ros_setup.bash /usr/local/bin/lgh_ws_ros_setup.bash

RUN chmod +x /usr/local/bin/lgh_ws_entrypoint.sh \
    && printf '\nsource /usr/local/bin/lgh_ws_ros_setup.bash\n' \
        >> "/home/${USERNAME}/.bashrc"

USER ${USERNAME}
WORKDIR /ws

ENTRYPOINT ["/usr/local/bin/lgh_ws_entrypoint.sh"]
CMD ["bash"]
