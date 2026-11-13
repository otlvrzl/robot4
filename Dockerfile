# Dockerfile - a recipe that builds the "computer inside your computer" (the image)
# where ROS 2, Gazebo and RViz are installed. Your own code is NOT baked in:
# it is mounted from your host at run time (see docker-compose.yml), so you can edit
# it with your normal editor and keep it in git.

# Start from the official ROS 2 Jazzy image (Ubuntu 24.04 + ROS 2 + RViz + basic tools).
FROM osrf/ros:jazzy-desktop

# Settings that can be overridden at build time (docker-compose passes your real IDs).
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
# Stops apt from asking questions during the build.
ENV DEBIAN_FRONTEND=noninteractive

# Install what the base image does not have yet.
RUN apt-get update && apt-get install -y --no-install-recommends \
        # ros-gz = Gazebo Harmonic + the ROS <-> Gazebo bridge and launch tools
        ros-jazzy-ros-gz \
        # publishes the robot's TF tree from the URDF
        ros-jazzy-robot-state-publisher \
        ros-jazzy-rviz2 \
        # drive the robot with the keyboard (useful later)
        ros-jazzy-teleop-twist-keyboard \
        # SLAM: builds a map from the LIDAR data while the robot drives (Stage 2)
        ros-jazzy-slam-toolbox \
        # map_saver_cli: the tool that saves the finished map to files (Stage 2).
        # It is part of Nav2, which will also use the saved map later.
        ros-jazzy-nav2-map-server \
        # the `colcon` build tool for ROS packages
        python3-colcon-common-extensions \
        # OpenGL libraries: Gazebo and RViz need them to draw 3D graphics
        mesa-utils \
        libgl1-mesa-dri \
        libglx-mesa0 \
        libegl1 \
        # tiny test program (xeyes) to check that windows can open
        x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Create a normal (non-root) user with the SAME numeric ID as your host user.
# Why: files created inside the container (e.g. in ./src) then belong to YOU on the host,
# not to root, so you can edit and commit them without permission problems.
# (Ubuntu 24.04 images contain a default user "ubuntu" with ID 1000; we remove it first.)
RUN (userdel -r ubuntu 2>/dev/null || true) \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    # /ws is the ROS "workspace": src = your code, build/install/log = build output
    && mkdir -p /ws/src /ws/build /ws/install /ws/log \
    && chown -R ${USERNAME}:${USERNAME} /ws

# The entrypoint script runs every time the container starts (it loads the ROS environment).
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Settings so GUI programs work inside Docker:
#   QT_X11_NO_MITSHM         - avoids a shared-memory problem with the X11 display
#   QT_QPA_PLATFORM=xcb      - makes Qt (used by RViz/Gazebo) use X11, which also works on Wayland
#   ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST - ROS 2 only talks to programs on this computer,
#                            so it does not mix with other ROS users on your network
#   NVIDIA_*                 - lets NVIDIA graphics cards be used (needs nvidia-container-toolkit)
ENV QT_X11_NO_MITSHM=1 \
    QT_QPA_PLATFORM=xcb \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

# Load ROS automatically when you open a terminal inside the container.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /home/${USERNAME}/.bashrc

# From here on, run as the normal user and start in the workspace folder.
USER ${USERNAME}
WORKDIR /ws

ENTRYPOINT ["/ros_entrypoint.sh"]
# Default command if none is given: open a bash terminal.
CMD ["bash"]
