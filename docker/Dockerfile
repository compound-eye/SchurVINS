FROM ros:melodic-perception-bionic

# Build tools and non-ROS system libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git sudo vim x11-apps python3-pip \
    autoconf automake libtool \
    python-catkin-tools python-vcstool \
    libglew-dev libopencv-dev libyaml-cpp-dev \
    libblas-dev liblapack-dev libsuitesparse-dev \
    ros-melodic-rqt-gui ros-melodic-rqt-gui-py \
    ros-melodic-tf-conversions ros-melodic-rviz \
    python-rospkg python-yaml \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies used by CE companion scripts (py/run_schurvins_batch.py, py/ce_pose.py)
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt && rm /tmp/requirements.txt

# Create catkin workspace and clone third-party deps (baked into image)
RUN mkdir -p /catkin_ws/src
COPY dependencies.yaml /tmp/dependencies.yaml
RUN cd /catkin_ws/src && vcs-import < /tmp/dependencies.yaml \
    && touch minkindr/minkindr_python/CATKIN_IGNORE \
    && rm /tmp/dependencies.yaml \
    && sed -i 's|git@github.com:dorian3d/DBoW2.git|https://github.com/dorian3d/DBoW2.git|' \
       dbow2_catkin/CMakeLists.txt

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
