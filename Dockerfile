ARG AUTOWARE_VERSION=latest-melodic-cuda

FROM autoware/autoware:$AUTOWARE_VERSION

USER autoware
ENV USERNAME autoware

WORKDIR /home/autoware

# Update simulation repo to latest master.
COPY --chown=autoware update_sim.patch /home/$USERNAME/Autoware
RUN patch ./Autoware/autoware.ai.repos /home/$USERNAME/Autoware/update_sim.patch

# Change code in simulation package.
COPY --chown=autoware update_code.patch /home/$USERNAME/Autoware/src/autoware/simulation
RUN cd /home/$USERNAME/Autoware \
    && vcs import src < autoware.ai.repos \
    && cd /home/$USERNAME/Autoware/src/autoware/simulation \
    && git apply update_code.patch

# Compile with colcon build.
RUN cd ./Autoware \
    && source /opt/ros/melodic/setup.bash \
    && AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# CARLA PythonAPI
RUN mkdir ./PythonAPI
ADD --chown=autoware https://carla-releases.s3.eu-west-3.amazonaws.com/Backup/carla-0.9.11-py2.7-linux-x86_64.egg ./PythonAPI
RUN echo "export PYTHON2_EGG=$(ls /home/autoware/PythonAPI | grep py2.)" >> .bashrc \
    && echo "export PYTHONPATH=\$PYTHONPATH:~/PythonAPI/\$PYTHON2_EGG" >> .bashrc

# CARLA ROS Bridge
# There is some kind of mismatch between the ROS debian packages installed in the Autoware image and
# the latest ros-melodic-ackermann-msgs and ros-melodic-derived-objects-msgs packages. As a
# workaround we use a snapshot of the ROS apt repository to install an older version of the required
# packages.
USER root
RUN rm -f /etc/apt/sources.list.d/ros1-latest.list
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
RUN sh -c 'echo "deb http://snapshots.ros.org/melodic/2020-08-07/ubuntu $(lsb_release -sc) main" >> /etc/apt/sources.list.d/ros-snapshots.list'
RUN apt-get update && apt-get install -y --no-install-recommends \
        python-pip \
        python-wheel \
        ros-melodic-ackermann-msgs \
        ros-melodic-derived-object-msgs \
    && rm -rf /var/lib/apt/lists/*
RUN pip install transforms3d simple-pid pygame networkx==2.2

USER autoware

RUN git clone -b '0.9.11' --recurse-submodules https://github.com/carla-simulator/ros-bridge.git

# CARLA Autoware agent
COPY --chown=autoware . ./carla-autoware

RUN mkdir -p carla_ws/src
RUN cd carla_ws/src \
    && ln -s ../../ros-bridge \
    && ln -s ../../carla-autoware/carla-autoware-agent \
    && cd .. \
    && source /opt/ros/melodic/setup.bash \
    && catkin_make

RUN echo "export CARLA_AUTOWARE_CONTENTS=~/autoware-contents" >> .bashrc \
    && echo "source ~/carla_ws/devel/setup.bash" >> .bashrc \
    && echo "source ~/Autoware/install/setup.bash" >> .bashrc

USER root

# (Optional) Install vscode
RUN apt-get update
RUN apt-get install -y software-properties-common apt-transport-https wget
RUN wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | apt-key add -
RUN add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
RUN apt-get -y install code

CMD ["/bin/bash"]
