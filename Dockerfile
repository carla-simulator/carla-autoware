ARG CARLA_VERSION=0.9.9
ARG AUTOWARE_VERSION=1.14.0-melodic-cuda

FROM carlasim/carla:$CARLA_VERSION AS carla
FROM autoware/autoware:$AUTOWARE_VERSION

USER autoware
WORKDIR /home/autoware

# CARLA PythonAPI
RUN mkdir ./PythonAPI
COPY --from=carla /home/carla/PythonAPI ./PythonAPI
RUN echo "export PYTHONPATH=\$PYTHONPATH:~/PythonAPI/carla/dist/carla-0.9.9-py2.7-linux-x86_64.egg" >> .bashrc \
    && echo "export PYTHONPATH=\$PYTHONPATH:~/PythonAPI/carla" >> .bashrc

# CARLA ROS Bridge
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
        python-pip \
        python-wheel \
	    ros-melodic-ackermann-msgs \
        ros-melodic-derived-object-msgs \
	&& sudo rm -rf /var/lib/apt/lists/*
RUN pip install simple-pid pygame networkx==2.2

RUN git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git

# CARLA Autoware agent
COPY --chown=autoware . ./carla-autoware

RUN mkdir -p carla_ws/src
RUN cd carla_ws/src \
    && ln -s ../../ros-bridge \
    && ln -s ../../carla-autoware/carla-autoware-agent \
    && cd .. \
    && source /opt/ros/melodic/setup.bash && catkin_make

RUN echo "export CARLA_AUTOWARE_CONTENTS=~/autoware-contents" >> .bashrc \
    && echo "source ~/carla_ws/devel/setup.bash" >> .bashrc \
    && echo "source ~/Autoware/install/setup.bash" >> .bashrc

CMD ["/bin/bash"]

