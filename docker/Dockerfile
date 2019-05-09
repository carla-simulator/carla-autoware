ARG CARLA_VERSION=0.9.5
ARG CARLA_BUILD=''

ARG AUTOWARE_VERSION=1.11.0-kinetic-cuda

FROM nvidia/opengl:1.1-glvnd-runtime-ubuntu16.04 as nvidia-opengl
FROM carlasim/carla:$CARLA_VERSION$CARLA_BUILD as carla

FROM autoware/autoware:$AUTOWARE_VERSION
ARG CARLA_VERSION

SHELL ["/bin/bash", "-c"]

USER root
RUN apt-get update && apt-get upgrade -y \
	&& apt-get install -y python-pip python-wheel libpng16-16 ros-kinetic-ackermann-msgs ros-kinetic-derived-object-msgs --no-install-recommends \
	&& rm -rf /var/lib/apt/lists/*
RUN pip install simple-pid pygame networkx==2.2

COPY --from=nvidia-opengl /usr/local/lib/x86_64-linux-gnu /usr/local/lib/x86_64-linux-gnu
COPY --from=nvidia-opengl /usr/local/lib/i386-linux-gnu /usr/local/lib/i386-linux-gnu
COPY --from=nvidia-opengl /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json

COPY --from=nvidia-opengl /etc/ld.so.conf.d/glvnd.conf /etc/ld.so.conf.d/glvnd.conf

ENV NVIDIA_VISIBLE_DEVICES \
         ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
         ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,display

ENV LD_LIBRARY_PATH /usr/local/lib/x86_64-linux-gnu:/usr/local/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

COPY --from=carla --chown=autoware /home/carla/PythonAPI home/$USERNAME/carla-autoware/PythonAPI
ENV PYTHONPATH=/home/$USERNAME/carla-autoware/PythonAPI/carla/dist/carla-$CARLA_VERSION-py2.7-linux-x86_64.egg:/home/$USERNAME/carla-autoware/PythonAPI/carla

USER autoware

#patch and (re-)compile Autoware stack
COPY --chown=autoware docker/Autoware-eliminate-pcl-warnings.patch /home/$USERNAME/
RUN patch -p1 -d /home/$USERNAME/Autoware < /home/$USERNAME/Autoware-eliminate-pcl-warnings.patch; \
                  source /opt/ros/$ROS_DISTRO/setup.bash; \
                  cd /home/$USERNAME/Autoware/ros; \
                  ./colcon_release

RUN cd && mkdir -p /home/$USERNAME/carla-autoware
COPY --chown=autoware catkin_ws /home/$USERNAME/carla-autoware/catkin_ws

RUN source /etc/profile.d/ros.sh; source /home/autoware/Autoware/ros/install/local_setup.bash; \
    cd /home/$USERNAME/carla-autoware/catkin_ws/src; catkin_init_workspace; cd ../; \
    catkin_make -DCMAKE_BUILD_TYPE=Release

COPY --chown=autoware autoware_data /home/$USERNAME/carla-autoware/autoware_data
COPY --chown=autoware autoware_launch /home/$USERNAME/carla-autoware/autoware_launch
COPY --chown=autoware docker/param.yaml /home/$USERNAME/Autoware/ros/src/util/packages/runtime_manager/scripts/param.yaml

RUN echo $'export CARLA_AUTOWARE_ROOT=/home/$USERNAME/carla-autoware/autoware_launch \n\
           export CARLA_MAPS_PATH=/home/$USERNAME/carla-autoware/autoware_data/maps \n\
           source /home/$USERNAME/Autoware/ros/install/setup.bash \n\
           source /home/$USERNAME/carla-autoware/catkin_ws/devel/setup.bash \n' \
           >> /home/$USERNAME/.bashrc
RUN echo "export PYTHONPATH=\$PYTHONPATH:${PYTHONPATH}" >> /home/$USERNAME/.bashrc

#replace Autoware entrypoint
WORKDIR /home/$USERNAME/
ENTRYPOINT []
CMD ["/bin/bash"]

