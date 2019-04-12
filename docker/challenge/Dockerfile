ARG CARLA_AUTOWARE=carla-autoware:latest

ARG CARLA_VERSION=0.9.5
ARG CARLA_BUILD=''

FROM carlasim/carla:$CARLA_VERSION$CARLA_BUILD as carla

FROM $CARLA_AUTOWARE

USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
        libjpeg8 \
        libpng16-16 \
        libtiff5 \
        python3.5 \
        python-setuptools \
        python3-setuptools \
        python-wheel \
        python3-wheel \
        python-pip \
        python3-pip && \
        rm -rf /var/lib/apt/lists/*

RUN pip install py_trees==0.8.3 shapely six numpy networkx==2.2 scipy && \
    pip3 install py_trees==0.8.3 shapely six numpy networkx==2.2 scipy


WORKDIR /workspace
COPY --from=carla /home/carla/PythonAPI /workspace/CARLA/PythonAPI
COPY --from=carla /home/carla/HDMaps /workspace/CARLA/HDMaps

ENV CARLA_ROOT /workspace/CARLA

# installing CARLA eggs
RUN /usr/bin/easy_install CARLA/PythonAPI/carla/dist/*-py2.7-linux-x86_64.egg
RUN easy_install-3.5 CARLA/PythonAPI/carla/dist/*-py3.5-linux-x86_64.egg

ENV ROOT_SCENARIO_RUNNER /workspace/scenario_runner

COPY scenario_runner /workspace/scenario_runner
COPY team_code /workspace/team_code

ENV CARLA_MAPS_PATH /home/autoware/carla-autoware/autoware_data/maps

RUN mkdir -p /workspace/results
RUN chmod +x /workspace/scenario_runner/srunner/challenge/run_evaluator.sh

ENV PYTHONPATH "${ROOT_SCENARIO_RUNNER}":"${CARLA_ROOT}/PythonAPI/carla":${PYTHONPATH}

########################################################################################################################
########################################################################################################################
############                                BEGINNING OF USER COMMANDS                                      ############
########################################################################################################################
########################################################################################################################

ENV TEAM_AGENT /workspace/team_code/AutowareRosAgent.py
ENV TEAM_CONFIG ''

ENV TEAM_CODE_ROOT /home/autoware/carla-autoware/autoware_launch/
ENV CARLA_AUTOWARE_ROOT ${TEAM_CODE_ROOT}

COPY entrypoint.sh /workspace/entrypoint.sh

ENTRYPOINT ["/workspace/entrypoint.sh"]
CMD ["/bin/bash"]

########################################################################################################################
########################################################################################################################
############                                   END OF USER COMMANDS                                         ############
########################################################################################################################
########################################################################################################################

