# CARLA-Autoware Docker image for the CARLA AD Challenge

https://carlachallenge.org/

This a a blueprint how to to build a container for the CARLA AD Challenge.


## Requirements

- Prebuilt CARLA-Autoware Docker image named "carla-autoware:latest"

### Build image
Select an image name, e.g. "carla-autoware-challenge:latest" as <CARLA_AUTOWARE_CHALLENGE_IMAGE_NAME>.

Call within this folder:

   docker build -t <CARLA_AUTOWARE_CHALLENGE_IMAGE_NAME> <ADDITIONAL_DOCKER_PARAMETERS>

<ADDITIONAL_DOCKER_PARAMETERS> may be required e.g. when building behind a network proxy.


### Run image
Please consult the main README regarding execution of Carla, the Carla Autoware Bridge and the
Autoware stack.
Start a Carla server as described there.

Start the container:
   docker run -it --rm --net=host <CARLA_AUTOWARE_CHALLENGE_IMAGE_NAME>

Within the Docker shell:
    /workspace/scenario_runner/srunner/challenge/run_evaluator.sh

or any python call refining the scenario runner execution.

Please consult https://carlachallenge.org/submit/ and
https://github.com/carla-simulator/scenario_runner/tree/carla_challenge for up-to-date
instructions.

