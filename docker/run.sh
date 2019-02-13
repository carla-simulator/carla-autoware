#!/bin/sh

usage() { echo "Usage: $0 [-t <tag>] [-r <repo>] [-s <Shared directory>]" 1>&2; exit 1; }

# Defaults
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
SHARED_DIR=/home/autoware/shared_dir
HOST_DIR=/home/$USER/shared_dir
DOCKER_HUB_REPO="carla-autoware"
TAG="latest"

while getopts ":ht:r:s:" opt; do
  case $opt in
    h)
      usage
      exit
      ;;
    t)
      TAG=$OPTARG
      ;;
    r )
      DOCKER_HUB_REPO=$OPTARG
      ;;
    s)
      HOST_DIR=$OPTARG
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done

echo "Using $DOCKER_HUB_REPO:$TAG"
echo "Shared directory: ${HOST_DIR}"

if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi


docker run -it --rm \
    --runtime=nvidia \
    --volume=$XAUTH:$XAUTH:rw \
    --volume=$XSOCK:$XSOCK:rw \
    --volume=$HOST_DIR:$SHARED_DIR:rw \
    --env="XAUTHORITY=$XAUTH" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    -u autoware \
    --net=host \
    $DOCKER_HUB_REPO:$TAG
