#!/bin/bash


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

if [[ ( $@ == "--help") ||  $@ == "-h" ]]
then 
    echo "Usage: $0 [IMAGE_NAME] [CONTAINER_NAME] [FOLDER_NAME]"
    echo "Example: ./run.sh my_image my_container src_folder"
    exit 0
fi

if [[ $# -lt 3 ]]
then 
    echo "Error: Missing arguments."
    echo "Usage: $0 [IMAGE_NAME] [CONTAINER_NAME] [FOLDER_NAME]"
    exit 1
else
    IMAGE_NAME=$1
    CONTAINER_NAME=$2
    FOLDER_NAME=$3
    
    
    WORK_DIR="$PROJECT_ROOT/$FOLDER_NAME"

   
    if [ ! -d "$WORK_DIR" ] 
    then
        echo "[WARNING] Folder $WORK_DIR doesn't exist, creating a new one..."
        mkdir -p "$WORK_DIR"
    else
        echo "Mounting workspace from: $WORK_DIR"
    fi
    
   
    xhost +local:root

   
    docker run --privileged --rm -it \
        --name="$CONTAINER_NAME" \
        --net=host \
        --env="DISPLAY=$DISPLAY" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:ro" \
        --volume="$WORK_DIR:/home/user/ros2_ws/src" \
        "$IMAGE_NAME"

   
    xhost -local:root
fi
