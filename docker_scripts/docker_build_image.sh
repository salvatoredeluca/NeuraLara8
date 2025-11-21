#!/bin/bash


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [[ ( $@ == "--help") ||  $@ == "-h" ]]
then 
    echo "Usage: $0 [IMAGE_NAME]"
    exit 0
fi

if [[ ($# -eq 0 ) ]]
then 
    echo "Usage: $0 [IMAGE_NAME]"
    exit 1
else
    IMAGE_NAME=$1
    echo "Building image $IMAGE_NAME using Dockerfile in $SCRIPT_DIR..."
    
   
    docker build -t $IMAGE_NAME \
        --build-arg USER_ID=$(id -u) \
        --build-arg GROUP_ID=$(id -g) \
        "$SCRIPT_DIR"
fi
