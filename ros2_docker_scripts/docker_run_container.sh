#!/bin/bash

if [[ ( $@ == "--help") ||  $@ == "-h" ]]
then 
	echo "Usage: $0 [IMAGE_NAME] [CONTAINER_NAME] [FOLDER_NAME]"
else
	if [[ ($# -eq 0 ) ]]
	then 
		echo "Usage: $0 [IMAGE_NAME] [CONTAINER_NAME] [FOLDER_NAME]"
		#exit 0
	else
		dev_folder=$3
		if [ ! -d /home/$USER/$dev_folder ] 
		then
			echo "[WARNING] devel folder doesn't exists, creating a new one"
			mkdir -p /home/$USER/$dev_folder
			#exit 0
		fi
		
		xhost +local:root
		IMAGE_ID=$1
		docker run --privileged --rm -it --name=$2 --net=host --env="DISPLAY=$DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:ro" --volume=/home/$USER/$dev_folder:/home/user/ros2_ws/src $1
		xhost -local:root
	fi 	
fi