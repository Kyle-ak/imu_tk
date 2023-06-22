if [ -z "$1" ]; then echo "No names was provided for the image. Usage: ./docker_start_gui.sh image_name:tag" && exit 1;fi

xhost + #Allow Xserver connections from the docker machine
sudo docker run --rm --network=host -it --name motion_imitation -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v $(pwd):/home/imu_tk/host $1 &&
xhost - #Disable the Xserver remote connection for safety reasons