#!/bin/bash

# Change name also in stop.sh
TMUX_NAME=manipulation-tmux
DOCKER_CONTAINER_NAME=ergocub_manipulation_container

echo "Start this script inside the ergoCub visual perception rooot folder"
usage() { echo "Usage: $0 [-i ip_address] [-y (to start yarp server]" 1>&2; exit 1; }

while getopts i:yh flag
do
    case "${flag}" in
        i) SERVER_IP=${OPTARG};;
        y) START_YARP_SERVER='1';;
        h) usage;;
        *) usage;;
    esac
done

# Start the container with the right options
docker run -itd --rm --network=host \
  --env DISPLAY=:0 --env QT_X11_NO_MITSHM=1 --env XDG_RUNTIME_DIR=/root/1000 --env XAUTHORITY=/root/.Xauthority \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "${XAUTHORITY}":/root/.Xauthority:rw \
  -v "$(pwd)":/root/ergocub-manipulation \
  --name $DOCKER_CONTAINER_NAME \
  ar0s/ergocub-manipulation bash

# Create tmux session
tmux new-session -d -s $TMUX_NAME

# Set server
tmux send-keys -t $TMUX_NAME "docker exec -it $DOCKER_CONTAINER_NAME bash" Enter
if [ -n "$SERVER_IP" ] # Variable is non-null
then
  tmux send-keys -t $TMUX_NAME "yarp namespace ergocub00" Enter
  tmux send-keys -t $TMUX_NAME "yarp conf $SERVER_IP 10000" Enter
else
  if [ -n "$START_YARP_SERVER" ]
  then
    tmux send-keys -t $TMUX_NAME "yarpserver --write" Enter
  else 
    tmux send-keys -t $TMUX_NAME "yarp detect --write" Enter
  fi
fi

tmux split-window -h -t $TMUX_NAME

# Start Gazebo with iCub
tmux send-keys -t $TMUX_NAME "docker exec -it $DOCKER_CONTAINER_NAME bash" Enter
tmux send-keys -t $TMUX_NAME "gazebo ~/ergocub-manipulation/gazebo/worlds/ergocub_grasp_demo.sdf" Enter
tmux select-pane -t $TMUX_NAME:0.0
tmux split-window -v -t $TMUX_NAME

# Start grasping script
tmux send-keys -t $TMUX_NAME "docker exec -it $DOCKER_CONTAINER_NAME bash" Enter
tmux send-keys -t $TMUX_NAME "cd /root/ergocub-manipulation/build/bin" Enter
tmux send-keys -t $TMUX_NAME "sleep 10" Enter
tmux send-keys -t $TMUX_NAME "./ergocub_grasp_demo /ergocubSim /robotology-superbuild/build/install/share/ergoCub/robots/ergoCubGazeboV1/model.urdf" Enter
tmux select-pane -t $TMUX_NAME:0.2
tmux split-window -v -t $TMUX_NAME

# Start bash for fun
tmux send-keys -t $TMUX_NAME "docker exec -it $DOCKER_CONTAINER_NAME bash" Enter
tmux send-keys -t $TMUX_NAME "sleep 12" Enter
tmux send-keys -t $TMUX_NAME "yarp rpc /Components/Manipulation" Enter

# Attach
tmux a -t $TMUX_NAME
