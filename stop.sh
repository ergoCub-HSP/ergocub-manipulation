#!/bin/bash

TMUX_NAME=manipulation-tmux
DOCKER_CONTAINER_NAME=ergocub_manipulation_container

docker rm -f $DOCKER_CONTAINER_NAME
tmux kill-session -t $TMUX_NAME
