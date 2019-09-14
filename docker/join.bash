#!/usr/bin/env bash
#
# Typical usage: ./join.bash subt
#

IMG=("$@")

xhost +
containerid=$(docker ps -aqf "ancestor=${IMG}")
docker exec --privileged -e DISPLAY=${DISPLAY} -e LINES=`tput lines` -it ${containerid} bash
xhost -
