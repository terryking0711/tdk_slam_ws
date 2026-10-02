#!/bin/bash
set -e
cd "$(dirname "$0")"

if grep -qi microsoft /proc/version; then
    export DISPLAY="host.docker.internal:0.0"
else
    # allow the container to open GUI windows (rviz2 / gazebo)
    xhost +local:docker >/dev/null 2>&1 || true
fi

export HOST_UID=$(id -u)
export HOST_GID=$(id -g)

docker compose up -d --build
echo "Container started. Enter with: docker exec -it tdk_slam bash"
