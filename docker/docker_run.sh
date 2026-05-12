#!/bin/bash

if grep -qi microsoft /proc/version; then
    export DEV_DISPLAY="host.docker.internal:0.0"
else
    export DEV_DISPLAY="${DISPLAY}"
fi

DISPLAY=$DEV_DISPLAY docker compose up -d