#!/bin/bash
sudo docker run -it --rm --platform linux/x86_64 --mount type=bind,source=${PWD},target=/home/srbd-mpc srbd-mpc bash
