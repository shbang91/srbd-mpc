#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd $SCRIPT_DIR/..

if [ -d "build/" ]
then
cd build
else
mkdir build && cd build
fi

cmake ..
make -j
