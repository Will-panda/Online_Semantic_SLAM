#!/bin/bash


echo "Configuring and building ORB_SLAM2 ..."
if [ ! -d build ]; then
    mkdir build
    cd build
else
    echo "clean build files ..."
    cd build
    rm -rf *
fi

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
cd -