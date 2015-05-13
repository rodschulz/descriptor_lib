#!/bin/sh

cd ./build/
rm -rf *
cmake -DCMAKE_BUILD_TYPE=Debug ../src/
cd ..
