#!/bin/bash

type=$1

cd ./build/
rm -rf *

if [ "$type" == "-r" ] ; then
	cmake -DCMAKE_BUILD_TYPE=Release ../src/
else
	cmake -DCMAKE_BUILD_TYPE=Debug ../src/
fi

cd ..
./make.sh
