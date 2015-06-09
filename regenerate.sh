#!/bin/bash

type=$1

cd ./build/
rm -rf *

if [ "$type" == "-r" ] ; then
	echo "Generating project for release"
	cmake -DCMAKE_BUILD_TYPE=Release ../src/
else
	echo "Generating project for debug"
	cmake -DCMAKE_BUILD_TYPE=Debug ../src/
fi

cd ..
./make.sh
