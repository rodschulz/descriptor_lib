#!/bin/bash

type=$1

# Remove old build folder
if [ -d "./build" ]; then
	echo "Removing build folder"
	rm -rf ./build
fi

# Create build folder
echo "Generating new build folder"
mkdir ./build/

# Generate with cmake
cd ./build/
if [ "$type" == "-r" ] ; then
	echo "Generating project for release"
	cmake -DCMAKE_BUILD_TYPE=Release ../src/
else
	echo "Generating project for debug"
	cmake -DCMAKE_BUILD_TYPE=Debug ../src/
fi
cd ..

# Create folders if needed
if [ ! -d "./input" ]; then
	mkdir ./input/
fi
if [ ! -d "./output" ]; then
	mkdir ./output/
fi
if [ ! -d "./cache" ]; then
	mkdir ./cache/
fi

# Make 
./make.sh
