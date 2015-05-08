#!/bin/sh

alias pcv='pcl_viewer'

cd ./output/
pcv pointPosition.pcd &
pcv pointPosition.pcd band* &
