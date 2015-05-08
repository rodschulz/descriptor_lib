#!/bin/sh

alias pcv='pcl_viewer'

cd ./output/
pcv pointPosition.pcd &
pcv patch.pcd band0.pcd band45.pcd band90.pcd band135.pcd &
