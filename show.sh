#!/bin/sh

alias pcv='pcl_viewer'

cd ./output/
pcv pointPosition.pcd -use_point_picking &
pcv pointPosition.pcd band* -use_point_picking &
