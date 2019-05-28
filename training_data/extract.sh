#!/bin/bash
umask 0000

source ../tools/devel/setup.bash
mkdir raw_decompose

for file in $(ls *.bag)
do 
rosrun velodyne2image packet2image $file ./raw_decompose/
done

cd ./raw_decompose

mkdir rotation
mkdir distance
mkdir intensity

mv *.txt ./rotation
mv *_distance.png ./distance
mv *_intensity.png ./intensity

