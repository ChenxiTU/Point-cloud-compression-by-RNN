#!/bin/bash
umask 0000
source ../../tools/devel/setup.bash 

mkdir ./recon
mkdir ./recon/raw_decompose

cp -r raw_decompose/* recon/raw_decompose/

interation_num=31

mkdir ./recon/origin
mkdir ./recon/origin/pcds
#
cd recon/origin

rosrun velodyne2image image2bag ../ ori 2 _calibration:=../../32db.yaml

cd ../../

for factor in `seq 10 $interation_num`
do
echo $factor
mkdir ./recon/$factor
mkdir ./recon/$factor/pcds
cp -r ./rearrange/$factor/* recon/raw_decompose/distance/

cd recon/$factor
rosrun velodyne2image image2bag ../ $factor 2 _calibration:=../../32db.yaml

cd ../../

done
