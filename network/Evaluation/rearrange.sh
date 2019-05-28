#!/bin/bash
umask 0000

mkdir rearrange

interation_num=32
interation_num=`expr $interation_num - 1`

for iter in `seq 0 $interation_num`
do
mkdir rearrange/`printf "%02d" $iter`
done

cd decoded
for file in $(ls)
do
for iter in `seq 0 $interation_num`
do
cp ${file}/`printf "%02d" $iter`.png ../rearrange/`printf "%02d" $iter`/${file}
done
done

