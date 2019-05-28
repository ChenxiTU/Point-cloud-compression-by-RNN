#!/bin/bash
source /home/tu/anaconda3/bin/activate
source activate /home/tu/research/py35/

interation_num=32

for file in $(ls ./raw_decompose/distance/)
do
  echo Encoding $file
  mkdir -p ./codes
  python ../encoder.py --model ../checkpoint/encoder_epoch_00000200.pth --input ./raw_decompose/distance/$file --cuda --output ./codes/$file --iterations $interation_num
  echo Decoding $file
  mkdir -p ./decoded/$file
  python ../decoder.py --model ../checkpoint/decoder_epoch_00000200.pth --input ./codes/$file.npz --cuda --output ./decoded/$file --iterations $interation_num
done


