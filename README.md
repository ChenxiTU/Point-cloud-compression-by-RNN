# Point-cloud-compression-by-RNN

* PyTorch implementation of [our work published in ICRA2019](https://www.researchgate.net/publication/333419310_Point_Cloud_Compression_for_3D_LiDAR_Sensor_using_Recurrent_Neural_Network_with_Residual_Blocks)
* Support data from Velodyne HDL64E(S2,S3), HDL32E and VLP16 sensor 

### Requirements
* Linux (tested on Ubuntu 16.04)
* Python 3.5+
* PyTorch 0.3.1+
* ROS 
* Opencv

## Training your own network
### Install 
Build tools library (to convert LiDAR packet data into 2D matrix or reconstruct packet data back) by running the following 
command:

```
cd tools/src/
catkin_init_workspace
cd ../
catkin_make -DCMAKE_BUILD_TYPE=Release
```
### Build training data
Download training data from [here](https://data.tier4.jp/) or use you own data.  
Make sure rosbags have /velodyne_packets topic and copy them to /training_data.
Run 
```
cd training_data
./extract.sh
```
to build the trainging data.  
Each frame of point cloud from LiDAR is decomposed into a 2D intensity matrix (.png unrelated to our compression) and a 
rotation angle vector (.txt can be further compressed into a few bits by run length code) and a distance matrix (.png
our main target and input of network) 

### Training
```
cd network
python training.py  
```

## Examples
Here we provide an example of compression and decompression. We provide 32 frames' LiDAR data from Velodyne HDL32E
sensor in network/Evaluation/raw_decompose
 
1. Compression and decompression
```
./enc_dec.sh
```
All frames are compressed using max number (32) of interval, while decompressed from 10 to max number of interval.  
 
2. Reconstruction to point cloud (.pcd) file 
```
./rearrange.sh
./rearr2recon.sh 
```

## Acknowledgement
* [1zb/pytorch-image-comp-rnn](https://github.com/1zb/pytorch-image-comp-rnn): Our network's code references this work