#include "ros/ros.h"
#include <ros/time.h>
#include "velodyne_pointcloud/rawdata.h"
#include "ros/package.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>  
#include <fstream>  
#include <iostream>  
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include "std_msgs/UInt16MultiArray.h"
#include <cmath>

#include <rosbag/bag.h>
#include <boost/foreach.hpp>

#include <vector>
#include <sstream>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>


using namespace std;
using namespace velodyne_rawdata;
using namespace cv;
#define foreach BOOST_FOREACH
uint scan_row,scan_column;
string file_in,route_out;
int frame_num= 1;
int interval= 1;
bool pad;

const int LASER_RING[64] =
{
  36, 37, 58, 59, 38, 39, 32, 33,
  40, 41, 34, 35, 48, 49, 42, 43,
  50, 51, 44, 45, 52, 53, 46, 47,
  60, 61, 54, 55, 62, 63, 56, 57,
  4, 5, 26, 27, 6, 7, 0, 1,
  8, 9, 2, 3, 16, 17, 10, 11,
  18, 19, 12, 13, 20, 21, 14, 15,
  28, 29, 22, 23, 30, 31, 24, 25

};

int main(int argc, char **argv)
{
  rosbag::Bag bag;

  if (argc == 4)
  {
//    interval = atof (argv[3]);
//    std::cout << "Change extract interval to" << interval << std::endl;
    pad = atof (argv[3]);
  }
  else
    pad = 0;



  printf("Extraction start! \n");
  file_in=string(argv[1]);
  route_out=string(argv[2]);

  bag.open(file_in, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_packets"));
//  topics.push_back(std::string("/hdl64/velodyne_packets"));
//  topics.push_back(std::string("/newpacket"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::cout << "Toatally" << view.size() <<" messages" << std::endl;
  std::cout << "Converting interval: " << interval << std::endl;
  int count=1;

  foreach(rosbag::MessageInstance const m, view)
  {
//  if (count < 61)
  if (count % interval == 0)
    {
      string name_distance;
      string name_intensity;
      string name_rotation;

      stringstream ss;
      ss<<setw(5)<<setfill('0')<<count;
      string str_count=ss.str();

      name_distance=file_in.substr(0,file_in.rfind("."));
      name_distance=route_out+name_distance+"_"+str_count+"_distance.png";

      name_intensity=file_in.substr(0,file_in.rfind("."));
      name_intensity=route_out+name_intensity+"_"+str_count+"_intensity.png";

      name_rotation=file_in.substr(0,file_in.rfind("."));
      name_rotation=route_out+name_rotation+"_"+str_count+"_roation.txt";

     ofstream ofile;
     ofile.open(name_rotation.c_str());


        velodyne_msgs::VelodyneScan::ConstPtr msg = m.instantiate<velodyne_msgs::VelodyneScan>();

        cout << msg->header.stamp << endl;

        if (msg->packets.size()==348) //64ES2
        {

          cout<<"Senor version:HDL64ES2/HDL64ES2.1"<<endl;
          scan_row=64;
          scan_column=2088;
        }
        else if (msg->packets.size()==580)//64ES3
        {
          scan_row=64;
          scan_column=6*580;;
          cout<<"Senor version:HDL64ES3"<<endl;
        }
        else if (msg->packets.size()==260)//64E 260
        {
          scan_row=64;
          scan_column=6*260;
          cout<<"Senor version:HDL64E"<<endl;
        }
        else if (msg->packets.size()==181)//32E 181
        {
          scan_row=32;
          scan_column=12*181;;
          cout<<"Senor version:HDL32E"<<endl;
        }

        else if (msg->packets.size()==79)//VLP16
        {
          scan_row=32;
          scan_column=12*79;;
          cout<<"Senor version:VLP16"<<endl;
        }
        else
        {
           cout<<"Unknown device"<<endl;
        }
        Mat img(scan_row, scan_column, CV_16U);
        Mat img2(scan_row, scan_column, CV_8U);
        ofile<<scan_row<<endl;
        ofile<<scan_column<<endl;


        vector<int>compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);



        for (int next=0; next<msg->packets.size();++next)
        {
         const raw_packet_t *raw=(const raw_packet_t *) &msg->packets[next].data[0];

        for (int i = 0; i < BLOCKS_PER_PACKET; i++) {

              // upper bank lasers are numbered [0..31]
              // NOTE: this is a change from the old velodyne_common implementation
              int bank_origin = 0;
              if (raw->blocks[i].header == LOWER_BANK) {
                // lower bank lasers are [32..63]
                bank_origin = 32;
              }
        for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {

                uint8_t laser_number;       ///< hardware laser number

                laser_number=j+bank_origin;
                union two_bytes tmp;
                tmp.bytes[0] = raw->blocks[i].data[k];
                tmp.bytes[1] = raw->blocks[i].data[k+1];
                uint d = tmp.uint;
                //printf("get %d\n",d);
                int col;
                if (scan_row==32)
                {col=next*12+i;}
                else
                {col=floor(next*6+i/2);}

                img.at<ushort>(laser_number, col) = d;
                img2.at<uchar>(laser_number, col) = raw->blocks[i].data[k+2];

                if (j==0)
                {
                  if (scan_row==32)
                  {
                    ofile<<raw->blocks[i].rotation<<endl;
                  }
                  else
                    if (i%2==0)
                    {
                      ofile<<raw->blocks[i].rotation<<endl;
                    }
                }

        }
        }

        }
//        if (pad == 1)
//        {
//          Mat img_process(scan_row, scan_column, CV_16U);
//          for (int i = 0; i < scan_row; i++ )
//          {img.row(LASER_RING[i]).copyTo(img_process.row(i));}

//          for (int i = 1; i < scan_row - 1; i++ )
//          {
//            for (int j = 1; j < scan_column - 1; j++ )
//            {
//              if (img_process.at<ushort>(i,j) == 0)
//              {
//                if (img_process.at<ushort>(i , j - 1) != 0)
//                {img_process.at<ushort>(i , j) = img_process.at<ushort>(i , j -1);}
//                else
//                {
//                  if (img_process.at<ushort>(i - 1, j) != 0)
//                  {img_process.at<ushort>(i, j) = img_process.at<ushort>(i - 1, j);}
//                  else
//                    if (img_process.at<ushort>(i, j + 1) != 0)
//                    {img_process.at<ushort>(i, j) = img_process.at<ushort>(i, j + 1);}
//                  else
//                      img_process.at<ushort>(i, j) = img_process.at<ushort>(i + 1, j);
//                }
//              }

//            }
//          }
//          if (cv::imwrite(name_distance,img_process,compression_params))
//          {

//              cv::imwrite(name_intensity,img2,compression_params);
//              cout<<"Frame "<<count<<" is extreacting"<<endl;
//          }
//        }
//        else
//        {
//          if (cv::imwrite(name_distance,img,compression_params))
//          {

//              cv::imwrite(name_intensity,img2,compression_params);
//              cout<<"Frame "<<count<<" is extreacting"<<endl;
//          }
//        }


        if (cv::imwrite(name_distance,img,compression_params))
        {

            cv::imwrite(name_intensity,img2,compression_params);
            cout<<"Frame "<<count<<" is extreacting"<<endl;
        }


        ofile.close();
    }
    count++;
  }
  bag.close();
}





