#include <ros/ros.h>
//#include "velodyne_pointcloud/rawdata.h"
#include "ros/package.h"
#include "velodyne_pointcloud/calibration.h"

#include <string>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <tf/transform_listener.h>
#include "velodyne_msgs/VelodyneScan.h"
#include "velodyne_pointcloud/point_types.h"
#include <velodyne_pointcloud/rawdata.h>

#include <unistd.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "driver.h"
#include "std_msgs/UInt16MultiArray.h"
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/foreach.hpp>
#include <sstream>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <dirent.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


using namespace std;
using namespace velodyne_rawdata;
using namespace cv;

typedef unsigned short uint16;

int recon_frame;
uint16* rota;
uint16 row;
uint16 column;
int mode = 1; //defalut mode: reconstruct packet and pointcloud topic
string name_pcd;

typedef VPoint PointT;



namespace velodyne_driver
{

VelodyneDriver::VelodyneDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh
                               )
{
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("velodyne"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  // get model name, validate string, determine packet rate
  if (column==2088)
  {
  private_nh.param("model", config_.model, std::string("64E_S2"));
  }
  else if (column==6*260)
    {
    private_nh.param("model", config_.model, std::string("64E"));
    }
  else if (column==12*181)
    {
    private_nh.param("model", config_.model, std::string("32E"));
    }
  else if (column==6*580)
    {
    private_nh.param("model", config_.model, std::string("64E_S3"));
    }
  else if (column==12*79)
    {
    private_nh.param("model", config_.model, std::string("VLP16"));
    }


  double packet_rate;                   // packet frequency (Hz)
   std::string model_full_name;
   if ((config_.model == "64E_S2") ||
       (config_.model == "64E_S2.1"))    // generates 1333312 points per second
     {                                   // 1 packet holds 384 points
       packet_rate = 3472.17;            // 1333312 / 384
       model_full_name = std::string("HDL-") + config_.model;
     }
   else if (config_.model == "64E_S3")
     {
       packet_rate = 5800.0; // experimental
       model_full_name = std::string("HDL-") + config_.model;
     }
   else if (config_.model == "64E")
     {
       packet_rate = 2600.0;
       model_full_name = std::string("HDL-") + config_.model;
     }
   else if (config_.model == "32E")
     {
       packet_rate = 1808.0;
       model_full_name = std::string("HDL-") + config_.model;
     }
   else if (config_.model == "VLP16")
     {
       packet_rate = 781.25;             // 300000 / 384
       model_full_name = "VLP-16";
     }
   else
     {
       ROS_ERROR_STREAM("unknown Velodyne LIDAR model: " << config_.model);
       packet_rate = 2600.0;
 }
  std::string deviceName("Velodyne HDL-" + config_.model);

  private_nh.param("rpm", config_.rpm, 600.0);
  ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
  double frequency = (config_.rpm / 60.0);     // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = (int) ceil(packet_rate / frequency);
  private_nh.getParam("npackets", config_.npackets);
  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate/config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic("velodyne_packets", diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_,
                                                             &diag_max_freq_,
                                                             0.1, 10),
                                        TimeStampStatusParam()));

  // open Velodyne input device or file
  // raw data output topic
  output_ = node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);
}





/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::poll_bag(Mat img_distance,Mat img_inten, rosbag::Bag &bag, ros::NodeHandle private_nh)
{
  uint16_t rot_angle;
  long i, j, ID_block,ID_packet;
  union two_bytes tmp;
  float dis;
  int intensity;
  uint8_t packet[1206];
  uint8_t head_p[6] = { 30, 30, 30, 30, 30, 30 };

  recon_frame++;
  //cout << config_.npackets;
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);
  scan->packets.resize(config_.npackets);


  cout << "Frame " << recon_frame << " is reconstructing"<<endl;


  if (row==64)
  {

  ID_block = 0;
  ID_packet=0;
  for (i = 0; i < column; i++)
  {
  rot_angle=*(rota+i);
  tmp.uint = uint16_t(0xeeff);
  packet[ID_block * 100] = tmp.bytes[0];
  packet[ID_block * 100 + 1] = tmp.bytes[1];

  tmp.uint = rot_angle;
  packet[ID_block * 100 + 2] = tmp.bytes[0];
  packet[ID_block * 100 + 3] = tmp.bytes[1];

  for (j = 0; j < 32; j++)
  {

  intensity=img_inten.at<uchar>(j,i);
  dis= img_distance.at<ushort>(j, i);
 //is = (dis+0.5) / 100.0 / 0.002f;
  tmp.uint = dis;
  int loca = ID_block * 100 + 4 + j * 3;
  packet[loca] = tmp.bytes[0];
  packet[loca + 1] = tmp.bytes[1];
  packet[loca + 2] = intensity;  //intensity
  }

  ID_block++;

  tmp.uint = uint16_t(0xddff);
  packet[ID_block * 100] = tmp.bytes[0];
  packet[ID_block * 100 + 1] = tmp.bytes[1];

  tmp.uint = rot_angle;
  packet[ID_block * 100 + 2] = tmp.bytes[0];
  packet[ID_block * 100 + 3] = tmp.bytes[1];

  for (j = 32; j < 64; j++)
  {
    dis = img_distance.at<ushort>(j, i);
    intensity=img_inten.at<uchar>(j,i);
//		dis = (dis+0.5) / 100.0 / 0.002f;
    tmp.uint = dis;
    int loca = ID_block * 100 + 4 + (j-32) * 3;
    packet[loca] = tmp.bytes[0];
    packet[loca + 1] = tmp.bytes[1];
    packet[loca + 2] = intensity;  //intensity
  }

  ID_block++;


  if (ID_block == 12)
        {

        memcpy(packet+1200, head_p, 6);
        velodyne_msgs::VelodynePacket *pkt=&scan->packets[ID_packet];
        memcpy(&pkt->data[0],packet,1206);
        pkt->stamp=ros::Time::now();
        ID_block = 0;
        ID_packet++;
        } //status
  }
  }

  if (row==32)
  {
    ID_block = 0;
    ID_packet=0;
    for (i = 0; i < column; i++)
    {
    rot_angle=*(rota+i);
    tmp.uint = uint16_t(0xeeff);
    packet[ID_block * 100] = tmp.bytes[0];
    packet[ID_block * 100 + 1] = tmp.bytes[1];

    tmp.uint = rot_angle;
    packet[ID_block * 100 + 2] = tmp.bytes[0];
    packet[ID_block * 100 + 3] = tmp.bytes[1];
    for (j = 0; j < 32; j++)
    {
    dis= img_distance.at<ushort>(j, i);
    intensity=img_inten.at<uchar>(j,i);
  //dis = (dis+0.5) / 100.0 / 0.002f;
    tmp.uint = dis;
    int loca = ID_block * 100 + 4 + j * 3;
    packet[loca] = tmp.bytes[0];
    packet[loca + 1] = tmp.bytes[1];
    packet[loca + 2] = intensity;  //intensity
    }
    ID_block++;
    if (ID_block == 12)
          {

          memcpy(packet+1200, head_p, 6);
          velodyne_msgs::VelodynePacket *pkt=&scan->packets[ID_packet];
          memcpy(&pkt->data[0],packet,1206);
          pkt->stamp=ros::Time::now();
          ID_block = 0;
          ID_packet++;
          }

  }


}

  ROS_DEBUG("Publishing a full Velodyne scan.");

  scan->header.stamp = ros::Time(scan->packets[config_.npackets - 1].stamp + ros::Duration(0.1 * recon_frame ));
//  scan->header.stamp = ros::Time(scan->packets[config_.npackets - 1].stamp);
  scan->header.frame_id = config_.frame_id;
//  bag.write("velodyne_packets", ros::Time::now() + ros::Duration(0.1) , scan);
  bag.write("velodyne_packets", scan->header.stamp , scan);

  cout << "mode: " << mode <<endl;

  if (mode == 2) //save point cloud
  {
    velodyne_rawdata::VPointCloud::Ptr
      outMsg(new velodyne_rawdata::VPointCloud());
    outMsg->header.stamp = pcl_conversions::toPCL(scan->header).stamp;
    outMsg->header.frame_id = scan->header.frame_id;
    outMsg->height = 1;

    velodyne_rawdata::RawData *data_(new velodyne_rawdata::RawData());

    data_->setup(private_nh);
//    data_->setParameters(1.5, 130, 0, 0);
    data_->setParameters(0, 500, 0, 0);

    for (size_t i = 0; i < scan->packets.size(); ++i)
      {

//        const raw_packet_t *raw = (const raw_packet_t *) scan->packets[i].data[1];
//        cout << raw->blocks[0].header <<endl;

        data_->unpack(scan->packets[i], *outMsg);
//        cout << outMsg->height << "." << outMsg->width <<endl;
      }

    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Saving " << outMsg->height * outMsg->width
                     << " Velodyne points, time: " << outMsg->header.stamp);

//    pcl::PointCloud<PointT>::Ptr pclCloud(new pcl::PointCloud<PointT>);

//    pcl::PCLPointCloud2 pcl_pc2;
//    pcl_conversions::toPCL(*outMsg,pcl_pc2);
//    pcl::fromPCLPointCloud2 (pcl_pc2, *pclCloud);
    cout <<  "Saving " << outMsg->height * outMsg->width
          << " Velodyne points, time: " << outMsg->header.stamp << endl;
    cout << name_pcd << endl;
    pcl::io::savePCDFileASCII ("./pcds/" + name_pcd, *outMsg);
  }



//  pubp->publish(scan);

//  diag_topic_->tick(scan->header.stamp);
//  diagnostics_.update();
  delete[] rota;
  return true;
}

} // namespace velodyne_driver



/*Extract file names*/
int getdir (string dir, vector<string> &files)
{
//DIR *dp;
struct dirent **namelist;
int n;

n = scandir(dir.data(), &namelist, 0, alphasort);
if (n < 0)
    perror("scandir");
else {
    while (n--) {
      files.push_back(string(namelist[n]->d_name));
      free(namelist[n]);
      }
    free(namelist);
    }

return 0;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image2bag");
//  ros::NodeHandle np;
  ros::NodeHandle private_nh("~");
  ros::NodeHandle node2;


  std::string s;
  if (private_nh.getParam("calibration", s))
  {
      ROS_INFO("Got param: %s", s.c_str());
  }
      else
  {
    ROS_ERROR("Failed to get param 'my_param'");
  }

//  string dir = string("./");
  string dir = string(argv[1]);
  string dir_rotation;
//  string rosbag_name = "aaa";
  string rosbag_name = string(argv[2]);

  if (argc == 4)
    mode = atof (argv[3]);


  dir_rotation = dir + "raw_decompose/rotation/";
//  dir_rotation = dir + "one_frame_bag/rotation/";
//  cout << "Reconstruct from:" << dir <<endl;

  vector<string> files = vector<string>();

  string name_distance;
  string name_intensity;
  string name_rotation;


  recon_frame=0;

  getdir(dir_rotation,files);

  rosbag::Bag bag;
  bag.open(rosbag_name.data(), rosbag::bagmode::Write);

  for (unsigned int i = files.size()-3;i >= 0;i--) {

    cout << files[i] << endl;
    ////////////////////////////////////////////////////////////////////////////////
    name_rotation = dir_rotation + files[i];
    name_distance = files[i].substr(0,files[i].rfind("_"));
    name_distance = dir+ "raw_decompose/distance/" + name_distance+"_distance.png";

    name_intensity = files[i].substr(0,files[i].rfind("_"));
    name_intensity = dir+ "raw_decompose/intensity/" + name_intensity+"_intensity.png";

    name_pcd = files[i].substr(0,files[i].rfind("_"))+".pcd";
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////
//    name_rotation = dir_rotation + files[i];
//    name_distance = files[i].substr(0,files[i].rfind("."));
//    name_distance = dir+ "one_frame_bag/distance/" + name_distance+".png";

//    name_intensity = files[i].substr(0,files[i].rfind("."));
//    name_intensity = dir+ "one_frame_bag/intensity/" + name_intensity+".png";
    /////////////////////////////////////////////////////////////////////////////////

    cout << name_distance <<endl;
    cout << name_intensity <<endl;

    ifstream ifile;
    ifile.open(name_rotation.c_str());
    ifile>>row>>column;

    rota=new uint16[column];
    Mat img_distance(row, column, CV_16U);
    Mat img_inten(row, column, CV_8U);

    cout << img_distance.at<ushort>(10, 10);

     for(int i=0; i<column;i++ )
     {
         ifile>>*(rota+i);
     }


     img_distance=cv::imread(name_distance, CV_LOAD_IMAGE_UNCHANGED);
     img_inten=cv::imread(name_intensity, CV_LOAD_IMAGE_UNCHANGED);


     velodyne_driver::VelodyneDriver dvr(node2, private_nh);
     dvr.poll_bag(img_distance,img_inten, bag, private_nh);
//     ros::Duration(0.1).sleep();
  }
  bag.close();


  return 0;
}
