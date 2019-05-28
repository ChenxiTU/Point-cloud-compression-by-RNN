#include <ros/ros.h>
#include "ros/package.h"
#include "velodyne_pointcloud/calibration.h"

#include <string>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <iostream>
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

#include <boost/foreach.hpp>
#include <sstream>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
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
typedef VPoint PointT;
#define foreach BOOST_FOREACH

int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_calibration");
  ros::NodeHandle private_nh("~");

  std::string s;
  if (private_nh.getParam("calibration", s))
  {
      ROS_INFO("Got param: %s", s.c_str());
  }
      else
  {
    ROS_ERROR("Failed to get param 'my_param'");
  }

  rosbag::Bag bag;
  string name_pcd;
  string file_in,route_out;

  file_in=string(argv[1]);
  route_out=string(argv[2]);

  bag.open(file_in, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("/newpacket"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::cout << "Toatally" << view.size() <<" messages" << std::endl;

  int frame_ID = 1;

  foreach(rosbag::MessageInstance const m, view)
  {
    velodyne_msgs::VelodyneScan::ConstPtr scan = m.instantiate<velodyne_msgs::VelodyneScan>();

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


        data_->unpack(scan->packets[i], *outMsg);
//        cout << outMsg->height << "." << outMsg->width <<endl;
      }

    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Saving " << outMsg->height * outMsg->width
                     << " Velodyne points, time: " << outMsg->header.stamp);

    cout <<  "Saving " << outMsg->height * outMsg->width
          << " Velodyne points, time: " << outMsg->header.stamp << endl;

    stringstream ss;
    ss<<setw(5)<<setfill('0')<<frame_ID;
    string str_frame_ID=ss.str();

    string whole_name = file_in.substr(file_in.rfind("/"), file_in.length());
//    cout << whole_name.substr(1, whole_name.rfind(".") - 1) << endl;


    name_pcd = whole_name.substr(1, whole_name.rfind(".") - 1) + "_" + str_frame_ID +".pcd";

    cout << name_pcd << endl;
    pcl::io::savePCDFileASCII ("./" + route_out + "/" + name_pcd, *outMsg);

    frame_ID++;

  }
  bag.close();
}

