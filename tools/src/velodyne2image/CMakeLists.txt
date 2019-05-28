cmake_minimum_required(VERSION 2.8.3)
project(velodyne2image)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  message_generation
  pcl_ros
  sensor_msgs
  velodyne_msgs
  pcl_conversions
  velodyne_pointcloud
  )
find_package( OpenCV REQUIRED )


set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    nodelet
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    tf
    velodyne_driver
    velodyne_msgs
    dynamic_reconfigure
)

add_message_files(
  DIRECTORY msg
  FILES
  CompressedPacket.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS system_lib message_runtime velodyne_rawdata
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(image2packet src/image2packet.cpp)
target_link_libraries(image2packet ${catkin_LIBRARIES})
target_link_libraries(image2packet ${PCL_LIBRARIES})
target_link_libraries(image2packet ${OpenCV_LIBRARIES})

add_executable(packet2image src/packet2image.cpp)
target_link_libraries(packet2image ${catkin_LIBRARIES})
target_link_libraries(packet2image ${PCL_LIBRARIES})
target_link_libraries(packet2image ${OpenCV_LIBRARIES})
target_link_libraries(packet2image ${libpcap_LIBRARIES})

add_executable(singlePCDextraction src/singlePCDextraction.cpp)
target_link_libraries(singlePCDextraction ${catkin_LIBRARIES})
target_link_libraries(singlePCDextraction ${PCL_LIBRARIES})
target_link_libraries(singlePCDextraction ${OpenCV_LIBRARIES})
target_link_libraries(singlePCDextraction ${libpcap_LIBRARIES})

add_executable(image2bag src/image2bag.cpp)
target_link_libraries(image2bag
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES}
    )

add_executable(local_calibration src/local_calibration.cpp)
target_link_libraries(local_calibration
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES}
    )





