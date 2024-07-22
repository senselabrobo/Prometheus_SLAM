#include <glog/logging.h>
#include <ros/ros.h>
#include <gflags/gflags.h>
#include "prometheus_slam/client/prometheus_slam_client.h"
#include "prometheus_slam/common.h"

int main(int argc, char** argv) {
  // Start logging
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  google::SetLogDestination(google::INFO, 
  "/home/prometheus_slam_ws/src/prometheus_slam/output/prometheus_slam_Client_log/info.log");  
  google::SetLogDestination(google::WARNING, 
  "/home/prometheus_slam_ws/src/prometheus_slam/output/prometheus_slam_Client_log/warning.log");  
  google::SetLogDestination(google::ERROR, 
  "/home/prometheus_slam_ws/src/prometheus_slam/output/prometheus_slam_Client_log/error.log"); 

  // Register with ROS master
  ros::init(argc, argv, "prometheus_slam_client");

  // Create node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create the mapper
  prometheus_slam::prometheus_slam_Client prometheus_slam_client(nh, nh_private);

  // Spin
  ros::spin();

  // Exit normally
  return 0;
}
