#include <glog/logging.h>
#include <ros/ros.h>
#include "prometheus_slam/server/prometheus_slam_server.h"

int main(int argc, char** argv) {
  // Start logging
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  google::SetLogDestination(google::INFO, 
  "/home/prometheus_slam_ws/src/prometheus_slam/output/prometheus_slam_Server_log/info.log");  
  google::SetLogDestination(google::WARNING, 
  "/home/prometheus_slam_ws/src/prometheus_slam/output/prometheus_slam_Server_log/warning.log");  
  google::SetLogDestination(google::ERROR, 
  "/home/prometheus_slam_ws/src/prometheus_slam/output/prometheus_slam_Server_log/error.log"); 

  // Register with ROS master
  ros::init(argc, argv, "prometheus_slam_server");

  // Create node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create the mapper
  prometheus_slam::prometheus_slam_Server prometheus_slam_server(nh, nh_private);

  // Spin
  ros::spin();

  // Exit normally
  return 0;
}
