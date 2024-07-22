#include "prometheus_slam/server/global_tf_controller.h"

#include <mutex>
#include <string>
#include <vector>

namespace prometheus_slam {
namespace server {

GlobalTfController::Config GlobalTfController::getConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  Config config;
  nh_private.param<int>("global_tf_controller/init_cli_map_dist",
                        config.init_cli_map_dist, config.init_cli_map_dist);
  return config;
}

void GlobalTfController::initCliMapPose() {
  int j = 0 ;
  for (int i = 0; i < client_number_; i++) {
    cli_mission_frames_.emplace_back(map_frame_prefix_ + "_" +
                                     std::to_string(i)); 

    client_tf_optimizer_.addClient(i, Transformation()); 
    tf::Transform odom_initial_pose;
    
    odom_initial_pose.setOrigin(tf::Vector3(j, j, j));
    //odom_initial_pose.setRotation(tf::Quaternion(0, 0, 0, 1));
    odom_initial_pose.setRotation(tf::Quaternion(0, 0, 0, 1));
    T_G_CLI_opt_.emplace_back(tf::StampedTransform(odom_initial_pose, ros::Time::now(),
                                                   global_mission_frame_,
                                                   cli_mission_frames_[i]));
    j = j + 10 ;
  }

  cli_tf_fused_.resize(4, false);
  cli_tf_fused_[0] = true;
  cli_tf_fused_[1] = true;

  tf_pub_timer_ =
      nh_private_.createTimer(ros::Duration(1 / kTfPubFreq),
                              &GlobalTfController::pubCliTfCallback, this);
}

void GlobalTfController::pubCliTfCallback(const ros::TimerEvent& event) {
  if (!inControl()) 
  { ROS_ERROR("pubCliTfCallback NOT IN CONTROL");
    return;
  }
  updateCliMapPose();
  for (int i = 0; i < T_G_CLI_opt_.size(); i++) {
    if (cli_tf_fused_[i]) tf_boardcaster_.sendTransform(T_G_CLI_opt_[i]);
  }
}

void GlobalTfController::addCliMapRelativePose(const CliId& first_cid,
                                               const CliId& second_cid,
                                               const Transformation& T_C1_C2) {
  client_tf_optimizer_.addClientRelativePoseMeasurement(first_cid, second_cid,
                                                        T_C1_C2);   
  pose_updated_ = true;
}

void GlobalTfController::updateCliMapPose() {
  if (pose_updated_) {
    std::lock_guard<std::mutex> pose_update_lock(pose_update_mutex);
    computeOptCliMapPose();
    PoseMap new_cli_map_poses = client_tf_optimizer_.getClientMapTfs();
    CHECK(new_cli_map_poses[0] == Transformation()); 
                                                     
    for (auto const& cli_map_pose_kv : new_cli_map_poses) {
      cli_tf_fused_[cli_map_pose_kv.first] = true;  
      tf::Transform pose;
      tf::transformKindrToTF(cli_map_pose_kv.second.cast<double>(), &pose);
      T_G_CLI_opt_[cli_map_pose_kv.first] = tf::StampedTransform(
          pose, ros::Time::now(), T_G_CLI_opt_[cli_map_pose_kv.first].frame_id_,
          T_G_CLI_opt_[cli_map_pose_kv.first].child_frame_id_);
      LOG_IF(INFO, verbose_)
          << "Updated pose for Client " << int(cli_map_pose_kv.first) << " with pose" << std::endl
          << cli_map_pose_kv.second;     
    }
    pose_updated_ = false;
  } else {
    for (auto& T_G_CLI : T_G_CLI_opt_) {
      T_G_CLI.stamp_ = ros::Time::now();
    }
  }
}

void GlobalTfController::computeOptCliMapPose() {

  client_tf_optimizer_.optimize();

}

void GlobalTfController::publishTfGloCli() {}

}  // namespace server
}  // namespace prometheus_slam
