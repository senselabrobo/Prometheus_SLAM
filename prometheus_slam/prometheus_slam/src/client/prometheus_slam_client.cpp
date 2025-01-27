#include "prometheus_slam/client/prometheus_slam_client.h"

#include <prometheus_slam_msgs/MapPoseUpdates.h>
#include <prometheus_slam_msgs/TimeLine.h>
#include <voxblox_msgs/MultiMesh.h>
#include <voxgraph/tools/tf_helper.h>

#include <chrono>
#include <string>

#include "prometheus_slam/common.h"

namespace prometheus_slam {

void prometheus_slam_Client::subscribeToClientTopics() {
}

void prometheus_slam_Client::advertiseClientTopics() {
  time_line_pub_ = nh_private_.advertise<prometheus_slam_msgs::TimeLine>(
      "time_line", publisher_queue_length_, true);
  map_pose_pub_ = nh_private_.advertise<prometheus_slam_msgs::MapPoseUpdates>(
      "map_pose_updates", publisher_queue_length_, true);
}

void prometheus_slam_Client::advertiseClientServices() {
  get_client_submap_srv_ = nh_private_.advertiseService(
      "get_client_submap", &prometheus_slam_Client::getClientSubmapCallback, this);
  get_all_client_submaps_srv_ = nh_private_.advertiseService(
      "get_all_submaps", &prometheus_slam_Client::getAllClientSubmapsCallback, this);
  get_pose_history_srv_ = nh_private_.advertiseService(
      "get_pose_history", &prometheus_slam_Client::getPoseHistory, this);
}

bool prometheus_slam_Client::getClientSubmapCallback(
    prometheus_slam_msgs::ClientSubmapSrv::Request& request,
    prometheus_slam_msgs::ClientSubmapSrv::Response& response) {
  CliSmId submap_id;
  if (
    submap_collection_ptr_->lookupActiveSubmapByTime(request.timestamp,&submap_id) 
     )    
    {
    const CliSm& submap = submap_collection_ptr_->getSubmap(submap_id);  
    Transformation T_submap_t;
    if (submap.lookupPoseByTime(request.timestamp, &T_submap_t)) {
      response.submap.map_header.id = submap_id;
      tf::transformKindrToMsg(T_submap_t.cast<double>(), &response.transform); 
      if (!ser_sm_id_pose_map_.count(submap_id)) {
        response.submap =
            utils::msgFromCliSubmap(submap, frame_names_.output_odom_frame);
        ser_sm_id_pose_map_.emplace(submap_id, submap.getPose());
        LOG(INFO) << log_prefix_ << " Submap " << submap_id
                  << " is successfully sent to server";
      }
      response.pub_time = ros::Time::now();
      return true;
    } else {
      LOG(WARNING) << "Client " << client_id_ << ": Requested time "
                   << request.timestamp << " has no corresponding robot pose!";
      return false;
    }
  } else {
    LOG(WARNING) << "Client " << client_id_
                 << ": No active submap containing requested time "
                 << request.timestamp << "!";
    return false;
  }
  return false;
}

bool prometheus_slam_Client::getAllClientSubmapsCallback(
    prometheus_slam_msgs::SubmapsSrv::Request& request,      // NOLINT
    prometheus_slam_msgs::SubmapsSrv::Response& response) {  // NOLINT
  LOG(INFO) << log_prefix_
            << "Server is requesting all submaps! pausing submap process";
  uint8_t trials_ = 0;
  while (!submap_proc_mutex_.try_lock_for(std::chrono::milliseconds(500))) {
    LOG(INFO) << log_prefix_
              << "current submap is still being processed, waiting";
    trials_++;
    CHECK_LT(trials_, 3) << " Tried 3 times, submap process is still running";
  }
  LOG(INFO) << log_prefix_ << "Submap process is paused, sending all submaps";

  for (auto const& submap_ptr : submap_collection_ptr_->getSubmapPtrs()) {
    ROS_WARN_STREAM("submap_ptr->getID() = " << submap_ptr->getID());
    if (ser_sm_id_pose_map_.count(submap_ptr->getID())) continue;
    response.submaps.emplace_back(
        utils::msgFromCliSubmap(*submap_ptr, frame_names_.output_odom_frame));
  }

  submap_proc_mutex_.unlock();

  return true;
}

bool prometheus_slam_Client::submapCallback(
    const voxblox_msgs::LayerWithTrajectory& submap_msg, bool transform_layer) {
  std::lock_guard<std::timed_mutex> submap_proc_lock(submap_proc_mutex_);
  if (!VoxgraphMapper::submapCallback(submap_msg, transform_layer))
    return false;
  if (submap_collection_ptr_->size()) {
    publishTimeLine();   
    
    publishMapPoseUpdates();
  }
  return true;
}

void prometheus_slam_Client::publishTimeLine() {
  prometheus_slam_msgs::TimeLine time_line_msg;
  time_line_msg.start =
      submap_collection_ptr_
          ->getSubmapConstPtr(submap_collection_ptr_->getFirstSubmapId())
          ->getStartTime(); 
  time_line_msg.end =
      submap_collection_ptr_
          ->getSubmapConstPtr(submap_collection_ptr_->getLastSubmapId())
          ->getEndTime();  
  ROS_ERROR("publishTimeLine");
  LOG(INFO) << log_prefix_ << "Updating client time Line from "
            << time_line_msg.start << " to " << time_line_msg.end;
  time_line_pub_.publish(time_line_msg);
}

void prometheus_slam_Client::publishMapPoseUpdates() {
  TransformationVector submap_poses;
  submap_collection_ptr_->getSubmapPoses(&submap_poses);

  prometheus_slam_msgs::MapPoseUpdates map_pose_updates_msg;
  for (auto& sm_id_pose_kv : ser_sm_id_pose_map_) {
    if (!(submap_poses[sm_id_pose_kv.first] == sm_id_pose_kv.second)) {
      LOG(ERROR) << log_prefix_ << "Updating pose of submap "
                << sm_id_pose_kv.first << " to server";
      sm_id_pose_kv.second = submap_poses[sm_id_pose_kv.first];
      map_pose_updates_msg.submap_id.emplace_back(sm_id_pose_kv.first);
      geometry_msgs::Pose new_pose;
      tf::poseKindrToMsg(sm_id_pose_kv.second.cast<double>(), &new_pose);
      map_pose_updates_msg.new_pose.emplace_back(new_pose);
    }
  }
  if (map_pose_updates_msg.submap_id.size())
    map_pose_pub_.publish(map_pose_updates_msg);
}

void prometheus_slam_Client::publishSubmapPoseTFs() {
  // Publish the submap poses as TFs
  const ros::Time current_timestamp = ros::Time::now();
  // NOTE: The rest of the voxgraph is purely driven by the sensor message
  //       timestamps, but the submap pose TFs are published at the current ROS
  //       time (e.g. computer time or /clock topic if use_time_time:=true).
  //       This is necessary because TF lookups only interpolate (i.e. no
  //       extrapolation), and TFs subscribers typically have a limited buffer
  //       time length (e.g. Rviz). The TFs therefore have to be published at a
  //       frequency that exceeds the rate at which new submaps come in (e.g.
  //       once every 10s). In case you intend to use the TFs for more than
  //       visualization, it would be important that the message timestamps and
  //       ros::Time::now() are well synchronized.

  //! override to add client id suffix to tf frames
  for (auto const& submap_ptr : submap_collection_ptr_->getSubmapConstPtrs()) {
    TfHelper::publishTransform(submap_ptr->getPose(),
                               frame_names_.output_odom_frame,
                               "submap_" + std::to_string(submap_ptr->getID()) +
                                   "_" + std::to_string(client_id_),
                               false, current_timestamp);
  }
  if (!submap_collection_ptr_->empty()) {
    auto const first_submap_id = submap_collection_ptr_->getFirstSubmapId();
    auto const& first_submap =
        submap_collection_ptr_->getSubmap(first_submap_id);
    if (!first_submap.getPoseHistory().empty()) {
      const Transformation T_odom__initial_pose =
          first_submap.getPose() *
          first_submap.getPoseHistory().begin()->second;
      TfHelper::publishTransform(T_odom__initial_pose,
                                 frame_names_.output_odom_frame,
                                 "initial_pose_" + std::to_string(client_id_),
                                 false, current_timestamp);
    }
  }
}

void prometheus_slam_Client::savePoseHistory(std::string file_path) {
  std::ofstream f;
  f.open(file_path);
  if (!f.is_open()) LOG(ERROR) << "Failed to open file " << file_path;
  f << std::fixed;

  for (auto const& pose : submap_collection_ptr_->getPoseHistory()) {
    f << std::setprecision(6) << pose.header.stamp.toSec()
      << std::setprecision(7) << " " << pose.pose.position.x << " "
      << pose.pose.position.y << " " << pose.pose.position.z << " "
      << pose.pose.orientation.x << " " << pose.pose.orientation.y << " "
      << pose.pose.orientation.z << " " << pose.pose.orientation.w << std::endl;
  }

  f.close();
  std::string ok_str = "Pose history saved to " + file_path;
  LOG(INFO) << log_prefix_ << ok_str;
}

}  // namespace prometheus_slam
