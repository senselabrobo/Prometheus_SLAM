#ifndef PROMETHEUS_SLAM_SERVER_CLIENT_TF_OPTIMIZER_H_
#define PROMETHEUS_SLAM_SERVER_CLIENT_TF_OPTIMIZER_H_

#include <ros/ros.h>

#include <map>

#include "prometheus_slam/common.h"
#include "prometheus_slam/server/backend/client_frame_node.h"
#include "prometheus_slam/server/backend/pose_graph.h"
#include "prometheus_slam/utils/ros_params.h"

namespace prometheus_slam {
namespace server {

class ClientTfOptimizer {
 public:
  using PoseMap = PoseGraph::PoseMap;

  ClientTfOptimizer(const ros::NodeHandle& nh_private, bool verbose)
      : verbose_(verbose) {
    utils::setInformationMatrixFromRosParams(
        ros::NodeHandle(nh_private,
                        "client_map_relative_pose/information_matrix"),
        &cli_rp_info_matrix_);
  }

  void addClient(const CliId& cid, const Transformation& pose);

  void addClientRelativePoseMeasurement(const CliId& first_cid,
                                        const CliId& second_cid,
                                        const Transformation& T_C1_C2);

  void resetClientRelativePoseConstraints() {
    pose_graph_.resetClientRelativePoseConstraint();
  }

  void optimize() { pose_graph_.optimize();}

  PoseMap getClientMapTfs() { return pose_graph_.getClientMapTf(); }

 private:
  bool verbose_;

  PoseGraph pose_graph_;

  InformationMatrix cli_rp_info_matrix_;
};

}  // namespace server
}  // namespace prometheus_slam
#endif  // PROMETHEUS_SLAM_SERVER_CLIENT_TF_OPTIMIZER_H_
