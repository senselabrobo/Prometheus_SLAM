#ifndef PROMETHEUS_SLAM_CLIENT_PROMETHEUS_SLAM_CLIENT_H_
#define PROMETHEUS_SLAM_CLIENT_PROMETHEUS_SLAM_CLIENT_H_

#include <prometheus_slam_msgs/ClientSubmap.h>
#include <prometheus_slam_msgs/ClientSubmapSrv.h>
#include <prometheus_slam_msgs/PoseHistorySrv.h>
#include <prometheus_slam_msgs/SubmapsSrv.h>
#include <prometheus_slam_msgs/TimeLine.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <voxgraph/frontend/voxgraph_mapper.h>
#include <voxgraph_msgs/LoopClosure.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "prometheus_slam/client/map_server.h"
#include "prometheus_slam/common.h"
#include "prometheus_slam/utils/msg_converter.h"

namespace prometheus_slam {
class prometheus_slam_Client : public voxgraph::VoxgraphMapper {
 public:
  prometheus_slam_Client(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : VoxgraphMapper(nh, nh_private)
        {
    int client_id;
    nh_private.param<int>("client_id", client_id, -1);
    client_id_ = static_cast<CliId>(client_id);

    subscribeToClientTopics();  
    advertiseClientTopics();
    advertiseClientServices();  
    if (client_id_ < 0) {
      LOG(FATAL) << "Invalid Client Id " << client_id_;
    } else {
      LOG(INFO) << "Started prometheus_slam Client " << client_id_;
    }
    log_prefix_ = "Client " + std::to_string(client_id_) + ": ";

    map_server_.reset(new MapServer(nh_, nh_private_, client_id_,
                                    submap_config_, frame_names_,
                                    submap_collection_ptr_));
  }

  ~prometheus_slam_Client() = default;

  inline const CliId& getClientId() const { return client_id_; }

  void subscribeToClientTopics();
  void advertiseClientTopics();
  void advertiseClientServices();

  bool getClientSubmapCallback(
      prometheus_slam_msgs::ClientSubmapSrv::Request& request,     // NOLINT
      prometheus_slam_msgs::ClientSubmapSrv::Response& response);  // NOLINT

  bool getAllClientSubmapsCallback(
      prometheus_slam_msgs::SubmapsSrv::Request& request,     // NOLINT
      prometheus_slam_msgs::SubmapsSrv::Response& response);  // NOLINT

  bool getPoseHistory(
      prometheus_slam_msgs::PoseHistorySrv::Request& request,      // NOLINT
      prometheus_slam_msgs::PoseHistorySrv::Response& response) {  // NOLINT
    response.pose_history.pose_history =
        submap_collection_ptr_->getPoseHistory();
    boost::filesystem::path p(request.file_path);
    p.append("prometheus_slam_client_traj_" + std::to_string(client_id_) + ".txt");
    savePoseHistory(p.string());
    return true;
  }

  bool submapCallback(const voxblox_msgs::LayerWithTrajectory& submap_msg,
                      bool transform_layer) override;

 private:
  using VoxgraphMapper = voxgraph::VoxgraphMapper;
  using MapServer = client::MapServer;
  typedef std::map<CliSmId, Transformation> SmIdTfMap;

  void publishTimeLine();
  void publishMapPoseUpdates();
  void publishSubmapPoseTFs() override;

  void savePoseHistory(std::string file_path);

  CliId client_id_;
  std::string log_prefix_;

  ros::Publisher time_line_pub_;
  ros::Publisher map_pose_pub_;
  ros::Publisher submap_mesh_pub_;
  ros::ServiceServer get_client_submap_srv_;
  ros::ServiceServer get_all_client_submaps_srv_;
  ros::ServiceServer get_pose_history_srv_;

  SmIdTfMap ser_sm_id_pose_map_;  

  std::timed_mutex submap_proc_mutex_;

  MapServer::Ptr map_server_;

  typedef message_filters::sync_policies::ApproximateTime<
      voxblox_msgs::LayerWithTrajectory, sensor_msgs::PointCloud2>
      sync_pol;
  message_filters::Synchronizer<sync_pol>* synchronizer_;
  message_filters::Subscriber<voxblox_msgs::LayerWithTrajectory>*
      submap_sync_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2>*
      mesh_pointcloud_sync_sub_;

  Eigen::Vector3d traj_color_;
};

}  // namespace prometheus_slam

#endif  // PROMETHEUS_SLAM_CLIENT_PROMETHEUS_SLAM_CLIENT_H_
