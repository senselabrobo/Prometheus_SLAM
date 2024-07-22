#ifndef PROMETHEUS_SLAM_SERVER_DISTRIBUTION_DISTRIBUTION_CONTROLLER_H_
#define PROMETHEUS_SLAM_SERVER_DISTRIBUTION_DISTRIBUTION_CONTROLLER_H_

#include <prometheus_slam_msgs/ControlTrigger.h>
#include <prometheus_slam_msgs/StateQuery.h>

#include <memory>
#include <string>

#include "prometheus_slam/common.h"
#include "prometheus_slam/server/submap_collection.h"
#include "prometheus_slam/utils/msg_converter.h"

namespace prometheus_slam {
namespace server {

class DistributionController {
 public:
  struct Config {
    Config() {}
    friend inline std::ostream& operator<<(std::ostream& s, const Config& v) {
      s << std::endl
        << "Distribution Controller using Config:" << std::endl
        << std::endl
        << "-------------------------------------------" << std::endl;
      return (s);
    }
  };

  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private);

  typedef std::shared_ptr<DistributionController> Ptr;

  DistributionController(const ros::NodeHandle& nh,
                         const ros::NodeHandle& nh_private,
                         const SubmapCollection::Ptr& submap_collection_ptr)
      : nh_(nh),
        nh_private_(nh_private),
        submap_collection_ptr_(submap_collection_ptr) {
    nh_private_.param<bool>("in_control", in_control_, true);
    LOG(INFO) << "Server in control: "
              << static_cast<std::string>(in_control_ ? "true" : "false");

    advertiseServices();
  }

  ~DistributionController() = default;

  void advertiseServices() {
    control_trigger_srv_ = nh_private_.advertiseService(
        "control_trigger", &DistributionController::ControlTriggerCallback,
        this);
    state_query_srv_ = nh_private_.advertiseService(
        "state_query", &DistributionController::StateQueryCallback, this);
  }

  inline bool inControl() const { return in_control_; }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Control trigger service
  ros::ServiceServer control_trigger_srv_;
  bool in_control_;
  bool ControlTriggerCallback(
      prometheus_slam_msgs::ControlTrigger::Request& request,      // NOLINT
      prometheus_slam_msgs::ControlTrigger::Response& response) {  // NOLINT
    ROS_ERROR("ControlTriggerCallback");
    LOG(INFO) << "Triggering control state to: "
              << static_cast<std::string>(request.in_control ? "true"
                                                             : "false");
    in_control_ = request.in_control;
    return true;
  }

  ros::ServiceServer state_query_srv_;
  SubmapCollection::Ptr submap_collection_ptr_;
  bool StateQueryCallback(
      prometheus_slam_msgs::StateQuery::Request& request,      // NOLINT
      prometheus_slam_msgs::StateQuery::Response& response) {  // NOLINT
    response.n_submaps = submap_collection_ptr_->getSubmapPtrs().size();
    ROS_ERROR("StateQueryCallback");
    for (auto const& submap_ptr : submap_collection_ptr_->getSubmapPtrs()) {
      response.bb.emplace_back(
          utils::msgFromBb(submap_ptr->getOdomFrameSurfaceAabb()));
    }
    return true;
  }
};

}  // namespace server
}  // namespace prometheus_slam

#endif  // PROMETHEUS_SLAM_SERVER_DISTRIBUTION_DISTRIBUTION_CONTROLLER_H_
