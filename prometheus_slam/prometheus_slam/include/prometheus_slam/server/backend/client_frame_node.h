#ifndef PROMETHEUS_SLAM_SERVER_BACKEND_CLIENT_FRAME_NODE_H_
#define PROMETHEUS_SLAM_SERVER_BACKEND_CLIENT_FRAME_NODE_H_

#include <voxgraph/backend/node/node.h>

#include <memory>

#include "prometheus_slam/common.h"

namespace prometheus_slam {
namespace server {

class ClientFrameNode : public voxgraph::Node {
 public:
  typedef std::shared_ptr<ClientFrameNode> Ptr;

  struct Config : Node::Config {
    CliId client_id;
  };

  ClientFrameNode(const NodeId& node_id, const Config& config)
      : Node(node_id, config), config_(config) {}
  ~ClientFrameNode() = default;

  CliId getCliId() const { return config_.client_id; }

 private:
  using Node = voxgraph::Node;

  Config config_;
};

}  // namespace server
}  // namespace prometheus_slam
#endif  // PROMETHEUS_SLAM_SERVER_BACKEND_CLIENT_FRAME_NODE_H_
