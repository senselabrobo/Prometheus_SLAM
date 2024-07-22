#include "prometheus_slam/server/submap_collection.h"

#include <voxblox/integrator/merge_integration.h>

#include <vector>

namespace prometheus_slam {
namespace server {

Transformation SubmapCollection::addSubmap(const CliSm::Ptr& submap_ptr,
                                           const CliId& cid,
                                           const CliSmId& cli_sm_id) {
  CHECK(submap_ptr != nullptr);
  voxgraph::VoxgraphSubmapCollection::addSubmap(submap_ptr);
  sm_cli_id_map_.emplace(submap_ptr->getID(), CIdCSIdPair(cid, cli_sm_id));

  cli_ser_sm_id_map_[cid].emplace_back(submap_ptr->getID());
  sm_id_ori_pose_map_.emplace(submap_ptr->getID(), submap_ptr->getPose());
  return Transformation();
}

Transformation SubmapCollection::mergeToCliMap(const CliSm::Ptr& submap_ptr) {
  CHECK(exists(submap_ptr->getID()));

  auto const& cli_map_ptr = getSubmapPtr(submap_ptr->getID());
  voxblox::mergeLayerAintoLayerB(
      submap_ptr->getTsdfMapPtr()->getTsdfLayer(),
      cli_map_ptr->getTsdfMapPtr()->getTsdfLayerPtr());
  cli_map_ptr->finishSubmap();
  return submap_ptr->getPose() * cli_map_ptr->getPose().inverse();
}

}  // namespace server
}  // namespace prometheus_slam
