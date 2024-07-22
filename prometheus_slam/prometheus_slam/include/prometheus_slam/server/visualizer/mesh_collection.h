#ifndef PROMETHEUS_SLAM_SERVER_VISUALIZER_MESH_COLLECTION_H_
#define PROMETHEUS_SLAM_SERVER_VISUALIZER_MESH_COLLECTION_H_

#include <prometheus_slam/common.h>
#include <prometheus_slam_msgs/MeshWithTrajectory.h>

#include <map>
#include <memory>
#include <mutex>
#include <utility>

namespace prometheus_slam {
namespace server {

class MeshCollection {
 public:
  typedef std::shared_ptr<MeshCollection> Ptr;

  using CSIdMeshMap = std::map<CIdCSIdPair, prometheus_slam_msgs::MeshWithTrajectory>;
  using CSIdMeshMapPtr = std::shared_ptr<CSIdMeshMap>;

  MeshCollection() : csid_mesh_map_ptr_(new CSIdMeshMap()) {}
  ~MeshCollection() = default;

  void addSubmapMesh(CliId cid, CliSmId csid,
                     prometheus_slam_msgs::MeshWithTrajectory mesh_with_traj) {
    if (!csid_mesh_map_ptr_->count(std::make_pair(cid, csid)))
      csid_mesh_map_ptr_->emplace(std::make_pair(cid, csid), mesh_with_traj);
    else
      (*csid_mesh_map_ptr_)[std::make_pair(cid, csid)] = mesh_with_traj;
  }

  CSIdMeshMapPtr getSubmapMeshesPtr() { return csid_mesh_map_ptr_; }

 private:
  CSIdMeshMapPtr csid_mesh_map_ptr_;
};

}  // namespace server
}  // namespace prometheus_slam

#endif  // PROMETHEUS_SLAM_SERVER_VISUALIZER_MESH_COLLECTION_H_
