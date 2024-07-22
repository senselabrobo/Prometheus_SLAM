#include "prometheus_slam/server/visualizer/server_visualizer.h"

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "prometheus_slam/common.h"
#include "prometheus_slam/server/submap_collection.h"
#include "prometheus_slam/utils/msg_converter.h"

namespace prometheus_slam {
namespace server {
void ServerVisualizer::getFinalGlobalMesh(
    const SubmapCollection::Ptr& submap_collection_ptr,
    const PoseGraphInterface& pose_graph_interface,
    const std::vector<CliSmPack>& other_submaps,
    const std::string& mission_frame, const ros::Publisher& publisher,
    const std::string& file_path, bool save_to_file) {

  SubmapCollection::Ptr global_submap_collection_ptr(
      new SubmapCollection(*submap_collection_ptr));  

  PoseGraphInterface global_pg_interface(pose_graph_interface,
                                         global_submap_collection_ptr); 


  for (auto const& submap_pack : other_submaps) {
    global_submap_collection_ptr->addSubmap(
        submap_pack.submap_ptr, submap_pack.cid, submap_pack.cli_sm_id);
    
    global_pg_interface.addSubmap(submap_pack.submap_ptr->getID());
  }

  if (global_submap_collection_ptr->getSubmapConstPtrs().empty()) return;

  global_pg_interface.updateSubmapRPConstraints();  

  auto opt_async = std::async(std::launch::async, &PoseGraphInterface::optimize,
                 &global_pg_interface, config_.registration_enable);

  while (opt_async.wait_for(std::chrono::milliseconds(100)) !=
         std::future_status::ready) {
    LOG_EVERY_N(INFO, 10) << "Global optimzation is still running...";
  }
  global_pg_interface.printResiduals(  
      PoseGraphInterface::ConstraintType::RelPose);
  std::cout << "" << std::endl;
  global_pg_interface.printResiduals(
      PoseGraphInterface::ConstraintType::SubmapRelPose); 
  ROS_ERROR("Optimization finished, generating global mesh...");

  auto pose_map3 = global_pg_interface.getPoseMap();  
  for (auto const& kv3 : pose_map3) {
    LOG(INFO) << "submap_id_" << int(kv3.first) << std::endl << "pose = "<< std::endl << kv3.second;
  }

  global_pg_interface.updateSubmapCollectionPoses();  

  auto pose_map = global_pg_interface.getPoseMap();   

  boost::filesystem::path mesh_p_voxblox(file_path);
  mesh_p_voxblox.append("global_mesh_voxblox.ply");

  if (config_.publish_combined_mesh)
    submap_vis_.saveAndPubCombinedMesh(
        *global_submap_collection_ptr, mission_frame, publisher,
        save_to_file ? mesh_p_voxblox.string() : "");   

  LOG(INFO) << "Global mesh generated, published and saved to " << file_path;

  if (save_to_file)
    global_submap_collection_ptr->savePoseHistoryToFile(file_path);

  LOG(INFO) << "Trajectory saved to " << file_path;
}

}  // namespace server
}  // namespace prometheus_slam
