#include "prometheus_slam/server/prometheus_slam_server.h"

#include <prometheus_slam_msgs/SubmapsSrv.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap_collection.h>
#include <boost/filesystem.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "prometheus_slam/common.h"
#include "prometheus_slam/utils/msg_converter.h"

namespace prometheus_slam {

prometheus_slam_Server::Config prometheus_slam_Server::getConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  prometheus_slam_Server::Config config;

  nh_private.param<int>("client_number", config.client_number,
                        config.client_number);
  LOG_IF(FATAL,
         !(config.client_number > 0 && config.client_number <= kMaxClientNum))
      << "Invalid client number, must > 0, and only max 2 clients supported "
         "now. Given: "
      << config.client_number;

  nh_private.param<int>("map_fusion_queue_size", config.map_fusion_queue_size,
                        config.map_fusion_queue_size);
  float refuse_interval;
  nh_private.param<float>("refuse_interval", refuse_interval, refuse_interval);
  config.refuse_interval.fromSec(refuse_interval);
  nh_private.param<int>("fixed_map_client_id", config.fixed_map_client_id,
                        config.fixed_map_client_id);
  nh_private.param<std::string>("map_frame_prefix", config.map_frame_prefix,
                                config.map_frame_prefix);
  nh_private.param<std::string>("output_mission_frame", config.output_map_frame,
                                config.output_map_frame);
  nh_private.param<bool>("submap_registration/enabled",
                         config.enable_registration_constraints,
                         config.enable_registration_constraints);
  nh_private.param<bool>("loop_closure/enabled",
                         config.enable_map_fusion_constraints,
                         config.enable_map_fusion_constraints);
  nh_private.param<bool>("enable_client_loop_closure",
                         config.enable_client_loop_closure,
                         config.enable_client_loop_closure);
  nh_private.param<int>("publisher_queue_length", config.publisher_queue_length,
                        config.publisher_queue_length);
  nh_private.param<bool>("use_tf_submap_pose", config.use_tf_submap_pose,
                         config.use_tf_submap_pose);
  nh_private.param("publish_global_mesh_on_update",
                   config.publish_global_mesh_on_update,
                   config.publish_global_mesh_on_update);
  nh_private.param("global_mesh_need_update_threshold",
                  config.global_mesh_need_update_threshold,
                  config.global_mesh_need_update_threshold);
  return config;
}

void prometheus_slam_Server::initClientHandlers(const ros::NodeHandle& nh,
                                        const ros::NodeHandle& nh_private) {
  CHECK_LT(config_.fixed_map_client_id, kMaxClientNum);
  for (int i = 0; i < config_.client_number; i++) {
    client_handlers_.emplace_back(new ClientHandler(
        nh, nh_private, i, config_.map_frame_prefix, submap_config_,
        submap_collection_ptr_, server_vis_->getMeshCollectionPtr(),
        std::bind(&prometheus_slam_Server::timeLineUpdateCallback, this)));

    force_fuse_.emplace_back(true);
    fused_time_line_.emplace_back(TimeLine());
  }
  force_fuse_[config_.fixed_map_client_id] = false;
}

void prometheus_slam_Server::subscribeTopics() {
  map_fusion_sub_ =
      nh_.subscribe("map_fusion_in", config_.map_fusion_queue_size,
                    &prometheus_slam_Server::mapFusionMsgCallback, this);
}

void prometheus_slam_Server::advertiseTopics() {}

void prometheus_slam_Server::advertiseServices() {
  get_final_global_mesh_srv_ = nh_private_.advertiseService(
      "get_final_global_mesh", &prometheus_slam_Server::getFinalMeshCallback,
      this);
  get_pose_history_srv_ = nh_private_.advertiseService(
      "get_pose_history", &prometheus_slam_Server::getPoseHistoryCallback, this);  
  need_to_fuse_srv_ = nh_private_.advertiseService(
      "need_to_fuse", &prometheus_slam_Server::needToFuseCallback, this);
}

bool prometheus_slam_Server::getFinalMeshCallback(
    prometheus_slam_msgs::FilePath::Request& request,
    prometheus_slam_msgs::FilePath::Response& response) {
  LOG(INFO) << "Service called to get final global mesh, pausing map fusion "
               "process";

  processMFFuture();  

  std::string file_path = request.file_path;
  LOG_IF(INFO, file_path.empty())
      << "Mesh file path is not given, mesh will not be saved as file";

  uint8_t trials_ = 0;
  while (!final_mesh_gen_mutex_.try_lock_for(std::chrono::milliseconds(500))) {
    LOG(INFO) << "current map fusion is still being processing, waiting";
    trials_++;
    CHECK_LT(trials_, 3)
        << " Tried 3 times, map fusion process is still running";
  }
  LOG(INFO) << "Map fusion process is paused, generating final mesh";

  // requesting submaps one by one to avoid bandwidth peak,
  std::vector<CliSmPack> all_submaps;  
  SerSmId start_ser_sm_id = submap_collection_ptr_->getNextSubmapID(); 
  ROS_WARN_STREAM( "start_ser_sm_id = " << start_ser_sm_id ); 


  for (auto const& ch : client_handlers_) {
    if (!tf_controller_->ifClientFused(ch->getCliId())) continue;
    std::vector<CliSmPack> submaps_in_client;
    CHECK(ch->requestAllSubmaps(&submaps_in_client, &start_ser_sm_id));  
    all_submaps.insert(all_submaps.end(), submaps_in_client.begin(),
                       submaps_in_client.end());
  }


  server_vis_->getFinalGlobalMesh(
      submap_collection_ptr_, 
      pose_graph_interface_, 
      all_submaps,
      tf_controller_->getGlobalMissionFrame(), 
      file_path, 
      true);

  LOG(INFO) << "Global mesh generated, map fusion process unpaused";

  final_mesh_gen_mutex_.unlock();
  std::string ok_str = "Global mesh saved to " + request.file_path;
  LOG(INFO) << ok_str;
  response.message = ok_str;
  return true;  
}

bool prometheus_slam_Server::getPoseHistoryCallback(
    prometheus_slam_msgs::FilePath::Request& request,
    prometheus_slam_msgs::FilePath::Response& response) {
  LOG(INFO) << "Generating pose history for all clients";

  std::vector<PoseStampedVector> pose_histories(config_.client_number);
  for (auto const& ch : client_handlers_) {
    if (!ch->requestPoseHistory(request.file_path,
                                &pose_histories[ch->getCliId()])) {
      LOG(ERROR) << "Request pose history of Client " << ch->getCliId()
                 << " failed";
      return false;
    }
  }

  boost::filesystem::path p(request.file_path);
  p.append("prometheus_slam_server_traj.txt");
  std::ofstream f;
  f.open(p.string());
  if (!f.is_open()) LOG(ERROR) << "Failed to open file " << request.file_path;
  f << std::fixed;

  for (int cid = 0; cid < pose_histories.size(); cid++) {
    TransformationD T_G_Cli;
    tf::transformTFToKindr(tf_controller_->getTGCliOpt(cid), &T_G_Cli);

    for (auto const& pose : pose_histories[cid]) {
      TransformationD T_Cli_C;
      tf::poseMsgToKindr(pose.pose, &T_Cli_C);
      TransformationD T_G_C = T_G_Cli * T_Cli_C;
      f << std::setprecision(6) << pose.header.stamp.toSec()
        << std::setprecision(7) << " " << T_G_C.getPosition().x() << " "
        << T_G_C.getPosition().y() << " " << T_G_C.getPosition().z() << " "
        << T_G_C.getRotation().x() << " " << T_G_C.getRotation().y() << " "
        << T_G_C.getRotation().z() << " " << T_G_C.getRotation().w()
        << std::endl;
    }
  }

  f.close();
  std::string ok_str = "Pose history saved to " + p.string();
  LOG(INFO) << ok_str;
  response.message = ok_str;
  return true;
}

bool prometheus_slam_Server::needToFuseCallback(
    prometheus_slam_msgs::NeedToFuseSrv::Request& request,
    prometheus_slam_msgs::NeedToFuseSrv::Response& response) {
  ROS_ERROR("CALLING needToFuseCallback");
  response.need_to_fuse = needRefuse(
      request.cid_a, client_handlers_[request.cid_a]->getTimeLine().end,
      request.cid_b, client_handlers_[request.cid_b]->getTimeLine().end);
  return true;
}

void prometheus_slam_Server::mapFusionMsgCallback(
    const prometheus_slam_msgs::MapFusion& map_fusion_msg) {
  if (map_fusion_msg.from_client_id == map_fusion_msg.to_client_id) {
    LOG(INFO) << "Received loop closure msg in client "
              << map_fusion_msg.from_client_id << " from "
              << map_fusion_msg.from_timestamp << " to "
              << map_fusion_msg.to_timestamp;
    loopClosureCallback(map_fusion_msg.from_client_id,
                        utils::fromMapFusionMsg(map_fusion_msg));
  } else {
    LOG(INFO) << "Received map fusion msg from client "
              << map_fusion_msg.from_client_id << " at "
              << map_fusion_msg.from_timestamp << " to client "
              << map_fusion_msg.to_client_id << " at "
              << map_fusion_msg.to_timestamp;
    mapFusionCallback(map_fusion_msg);  
  }
}

void prometheus_slam_Server::loopClosureCallback(
    const CliId& client_id,
    const voxgraph_msgs::LoopClosure& loop_closure_msg) {
  if (config_.enable_client_loop_closure)
    client_handlers_[client_id]->pubLoopClosureMsg(loop_closure_msg);
}

bool prometheus_slam_Server::mapFusionCallback(
    const prometheus_slam_msgs::MapFusion& map_fusion_msg, bool future) {
  std::lock_guard<std::timed_mutex> map_fusion_proc_lock(final_mesh_gen_mutex_);

  CHECK_NE(map_fusion_msg.from_client_id, map_fusion_msg.to_client_id);
  if(!future) 
  LOG(INFO)<<"mapFusionCallback called from mapFusionMsgCallback()";
  else
  LOG(INFO)<<"mapFusionCallback called from processMFFuture()";
  CliSm::Ptr submap_a, submap_b;
  CliSmId cli_sm_id_a, cli_sm_id_b;
  Transformation T_A_t1, T_B_t2;
  const CliId& cid_a = map_fusion_msg.from_client_id;
  const CliId& cid_b = map_fusion_msg.to_client_id;
  const ros::Time& t1 = map_fusion_msg.from_timestamp;
  const ros::Time& t2 = map_fusion_msg.to_timestamp;
  TransformationD T_t1_t2;
  tf::transformMsgToKindr(map_fusion_msg.transform, &T_t1_t2);

  if (!needRefuse(cid_a, t1, cid_b, t2)) return true;
  CHECK((!fused_time_line_[cid_a].hasTime(t1)) ||
        (!fused_time_line_[cid_b].hasTime(t2)));

  ReqState ok_a, ok_b;

  bool has_time_a = client_handlers_[cid_a]->hasTime(t1);
  bool has_time_b = client_handlers_[cid_b]->hasTime(t2);


  if (has_time_a && has_time_b) {
    ok_a = client_handlers_[cid_a]->requestSubmapByTime(
        t1,
        submap_collection_ptr_->getNextSubmapID(),   
        &cli_sm_id_a, 
        &submap_a,  
        &T_A_t1  
        );
    ok_b = client_handlers_[cid_b]->requestSubmapByTime(
        t2, 
        submap_collection_ptr_->getNextSubmapID() + 1, 
        &cli_sm_id_b,
        &submap_b, 
        &T_B_t2
        );

    CHECK_NE(ok_a, ReqState::FUTURE);
    CHECK_NE(ok_b, ReqState::FUTURE);
    verbose_ = true;
    if (verbose_) {
      LOG_IF(INFO, ok_a == ReqState::FAILED && verbose_)
          << "Requesting submap from Client " << map_fusion_msg.from_client_id
          << " failed!";

      LOG_IF(INFO, ok_a == ReqState::FUTURE && verbose_)
          << "Requested timestamp from Client " << map_fusion_msg.from_client_id
          << " is ahead of client timeline, map fusion msg saved for later";
      LOG_IF(INFO, ok_b == ReqState::FAILED && verbose_)
          << "Requesting submap from Client " << map_fusion_msg.to_client_id
          << " failed!";
      LOG_IF(INFO, ok_b == ReqState::FUTURE && verbose_)
          << "Requested timestamp from Client " << map_fusion_msg.to_client_id
          << " is ahead of client timeline, map fusion msg saved for later";
    }
    LOG_IF(INFO, ok_a == ReqState::SUCCESS && verbose_)
        << "Received submap from Client " << map_fusion_msg.from_client_id
        << " with layer memory "
        << submap_a->getTsdfMapPtr()->getTsdfLayerPtr()->getMemorySize();
    LOG_IF(INFO, ok_b == ReqState::SUCCESS && verbose_)
        << "Received submap from Client " << map_fusion_msg.to_client_id
        << " with layer memory "
        << submap_b->getTsdfMapPtr()->getTsdfLayerPtr()->getMemorySize();

    if (ok_a != ReqState::SUCCESS || ok_b != ReqState::SUCCESS) {
      if (ok_a == ReqState::SUCCESS) {
        addSubmap(submap_a, cid_a, cli_sm_id_a);
      }
      if (ok_b == ReqState::SUCCESS) {
        addSubmap(submap_b, cid_b, cli_sm_id_b);
      }
      return false;
    }
  }

  bool fused_any = false; 
  if (future) {
    CHECK_EQ(ok_a, ReqState::SUCCESS);

    CHECK_EQ(ok_b, ReqState::SUCCESS);
    fused_any = fuseMap(cid_a, t1, cli_sm_id_a, submap_a, T_A_t1,  // NOLINT
                        cid_b, t2, cli_sm_id_b, submap_b, T_B_t2,
                        T_t1_t2.cast<voxblox::FloatingPoint>());
  } else {
    if (has_time_a && has_time_b) {
      if ((ok_a == ReqState::SUCCESS && ok_b == ReqState::FUTURE) ||
          (ok_b == ReqState::SUCCESS && ok_a == ReqState::FUTURE)) {
        addToMFFuture(map_fusion_msg);
      }
      if (ok_a == ReqState::SUCCESS && ok_b == ReqState::SUCCESS) {
        fused_any = fuseMap(cid_a, t1, cli_sm_id_a, submap_a, T_A_t1,  // NOLINT
                            cid_b, t2, cli_sm_id_b, submap_b, T_B_t2,
                            T_t1_t2.cast<voxblox::FloatingPoint>());
      }
    } else {
      addToMFFuture(map_fusion_msg); 
    }
  }

  if (fused_any) {
    updateNeedRefuse(cid_a, t1, cid_b, t2);
    return true;
  }
  return false;
}

void prometheus_slam_Server::addToMFFuture(
    const prometheus_slam_msgs::MapFusion& map_fusion_msg) {
  std::lock_guard<std::mutex> future_mf_queue_lock(future_mf_queue_mutex_);
  if (map_fusion_msgs_future_.size() < config_.map_fusion_queue_size)
    map_fusion_msgs_future_.push_back(
        std::pair<prometheus_slam_msgs::MapFusion, int>(map_fusion_msg, 0));
}

void prometheus_slam_Server::processMFFuture() {
  std::lock_guard<std::mutex> future_mf_queue_lock(future_mf_queue_mutex_);
  bool processed_any = false;
  for (auto it = map_fusion_msgs_future_.begin();
       it != map_fusion_msgs_future_.end();) {                                        
    prometheus_slam_msgs::MapFusion map_fusion_msg = it->first;
    CliId cid_a = map_fusion_msg.from_client_id;
    CliId cid_b = map_fusion_msg.to_client_id;
    ros::Time t1 = map_fusion_msg.from_timestamp;
    ros::Time t2 = map_fusion_msg.to_timestamp;
    if (client_handlers_[cid_a]->hasTime(t1) &&
        client_handlers_[cid_b]->hasTime(t2)) {
      if (
        mapFusionCallback(map_fusion_msg, true)  
        ) 
        {
        processed_any = true;
        break;
      }
    }

    if (it->second >= kMaxFutureUncatchedN) {
      it = map_fusion_msgs_future_.erase(it);
    } else {
      it->second++;
      ++it;
    }
  }
  
  if (processed_any) {
    map_fusion_msgs_future_.clear(); 
  }
}


bool prometheus_slam_Server::needRefuse(const CliId& cid_a, const ros::Time& t1,
                                const CliId& cid_b, const ros::Time& t2) {
  CHECK_EQ(force_fuse_[config_.fixed_map_client_id], false);

  if ((cid_a != config_.fixed_map_client_id &&
       (isTimeNeedRefuse(cid_a, t1) || force_fuse_[cid_a])) ||
      (cid_b != config_.fixed_map_client_id &&
       (isTimeNeedRefuse(cid_b, t2) || force_fuse_[cid_b])))
    return true;
  return false;
}

bool prometheus_slam_Server::updateNeedRefuse(const CliId& cid_a, const ros::Time& t1,
                                      const CliId& cid_b, const ros::Time& t2) {
  CHECK_EQ(force_fuse_[config_.fixed_map_client_id], false);
  force_fuse_[cid_a] = false;
  force_fuse_[cid_b] = false;

  fused_time_line_[cid_a].update(t1);
  fused_time_line_[cid_b].update(t2);
  return true;
}

bool prometheus_slam_Server::fuseMap(const CliId& cid_a, const ros::Time& t1,
                             const CliSmId& cli_sm_id_a,
                             const CliSm::Ptr& submap_a,
                             const Transformation& T_A_t1, const CliId& cid_b,
                             const ros::Time& t2, const CliSmId& cli_sm_id_b,
                             const CliSm::Ptr& submap_b,
                             const Transformation& T_B_t2,
                             const Transformation& T_t1_t2) {
  std::lock_guard<std::mutex> map_fuse_lock(map_fuse_mutex_);
  //global_mesh_need_update_ = 6 ;
  LOG(INFO) << "Fusing: " << std::endl
            << "  Client: " << static_cast<int>(cid_a)
            << " -> Submap: " << static_cast<int>(cli_sm_id_a) << std::endl
            << "  Client: " << static_cast<int>(cid_b)
            << " -> Submap: " << static_cast<int>(cli_sm_id_b);
  LOG_IF(INFO, verbose_) << " T_A_t1: " << std::endl << T_A_t1;  
  LOG_IF(INFO, verbose_) << " T_B_t2: " << std::endl << T_B_t2;  
  LOG_IF(INFO, verbose_) << " T_t1_t2: " << std::endl << T_t1_t2; 

  bool prev_result = false;

  if (optimization_async_handle_.valid() &&
      optimization_async_handle_.wait_for(std::chrono::milliseconds(10)) !=
          std::future_status::ready) {
    LOG(INFO)
        << "Previous pose graph optimization not yet complete. Waiting...";
    optimization_async_handle_.wait();
  }

  prev_result = optimization_async_handle_.valid()
                    ? (optimization_async_handle_.get() == OptState::OK)
                    : true;
  LOG(INFO) << "Result of Last Optimization" << prev_result;

  SerSmId ser_sm_id_a, ser_sm_id_b;
  if (submap_a->getPoseHistory().empty()) {
    CHECK(submap_collection_ptr_->getSerSmIdByCliSmId(cid_a, cli_sm_id_a,
                                                      &ser_sm_id_a));
  } else {
    ser_sm_id_a = addSubmap(submap_a, cid_a, cli_sm_id_a);  
  }

  if (submap_b->getPoseHistory().empty()) {
    CHECK(submap_collection_ptr_->getSerSmIdByCliSmId(cid_b, cli_sm_id_b,
                                                      &ser_sm_id_b));
  } else {
    ser_sm_id_b = addSubmap(submap_b, cid_b, cli_sm_id_b);  
  }                                                         

  bool added_loop;
  if (config_.enable_map_fusion_constraints) {
    Transformation T_A_B = T_A_t1 * T_t1_t2 * T_B_t2.inverse();
    added_loop = pose_graph_interface_.addLoopClosureMeasurement(  
        ser_sm_id_a, ser_sm_id_b, T_A_B, false);                   
    if (!added_loop) return false;
  }

  pose_graph_interface_.addForceRegistrationConstraint(ser_sm_id_a,
                                                       ser_sm_id_b);  
                                                                      
  updateSubmapRPConstraints();  

  // Optimize the pose graph in a separate thread
  optimization_async_handle_ =
      std::async(std::launch::async, &prometheus_slam_Server::optimizePoseGraph, this,
                 this->config_.enable_registration_constraints);  
  return true;
}

void prometheus_slam_Server::updateSubmapRPConstraints() {
  if (config_.use_tf_submap_pose) {
    LOG(FATAL)
        << "Don't turn on use_tf_submap_pose, somehow it doesn't work for now";

    for (int cid = 0; cid < submap_collection_ptr_->getClientNumber(); cid++) {
      std::vector<CliSmId> cli_sids;
      if (!submap_collection_ptr_->getCliSmIdsByCliId(cid, &cli_sids)) continue;
      for (auto const& cli_sid : cli_sids) {
        Transformation T_Cli_Sm;
        if (!client_handlers_[cid]->lookUpSubmapPoseFromTf(cli_sid,
                                                           &T_Cli_Sm)) {
          continue;
        }
        SerSmId ser_sid;
        CHECK(submap_collection_ptr_->getSerSmIdByCliSmId(cid, cli_sid,
                                                          &ser_sid));
        submap_collection_ptr_->setSubmapPose(ser_sid, T_Cli_Sm);
      }
    }
  }
  pose_graph_interface_.updateSubmapRPConstraints();
}

prometheus_slam_Server::OptState prometheus_slam_Server::optimizePoseGraph(
    bool enable_registration) {

  // Optimize the pose graph
  ROS_INFO("Optimizing the pose graph");

  
  LOG(INFO) << "before optimizing***************";
  auto pose_map = pose_graph_interface_.getPoseMap();  
  for (auto const& kv : pose_map) {
    LOG(INFO) << "submap_id_" << int(kv.first) << std::endl << "pose = "<< std::endl << kv.second;
  }
  enable_registration = false;
  pose_graph_interface_.optimize(enable_registration);

  pose_graph_interface_.updateSubmapCollectionPoses();  

  pose_map = pose_graph_interface_.getPoseMap();
  LOG(INFO) << "after optimizing----------";
  for (auto const& kv : pose_map) {
   LOG(INFO) << "submap_id_" << int(kv.first) << std::endl << "pose = "<< std::endl << kv.second;
  }

  updateCliMapRelativePose();  

  global_mesh_initialized_ = true;
  return OptState::OK;
}

void prometheus_slam_Server::evaluateResiduals() {
  if (config_.enable_map_fusion_constraints) {
    LOG(INFO) << "Evaluating Residuals of Map Fusion Constraints";
    pose_graph_interface_.printResiduals(
        PoseGraphInterface::ConstraintType::RelPose);
  }
  if (submap_collection_ptr_->size() > 2) {
    LOG(INFO) << "Evaluating Residuals of Submap RelPose Constraints";
    pose_graph_interface_.printResiduals(
        PoseGraphInterface::ConstraintType::SubmapRelPose);
  } else {
    LOG(INFO) << "No Submap RelPose Constraints added yet";
  }
}

void prometheus_slam_Server::updateCliMapRelativePose() {
  std::lock_guard<std::mutex> pose_update_lock( *(tf_controller_->getPoseUpdateMutex()) );  
  tf_controller_->resetCliMapRelativePoses();
  PoseMap pose_map = pose_graph_interface_.getPoseMap();  
  for (int i = 0; i < config_.client_number; i++) {
    std::vector<SerSmId> ser_sm_ids_a;
    if (!submap_collection_ptr_->getSerSmIdsByCliId(i, &ser_sm_ids_a)) continue;
    for (int j = i + 1; j < config_.client_number; j++) {
      std::vector<SerSmId> ser_sm_ids_b;
      if (!submap_collection_ptr_->getSerSmIdsByCliId(j, &ser_sm_ids_b))
        continue;
      for (auto const& sm_id_a : ser_sm_ids_a) {
        for (auto const& sm_id_b : ser_sm_ids_b) {
          Transformation T_CA_SMA = submap_collection_ptr_->getOriPose(sm_id_a);
          Transformation T_CB_SMB = submap_collection_ptr_->getOriPose(sm_id_b);
          Transformation T_SMA_SMB = pose_map[sm_id_a].inverse() * pose_map[sm_id_b]; 
          Transformation T_CA_CB = T_CA_SMA * T_SMA_SMB * T_CB_SMB.inverse();
 
          tf_controller_->addCliMapRelativePose(i, j, T_CA_CB); 
        }
      }
    }
  }
}

}  // namespace prometheus_slam
