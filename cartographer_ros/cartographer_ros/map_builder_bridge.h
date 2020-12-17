/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H

#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "absl/synchronization/mutex.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/TrajectoryQuery.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"

// Abseil unfortunately pulls in winnt.h, which #defines DELETE.
// Clean up to unbreak visualization_msgs::Marker::DELETE.
#ifdef DELETE
#undef DELETE
#endif
#include "visualization_msgs/MarkerArray.h"

namespace cartographer_ros {

class MapBuilderBridge {
 public:
  struct LocalTrajectoryData {
    // Contains the trajectory data received from local SLAM, after
    // it had processed accumulated 'range_data_in_local' and estimated
    // current 'local_pose' at 'time'.
    struct LocalSlamData {
      ::cartographer::common::Time time;
      ::cartographer::transform::Rigid3d local_pose;
      ::cartographer::sensor::RangeData range_data_in_local;
    };
    std::shared_ptr<const LocalSlamData> local_slam_data;
    cartographer::transform::Rigid3d local_to_map;
    std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking;
    TrajectoryOptions trajectory_options;
  };

  MapBuilderBridge(
      const NodeOptions& node_options,
      std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
      tf2_ros::Buffer* tf_buffer);

  MapBuilderBridge(const MapBuilderBridge&) = delete;
  MapBuilderBridge& operator=(const MapBuilderBridge&) = delete;

  //add by zz
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> submap_slices_;
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> *GetSubmapSlice()
  {
    // Keep track of submap IDs that don't appear in the message anymore.
    std::set<::cartographer::mapping::SubmapId> submap_ids_to_delete;
    for (auto& pair : submap_slices_) {
      submap_ids_to_delete.insert(pair.first);
    }
    for (const auto& submap_id_pose : map_builder_->pose_graph()->GetAllSubmapPoses())
    {
      ::cartographer::mapping::SubmapId submap_id{submap_id_pose.id.trajectory_id,
                                                submap_id_pose.id.submap_index};
      submap_ids_to_delete.erase(submap_id);
      ::cartographer::io::SubmapSlice& submap_slice = submap_slices_[submap_id];
      submap_slice.pose = submap_id_pose.data.pose;
      submap_slice.metadata_version = submap_id_pose.data.version;
      if (submap_slice.surface != nullptr &&
          submap_slice.version == submap_id_pose.data.version) 
      continue;

      ::cartographer::mapping::proto::SubmapQuery::Response response_proto;
      const std::string error = map_builder_->SubmapToProto(submap_id, &response_proto);
      if (!error.empty()) 
      {
        LOG(ERROR) << error;
        submap_ids_to_delete.insert(submap_id);
        continue;
      }
      const auto& texture_proto = response_proto.textures().begin();
      ::cartographer::io::SubmapTexture texture = {
          ::cartographer::io::UnpackTextureData(texture_proto->cells(), texture_proto->width(),
                                                texture_proto->height()),
          texture_proto->width(), texture_proto->height(), texture_proto->resolution(),
          ::cartographer::transform::ToRigid3(texture_proto->slice_pose())};

      submap_slice.version = response_proto.submap_version();
      submap_slice.width = texture.width;
      submap_slice.height = texture.height;
      submap_slice.slice_pose = texture.slice_pose;
      submap_slice.resolution = texture.resolution;
      submap_slice.cairo_data.clear();
      submap_slice.surface = ::cartographer::io::DrawTexture(
          texture.pixels.intensity, texture.pixels.alpha,
          texture.width, texture.height,
          &submap_slice.cairo_data);
    }
    // Delete all submaps that didn't appear in the message.
    for (const auto& id : submap_ids_to_delete) {
      submap_slices_.erase(id);
    }
    return &submap_slices_;
  }

  void LoadState(const std::string& state_filename, bool load_frozen_state);
  int AddTrajectory(
      const std::set<
          ::cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
          expected_sensor_ids,
      const TrajectoryOptions& trajectory_options);
  void FinishTrajectory(int trajectory_id);
  void RunFinalOptimization();
  bool SerializeState(const std::string& filename,
                      const bool include_unfinished_submaps);

  void HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);
  void HandleTrajectoryQuery(
      cartographer_ros_msgs::TrajectoryQuery::Request& request,
      cartographer_ros_msgs::TrajectoryQuery::Response& response);

  std::map<int /* trajectory_id */,
           ::cartographer::mapping::PoseGraphInterface::TrajectoryState>
  GetTrajectoryStates();
  cartographer_ros_msgs::SubmapList GetSubmapList();
  std::unordered_map<int, LocalTrajectoryData> GetLocalTrajectoryData()
      LOCKS_EXCLUDED(mutex_);
  visualization_msgs::MarkerArray GetTrajectoryNodeList();
  visualization_msgs::MarkerArray GetLandmarkPosesList();
  visualization_msgs::MarkerArray GetConstraintList();

  SensorBridge* sensor_bridge(int trajectory_id);

 private:
  void OnLocalSlamResult(const int trajectory_id,
                         const ::cartographer::common::Time time,
                         const ::cartographer::transform::Rigid3d local_pose,
                         ::cartographer::sensor::RangeData range_data_in_local)
      LOCKS_EXCLUDED(mutex_);

  absl::Mutex mutex_;
  const NodeOptions node_options_;
  std::unordered_map<int,
                     std::shared_ptr<const LocalTrajectoryData::LocalSlamData>>
      local_slam_data_ GUARDED_BY(mutex_);
  std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_;
  tf2_ros::Buffer* const tf_buffer_;

  std::unordered_map<std::string /* landmark ID */, int> landmark_to_index_;

  // These are keyed with 'trajectory_id'.
  std::unordered_map<int, TrajectoryOptions> trajectory_options_;
  std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;
  std::unordered_map<int, size_t> trajectory_to_highest_marker_id_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H
