-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.02,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  publish_tracked_pose = true;
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 3

TRAJECTORY_BUILDER_2D.use_imu_data = true --是否使用IMU数据进行建图
TRAJECTORY_BUILDER_2D.min_range = 0.05 -- 激光数据的最小距离,超出该范围的将被删除掉
TRAJECTORY_BUILDER_2D.max_range = 40.0 -- 激光数据的最大距离
-- TRAJECTORY_BUILDER_2D.min_z = 0.1;
-- TRAJECTORY_BUILDER_2D.max_z = 1;
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data =90
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1 -- 点云信任度
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10 --15 --推演位姿平移信任度
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40  --推演位姿旋转信任度
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 --用于一组扫描匹配的激光的数据量

POSE_GRAPH.constraint_builder.log_matches = false  --是否打印直方图日志，查看ceres优化扫描匹配日志
POSE_GRAPH.optimize_every_n_nodes = 60
POSE_GRAPH.constraint_builder.min_score = 0.6
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
-- POSE_GRAPH.constraint_builder.max_constraint_distance = 15
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1
-- POSE_GRAPH.optimization_problem.huber_scale = 1e1 --huber损失函数处理离群点问题，该值越大，离群点对结果影响较大
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3
POSE_GRAPH.optimization_problem.rotation_weight = 3e5
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e3
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e3
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4 --里程计平移量的信任度，该值越大对里程计计算的位置信任度越大
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3 --里程计旋转量的信任度，该值越大对里程计计算的航向角信任度越大
return options
