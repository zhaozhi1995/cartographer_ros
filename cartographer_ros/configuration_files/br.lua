include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 2,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  -- fast_motion_filter_time = 1e-1,
  -- search_linear_window = 8.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true --使用二维轨迹构建
TRAJECTORY_BUILDER_2D.use_imu_data = false --是否使用IMU数据进行建图
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2 --用于一组扫描匹配的激光的数据量

-- TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05 --地图占有栅格分辨率
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60
-- TRAJECTORY_BUILDER_2D.min_range = 0.05 -- 激光数据的最小距离,超出该范围的将被删除掉
-- TRAJECTORY_BUILDER_2D.max_range = 40.0 -- 激光数据的最大距离
-- --TRAJECTORY_BUILDER_2D.min_z = 0.1;
-- --TRAJECTORY_BUILDER_2D.max_z = 1;

-- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true --在线构建相关scan_match,提供准确初值
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20 -- 点云信任度
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10 --15 --推演位姿平移信任度
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40  --推演位姿旋转信任度

-- POSE_GRAPH.optimize_every_n_nodes = 20

-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.2
-- POSE_GRAPH.constraint_builder.min_score = 0.6
-- POSE_GRAPH.constraint_builder.max_constraint_distance = 15
-- POSE_GRAPH.constraint_builder.log_matches = false  --是否打印直方图日志，查看ceres优化扫描匹配日志
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.  --局部回环搜索距离
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)   --局部回环搜索角度
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 40

-- POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e3
-- POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e3
-- POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3 --里程计平移量的信任度，该值越大对里程计计算的位置信任度越大
-- POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5 --里程计旋转量的信任度，该值越大对里程计计算的航向角信任度越大
return options
