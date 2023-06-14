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

#include <cmath>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/synchronization/mutex.h"
#include "cairo/cairo.h"
#include "cartographer/common/port.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "gflags/gflags.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

#include <mx_common/Common.hpp>
#include "movexbot_msgs/mapAndroid.h"
#include "movexbot_msgs/mapInfo.h"
#include "movexbot_msgs/robotPos.h"
#include "movexbot_msgs/RealtimeStatus.h"
#include <movexbot_msgs/SlamStatus.h>
#include "visualization_msgs/MarkerArray.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");
DEFINE_double(publish_period_sec, 1.0, "OccupancyGrid publishing period.");
DEFINE_bool(include_frozen_submaps, true,
            "Include frozen submaps in the occupancy grid.");
DEFINE_bool(include_unfrozen_submaps, true,
            "Include unfrozen submaps in the occupancy grid.");
DEFINE_string(occupancy_grid_topic, cartographer_ros::kOccupancyGridTopic,
              "Name of the topic on which the occupancy grid is published.");

namespace cartographer_ros {
namespace {

using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

class Node {
 public:
  explicit Node(double resolution, double publish_period_sec);
  ~Node() {}

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

 private:
  void HandleSubmapList(const cartographer_ros_msgs::SubmapList::ConstPtr& msg);
  void DrawAndPublish(const ::ros::WallTimerEvent& timer_event);
  

  void RobotStatusCallBack(const movexbot_msgs::RealtimeStatus &robot_status);
  void SlamStatusCallBack(const movexbot_msgs::SlamStatus &slam_status);
  void HandleRobotPose(const geometry_msgs::PoseStamped &pose);
  void TrajectoryNodeListCallBack(const ::visualization_msgs::MarkerArray& msg);
  void PointCloudCallBack(const ::sensor_msgs::PointCloud2& msg);

  ::ros::NodeHandle node_handle_;
  double resolution_;

  absl::Mutex mutex_;
  ::ros::ServiceClient client_ GUARDED_BY(mutex_);
  ::ros::Subscriber submap_list_subscriber_ GUARDED_BY(mutex_);
  ::ros::Publisher occupancy_grid_publisher_ GUARDED_BY(mutex_);
  std::map<SubmapId, SubmapSlice> submap_slices_ GUARDED_BY(mutex_);
  ::ros::WallTimer occupancy_grid_publisher_timer_;
  std::string last_frame_id_;
  ros::Time last_timestamp_;

  ros::Publisher map_android_pub_,robot_grid_pos_pub_;
  ros::Subscriber robot_pose_sub_,robot_realtime_status_sub_,trajectory_node_list_sub_,point_cloud_sub_,slam_status_sub_;
  movexbot_msgs::RealtimeStatus robot_status_;
  movexbot_msgs::mapInfo map_info;
  visualization_msgs::MarkerArray trajectory_node_list_;
  sensor_msgs::PointCloud2 point_cloud_;
  movexbot_msgs::SlamStatus slam_status_;
};

Node::Node(const double resolution, const double publish_period_sec)
    : resolution_(resolution),
      client_(node_handle_.serviceClient<::cartographer_ros_msgs::SubmapQuery>(
          kSubmapQueryServiceName)),
      submap_list_subscriber_(node_handle_.subscribe(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize,
          boost::function<void(
              const cartographer_ros_msgs::SubmapList::ConstPtr&)>(
              [this](const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
                HandleSubmapList(msg);
              }))),
      occupancy_grid_publisher_(
          node_handle_.advertise<::nav_msgs::OccupancyGrid>(
              FLAGS_occupancy_grid_topic, kLatestOnlyPublisherQueueSize,
              false /* latched */)),
      occupancy_grid_publisher_timer_(
          node_handle_.createWallTimer(::ros::WallDuration(publish_period_sec),
                                       &Node::DrawAndPublish, this)) {
      map_android_pub_ = node_handle_.advertise<movexbot_msgs::mapAndroid>("/map_android_picture", 1);
      robot_grid_pos_pub_ = node_handle_.advertise<movexbot_msgs::robotPos>("robot_pos", 1);

      robot_pose_sub_ = node_handle_.subscribe(kTrackedPoseTopic,1, &Node::HandleRobotPose,this);
      // robot_realtime_status_sub_ = node_handle_.subscribe("/robot_status/realtime_status",1,&Node::RobotStatusCallBack, this);
      trajectory_node_list_sub_ = node_handle_.subscribe(kTrajectoryNodeListTopic,1,&Node::TrajectoryNodeListCallBack, this);
      point_cloud_sub_ = node_handle_.subscribe(kScanMatchedPointCloudTopic,1,&Node::PointCloudCallBack, this);
      slam_status_sub_ = node_handle_.subscribe("/slam_status",1,&Node::SlamStatusCallBack, this);
                                       }

void Node::RobotStatusCallBack(const movexbot_msgs::RealtimeStatus &robot_status)
{
  robot_status_ = robot_status;
}

void Node::SlamStatusCallBack(const movexbot_msgs::SlamStatus &slam_status)
{
  slam_status_ = slam_status;
}

void Node::HandleRobotPose(const geometry_msgs::PoseStamped &pose)
{
  static bool flag_initial_done = true;
  if(slam_status_.slam_state == slam_status_.SLAM_STATE_MAPPING)
  {
    static movexbot_msgs::robotPos robot_grid_pose;
    robot_grid_pose.angle = tf::getYaw(pose.pose.orientation);
    robot_grid_pose.gridPosition.x = round(pose.pose.position.x / map_info.resolution);
    robot_grid_pose.gridPosition.y = round(pose.pose.position.y / map_info.resolution);
    robot_grid_pose.mapInfo = map_info;
    robot_grid_pos_pub_.publish(robot_grid_pose);
  }
  else if(!flag_initial_done)
  {
    resolution_ = 0.05;
    submap_slices_.clear();
    flag_initial_done = true;
  }
  
}

void Node::TrajectoryNodeListCallBack(const ::visualization_msgs::MarkerArray& msg)
{
  trajectory_node_list_ = msg;
}

void Node::PointCloudCallBack(const ::sensor_msgs::PointCloud2& msg)
{
  point_cloud_ = msg;
}

void Node::HandleSubmapList(
    const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
  absl::MutexLock locker(&mutex_);

  // We do not do any work if nobody listens.
  // if (occupancy_grid_publisher_.getNumSubscribers() == 0) {
  //   return;
  // }

  if(slam_status_.slam_state != slam_status_.SLAM_STATE_MAPPING)
    return;

  // Keep track of submap IDs that don't appear in the message anymore.
  std::set<SubmapId> submap_ids_to_delete;
  for (const auto& pair : submap_slices_) {
    submap_ids_to_delete.insert(pair.first);
  }

  for (const auto& submap_msg : msg->submap) {
    const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
    submap_ids_to_delete.erase(id);
    if ((submap_msg.is_frozen && !FLAGS_include_frozen_submaps) ||
        (!submap_msg.is_frozen && !FLAGS_include_unfrozen_submaps)) {
      continue;
    }
    SubmapSlice& submap_slice = submap_slices_[id];
    submap_slice.pose = ToRigid3d(submap_msg.pose);
    submap_slice.metadata_version = submap_msg.submap_version;
    if (submap_slice.surface != nullptr &&
        submap_slice.version == submap_msg.submap_version) {
      continue;
    }

    auto fetched_textures =
        ::cartographer_ros::FetchSubmapTextures(id, &client_);
    if (fetched_textures == nullptr) {
      continue;
    }
    CHECK(!fetched_textures->textures.empty());
    submap_slice.version = fetched_textures->version;

    // We use the first texture only. By convention this is the highest
    // resolution texture and that is the one we want to use to construct the
    // map for ROS.
    const auto fetched_texture = fetched_textures->textures.begin();
    submap_slice.width = fetched_texture->width;
    submap_slice.height = fetched_texture->height;
    submap_slice.slice_pose = fetched_texture->slice_pose;
    submap_slice.resolution = fetched_texture->resolution;
    submap_slice.cairo_data.clear();
    submap_slice.surface = ::cartographer::io::DrawTexture(
        fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
        fetched_texture->width, fetched_texture->height,
        &submap_slice.cairo_data);
  }

  // Delete all submaps that didn't appear in the message.
  for (const auto& id : submap_ids_to_delete) {
    submap_slices_.erase(id);
  }

  last_timestamp_ = msg->header.stamp;
  last_frame_id_ = msg->header.frame_id;
}

void Node::DrawAndPublish(const ::ros::WallTimerEvent& unused_timer_event) {
  absl::MutexLock locker(&mutex_);

  if(slam_status_.slam_state != slam_status_.SLAM_STATE_MAPPING)
    return;
  if (submap_slices_.empty() || last_frame_id_.empty()) {
    return;
  }
  ros::Time start = ros::Time::now();
  auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);
  std::unique_ptr<nav_msgs::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
      painted_slices, resolution_, last_frame_id_, last_timestamp_);
  if(occupancy_grid_publisher_.getNumSubscribers() > 0)
    occupancy_grid_publisher_.publish(*msg_ptr);

#if 1
  if(msg_ptr->data.size() > 400000)
  {
    resolution_ += 0.01;
    LOG(INFO) << "update resolution:" << resolution_;
  }

  cv::Mat mat = OccupancyGridToMat(*msg_ptr);
  // draw trajectory
  for(auto &trajectory_node: trajectory_node_list_.markers)
  {
    cv::Vec4b color(trajectory_node.color.b*255,trajectory_node.color.g*255,trajectory_node.color.r*255,0xFF);
    cv::Point last_point(0,0);
    for(auto &point:trajectory_node.points)
    {
      cv::Point new_point;
      new_point.x = (point.x - msg_ptr->info.origin.position.x) / msg_ptr->info.resolution;
      new_point.y = msg_ptr->info.height - ((point.y - msg_ptr->info.origin.position.y) / msg_ptr->info.resolution) - 1;
      if(last_point != cv::Point(0,0)/*  && (new_point.x < mat.cols && new_point.y < mat.rows) */)
        cv::line(mat,last_point,new_point,color);
      last_point = new_point;
    }
  }

  // draw point_cloud
  sensor_msgs::PointCloud out_pointcloud;
	sensor_msgs::convertPointCloud2ToPointCloud(point_cloud_, out_pointcloud);
  for(auto &point: out_pointcloud.points)
  {
    static cv::Vec4b color(0,0,0xFF,0xFF);
    cv::Point mat_point;
    mat_point.x = (point.x - msg_ptr->info.origin.position.x) / msg_ptr->info.resolution;
    mat_point.y = msg_ptr->info.height - ((point.y - msg_ptr->info.origin.position.y) / msg_ptr->info.resolution) - 1;
    if(mat_point.x < mat.cols && mat_point.y < mat.rows)
      mat.at<cv::Vec4b>(mat_point.y,mat_point.x) = color;
  }
  // ROS_INFO("time cost0:%f",(ros::Time::now() - start).toSec());
  // cv::imshow("a",mat);
  // cv::waitKey(1);
  //LOG(ERROR) << "test";
  static std::vector<int> compression_params = {CV_IMWRITE_PNG_COMPRESSION,5};
  std::vector<uchar> data_encode;
  cv::imencode(".png", mat, data_encode,compression_params);
  movexbot_msgs::mapAndroid map_android;
  map_android.width = mat.cols;
  map_android.height = mat.rows;
  map_android.resolution = msg_ptr->info.resolution;
  map_android.originX = round(msg_ptr->info.origin.position.x / map_android.resolution);
  map_android.originY = round(msg_ptr->info.origin.position.y / map_android.resolution);
  map_android.data = data_encode;
  map_android_pub_.publish(map_android);

  //useless
  map_info.gridWidth = map_android.width;
  map_info.gridHeight = map_android.height;
  map_info.resolution = map_android.resolution;
  map_info.originX = map_android.originX;
  map_info.originY = map_android.originY;
#endif
  ROS_ERROR("occupancy_grid_node_main: time cost:%f",(ros::Time::now() - start).toSec());
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(FLAGS_include_frozen_submaps || FLAGS_include_unfrozen_submaps)
      << "Ignoring both frozen and unfrozen submaps makes no sense.";

  ::ros::init(argc, argv, "cartographer_occupancy_grid_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  ::cartographer_ros::Node node(FLAGS_resolution, FLAGS_publish_period_sec);

  ::ros::spin();
  ::ros::shutdown();
}
