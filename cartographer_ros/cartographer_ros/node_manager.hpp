#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/io/status.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/msg_conversion.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"
#include "sys/stat.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "boost/filesystem.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "movexbot_msgs/androidConsole.h"
#include "movexbot_msgs/mapAndroid.h"
#include "movexbot_msgs/mapInfo.h"
#include "movexbot_msgs/robotPos.h"
#include "movexbot_msgs/destination_point.h"
#include "std_msgs/Bool.h"
#include "ros/wall_timer.h"
#include <syscall.h>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <aruco_msgs/MarkerArray.h>
#include <lib/StationID.hpp>
#include <lib/Common.hpp>
#include <movexbot_msgs/Relocalization.h>
#include <movexbot_msgs/SetSlamCmd.h>


DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(configuration_basename_localization, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(map_folder_path, "",
              "map_file_path");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(start_trajectory_with_default_topics, true,
            "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(save_state_filename, "",
            "If non-empty, serialize state and write it to disk before shutting down.");
DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");

namespace cartographer_ros {

using ::cartographer::transform::Rigid3d;
using TrajectoryState = ::cartographer::mapping::PoseGraphInterface::TrajectoryState;
using ::cartographer::io::SlamState;
using ::cartographer::mapping::SubmapId;
using ::cartographer::io::SubmapSlice;
#define slam_state_ ::cartographer::io::slam_state

class Manager
{
  struct MapData_t
  {
    //for location
    std::string map_name;
    nav_msgs::MapMetaData map_info;//TODO:可以删除，需要先统一坐标系为相对于"map"的栅格坐标
    //for mapping
    uint32_t app_map_size_max;
    movexbot_msgs::mapAndroid app_map;
    float mapping_resolution;
  };
  
private:
  boost::shared_ptr<Node> node_;
  NodeOptions node_options;
  TrajectoryOptions trajectory_options,trajectory_options_localization;
  tf2_ros::Buffer* tf_buffer_;
  ros::Subscriber initial_pose_sub_,app_initial_pose_sub_,robot_pose_sub_,slam_state_sub_,dock_station_sub_;
  ros::Publisher map_android_pub_,mapping_map_info_,robot_grid_pos_pub_,map_pub_,slam_state_pub_;
  ros::ServiceServer relocalization_service_,slam_cmd_service_;
  ros::NodeHandle nh;
  MapData_t map_data_;
  tf::TransformListener listener_;
  StationID dock_station_;

  void StartMapping(std::string map_name);
  void CloseMapping(std::string map_name);
  void StartLocalization(std::string map_name);
  void CloseLocalization(void);
  void SetInitialPose(const geometry_msgs::Pose &pose);
  void SetLocalizationSucceed(void);
  void SaveMap(std::string map_name);
  std::unique_ptr<nav_msgs::OccupancyGrid> GetOccupancyGridMap(float resolution);
  void HandleInitialPose(const geometry_msgs::PoseWithCovarianceStamped &pose_msg);
  void HandleAppInitialPose(const movexbot_msgs::destination_point &pose_msg);
  void HandleSlamState(const movexbot_msgs::androidConsole &msg);
  void HandleRobotPose(const geometry_msgs::PoseStamped &pose);
  void HandleDockStation(const aruco_msgs::MarkerArray &markers);
  void MapPublish(void);
  bool RelocalizationService(movexbot_msgs::Relocalization::Request &req,movexbot_msgs::Relocalization::Response &res);
  bool SetSlamCmdService(movexbot_msgs::SetSlamCmd::Request &req,movexbot_msgs::SetSlamCmd::Response &res);
  static void MapPublishThread(Manager *manager);
public:
  Manager(tf2_ros::Buffer* tf_buffer)
  {
    dock_station_.SetMapFolderPath(FLAGS_map_folder_path);
    tf_buffer_ = tf_buffer;
    std::tie(node_options, trajectory_options) = LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
    std::tie(node_options, trajectory_options_localization) = LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename_localization);
    initial_pose_sub_ = nh.subscribe("/initialpose",1, &Manager::HandleInitialPose,this);
    app_initial_pose_sub_ = nh.subscribe("/nav/cmd/initPose", 1, &Manager::HandleAppInitialPose, this);
    // slam_state_sub_ = nh.subscribe("/scan/cmd/slam_state", 1, &Manager::HandleSlamState,this);
    slam_state_pub_ = nh.advertise<movexbot_msgs::androidConsole>("/scan/cmd/slam_state",1);
    map_android_pub_ = nh.advertise<movexbot_msgs::mapAndroid>("/map_android_picture", 1);
    mapping_map_info_ = nh.advertise<movexbot_msgs::mapInfo>("/mapping_map_info", 1);
    robot_grid_pos_pub_ = nh.advertise<movexbot_msgs::robotPos>("robot_pos", 1);
    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    relocalization_service_ = nh.advertiseService("slam/relocalization_service",&Manager::RelocalizationService,this);
    slam_cmd_service_ = nh.advertiseService("slam/set_cmd",&Manager::SetSlamCmdService,this);
    std::thread map_publish_thread(MapPublishThread,this);
    map_publish_thread.detach();
  };
  ~Manager(){}
};

bool Manager::SetSlamCmdService(movexbot_msgs::SetSlamCmd::Request &req,movexbot_msgs::SetSlamCmd::Response &res)
{  
  switch (req.cmd)
  {
    case req.SLAM_CMD_STANDBY:
      {
        if(slam_state_ == SlamState::SLAM_STATE_MAPPING)
          CloseMapping(req.map_name);
        else if(slam_state_ == SlamState::SLAM_STATE_LOCATING || slam_state_ == SlamState::SLAM_STATE_LOCATE_SUCCEED)
          CloseLocalization();
      }
      break;
    case req.SLAM_CMD_LOCATING:
      StartLocalization(req.map_name);
      break;

    case req.SLAM_CMD_LOCATE_SUCCEED:
      SetLocalizationSucceed();
      break;

    case req.SLAM_CMD_MAPPING:
      StartMapping(req.map_name);
      break;
    
    default:
      break;
  }
  return true;
}

bool Manager::RelocalizationService(movexbot_msgs::Relocalization::Request &req,movexbot_msgs::Relocalization::Response &res)
{
  geometry_msgs::Pose initial_pose;
  initial_pose.position.x = req.robot_pose.x * map_data_.map_info.resolution;
  initial_pose.position.y = req.robot_pose.y * map_data_.map_info.resolution;
  initial_pose.orientation = tf::createQuaternionMsgFromYaw(req.robot_pose.theta);
  LOG(INFO) << "relocalization...";
  SetInitialPose(initial_pose);

  //TODO:增加对定位状态的判断，是否产生约束，如产生正确约束并跳转位置，就直接返回成功，如超过3s未产生正确约束并跳转位置就返回失败
  ros::Rate loop(100);
  ros::Time start_time = ros::Time::now();
  while (ros::ok())
  {
    if((ros::Time::now() - start_time).toSec() > 3)
      break;
    ros::spinOnce();
    loop.sleep();
  }
  if(slam_state_ == SlamState::SLAM_STATE_LOCATING)
    SetLocalizationSucceed();
  res.state = res.STATE_SUCCESS;
  return true;
}

void Manager::HandleInitialPose(const geometry_msgs::PoseWithCovarianceStamped &pose_msg)
{
  SetInitialPose(pose_msg.pose.pose);
}

//TODO:当前坐标点为图像像素坐标，需修改为map坐标系
void Manager::HandleAppInitialPose(const movexbot_msgs::destination_point &pose_msg)
{
  ROS_INFO("set initial_pose:(%d,%d,%f) by app...",pose_msg.gridPosition.x,pose_msg.gridPosition.y,pose_msg.angle);
  geometry_msgs::Pose initial_pose;
  initial_pose.position.x = pose_msg.gridPosition.x * map_data_.map_info.resolution + map_data_.map_info.origin.position.x;
  initial_pose.position.y = pose_msg.gridPosition.y * map_data_.map_info.resolution + map_data_.map_info.origin.position.y;
  initial_pose.orientation = tf::createQuaternionMsgFromYaw(pose_msg.angle);
  SetInitialPose(initial_pose);
}

void Manager::HandleSlamState(const movexbot_msgs::androidConsole &msg)
{
  if(msg.model.data == "mapping_start")
    StartMapping(msg.name.data);
  else if(msg.model.data == "mapping_close")
    CloseMapping(msg.name.data);
  else if(msg.model.data == "localization_start")
    StartLocalization(msg.name.data);
  else if(msg.model.data == "localization_close")
    CloseLocalization();
  else if(msg.model.data == "localization_success")
    SetLocalizationSucceed();
}

void Manager::HandleRobotPose(const geometry_msgs::PoseStamped &pose) //only for mapping mode
{
  static movexbot_msgs::robotPos robot_grid_pose;
  robot_grid_pose.angle = tf::getYaw(pose.pose.orientation);
  robot_grid_pose.gridPosition.x = round(pose.pose.position.x / map_data_.app_map.resolution);
  robot_grid_pose.gridPosition.y = round(pose.pose.position.y / map_data_.app_map.resolution);
  robot_grid_pose.mapInfo.gridWidth = map_data_.app_map.width;
  robot_grid_pose.mapInfo.gridHeight = map_data_.app_map.height;
  robot_grid_pose.mapInfo.originX = map_data_.app_map.originX;
  robot_grid_pose.mapInfo.originY = map_data_.app_map.originY;
  robot_grid_pose.mapInfo.resolution = map_data_.app_map.resolution;
  robot_grid_pos_pub_.publish(robot_grid_pose);
}

void Manager::HandleDockStation(const aruco_msgs::MarkerArray &markers) //TODO: consider move to navigation
{
  if(slam_state_ == SlamState::SLAM_STATE_LOCATING)
  {
    auto station = dock_station_.GetStation(markers.markers.front().id);
    if(station.first)
    {
      ROS_INFO("current map name:%s,station map name:%s",map_data_.map_name.c_str(),station.second.map_name.c_str());
      if(map_data_.map_name != station.second.map_name)
      {
        StartLocalization(station.second.map_name);
        ros::Duration(2.0).sleep();
        return;
      }
      else
      {
        if(listener_.waitForTransform("/base_link", markers.header.frame_id, markers.header.stamp, ros::Duration(1.0)))
        {
          geometry_msgs::PoseStamped tag_pose;
          tag_pose.header = markers.header;
          tag_pose.pose = markers.markers.front().pose.pose;
          listener_.transformPose("/base_link",tag_pose,tag_pose);
          tag_pose.pose.position.z = 0;
          tag_pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(tag_pose.pose.orientation));
          tf::Transform tag_to_base_link,tag_to_map;
          tf::poseMsgToTF(tag_pose.pose,tag_to_base_link);
          tf::poseMsgToTF(station.second.pose,tag_to_map);
          tf::Transform new_base_link_to_map = tag_to_map * tag_to_base_link.inverse();
          geometry_msgs::Pose new_base_link_pose;
          tf::poseTFToMsg(new_base_link_to_map,new_base_link_pose);
          SetInitialPose(new_base_link_pose);
          SetLocalizationSucceed();
          ROS_INFO("have found tag station,location success.");
        }
      }
    }
  }
  else
    dock_station_sub_.shutdown();
}

void Manager::MapPublishThread(Manager *manager)
{
  ros::Rate loop(1);
  while(ros::ok())
  {
    if(slam_state_ == SlamState::SLAM_STATE_MAPPING)
      manager->MapPublish();
    loop.sleep();
  }
}

void Manager::MapPublish(void)
{
  ros::Time start = ros::Time::now();
  std::unique_ptr<nav_msgs::OccupancyGrid> map = GetOccupancyGridMap(map_data_.mapping_resolution);
  if(map->data.empty())
  {
    ROS_ERROR("empty data of map!!");
    return;
  }

  //update mapping resolution
  // ROS_ERROR("map size:(%d,%d)%d,resolution:%f",map->info.width,map->info.height,map->data.size(),map->info.resolution);
  if(map->data.size() > map_data_.app_map_size_max)
  {
    map_data_.mapping_resolution += 0.01;
    // ROS_INFO("update resolution:%f",map_data_.mapping_resolution);
  }

  cv::Mat mat = OccupancyGridToMat(*map);

  //draw trajectory
  auto trajectory_node_list = node_->GetMapBuilderBridge()->GetTrajectoryNodeList();
  for(auto &trajectory_node: trajectory_node_list.markers)
  {
    cv::Vec4b color(trajectory_node.color.b*255,trajectory_node.color.g*255,trajectory_node.color.r*255,0xFF);
    cv::Point last_point(0,0);
    for(auto &point:trajectory_node.points)
    {
      cv::Point new_point;
      new_point.x = (point.x - map->info.origin.position.x) / map->info.resolution;
      new_point.y = (point.y - map->info.origin.position.y) / map->info.resolution;
      new_point.y = map->info.height - new_point.y - 1;
      if(last_point != cv::Point(0,0))
        cv::line(mat,last_point,new_point,color);
      last_point = new_point;
    }
  }
  ROS_INFO("time cost0:%f",(ros::Time::now() - start).toSec());
  // cv::imshow("a",mat);
  // cv::waitKey(1);
  
  static std::vector<int> compression_params = {CV_IMWRITE_PNG_COMPRESSION,5};
  std::vector<uchar> data_encode;
  start = ros::Time::now();
  cv::imencode(".png", mat, data_encode,compression_params);
  ROS_WARN("time cost1:%f",(ros::Time::now() - start).toSec());
  map_data_.app_map.width = mat.cols;
  map_data_.app_map.height = mat.rows;
  map_data_.app_map.resolution = map->info.resolution;
  map_data_.app_map.originX = round(map->info.origin.position.x / map_data_.app_map.resolution);
  map_data_.app_map.originY = round(map->info.origin.position.y / map_data_.app_map.resolution);
  map_data_.app_map.data = data_encode;
  map_android_pub_.publish(map_data_.app_map);
  
  movexbot_msgs::mapInfo map_info;
  map_info.gridWidth = map_data_.app_map.width;
  map_info.gridHeight = map_data_.app_map.height;
  map_info.resolution = map_data_.app_map.resolution;
  map_info.originX = map_data_.app_map.originX;
  map_info.originY = map_data_.app_map.originY;
  mapping_map_info_.publish(map_info);
  
  if(map_pub_.getNumSubscribers() > 0)
  {
    start = ros::Time::now();
    map = GetOccupancyGridMap(FLAGS_resolution);
    map_pub_.publish(*map);
    ROS_INFO("time cost2:%f",(ros::Time::now() - start).toSec());
  }
}

std::unique_ptr<nav_msgs::OccupancyGrid> Manager::GetOccupancyGridMap(float resolution)
{
  auto &submap_slices = *(node_->GetMapBuilderBridge()->GetSubmapSlice());
  auto painted_slices = PaintSubmapSlices(submap_slices, resolution);
  return CreateOccupancyGridMsg(painted_slices, resolution, node_options.map_frame, ros::Time::now());
}

void Manager::StartMapping(std::string map_name)
{
  ROS_INFO("start mapping!map_name:%s",map_name.c_str());
  if(node_)
  {
    node_->FinishAllTrajectories();
    LOG(INFO) << "reset node ...";
    node_.reset();
    LOG(INFO) << "reset node done";
  }
  map_data_.app_map_size_max = 200000; //png compress will decrease data size tenfold
  auto map_builder = cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  node_ = boost::shared_ptr<Node>(new Node(node_options, std::move(map_builder), tf_buffer_,FLAGS_collect_metrics));
  if(!map_name.empty() && boost::filesystem::exists(FLAGS_map_folder_path + map_name + "/" + map_name + ".pbstream")) //extend map
  {
    node_->LoadState(FLAGS_map_folder_path + map_name + "/" + map_name + ".pbstream",FLAGS_load_frozen_state);
    std::unique_ptr<nav_msgs::OccupancyGrid> map = GetOccupancyGridMap(FLAGS_resolution);
    map_data_.mapping_resolution = sqrt((float)map->data.size() / map_data_.app_map_size_max) * FLAGS_resolution;
  }
  else
    map_data_.mapping_resolution = FLAGS_resolution;
  node_->StartTrajectoryWithDefaultTopics(trajectory_options);
  robot_pose_sub_ = nh.subscribe("/tracked_pose",1, &Manager::HandleRobotPose,this);
  slam_state_ = SlamState::SLAM_STATE_MAPPING;
  
  movexbot_msgs::androidConsole slam_state_msg;
  slam_state_msg.model.data = "mapping_start";
  slam_state_msg.name.data = map_name;
  slam_state_pub_.publish(slam_state_msg);

  //TODO:start record rosbag (code is in navigation pkg now)
}

void Manager::CloseMapping(std::string map_name)
{
  LOG(INFO) << "close mapping!map name:" << map_name.c_str();
  if(!node_)
    return;
  slam_state_ = SlamState::SLAM_STATE_STANDBY;
  robot_pose_sub_.shutdown();
  node_->FinishAllTrajectories();

  if (!map_name.empty() && !FLAGS_map_folder_path.empty()) 
  {
    node_->RunFinalOptimization();
    SaveMap(map_name);
  }
  LOG(INFO) << "reset node ...";
  node_.reset();
  LOG(INFO) << "reset node done";

  
  if(map_name.empty())
  {
    movexbot_msgs::androidConsole slam_state_msg;
    slam_state_msg.model.data = "mapping_close";
    slam_state_msg.name.data = map_name;
    slam_state_pub_.publish(slam_state_msg);
  }
  else
    StartLocalization(map_name);
}

void Manager::StartLocalization(std::string map_name)
{
  LOG(INFO) << "start localization!map name:" << map_name;
  if(map_name.empty() || FLAGS_map_folder_path.empty())
    return;
  if(node_)
  {
    node_->FinishAllTrajectories();
    LOG(INFO) << "reset node ...";
    node_.reset();
    LOG(INFO) << "reset node done";
  }
  if(!boost::filesystem::exists(FLAGS_map_folder_path + map_name + "/" + map_name + ".pbstream"))
  {
    ROS_ERROR("the pbstream file is not exist,return!");
    return;
  }
  map_data_.map_name = map_name;
  slam_state_ = SlamState::SLAM_STATE_LOCATING;
  
  auto map_builder = cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  node_ = boost::shared_ptr<Node>(new Node(node_options, std::move(map_builder), tf_buffer_,
            FLAGS_collect_metrics));
  node_->LoadState(FLAGS_map_folder_path + map_name + "/" + map_name + ".pbstream",
                  FLAGS_load_frozen_state);
  std::unique_ptr<nav_msgs::OccupancyGrid> map = GetOccupancyGridMap(FLAGS_resolution);
  map_data_.map_info = map->info;
  map_pub_.publish(*map);
  if (FLAGS_start_trajectory_with_default_topics) {
    node_->StartTrajectoryWithDefaultTopics(trajectory_options_localization);
  }
  dock_station_sub_ = nh.subscribe("/aruco_marker_publisher/markers",1,&Manager::HandleDockStation,this);
  
  movexbot_msgs::androidConsole slam_state_msg;
  slam_state_msg.model.data = "localization_start";
  slam_state_msg.name.data = map_name;
  slam_state_pub_.publish(slam_state_msg);
}

void Manager::SetLocalizationSucceed(void)
{
  LOG(INFO) << "Set localization succeed";
  slam_state_ = SlamState::SLAM_STATE_LOCATE_SUCCEED;

  movexbot_msgs::androidConsole slam_state_msg;
  slam_state_msg.model.data = "localization_success";
  slam_state_msg.name.data = map_data_.map_name;
  slam_state_pub_.publish(slam_state_msg);
}

void Manager::CloseLocalization(void)
{
  ROS_INFO("close localization!");
  if(node_)
  {
    node_->FinishAllTrajectories();
    LOG(INFO) << "reset node ...";
    node_.reset();
    LOG(INFO) << "reset node done";
    slam_state_ = SlamState::SLAM_STATE_STANDBY;
  }
}

void Manager::SetInitialPose(const geometry_msgs::Pose &initial_pose)
{
  if(!node_)
    return;
  LOG(INFO) << GetFormatString("initial_pose:(%f,%f,%f)",initial_pose.position.x,initial_pose.position.y,tf::getYaw(initial_pose.orientation));
  if(slam_state_ == SlamState::SLAM_STATE_LOCATE_SUCCEED)
    slam_state_ = SlamState::SLAM_STATE_LOCATING;
  node_->FinishLastTrajectory(); //TODO: delete the last trajectory to avoid memory increase
  TrajectoryOptions initial_pose_trajectory_options_localization = trajectory_options_localization;

  const auto pose = ToRigid3d(initial_pose);
  if (!pose.IsValid()) {
    LOG(ERROR) << "Invalid pose argument. Orientation quaternion must be normalized.";
    return;
  }

  ::cartographer::mapping::proto::InitialTrajectoryPose initial_trajectory_pose;
  initial_trajectory_pose.set_to_trajectory_id(0);
  *initial_trajectory_pose.mutable_relative_pose() = cartographer::transform::ToProto(pose);
  initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(::cartographer_ros::FromRos(ros::Time(0))));
  *initial_pose_trajectory_options_localization.trajectory_builder_options.mutable_initial_trajectory_pose() = initial_trajectory_pose;

  node_->StartTrajectoryWithDefaultTopics(initial_pose_trajectory_options_localization);
}

void Manager::SaveMap(std::string map_name)
{
  std::string map_name_folder_path = FLAGS_map_folder_path + map_name;
  ROS_INFO("map_name_folder_path:%s",map_name_folder_path.c_str());
  if(access(map_name_folder_path.c_str(),0) != 0)
    mkdir(map_name_folder_path.c_str(),S_IRWXU | S_IRWXG | S_IRWXO);
  node_->SerializeState(map_name_folder_path + "/" + map_name + ".pbstream",true /* include_unfinished_submaps */);
  
  /*******************save png****************/
  std::unique_ptr<nav_msgs::OccupancyGrid> map = GetOccupancyGridMap(FLAGS_resolution);
  cv::Mat colorMat = OccupancyGridToMat(*map);
  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(9);
  if(!cv::imwrite(map_name_folder_path + "/" + map_name + ".png", colorMat, compression_params))
  {
    LOG(ERROR) << "save map.png error:" << map_name_folder_path + "/" + map_name;
    return;
  }
  cv::imwrite(map_name_folder_path + "/" + map_name + "_temp.png", colorMat, compression_params);

  /*******************save yaml****************/
  FILE *yaml = fopen((map_name_folder_path + "/" + map_name + ".yaml").c_str(), "w");
  fprintf(yaml, "image: %s\nresolution: %f\nwidth: %d\nheight: %d\norigin: [%f, %f, %f]\nnegate: 0\n\n",
          (map_name + ".png").c_str(), map->info.resolution, map->info.width, map->info.height,
          map->info.origin.position.x, map->info.origin.position.y, tf::getYaw(map->info.origin.orientation));
  fclose(yaml);
  ROS_INFO("map saved done!");
}

}  // namespace cartographer_ros