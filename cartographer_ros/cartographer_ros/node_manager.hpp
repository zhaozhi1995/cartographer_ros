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
#include "skyee_msg/androidConsole.h"
#include "skyee_msg/mapAndroid.h"
#include "skyee_msg/mapInfo.h"
#include "skyee_msg/robotPos.h"
#include "skyee_msg/destination_point.h"
#include "std_msgs/Bool.h"
#include "ros/wall_timer.h"
#include <syscall.h>
#include <yaml-cpp/yaml.h>

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
    skyee_msg::mapAndroid app_map;
    float mapping_resolution;
  };
  

private:
  boost::shared_ptr<Node> node_;
  NodeOptions node_options;
  TrajectoryOptions trajectory_options,trajectory_options_localization;
  tf2_ros::Buffer* tf_buffer_;
  ros::Subscriber initial_pose_sub_,app_initial_pose_sub_,robot_pose_sub_,slam_state_sub_,aruco_tag_sub_;
  ros::Publisher map_android_pub_,mapping_map_info_,robot_grid_pos_pub_,map_pub_,start_pub_,close_pub_,slam_state_pub_,app_slam_state_pub_;
  ros::NodeHandle nh;
  ros::WallTimer map_publisher_timer_;
  MapData_t map_data_;
  tf::TransformListener listener_;

  void StartMapping(std::string map_name,uint32_t app_map_size_max);
  void CloseMapping(std::string map_name);
  void StartLocalization(std::string map_name);
  void CloseLocalization(void);
  void SetInitialPose(const geometry_msgs::Pose &pose);
  void SaveMap(std::string map_name);
  cv::Mat OccupancyGridToMat(const nav_msgs::OccupancyGrid &map);
  std::unique_ptr<nav_msgs::OccupancyGrid> GetOccupancyGridMap(float resolution);
  void HandleInitialPose(const geometry_msgs::PoseWithCovarianceStamped &pose_msg);
  void HandleAppInitialPose(const skyee_msg::destination_point &pose_msg);
  void HandleSlamState(const skyee_msg::androidConsole &msg);
  void HandleRobotPose(const geometry_msgs::PoseStamped &pose);
  void HandleArucoTag(const geometry_msgs::PoseStamped &tag_pose);
  void MapPublish(const ::ros::WallTimerEvent& unused_timer_event);
public:
  Manager(tf2_ros::Buffer* tf_buffer)
  {
    tf_buffer_ = tf_buffer;
    std::tie(node_options, trajectory_options) = LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
    std::tie(node_options, trajectory_options_localization) = LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename_localization);
    initial_pose_sub_ = nh.subscribe("/initialpose",1, &Manager::HandleInitialPose,this);
    app_initial_pose_sub_ = nh.subscribe("/nav/cmd/initPose", 1, &Manager::HandleAppInitialPose, this);
    slam_state_sub_ = nh.subscribe("/scan/cmd/slam_state", 1, &Manager::HandleSlamState,this);
    slam_state_pub_ = nh.advertise<skyee_msg::androidConsole>("/scan/cmd/slam_state",1);
    app_slam_state_pub_ = nh.advertise<std_msgs::String>("app_slam_state", 1);//TODO:可以用/scan/cmd/slam_state取代
    map_android_pub_ = nh.advertise<skyee_msg::mapAndroid>("/map_android_picture", 1);
    mapping_map_info_ = nh.advertise<skyee_msg::mapInfo>("/mapping_map_info", 1);
    robot_grid_pos_pub_ = nh.advertise<skyee_msg::robotPos>("robot_pos", 1);
    close_pub_ = nh.advertise<std_msgs::Bool>("cartographer_complete", 1, true);
    start_pub_ = nh.advertise<std_msgs::Bool>("cartographer_start", 1, true);
    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    map_publisher_timer_ = nh.createWallTimer(ros::WallDuration(1),&Manager::MapPublish, this);
    map_publisher_timer_.stop();
  };
  ~Manager(){}
};

void Manager::HandleInitialPose(const geometry_msgs::PoseWithCovarianceStamped &pose_msg)
{
  SetInitialPose(pose_msg.pose.pose);
}

//TODO:当前坐标点为图像像素坐标，需修改为map坐标系
void Manager::HandleAppInitialPose(const skyee_msg::destination_point &pose_msg)
{
  if(slam_state_ == SlamState::SLAM_STATE_LOCATING)
  {
    geometry_msgs::Pose initial_pose;
    initial_pose.position.x = pose_msg.gridPosition.x * map_data_.map_info.resolution + map_data_.map_info.origin.position.x;
    initial_pose.position.y = pose_msg.gridPosition.y * map_data_.map_info.resolution + map_data_.map_info.origin.position.y;
    initial_pose.orientation = tf::createQuaternionMsgFromYaw(pose_msg.angle);
    SetInitialPose(initial_pose);
  }
}

void Manager::HandleSlamState(const skyee_msg::androidConsole &msg)
{
  static std_msgs::Bool flag_msg;
  if(msg.model.data == "mapping_start")
  {
    flag_msg.data = true;
    start_pub_.publish(flag_msg);
    StartMapping(msg.name.data,(msg.mapArea!=0)?msg.mapArea:40000);
  }
  else if(msg.model.data == "mapping_close")
  {
    flag_msg.data = true;
    close_pub_.publish(flag_msg);
    CloseMapping(msg.name.data);
  }
  else if(msg.model.data == "localization_start")
  {
    flag_msg.data = true;
    start_pub_.publish(flag_msg);
    StartLocalization(msg.name.data);
  }
  else if(msg.model.data == "localization_close")
  {
    flag_msg.data = true;
    close_pub_.publish(flag_msg);
    CloseLocalization();
  }
  else if(msg.model.data == "localization_success")
  {
    slam_state_ = SlamState::SLAM_STATE_LOCATE_SUCCEED;
  }
  else if(msg.model.data == "cancel_localization_success_flag")
  {
    slam_state_ = SlamState::SLAM_STATE_LOCATING;
  }
}

void Manager::HandleRobotPose(const geometry_msgs::PoseStamped &pose) //only for mapping mode
{
  static skyee_msg::robotPos robot_grid_pose;
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

void Manager::HandleArucoTag(const geometry_msgs::PoseStamped &tag_pose)
{
  if(slam_state_ == SlamState::SLAM_STATE_LOCATING)
  {
    std::ifstream fin(FLAGS_map_folder_path + map_data_.map_name + "/ChargerStation/station.yaml");
    if (fin.is_open())
    {
      YAML::Node charger_station_yaml;
      charger_station_yaml = YAML::Load(fin);
      if (charger_station_yaml.IsNull())
        ROS_ERROR("station yaml is null!");
      else
      {
        tf::Transform tag_to_map;
        tag_to_map.getOrigin().setX(charger_station_yaml["gridX"].as<int>()*FLAGS_resolution);
        tag_to_map.getOrigin().setY(charger_station_yaml["gridY"].as<int>()*FLAGS_resolution);
        tag_to_map.setRotation(tf::createQuaternionFromYaw(charger_station_yaml["angle"].as<float>()));
        if(listener_.waitForTransform("/base_link", tag_pose.header.frame_id, tag_pose.header.stamp, ros::Duration(1.0)))
        {
          geometry_msgs::PoseStamped tag_pose_base_link;
          listener_.transformPose("/base_link",tag_pose,tag_pose_base_link);
          tf::StampedTransform tag_to_base_link;
          tag_to_base_link.getOrigin().setX(tag_pose_base_link.pose.position.x);
          tag_to_base_link.getOrigin().setY(tag_pose_base_link.pose.position.y);
          tag_to_base_link.setRotation(tf::createQuaternionFromYaw(tf::getYaw(tag_pose_base_link.pose.orientation)));
          tf::Transform new_base_link_to_map = tag_to_map * tag_to_base_link.inverse();
          geometry_msgs::Pose new_base_link_pose;
          tf::poseTFToMsg(new_base_link_to_map,new_base_link_pose);
          SetInitialPose(new_base_link_pose);
          skyee_msg::androidConsole slam_state_msg;
          slam_state_msg.model.data = "localization_success";
          slam_state_pub_.publish(slam_state_msg);
          std_msgs::String app_slam_state_msg;
          app_slam_state_msg.data = "slam_success";
          app_slam_state_pub_.publish(app_slam_state_msg);
          //pub slam state to succeed
          ROS_INFO("have found tag station,location success.");
        }
      }
    }
    else
      ROS_ERROR("no station yaml file!");
  }
  aruco_tag_sub_.shutdown();
}

void Manager::MapPublish(const ::ros::WallTimerEvent& unused_timer_event)
{
  if(slam_state_ != SlamState::SLAM_STATE_MAPPING)
  {
    map_publisher_timer_.stop();
    return;
  }
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
  std::vector<uchar> data_encode;
  cv::imencode(".png", mat, data_encode);
  map_data_.app_map.width = mat.cols;
  map_data_.app_map.height = mat.rows;
  map_data_.app_map.resolution = map->info.resolution;
  map_data_.app_map.originX = round(map->info.origin.position.x / map_data_.app_map.resolution);
  map_data_.app_map.originY = round(map->info.origin.position.y / map_data_.app_map.resolution);
  map_data_.app_map.data = data_encode;
  map_android_pub_.publish(map_data_.app_map);
  
  skyee_msg::mapInfo map_info;
  map_info.gridWidth = map_data_.app_map.width;
  map_info.gridHeight = map_data_.app_map.height;
  map_info.resolution = map_data_.app_map.resolution;
  map_info.originX = map_data_.app_map.originX;
  map_info.originY = map_data_.app_map.originY;
  mapping_map_info_.publish(map_info);

  map_pub_.publish(*map);
}

std::unique_ptr<nav_msgs::OccupancyGrid> Manager::GetOccupancyGridMap(float resolution)
{
  auto &submap_slices = *(node_->GetMapBuilderBridge()->GetSubmapSlice());
  auto painted_slices = PaintSubmapSlices(submap_slices, resolution);
  return CreateOccupancyGridMsg(painted_slices, resolution, node_options.map_frame, ros::Time::now());
}

void Manager::StartMapping(std::string map_name,uint32_t app_map_size_max)
{
  ROS_INFO("start mapping!map_name:%s",map_name.c_str());
  if(node_)
    node_.reset();
  map_data_.app_map_size_max = app_map_size_max * 10; //png compress will decrease data size tenfold
  slam_state_ = SlamState::SLAM_STATE_MAPPING;
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
  map_publisher_timer_.start();
  robot_pose_sub_ = nh.subscribe("/tracked_pose",1, &Manager::HandleRobotPose,this);
}

void Manager::CloseMapping(std::string map_name)
{
  ROS_INFO("close mapping!map name:%s",map_name.c_str());
  if(!node_)
    return;
  robot_pose_sub_.shutdown();
  map_publisher_timer_.stop();
  node_->FinishAllTrajectories();

  if (!map_name.empty() && !FLAGS_map_folder_path.empty()) 
  {
    node_->RunFinalOptimization();
    SaveMap(map_name);
  }
  node_.reset();
  slam_state_ = SlamState::SLAM_STATE_STANDBY;
}

void Manager::StartLocalization(std::string map_name)
{
  ROS_INFO("start localization!map name:%s",map_name.c_str());
  if(map_name.empty() || FLAGS_map_folder_path.empty())
    return;
  if(node_)
    node_.reset();
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
  aruco_tag_sub_ = nh.subscribe("/aruco_single/pose",1,&Manager::HandleArucoTag,this);
}

void Manager::CloseLocalization(void)
{
  ROS_INFO("close localization!");
  if(node_)
  {
    node_.reset();
    slam_state_ = SlamState::SLAM_STATE_STANDBY;
  }
}

void Manager::SetInitialPose(const geometry_msgs::Pose &initial_pose)
{
  if(!node_)
    return;
  node_->FinishLastTrajectory(); //TODO: delete the last trajectory to avoid memory increase
  TrajectoryOptions initial_pose_trajectory_options_localization = trajectory_options_localization;

  const auto pose = ToRigid3d(initial_pose);
  if (!pose.IsValid()) {
    ROS_ERROR("Invalid pose argument. Orientation quaternion must be normalized.");
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

cv::Mat Manager::OccupancyGridToMat(const nav_msgs::OccupancyGrid &map)
{
#if 1
  cv::Mat colorMat = cv::Mat(map.info.height, map.info.width, CV_8UC3);
  for (unsigned int y = 0; y < map.info.height; y++)
  {
    for (unsigned int x = 0; x < map.info.width; x++)
    {
      cv::Vec3b &rgb = colorMat.at<cv::Vec3b>(y, x);
      unsigned int i = x + (map.info.height - y - 1) * map.info.width;
      if ((map.data[i] >= 0) && (map.data[i] <= 40))        //free space
        rgb[0] = rgb[1] = rgb[2] = 0xFF;//white
      else if ((map.data[i] <= 50) && (map.data[i] > 40))   //
        rgb[0] = rgb[1] = rgb[2] = 0xBE;
      else if ((map.data[i] <= 100) && (map.data[i] > 50))  //obstacle
        rgb[0] = rgb[1] = rgb[2] = 0;//black
      else                                                  //unknow
        rgb[0] = rgb[1] = rgb[2] = 0xd9;
    }
  }
#else
  cv::Mat colorMat = cv::Mat(map.info.height, map.info.width, CV_8U);
  for (unsigned int y = 0; y < map.info.height; y++)
  {
    for (unsigned int x = 0; x < map.info.width; x++)
    {
      unsigned int i = x + (map.info.height - y - 1) * map.info.width;
      if ((map.data[i] >= 0) && (map.data[i] <= 40))
        colorMat.at<uchar>(y, x) = 0xFF;
      else if ((map.data[i] <= 65) && (map.data[i] > 40))
        colorMat.at<uchar>(y, x) = 0xBE;
      else if ((map.data[i] <= 100) && (map.data[i] > 65))
        colorMat.at<uchar>(y, x) = 0;
      else
        colorMat.at<uchar>(y, x) = 0xd9;
    }
  }
#endif
  return colorMat;
}

}  // namespace cartographer_ros