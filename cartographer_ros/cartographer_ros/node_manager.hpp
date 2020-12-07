#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/io/status.h"
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
#include "std_msgs/Bool.h"
#include "ros/wall_timer.h"

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

namespace cartographer_ros {

namespace carto = ::cartographer;
using carto::transform::Rigid3d;
using TrajectoryState = ::cartographer::mapping::PoseGraphInterface::TrajectoryState;
#define slam_state_ ::cartographer::io::slam_state
typedef cartographer::io::SlamState SlamState;

class Manager
{
private:
  boost::shared_ptr<Node> node_;
  NodeOptions node_options;
  TrajectoryOptions trajectory_options,trajectory_options_localization;
  tf2_ros::Buffer* tf_buffer_;
  ros::Subscriber initial_pose_sub_,initial_grid_pose_sub_,robot_pose_sub_,slam_state_sub_,map_sub_;
  ros::Publisher map_android_pub_,mapping_map_info_,robot_grid_pos_pub_,start_pub_,close_pub_;
  ros::NodeHandle nh;
  nav_msgs::OccupancyGrid saved_map_;
  skyee_msg::mapAndroid map_android_;
  uint32_t app_map_size_max_;
  tf::StampedTransform current_pose_;

  void StartMapping(std::string map_name,uint32_t app_map_size_max);
  void CloseMapping(std::string map_name);
  void StartLocalization(std::string map_name);
  void CloseLocalization(void);
  void SetInitialPose(const geometry_msgs::Pose &pose);
  void SaveMap(std::string map_name);
  cv::Mat OccupancyGridToMat(const nav_msgs::OccupancyGrid &map);
  void HandleInitialPose(const geometry_msgs::PoseWithCovarianceStamped &pose_msg);
  void HandleSlamState(const skyee_msg::androidConsole &msg);
  void HandleGetMap(const nav_msgs::OccupancyGrid &map);
  void HandleRobotPose(const geometry_msgs::PoseStamped &pose);
public:
  Manager(tf2_ros::Buffer* tf_buffer)
  {
    tf_buffer_ = tf_buffer;
    std::tie(node_options, trajectory_options) = LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
    std::tie(node_options, trajectory_options_localization) = LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename_localization);
    initial_pose_sub_ = nh.subscribe("/initialpose",1, &Manager::HandleInitialPose,this);
    slam_state_sub_ = nh.subscribe("/scan/cmd/slam_state", 1, &Manager::HandleSlamState,this);
    map_android_pub_ = nh.advertise<skyee_msg::mapAndroid>("/map_android_picture", 1);
    mapping_map_info_ = nh.advertise<skyee_msg::mapInfo>("/mapping_map_info", 1);
    robot_grid_pos_pub_ = nh.advertise<skyee_msg::robotPos>("robot_pos", 1);
    close_pub_ = nh.advertise<std_msgs::Bool>("cartographer_complete", 1, true);
    start_pub_ = nh.advertise<std_msgs::Bool>("cartographer_start", 1, true);
  };
  ~Manager(){}
};

void Manager::HandleInitialPose(const geometry_msgs::PoseWithCovarianceStamped &pose_msg)
{
  SetInitialPose(pose_msg.pose.pose);
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

void Manager::HandleGetMap(const nav_msgs::OccupancyGrid &map)
{
  if(slam_state_ != SlamState::SLAM_STATE_MAPPING)
  {
    map_sub_.shutdown();
    robot_pose_sub_.shutdown();
    return;
  }

  saved_map_ = map;

  // pub map to APP
  cv::Mat mat = OccupancyGridToMat(map);
  float resolution_ratio = sqrt((float)(mat.cols*mat.rows) / app_map_size_max_);
  float new_resolution = map.info.resolution;
  if(resolution_ratio > 1)
  {
    cv::resize(mat,mat,cv::Size(mat.cols/resolution_ratio, mat.rows/resolution_ratio),cv::INTER_NEAREST);
    new_resolution = map.info.resolution * resolution_ratio;
  }
  // ROS_ERROR("size:%d,resolution:%f",mat.cols*mat.rows,map.info.resolution * resolution_ratio);
  // cv::imshow("aa",mat);
  // cv::waitKey(1);
  std::vector<uchar> data_encode;
  cv::imencode(".png", mat, data_encode);
  map_android_.width = mat.cols;
  map_android_.height = mat.rows;
  map_android_.resolution = new_resolution;
  map_android_.originX = round(map.info.origin.position.x / map_android_.resolution);
  map_android_.originY = round(map.info.origin.position.y / map_android_.resolution);;
  map_android_.data = data_encode;
  map_android_pub_.publish(map_android_);
  
  skyee_msg::mapInfo map_info;
  map_info.gridWidth = map_android_.width;
  map_info.gridHeight = map_android_.height;
  map_info.resolution = map_android_.resolution;
  map_info.originX = map_android_.originX;
  map_info.originY = map_android_.originY;
  mapping_map_info_.publish(map_info);
}

void Manager::HandleRobotPose(const geometry_msgs::PoseStamped &pose)
{
  static skyee_msg::robotPos robot_grid_pose;
  robot_grid_pose.angle = tf::getYaw(pose.pose.orientation);
  robot_grid_pose.gridPosition.x = round(pose.pose.position.x / map_android_.resolution);
  robot_grid_pose.gridPosition.y = round(pose.pose.position.y / map_android_.resolution);
  robot_grid_pose.mapInfo.gridWidth = map_android_.width;
  robot_grid_pose.mapInfo.gridHeight = map_android_.height;
  robot_grid_pose.mapInfo.originX = map_android_.originX;
  robot_grid_pose.mapInfo.originY = map_android_.originY;
  robot_grid_pose.mapInfo.resolution = map_android_.resolution;
  robot_grid_pos_pub_.publish(robot_grid_pose);
}

void Manager::StartMapping(std::string map_name,uint32_t app_map_size_max)
{
  ROS_INFO("start mapping!map_name:%s",map_name.c_str());
  if(node_)
    node_.reset();
  slam_state_ = SlamState::SLAM_STATE_MAPPING;
  auto map_builder = cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  node_ = boost::shared_ptr<Node>(new Node(node_options, std::move(map_builder), tf_buffer_,
            FLAGS_collect_metrics));
  if(!map_name.empty() && boost::filesystem::exists(FLAGS_map_folder_path + map_name + "/" + map_name + ".pbstream")) //extend map
    node_->LoadState(FLAGS_map_folder_path + map_name + "/" + map_name + ".pbstream",FLAGS_load_frozen_state);
  if (FLAGS_start_trajectory_with_default_topics) {
    node_->StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  //start uploading map to APP
  app_map_size_max_ = app_map_size_max;
  saved_map_.data.clear();
  map_sub_ = nh.subscribe("/map", 1, &Manager::HandleGetMap,this);
  robot_pose_sub_ = nh.subscribe("/tracked_pose",1, &Manager::HandleRobotPose,this);
}

void Manager::CloseMapping(std::string map_name)
{
  ROS_INFO("close mapping!map name:%s",map_name.c_str());
  if(!node_)
    return;
  map_sub_.shutdown();
  robot_pose_sub_.shutdown();
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
  slam_state_ = SlamState::SLAM_STATE_LOCATING;
  
  auto map_builder = cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  node_ = boost::shared_ptr<Node>(new Node(node_options, std::move(map_builder), tf_buffer_,
            FLAGS_collect_metrics));
  node_->LoadState(FLAGS_map_folder_path + map_name + "/" + map_name + ".pbstream",
                  FLAGS_load_frozen_state);
  if (FLAGS_start_trajectory_with_default_topics) {
    node_->StartTrajectoryWithDefaultTopics(trajectory_options_localization);
  }
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
  ROS_DEBUG("map_name_folder_path:%s",map_name_folder_path.c_str());
  if(access(map_name_folder_path.c_str(),0) != 0)
    mkdir(map_name_folder_path.c_str(),S_IRWXU | S_IRWXG | S_IRWXO);
  node_->SerializeState(map_name_folder_path + "/" + map_name + ".pbstream",true /* include_unfinished_submaps */);
  
  /*******************save png****************/
  if(saved_map_.data.empty())
    ROS_ERROR("saved_map data is empty!");
  cv::Mat colorMat = OccupancyGridToMat(saved_map_);
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
          (map_name + ".png").c_str(), saved_map_.info.resolution, saved_map_.info.width, saved_map_.info.height,
          saved_map_.info.origin.position.x, saved_map_.info.origin.position.y, tf::getYaw(saved_map_.info.origin.orientation));
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
      if ((map.data[i] >= 0) && (map.data[i] <= 40))
        rgb[0] = rgb[1] = rgb[2] = 0xFF;
      else if ((map.data[i] <= 65) && (map.data[i] > 40))
        rgb[0] = rgb[1] = rgb[2] = 0xBE;
      else if ((map.data[i] <= 100) && (map.data[i] > 65))
        rgb[0] = rgb[1] = rgb[2] = 0;
      else
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