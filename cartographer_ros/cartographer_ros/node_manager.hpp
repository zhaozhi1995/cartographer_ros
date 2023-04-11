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
#include <mutex>
#include "lidar_localizer/LocalizerPostion.h"
#include <movexbot_msgs/SlamStatus.h>


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
DEFINE_bool(use_lidar_relocalizer, false,
            "whether use lidar realocalizer to relocate the robot in the whole map");
DEFINE_bool(is_check_location_lost, true,
            "whether check location lost");

namespace cartographer_ros {

using ::cartographer::transform::Rigid3d;
using TrajectoryState = ::cartographer::mapping::PoseGraphInterface::TrajectoryState;
using ::cartographer::io::SlamState;
using ::cartographer::mapping::SubmapId;
using ::cartographer::io::SubmapSlice;
//TODO: 和上传的slam状态同步
#define slam_state_ ::cartographer::io::slam_state
#define MapNameFolderPath(map_name) FLAGS_map_folder_path + map_name + "/"

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
  ros::Subscriber initial_pose_sub_,app_initial_pose_sub_,emergency_stop_button_sub_,robot_pose_sub_,slam_state_sub_,dock_station_sub_;
  ros::Publisher map_android_pub_,robot_grid_pos_pub_,map_pub_,slam_state_pub_,slam_status_pub_;
  ros::ServiceServer relocalization_service_,slam_cmd_service_;
  ros::ServiceClient lidar_localizer_client_;
  geometry_msgs::PoseStamped robot_pose_;
  ros::NodeHandle nh_;
  MapData_t map_data_;
  tf::TransformListener listener_;
  StationID dock_station_;
  std::recursive_mutex node_mutex_;
  ros::WallTimer map_publish_timer_,slam_staus_pub_timer_;
  bool flag_emergency_stop_;
  
  void StartMapping(std::string map_name,geometry_msgs::Pose initial_pose = geometry_msgs::Pose());
  void CloseMapping(std::string map_name);
  void StartLocalization(std::string map_name,geometry_msgs::Pose initial_pose = geometry_msgs::Pose());
  void CloseLocalization(void);
  void SetInitialPose(const geometry_msgs::Pose &pose);
  void SetLocalizationSucceed(void);
  void SetLocalizationWeak(void);
  void CancelLocalizationSucceed(void);
  void LocalizationLost(void);
  void SaveMap(std::string map_name);
  std::unique_ptr<nav_msgs::OccupancyGrid> GetOccupancyGridMap(float resolution);
  bool RelocateWithDockMarker(const aruco_msgs::Marker &marker);
  void HandleInitialPose(const geometry_msgs::PoseWithCovarianceStamped &pose_msg);
  void HandleAppInitialPose(const movexbot_msgs::destination_point &pose_msg);
  void EmergencyStopButton(const std_msgs::Bool &state);
  void HandleRobotPose(const geometry_msgs::PoseStamped &pose);
  void HandleDockStation(const aruco_msgs::MarkerArray &markers);
  void MapPublish(const ros::WallTimerEvent& event);
  void SlamStatusPublish(const ros::WallTimerEvent& event);
  bool RelocalizationService(movexbot_msgs::Relocalization::Request &req,movexbot_msgs::Relocalization::Response &res);
  bool SetSlamCmdService(movexbot_msgs::SetSlamCmd::Request &req,movexbot_msgs::SetSlamCmd::Response &res);
public:
  Manager(tf2_ros::Buffer* tf_buffer):flag_emergency_stop_(false)
  {
    dock_station_.SetMapFolderPath(FLAGS_map_folder_path);
    tf_buffer_ = tf_buffer;
    std::tie(node_options, trajectory_options) = LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
    std::tie(node_options, trajectory_options_localization) = LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename_localization);
    initial_pose_sub_ = nh_.subscribe("/initialpose",1, &Manager::HandleInitialPose,this);
    app_initial_pose_sub_ = nh_.subscribe("/nav/cmd/initPose", 1, &Manager::HandleAppInitialPose, this);
    emergency_stop_button_sub_ = nh_.subscribe("/robot_state/scram_button",1,&Manager::EmergencyStopButton,this);
    robot_pose_sub_ = nh_.subscribe("/tracked_pose",1, &Manager::HandleRobotPose,this);
    dock_station_sub_ = nh_.subscribe("/aruco_marker_publisher/markers",1,&Manager::HandleDockStation,this);

    slam_state_pub_ = nh_.advertise<movexbot_msgs::androidConsole>("/scan/cmd/slam_state",1,true);
    slam_status_pub_ = nh_.advertise<movexbot_msgs::SlamStatus>("/slam_status",1,true);
    map_android_pub_ = nh_.advertise<movexbot_msgs::mapAndroid>("/map_android_picture", 1);
    robot_grid_pos_pub_ = nh_.advertise<movexbot_msgs::robotPos>("robot_pos", 1);
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    relocalization_service_ = nh_.advertiseService("slam/relocalization_service",&Manager::RelocalizationService,this);
    slam_cmd_service_ = nh_.advertiseService("slam/set_cmd",&Manager::SetSlamCmdService,this);
    // map_publish_timer_ = nh_.createWallTimer(ros::WallDuration(1.0),&Manager::MapPublish,this);
    slam_staus_pub_timer_ = nh_.createWallTimer(ros::WallDuration(1.0),&Manager::SlamStatusPublish,this);

    lidar_localizer_client_ = nh_.serviceClient<lidar_localizer::LocalizerPostion>("/localizer");

    ros::Duration(0.5).sleep();
    CloseLocalization();
  };
  ~Manager(){}
};

bool Manager::SetSlamCmdService(movexbot_msgs::SetSlamCmd::Request &req,movexbot_msgs::SetSlamCmd::Response &res)
{  
  LOG(INFO) << GetFormatString("req.cmd:%d,slam_state:%d,map_name:%s",(int)req.cmd,(int)slam_state_,req.map_name.c_str());
  switch (req.cmd)
  {
    case req.SLAM_CMD_STANDBY:
      {
        if(slam_state_ == SlamState::SLAM_STATE_MAPPING)
          CloseMapping(req.map_name);
        else if(slam_state_ == SlamState::SLAM_STATE_LOCATING || slam_state_ == SlamState::SLAM_STATE_RELOCATING || slam_state_ == SlamState::SLAM_STATE_LOCATE_SUCCEED)
          CloseLocalization();
      }
      break;
    case req.SLAM_CMD_LOCATING:
      StartLocalization(req.map_name);
      break;

    case req.SLAM_CMD_LOCATE_SUCCEED:
      SetLocalizationSucceed();
      break;

    case req.SLAM_CMD_CANCEL_LOCATE_SUCCEED:
      CancelLocalizationSucceed();
      break;

    case req.SLAM_CMD_MAPPING:
      {
        if(map_data_.map_name == req.map_name && slam_state_ == SlamState::SLAM_STATE_LOCATE_SUCCEED)
        {
          // geometry_msgs::PoseStampedConstPtr robot_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/tracked_pose", ros::Duration(1.0)); //TODO:获取不到数据
          // if(robot_pose)
          //   StartMapping(req.map_name, robot_pose->pose);
          // else
          // {
          //   res.state = res.STATE_ERROR;
          //   LOG(ERROR) << "con't get robot pose";
          // }
          StartMapping(req.map_name, robot_pose_.pose);
        }
        else
          StartMapping(req.map_name);
        break;
      }
    
    default:
      break;
  }
  res.state = res.STATE_SUCCESS;
  return true;
}

bool Manager::RelocalizationService(movexbot_msgs::Relocalization::Request &req,movexbot_msgs::Relocalization::Response &res)
{
  if(req.robot_pose.theta == 0 && req.robot_pose.x == 0 && req.robot_pose.y == 0)
  {
    LOG(INFO) << "relocalization with marker...";
    aruco_msgs::MarkerArrayConstPtr markers = ros::topic::waitForMessage<aruco_msgs::MarkerArray>("/aruco_marker_publisher/markers", ros::Duration(2.0));
    if(markers)
    {
      LOG(INFO) << "have found tag station,station id:" << markers->markers.front().id;
      if(RelocateWithDockMarker(markers->markers.front()))
        res.state = res.STATE_SUCCESS;
      else
        res.state = res.STATE_ERROR;
      // auto station = dock_station_.FindStation(markers->markers.front().id);
      // if(station.first)
      // {
      //   LOG(INFO) << GetFormatString("current map name:%s,station map name:%s",map_data_.map_name.c_str(),station.second.map_name.c_str());
      //   if(map_data_.map_name != station.second.map_name)
      //   {
      //     StartLocalization(station.second.map_name);
      //     ros::Duration(2.0).sleep();
      //   }
      //   if(listener_.waitForTransform("/base_link", markers->header.frame_id, markers->header.stamp, ros::Duration(1.0)))
      //   {
      //     geometry_msgs::PoseStamped tag_pose;
      //     tag_pose.header = markers->header;
      //     tag_pose.pose = markers->markers.front().pose.pose;
      //     listener_.transformPose("/base_link",tag_pose,tag_pose);
      //     tag_pose.pose.position.z = 0;
      //     tag_pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(tag_pose.pose.orientation));
      //     tf::Transform tag_to_base_link,tag_to_map(tf::createQuaternionFromYaw(station.second.pose.theta),tf::Vector3(station.second.pose.x,station.second.pose.y,0));
      //     tf::poseMsgToTF(tag_pose.pose,tag_to_base_link);
      //     // tf::poseMsgToTF(station.second,tag_to_map);
      //     tf::Transform new_base_link_to_map = tag_to_map * tag_to_base_link.inverse();
      //     geometry_msgs::Pose new_base_link_pose;
      //     tf::poseTFToMsg(new_base_link_to_map,new_base_link_pose);
      //     SetInitialPose(new_base_link_pose);
      //     SetLocalizationSucceed();
      //     res.state = res.STATE_SUCCESS;
      //   }
      //   else
      //   {
      //     LOG(ERROR) << "can't get base_link tf data";
      //     res.state = res.STATE_ERROR;
      //   }
      // }
      // else
      // {
      //   LOG(INFO) << "no station record";
      //   res.state = res.STATE_ERROR;
      // }
    }
    else
    {
      LOG(ERROR) << "have found no markers";
      if(FLAGS_use_lidar_relocalizer)
      {
        LOG(INFO) << "relocalization in the whole map...";
        lidar_localizer::LocalizerPostion lidar_localizer_srv;
        lidar_localizer_srv.request.is_global = true;
        lidar_localizer_srv.request.min_score = 0.75;
        if(lidar_localizer_client_.call(lidar_localizer_srv))
        {
          if(lidar_localizer_srv.response.pose_found)
          {
            res.state = res.STATE_SUCCESS;
            LOG(INFO) << "found lidar relocate pose: " << lidar_localizer_srv.response.pose.pose << ",score: " << lidar_localizer_srv.response.score;
            SetInitialPose(lidar_localizer_srv.response.pose.pose);
            {
              ros::Rate loop(100);
              ros::Time start_time = ros::Time::now();
              while (ros::ok())
              {
                if((ros::Time::now() - start_time).toSec() > 2.0)
                  break;
                ros::spinOnce();
                loop.sleep();
              }
            }
            SetLocalizationSucceed();
            res.state = res.STATE_SUCCESS;
            return true;
          }
          else
          {
            LOG(INFO) << "relocalization in the whole map failed";
            res.state = res.STATE_ERROR;
          }
        }
        else
        {
          LOG(ERROR) << "lidar_localizer_srv error";
          res.state = res.STATE_ERROR;
        }
        // if(slam_state_ == SlamState::SLAM_STATE_LOCATE_SUCCEED)
        //   CancelLocalizationSucceed();
      }
      else
        res.state = res.STATE_ERROR;
    }
    return true;
  }

  LOG(INFO) << "relocalization with initial pose...";
  geometry_msgs::Pose initial_pose;
  initial_pose.position.x = req.robot_pose.x * map_data_.map_info.resolution;
  initial_pose.position.y = req.robot_pose.y * map_data_.map_info.resolution;
  initial_pose.orientation = tf::createQuaternionMsgFromYaw(req.robot_pose.theta);

  lidar_localizer::LocalizerPostion lidar_localizer_srv;
  lidar_localizer_srv.request.is_global = false;
  lidar_localizer_srv.request.linear_search_window = 2.0; //m
  lidar_localizer_srv.request.angular_search_window = 0.79; //45 degree
  lidar_localizer_srv.request.init_pose.pose = initial_pose;
  lidar_localizer_srv.request.min_score = 0.6;
  if(lidar_localizer_client_.call(lidar_localizer_srv))
  {
    if(lidar_localizer_srv.response.pose_found)
    {
      res.state = res.STATE_SUCCESS;
      LOG(INFO) << "found lidar relocate pose: " << lidar_localizer_srv.response.pose.pose << ",score: " << lidar_localizer_srv.response.score;
      SetInitialPose(lidar_localizer_srv.response.pose.pose);
      {
        ros::Rate loop(100);
        ros::Time start_time = ros::Time::now();
        while (ros::ok())
        {
          if((ros::Time::now() - start_time).toSec() > 2.0)
            break;
          ros::spinOnce();
          loop.sleep();
        }
      }
      SetLocalizationSucceed();
      res.state = res.STATE_SUCCESS;
      return true;
    }
    else
    {
      LOG(INFO) << "relocalization in the whole map failed";
      res.state = res.STATE_ERROR;
      return true;
    }
  }
  else
  {
    LOG(ERROR) << "lidar_localizer_srv error";
    res.state = res.STATE_ERROR;
    return true;
  }

  //TODO:增加对定位状态的判断，是否产生约束，如产生正确约束并跳转位置，就直接返回成功，如超过3s未产生正确约束并跳转位置就返回失败
  // ros::Rate loop(100);
  // ros::Time start_time = ros::Time::now();
  // while (ros::ok())
  // {
  //   if((ros::Time::now() - start_time).toSec() > 2.0)
  //     break;
  //   ros::spinOnce();
  //   loop.sleep();
  // }
  // if(slam_state_ == SlamState::SLAM_STATE_LOCATING || slam_state_ == SlamState::SLAM_STATE_RELOCATING)
  //   SetLocalizationSucceed();
  return true;
}

bool Manager::RelocateWithDockMarker(const aruco_msgs::Marker &marker)
{
  std::string check_station_file_path = MapNameFolderPath(map_data_.map_name) + "Station/" + std::to_string(marker.id) + ".yaml";
  if(access(check_station_file_path.c_str(), F_OK) == 0)
  {
    LOG(INFO) << "found station file in current map: " << map_data_.map_name;
    tf::Transform tag_to_map;
    try
    {
      YAML::Node station_yaml = YAML::LoadFile(check_station_file_path);
      tag_to_map.setRotation(tf::createQuaternionFromYaw(station_yaml[POSE_THETA].as<double>()));
      tag_to_map.setOrigin(tf::Vector3(station_yaml[POSE_X].as<double>(),station_yaml[POSE_Y].as<double>(),0));
    }
    catch(const std::exception& e)
    {
      LOG(ERROR) << e.what();
      return false;
    }

    if(listener_.waitForTransform("/base_link", marker.header.frame_id, marker.header.stamp, ros::Duration(1.0)))
    {
      geometry_msgs::PoseStamped tag_pose;
      tag_pose.header = marker.header;
      tag_pose.pose = marker.pose.pose;
      listener_.transformPose("/base_link",tag_pose,tag_pose);
      tag_pose.pose.position.z = 0;
      tag_pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(tag_pose.pose.orientation));
      tf::Transform tag_to_base_link;
      tf::poseMsgToTF(tag_pose.pose,tag_to_base_link);
      tf::Transform new_base_link_to_map = tag_to_map * tag_to_base_link.inverse();
      geometry_msgs::Pose new_base_link_pose;
      tf::poseTFToMsg(new_base_link_to_map,new_base_link_pose);
      SetInitialPose(new_base_link_pose);
      SetLocalizationSucceed();
      LOG(INFO) << "SetLocalizationSucceed: " << new_base_link_pose;
    }
    else
    {
      LOG(ERROR) << "can't get base_link tf data";
      return false;
    }
  }
  else
  {
    auto station = dock_station_.FindStation(marker.id);
    if(station.first)
    {
      LOG(INFO) << GetFormatString("current map name:%s,station map name:%s",map_data_.map_name.c_str(),station.second.map_name.c_str());
      StartLocalization(station.second.map_name);
      ros::Duration(2.0).sleep();
    }
    else
      return false;
  }
  return true;
}

void Manager::HandleInitialPose(const geometry_msgs::PoseWithCovarianceStamped &pose_msg)
{
  SetInitialPose(pose_msg.pose.pose);
  SetLocalizationSucceed();
}

//TODO:当前坐标点为图像像素坐标，需修改为map坐标系,(useless?)
void Manager::HandleAppInitialPose(const movexbot_msgs::destination_point &pose_msg)
{
  ROS_INFO("set initial_pose:(%d,%d,%f) by app...",pose_msg.gridPosition.x,pose_msg.gridPosition.y,pose_msg.angle);
  geometry_msgs::Pose initial_pose;
  initial_pose.position.x = pose_msg.gridPosition.x * map_data_.map_info.resolution + map_data_.map_info.origin.position.x;
  initial_pose.position.y = pose_msg.gridPosition.y * map_data_.map_info.resolution + map_data_.map_info.origin.position.y;
  initial_pose.orientation = tf::createQuaternionMsgFromYaw(pose_msg.angle);
  SetInitialPose(initial_pose);
  //TODO: 增加等待时间,以便立即产生约束以及纠正精确位置
  SetLocalizationSucceed();
}

void Manager::EmergencyStopButton(const std_msgs::Bool &state)
{
  if(!state.data && flag_emergency_stop_ && slam_state_ == SlamState::SLAM_STATE_LOCATING)
    dock_station_sub_ = nh_.subscribe("/aruco_marker_publisher/markers",1,&Manager::HandleDockStation,this);
  flag_emergency_stop_ = state.data;
  if (state.data && slam_state_ == SlamState::SLAM_STATE_LOCATE_SUCCEED)
  {
    LOG(INFO) << "Emergency stop!!";
    CancelLocalizationSucceed();
  }
}

void Manager::HandleRobotPose(const geometry_msgs::PoseStamped &pose)
{
  robot_pose_ = pose;
  /* if(slam_state_ == SlamState::SLAM_STATE_MAPPING)
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
  else  */if(slam_state_ == SlamState::SLAM_STATE_LOCATE_SUCCEED)
  {
    static int last_check_node_count = 0;
    if(cartographer::io::invalid_node_count > 100)
    {
      if(!last_check_node_count)
        SetLocalizationWeak();
      if(cartographer::io::invalid_node_count - last_check_node_count > 20)
      {
        last_check_node_count = cartographer::io::invalid_node_count;
        LOG(INFO) << "lost location when runing,check location around the current pose: " << pose;
        lidar_localizer::LocalizerPostion lidar_localizer_srv;
        lidar_localizer_srv.request.is_global = false;
        lidar_localizer_srv.request.linear_search_window = 1.0; //m
        lidar_localizer_srv.request.angular_search_window = 0.52; //30 degree
        lidar_localizer_srv.request.init_pose = pose;
        lidar_localizer_srv.request.min_score = 0.55;
        if(lidar_localizer_client_.call(lidar_localizer_srv))
        {
          if(lidar_localizer_srv.response.pose_found)
          {
            LOG(INFO) << "found lidar relocate pose: " << lidar_localizer_srv.response.pose.pose << ",score: " << lidar_localizer_srv.response.score;
            if(FLAGS_is_check_location_lost)
            {
              Position_t current_pos(pose.pose.position.x,pose.pose.position.y,tf::getYaw(pose.pose.orientation));
              Position_t calculate_pos(lidar_localizer_srv.response.pose.pose.position.x,lidar_localizer_srv.response.pose.pose.position.y,tf::getYaw(lidar_localizer_srv.response.pose.pose.orientation));
              double distance_diff = DistanceToDest(current_pos,calculate_pos);
              double orientation_diff = OrientationDiff(current_pos.orientation,calculate_pos.orientation);
              LOG(INFO) << "distance diif: " << distance_diff << ",orientation_diff: " << orientation_diff;
              if(FLAGS_is_check_location_lost && (distance_diff > 0.2 || orientation_diff > 0.35))
              {
                CancelLocalizationSucceed();
                SetInitialPose(lidar_localizer_srv.response.pose.pose);
                SetLocalizationSucceed();
              }
            }
            return;
          }
          else
            LOG(INFO) << "relocalization around the current pose failed";
        }
        else
          LOG(ERROR) << "lidar_localizer_srv error";
      }
    }
    else if(last_check_node_count)
    {
      last_check_node_count = 0;
      SetLocalizationSucceed();
    }
  }
}

void Manager::HandleDockStation(const aruco_msgs::MarkerArray &markers) //TODO: consider move to navigation
{
  if((slam_state_ == SlamState::SLAM_STATE_LOCATING || slam_state_ == SlamState::SLAM_STATE_STANDBY) && !flag_emergency_stop_)
  {
    LOG(INFO) << "have found tag station,station id:" << markers.markers.front().id;
    if(!RelocateWithDockMarker(markers.markers.front()))
      dock_station_sub_.shutdown();
  }
  else
  {
    LOG(INFO) << "slam_state is not SLAM_STATE_LOCATING or SLAM_STATE_STANDBY: " << slam_state_;
    LOG(INFO) << "flag_emergency_stop_: " << flag_emergency_stop_;
    dock_station_sub_.shutdown();
  }
}

void Manager::MapPublish(const ros::WallTimerEvent& event)
{
  if(slam_state_ != SlamState::SLAM_STATE_MAPPING)
    return;
  if (!node_mutex_.try_lock())
  {
    LOG(ERROR) << "try lock failed,return";
    return;
  }
//LOG(ERROR) << "test";
  std::lock_guard<std::recursive_mutex> lock(node_mutex_, std::adopt_lock);
//LOG(ERROR) << "test";
  ros::Time start = ros::Time::now();
  std::unique_ptr<nav_msgs::OccupancyGrid> map = GetOccupancyGridMap(map_data_.mapping_resolution);
  if(map->data.empty())
  {
    ROS_ERROR("empty data of map!!");
    return;
  }
//LOG(ERROR) << "test";
  //update mapping resolution
  // ROS_ERROR("map size:(%d,%d)%d,resolution:%f",map->info.width,map->info.height,map->data.size(),map->info.resolution);
  if(map->data.size() > map_data_.app_map_size_max)
  {
    map_data_.mapping_resolution += 0.01;
    // ROS_INFO("update resolution:%f",map_data_.mapping_resolution);
  }

  cv::Mat mat = OccupancyGridToMat(*map);
  ROS_INFO("time cost:%f",(ros::Time::now() - start).toSec());
  start = ros::Time::now();
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
  //LOG(ERROR) << "test";
  static std::vector<int> compression_params = {CV_IMWRITE_PNG_COMPRESSION,5};
  std::vector<uchar> data_encode;
  start = ros::Time::now();
  cv::imencode(".png", mat, data_encode,compression_params);
  ROS_WARN("time cost1:%f",(ros::Time::now() - start).toSec());
  start = ros::Time::now();
  map_data_.app_map.width = mat.cols;
  map_data_.app_map.height = mat.rows;
  map_data_.app_map.resolution = map->info.resolution;
  map_data_.app_map.originX = round(map->info.origin.position.x / map_data_.app_map.resolution);
  map_data_.app_map.originY = round(map->info.origin.position.y / map_data_.app_map.resolution);
  map_data_.app_map.data = data_encode;
  map_android_pub_.publish(map_data_.app_map);
  ROS_WARN("time cost2:%f",(ros::Time::now() - start).toSec());
  if(map_pub_.getNumSubscribers() > 0)
  {
    start = ros::Time::now();
    map = GetOccupancyGridMap(FLAGS_resolution);
    map_pub_.publish(*map);
    ROS_INFO("time cost4:%f",(ros::Time::now() - start).toSec());
  }
//LOG(ERROR) << "test";
}

void Manager::SlamStatusPublish(const ros::WallTimerEvent& event)
{
  movexbot_msgs::SlamStatus slam_status;
  slam_status.slam_state = slam_state_;
  slam_status.map_name = map_data_.map_name;
  slam_status_pub_.publish(slam_status);
}

std::unique_ptr<nav_msgs::OccupancyGrid> Manager::GetOccupancyGridMap(float resolution)
{
  auto &submap_slices = *(node_->GetMapBuilderBridge()->GetSubmapSlice());
  auto painted_slices = PaintSubmapSlices(submap_slices, resolution);
  return CreateOccupancyGridMsg(painted_slices, resolution, node_options.map_frame, ros::Time::now());
}

void Manager::StartMapping(std::string map_name,geometry_msgs::Pose initial_pose)
{
  LOG(INFO) << "start mapping!map_name:" << map_name;
  LOG(INFO) << "inital_pose:" << initial_pose;
  std::lock_guard<std::recursive_mutex> lock(node_mutex_);
  LOG(INFO) << "lock out";
  if(node_)
  {
    //LOG(ERROR) << "test";
    node_->FinishAllTrajectories();
    LOG(ERROR) << "FinishAllTrajectories...";
    node_->WaitFinishAllTrajectories();
    LOG(INFO) << "reset node ...";
    node_.reset();
    LOG(INFO) << "reset node done";
  }
  //LOG(ERROR) << "test";
  map_data_.app_map_size_max = 200000; //png compress will decrease data size tenfold
  auto map_builder = cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  node_ = boost::shared_ptr<Node>(new Node(node_options, std::move(map_builder), tf_buffer_,FLAGS_collect_metrics));
  if(!map_name.empty() && boost::filesystem::exists(MapNameFolderPath(map_name) + map_name + ".pbstream")) //extend map
  {
    node_->LoadState(MapNameFolderPath(map_name) + map_name + ".pbstream",FLAGS_load_frozen_state);
    std::unique_ptr<nav_msgs::OccupancyGrid> map = GetOccupancyGridMap(FLAGS_resolution);
    map_data_.mapping_resolution = sqrt((float)map->data.size() / map_data_.app_map_size_max) * FLAGS_resolution;
  }
  else
    map_data_.mapping_resolution = FLAGS_resolution;
  LOG(INFO) << "add trajectory...";
  auto mapping_trajectory_options = trajectory_options;
  const auto pose = ToRigid3d(initial_pose);
  if (pose.IsValid()) {
    ::cartographer::mapping::proto::InitialTrajectoryPose initial_trajectory_pose;
    initial_trajectory_pose.set_to_trajectory_id(0);
    *initial_trajectory_pose.mutable_relative_pose() = cartographer::transform::ToProto(pose);
    initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(::cartographer_ros::FromRos(ros::Time(0))));
    *mapping_trajectory_options.trajectory_builder_options.mutable_initial_trajectory_pose() = initial_trajectory_pose;
    LOG(INFO) << "start mapping with initial pose";
  }
  node_->StartTrajectoryWithDefaultTopics(mapping_trajectory_options);
  slam_state_ = SlamState::SLAM_STATE_MAPPING;
  
  movexbot_msgs::androidConsole slam_state_msg;
  slam_state_msg.model.data = "mapping_start";
  slam_state_msg.name.data = map_name;
  slam_state_pub_.publish(slam_state_msg);
  LOG(INFO) << "start mapping end";
  //TODO:start record rosbag (code is in navigation pkg now)
}

void Manager::CloseMapping(std::string map_name)
{
  LOG(INFO) << "close mapping!map name:" << map_name.c_str();
  std::lock_guard<std::recursive_mutex> lock(node_mutex_);
  LOG(WARNING) << "lock out";
  if(!node_)
  {
    LOG(WARNING) << "node_ is null";
    return;
  }
//LOG(ERROR) << "test";
  slam_state_ = SlamState::SLAM_STATE_STANDBY;
  node_->FinishAllTrajectories();
  //LOG(ERROR) << "test";
  node_->WaitFinishAllTrajectories();
//LOG(ERROR) << "test";
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
  {
    StartLocalization(map_name,robot_pose_.pose);
    SetLocalizationSucceed();
  }
}

void Manager::StartLocalization(std::string map_name,geometry_msgs::Pose initial_pose)
{
  LOG(INFO) << "start localization!map name:" << map_name;
  std::lock_guard<std::recursive_mutex> lock(node_mutex_);
  LOG(WARNING) << "lock out";
  if(map_name.empty() || FLAGS_map_folder_path.empty())
    return;
  if(node_)
  {
    node_->FinishAllTrajectories();
    //LOG(ERROR) << "test";
    node_->WaitFinishAllTrajectories();
    LOG(INFO) << "reset node ...";
    node_.reset();
    LOG(INFO) << "reset node done";
  }
  if(!boost::filesystem::exists(MapNameFolderPath(map_name) + map_name + ".pbstream"))
  {
    ROS_ERROR("the pbstream file is not exist,return!");
    return;
  }
//LOG(ERROR) << "test";
  map_data_.map_name = map_name;
  slam_state_ = SlamState::SLAM_STATE_LOCATING;
  
  auto map_builder = cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  node_ = boost::shared_ptr<Node>(new Node(node_options, std::move(map_builder), tf_buffer_,
            FLAGS_collect_metrics));
  node_->LoadState(MapNameFolderPath(map_name) + map_name + ".pbstream",
                  FLAGS_load_frozen_state);
  std::unique_ptr<nav_msgs::OccupancyGrid> map = GetOccupancyGridMap(FLAGS_resolution);
  map_data_.map_info = map->info;
//LOG(ERROR) << "test";
  if (FLAGS_start_trajectory_with_default_topics) {
    const auto pose = ToRigid3d(initial_pose);
    if (pose.IsValid()) {
      ::cartographer::mapping::proto::InitialTrajectoryPose initial_trajectory_pose;
      initial_trajectory_pose.set_to_trajectory_id(0);
      *initial_trajectory_pose.mutable_relative_pose() = cartographer::transform::ToProto(pose);
      initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(::cartographer_ros::FromRos(ros::Time(0))));
      *trajectory_options_localization.trajectory_builder_options.mutable_initial_trajectory_pose() = initial_trajectory_pose;
    }
    node_->StartTrajectoryWithDefaultTopics(trajectory_options_localization);
  }
// LOG(ERROR) << "test";
  dock_station_sub_ = nh_.subscribe("/aruco_marker_publisher/markers",1,&Manager::HandleDockStation,this);
// LOG(ERROR) << "test";
  movexbot_msgs::androidConsole slam_state_msg;
  slam_state_msg.model.data = "localization_start";
  slam_state_msg.name.data = map_name;
  slam_state_pub_.publish(slam_state_msg);
  ros::Duration(1.0).sleep(); //TODO:为了等待上传到app的信号改变，后续优化涉及到和navigation中的slam状态同步问题
  map_pub_.publish(*map);
}

void Manager::SetLocalizationSucceed(void)
{
  LOG(INFO) << "Set localization succeed";
  slam_state_ = SlamState::SLAM_STATE_LOCATE_SUCCEED;

  movexbot_msgs::androidConsole slam_state_msg;
  slam_state_msg.model.data = "localization_success";
  slam_state_msg.name.data = map_data_.map_name;
  slam_state_pub_.publish(slam_state_msg);
  dock_station_sub_.shutdown();
}

void Manager::SetLocalizationWeak(void)
{
  LOG(INFO) << "Set localization weak";
  // slam_state_ = SlamState::SLAM_STATE_LOCATE_WEAK;

  movexbot_msgs::androidConsole slam_state_msg;
  slam_state_msg.model.data = "localization_weak";
  slam_state_msg.name.data = map_data_.map_name;
  slam_state_pub_.publish(slam_state_msg);
  dock_station_sub_.shutdown();
}

void Manager::CancelLocalizationSucceed(void)
{
  LOG(INFO) << "Cancel localization succeed";
  slam_state_ = SlamState::SLAM_STATE_LOCATING;

  movexbot_msgs::androidConsole slam_state_msg;
  slam_state_msg.model.data = "cancel_localization_success";
  slam_state_msg.name.data = map_data_.map_name;
  slam_state_pub_.publish(slam_state_msg);
  
  dock_station_sub_ = nh_.subscribe("/aruco_marker_publisher/markers",1,&Manager::HandleDockStation,this);
}

void Manager::LocalizationLost(void)
{
  LOG(INFO) << "Localization lost";
  slam_state_ = SlamState::SLAM_STATE_LOCATING;

  movexbot_msgs::androidConsole slam_state_msg;
  slam_state_msg.model.data = "localization_lost";
  slam_state_msg.name.data = map_data_.map_name;
  slam_state_pub_.publish(slam_state_msg);
  
  dock_station_sub_ = nh_.subscribe("/aruco_marker_publisher/markers",1,&Manager::HandleDockStation,this);
}

void Manager::CloseLocalization(void)
{
  LOG(ERROR) << "close localization!";
  std::lock_guard<std::recursive_mutex> lock(node_mutex_);
  LOG(WARNING) << "lock out";
  if(node_)
  {
    node_->FinishAllTrajectories();
    node_->WaitFinishAllTrajectories();
    LOG(INFO) << "reset node ...";
    node_.reset();
    LOG(INFO) << "reset node done";
    slam_state_ = SlamState::SLAM_STATE_STANDBY;
  }

  movexbot_msgs::androidConsole slam_state_msg;
  slam_state_msg.model.data = "localization_close";
  slam_state_pub_.publish(slam_state_msg);
  dock_station_sub_.shutdown();
  LOG(ERROR) << "localization_close";
}

void Manager::SetInitialPose(const geometry_msgs::Pose &initial_pose)
{
  if(!node_)
    return;
  LOG(INFO) << GetFormatString("initial_pose:(%f,%f,%f)",initial_pose.position.x,initial_pose.position.y,tf::getYaw(initial_pose.orientation));
  slam_state_ = SlamState::SLAM_STATE_RELOCATING;
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

void UpdataVirtualWallPointCoor(const std::string &map_name, const Coordinate_t &png_coordinate_add)
{
  std::string virtualwall_folder_path = MapNameFolderPath(map_name) + "VirtualWall/";
  vector<std::string> virtualwall_file_names = GetFileNameListInFolder(virtualwall_folder_path,".json");
  for(auto &file_name:virtualwall_file_names)
  {
    nlohmann::json json;
    std::ifstream(virtualwall_folder_path + file_name) >> json;
    for(auto &path:json["pathList"])
    {
      path["gridPositions"][0]["x"] = path["gridPositions"][0]["x"].get<int>() + png_coordinate_add.x;
      path["gridPositions"][0]["y"] = path["gridPositions"][0]["y"].get<int>() + png_coordinate_add.y;
      path["gridPositions"][1]["x"] = path["gridPositions"][1]["x"].get<int>() + png_coordinate_add.x;
      path["gridPositions"][1]["y"] = path["gridPositions"][1]["y"].get<int>() + png_coordinate_add.y;
    }
    std::ofstream file(virtualwall_folder_path + file_name);
    if(!file.is_open())
    {
      LOG(ERROR) << "Open file failed";
      continue;
    }
    file << json.dump(2);
  }
}

void UpdataTaskPointCoor(const std::string &map_name, const Coordinate_t &png_coordinate_add)
{
  std::string task_folder_path = MapNameFolderPath(map_name) + "Task/";
  vector<std::string> task_file_names = GetFileNameListInFolder(task_folder_path,".json");
  for(auto &file_name:task_file_names)
  {
    nlohmann::json json;
    std::ifstream(task_folder_path + file_name) >> json;
    for(auto &task:json)
    {
      for(auto &pose:task["pose_list"])
      {
        pose["x"] = pose["x"].get<int>() + png_coordinate_add.x;
        pose["y"] = pose["y"].get<int>() + png_coordinate_add.y;
      }
      for(auto &arc_segment:task["arc_segment_list"])
      {
        arc_segment["start_pose"]["x"] = arc_segment["start_pose"]["x"].get<int>() + png_coordinate_add.x;
        arc_segment["start_pose"]["y"] = arc_segment["start_pose"]["y"].get<int>() + png_coordinate_add.y;
        arc_segment["end_pose"]["x"] = arc_segment["end_pose"]["x"].get<int>() + png_coordinate_add.x;
        arc_segment["end_pose"]["y"] = arc_segment["end_pose"]["y"].get<int>() + png_coordinate_add.y;
      }
    }
    std::ofstream file(task_folder_path + file_name);
    if(!file.is_open())
    {
      LOG(ERROR) << "Open file failed";
      continue;
    }
    file << json.dump(2);
  }
}

void Manager::SaveMap(std::string map_name)
{
  bool is_new_map;
  std::string map_name_folder_path = MapNameFolderPath(map_name);
  LOG(INFO) << "Save map, map_name_folder_path:" << map_name_folder_path;
  if(access(map_name_folder_path.c_str(),0) != 0)
  {
    is_new_map = true;
    mkdir(map_name_folder_path.c_str(),S_IRWXU | S_IRWXG | S_IRWXO);
  }
  else
    is_new_map = false;
  node_->SerializeState(map_name_folder_path + map_name + ".pbstream",true /* include_unfinished_submaps */);
  
  /*******************save png****************/
  std::unique_ptr<nav_msgs::OccupancyGrid> map = GetOccupancyGridMap(FLAGS_resolution);
  cv::Mat colorMat = OccupancyGridToMat(*map);
  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(9);
  if(!cv::imwrite(map_name_folder_path + map_name + ".png", colorMat, compression_params))
  {
    LOG(ERROR) << "save map.png error:" << map_name_folder_path + map_name;
    return;
  }
  cv::imwrite(map_name_folder_path + map_name + "_temp.png", colorMat, compression_params);

  //rewrite coordinate of all map data if zero point of png is changed
  if(!is_new_map)
  {
    LOG(INFO) << "Update map data...";
    MapInfo_t map_info = GetMapInfoFromYaml(map_name_folder_path  + map_name + ".yaml");
    Coordinate_t coordinate_of_zero_point_in_map = MatToMap(Coordinate_t(0,0),map_info);

    MapInfo_t new_map_info = MapMetaDataToMapInfo(map->info);
    Coordinate_t new_coordinate_of_zero_point_in_map = MatToMap(Coordinate_t(0,0),new_map_info);

    if(new_coordinate_of_zero_point_in_map != coordinate_of_zero_point_in_map)
    {
      Coordinate_t png_coordinate_add;
      png_coordinate_add.x = abs(new_coordinate_of_zero_point_in_map.x - coordinate_of_zero_point_in_map.x);
      png_coordinate_add.y = abs(new_coordinate_of_zero_point_in_map.y - coordinate_of_zero_point_in_map.y);
      LOG(INFO) << png_coordinate_add;

      UpdataVirtualWallPointCoor(map_name,png_coordinate_add);
      UpdataTaskPointCoor(map_name,png_coordinate_add);
    }
    LOG(INFO) << "Update map data done";
  }

  /*******************save yaml****************/
  FILE *yaml = fopen((map_name_folder_path + map_name + ".yaml").c_str(), "w");
  fprintf(yaml, "image: %s\nresolution: %f\nwidth: %d\nheight: %d\norigin: [%f, %f, %f]\nnegate: 0\n\n",
          (map_name + ".png").c_str(), map->info.resolution, map->info.width, map->info.height,
          map->info.origin.position.x, map->info.origin.position.y, tf::getYaw(map->info.origin.orientation));
  fclose(yaml);

  ROS_INFO("map saved done!");
}

}  // namespace cartographer_ros