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

#include "cartographer_ros/node_manager.hpp"
#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

// DEFINE_bool(collect_metrics, false,
//             "Activates the collection of runtime metrics. If activated, the "
//             "metrics can be accessed via a ROS service.");
// DEFINE_string(configuration_directory, "",
//               "First directory in which configuration files are searched, "
//               "second is always the Cartographer installation to allow "
//               "including files from there.");
// DEFINE_string(configuration_basename, "",
//               "Basename, i.e. not containing any directory prefix, of the "
//               "configuration file.");
// DEFINE_string(load_state_filename, "",
//               "If non-empty, filename of a .pbstream file to load, containing "
//               "a saved SLAM state.");
// DEFINE_bool(load_frozen_state, true,
//               "Load the saved state as frozen (non-optimized) trajectories.");
// DEFINE_bool(start_trajectory_with_default_topics, true,
//               "Enable to immediately start the first trajectory with default topics.");
// DEFINE_string(save_state_filename, "",
//               "If non-empty, serialize state and write it to disk before shutting down.");
DEFINE_bool(use_manager, true,
            "whether or not use manager for cartographer node");

namespace cartographer_ros {
namespace {

void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  ::ros::spin();

  node.FinishAllTrajectories();
  node.RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

void InitGoogleLog(std::string node_name)
{
  time_t raw_time;

   // 时间判断
  time(&raw_time);
  char tmp[64];
  strftime(tmp,sizeof(tmp),"%Y-%m-%d",localtime(&raw_time));
  std::string  str = tmp;
  std::string work_folder_path = std::getenv("WORK_FOLDER_PATH");
  std::string log_path = work_folder_path+"/Log/"+str+"/"+node_name+"/";
  google::InitGoogleLogging(node_name.c_str()); //链接文件名
  google::SetGoogleLogPath(log_path);  //判断当前路径下文件夹是否存在
  google::InstallFailureSignalHandler();
  google::SetStderrLogging(google::WARNING);
  FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色
   
  FLAGS_servitysinglelog = false;// 用来按照等级区分log文件
  google::SetLogSymlink(google::GLOG_FATAL,"");
  google::SetLogSymlink(google::GLOG_ERROR,"");
  google::SetLogSymlink(google::GLOG_WARNING,"");
  google::SetLogSymlink(google::GLOG_INFO,"");
  google::SetLogDestination(google::GLOG_FATAL,(log_path +"/"+ node_name+"_fatal").c_str()); // 设置 google::FATAL 级别的日志存储路径和文件名前缀
  google::SetLogDestination(google::GLOG_ERROR, (log_path+"/"+node_name+"_error").c_str()); //设置 google::ERROR 级别的日志存储路径和文件名前缀
  google::SetLogDestination(google::GLOG_WARNING,(log_path+"/"+node_name+"_warning").c_str()); //设置 google::WARNING 级别的日志存储路径和文件名前缀
  google::SetLogDestination(google::GLOG_INFO, (log_path+"/"+node_name+"_info").c_str()); //设置 google::INFO 级别的日志存储路径和文件名前缀
  google::SetLogFilenameExtension(".log");

  FLAGS_logbufsecs = 0; //缓冲日志输出，默认为30秒，此处改为立即输出
  FLAGS_max_log_size = 100; //最大日志大小为 100MB
  FLAGS_stop_logging_if_full_disk = true; //当磁盘被写满时，停止日志输出
}

int main(int argc, char** argv) {
  // google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  InitGoogleLog("cartographer_node");

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_node");
  ::ros::start();

  // cartographer_ros::ScopedRosLogSink ros_log_sink;
  if(FLAGS_use_manager)
  {
    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
    tf2_ros::TransformListener tf(tf_buffer);
    cartographer_ros::Manager manager(&tf_buffer);
    ::ros::spin();
  }
  else
    cartographer_ros::Run();

  ::ros::shutdown();
}
