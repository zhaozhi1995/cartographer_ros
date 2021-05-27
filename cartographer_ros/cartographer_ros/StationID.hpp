#ifndef __STATION_ID_HPP__
#define __STATION_ID_HPP__

#include <stdio.h>
#include <geometry_msgs/Pose.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <tf/tf.h>
#include <dirent.h>
#include <vector>

#define M_2PI 6.283185307179586476925286766559
#define STATION_FOLDER_NAME "/Station/"
#define POSE_X "pose_x"
#define POSE_Y "pose_y"
#define POSE_THETA "pose_theta"
#define STATION_NAME(id) "ID_" + std::to_string(id) + ".yaml"

struct Station_t
{
  std::string map_name;
  uint16_t id;
  geometry_msgs::Pose pose;
};

class StationID
{
private:
  std::string map_folder_path_;
  Station_t last_station_;
  double OrientationDiff(double target,double current)
  {
    double diff;
    
    while(target > M_PI)
      target -= M_2PI;
    while(target < -M_PI)
      target += M_2PI;
    while(current > M_PI)
      current -= M_2PI;
    while(current < -M_PI)
      current += M_2PI;
      
    diff = target - current;
    if(diff > M_PI)
      diff -= M_2PI;
    else if(diff < -M_PI)
      diff += M_2PI;
    
    return diff;
  }
public:
  StationID(std::string map_folder_path = "")
  {
    map_folder_path_ = map_folder_path;
    last_station_.map_name = "";
    last_station_.id = UINT16_MAX; //for initial id,reserved
  }
  ~StationID(){};
  void SetMapFolderPath(std::string map_folder_path)
  {
    map_folder_path_ = map_folder_path;
  }
  std::pair<bool,Station_t> GetStation(uint16_t id)
  {
    Station_t station;
    if(map_folder_path_.empty())
      return std::make_pair(false,station);
    DIR *directory_pointer;
    struct dirent *entry;
    std::vector<std::string> map_name_list;
    if((directory_pointer = opendir(map_folder_path_.c_str())) == NULL)
    {
      std::cout << "open map folder failed" << std::endl;
      return std::make_pair(false,station);
    }
    else 
    {
      while((entry = readdir(directory_pointer)) != NULL)
      {
        if(entry->d_type == DT_DIR && entry->d_name[0] != '.')
          map_name_list.push_back(entry->d_name);
      }
    }
    std::string station_file_name = STATION_NAME(id);
    for (auto &map_name:map_name_list)
    {
      std::string station_folder_path = map_folder_path_ + map_name + STATION_FOLDER_NAME;
      if((directory_pointer = opendir(station_folder_path.c_str())) != NULL)
      {
        while((entry = readdir(directory_pointer)) != NULL)
        {
          if(entry->d_name == station_file_name)
          {
            YAML::Node station_yaml = YAML::LoadFile(station_folder_path + station_file_name);
            station.pose.position.x = station_yaml[POSE_X].as<double>();
            station.pose.position.y = station_yaml[POSE_Y].as<double>();
            station.pose.orientation = tf::createQuaternionMsgFromYaw(station_yaml[POSE_THETA].as<double>());
            station.id = id;
            station.map_name = map_name;
            return std::make_pair(true,station);
          }
        }
      }
    }
    closedir(directory_pointer);
    std::cout << "have found no station of id:" << id << std::endl;
    return std::make_pair(false,station);
  }

  bool SaveStation(Station_t station)
  {
    if(map_folder_path_.empty())
      return false;

    if((station.id == last_station_.id) && (station.map_name == station.map_name))
    {
      if(hypot(station.pose.position.x - last_station_.pose.position.x,station.pose.position.y - last_station_.pose.position.y) < 0.1 //m
        && fabs(OrientationDiff(tf::getYaw(station.pose.orientation),tf::getYaw(last_station_.pose.orientation))) < 0.1) // ~ 5 degree
        return true;
    }

    auto check_station = GetStation(station.id);
    if(check_station.first)
      remove((map_folder_path_ + check_station.second.map_name + STATION_FOLDER_NAME + STATION_NAME(check_station.second.id)).c_str());

    std::string station_path = map_folder_path_ + station.map_name + STATION_FOLDER_NAME;
    if(access(station_path.c_str(),F_OK) != 0)
    {
      if(mkdir(station_path.c_str(),S_IRWXU) == -1)
      {
        std::cout << "mkdir station failed";
        return false;
      }
    }
    std::ofstream fout(station_path + STATION_NAME(station.id));

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << POSE_X << YAML::Value << station.pose.position.x;
    emitter << YAML::Key << POSE_Y << YAML::Value << station.pose.position.y;
    emitter << YAML::Key << POSE_THETA << YAML::Value << tf::getYaw(station.pose.orientation);

    // emitter << YAML::BeginSeq;
    // for (int i = 0; i < vi.size(); i++)
    // {
    //     emitter << YAML::BeginMap;
    //     emitter << YAML::Key << "name" << YAML::Value << vi[i].name;
    //     emitter << YAML::Key << "age" << YAML::Value << vi[i].age;;
    //     emitter << YAML::Key << "numbers" << YAML::Flow << vi[i].numbers;
    //     emitter << YAML::EndMap;
        
    // }
    // emitter << YAML::EndSeq;
    emitter << YAML::EndMap;
    fout << emitter.c_str();
    fout.close();

    last_station_ = station;
    return true;
  }
};

#endif