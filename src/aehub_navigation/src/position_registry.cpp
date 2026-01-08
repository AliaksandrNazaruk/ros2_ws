#include "aehub_navigation/position_registry.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>

namespace aehub_navigation
{

PositionRegistry::PositionRegistry()
{
}

PositionRegistry::~PositionRegistry()
{
}

bool PositionRegistry::loadFromYAML(const std::string & yaml_path)
{
  try {
    YAML::Node config = YAML::LoadFile(yaml_path);
    
    if (!config["positions"]) {
      return false;
    }

    positions_.clear();
    
    const YAML::Node & positions = config["positions"];
    for (const auto & item : positions) {
      std::string id = item.first.as<std::string>();
      const YAML::Node & pos = item.second;
      
      Position position;
      position.x = pos["x"].as<double>();
      position.y = pos["y"].as<double>();
      position.theta = pos["theta"].as<double>();
      
      if (pos["description"]) {
        position.description = pos["description"].as<std::string>();
      }
      
      positions_[id] = position;
    }

    // Allow any number of positions (dynamic positions)
    if (positions_.empty()) {
      return false;
    }

    return true;
  } catch (const YAML::Exception & e) {
    return false;
  } catch (const std::exception & e) {
    return false;
  }
}

bool PositionRegistry::getPosition(
  const std::string & position_id,
  geometry_msgs::msg::PoseStamped & pose) const
{
  auto it = positions_.find(position_id);
  if (it == positions_.end()) {
    return false;
  }

  const Position & pos = it->second;
  
  pose.header.frame_id = map_frame_id_;
  pose.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  
  pose.pose.position.x = pos.x;
  pose.pose.position.y = pos.y;
  pose.pose.position.z = 0.0;
  
  // Convert theta to quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, pos.theta);
  pose.pose.orientation = tf2::toMsg(q);
  
  return true;
}

bool PositionRegistry::hasPosition(const std::string & position_id) const
{
  return positions_.find(position_id) != positions_.end();
}

std::vector<std::string> PositionRegistry::getAllPositionIds() const
{
  std::vector<std::string> ids;
  for (const auto & pair : positions_) {
    ids.push_back(pair.first);
  }
  return ids;
}

size_t PositionRegistry::getPositionCount() const
{
  return positions_.size();
}

bool PositionRegistry::saveToYAML(const std::string & yaml_path) const
{
  try {
    YAML::Node config;
    YAML::Node positions;
    
    for (const auto & pair : positions_) {
      YAML::Node pos;
      pos["x"] = pair.second.x;
      pos["y"] = pair.second.y;
      pos["theta"] = pair.second.theta;
      if (!pair.second.description.empty()) {
        pos["description"] = pair.second.description;
      }
      positions[pair.first] = pos;
    }
    
    config["positions"] = positions;
    
    std::ofstream fout(yaml_path);
    if (!fout.is_open()) {
      return false;
    }
    
    fout << config;
    fout.close();
    
    return true;
  } catch (const YAML::Exception & e) {
    return false;
  } catch (const std::exception & e) {
    return false;
  }
}

bool PositionRegistry::addPosition(
  const std::string & position_id,
  double x,
  double y,
  double theta,
  const std::string & description)
{
  if (position_id.empty()) {
    return false;
  }
  
  Position position;
  position.x = x;
  position.y = y;
  position.theta = theta;
  position.description = description;
  
  positions_[position_id] = position;
  return true;
}

bool PositionRegistry::removePosition(const std::string & position_id)
{
  auto it = positions_.find(position_id);
  if (it == positions_.end()) {
    return false;
  }
  
  positions_.erase(it);
  return true;
}

}  // namespace aehub_navigation

