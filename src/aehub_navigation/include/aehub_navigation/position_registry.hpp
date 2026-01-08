#pragma once

#include <string>
#include <map>
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace aehub_navigation
{

struct Position
{
  double x;
  double y;
  double theta;
  std::string description;
};

/**
 * @brief Position Registry - manages dynamic positions
 * 
 * Supports adding, removing, and saving positions at runtime.
 */
class PositionRegistry
{
public:
  PositionRegistry();
  ~PositionRegistry();

  /**
   * @brief Load positions from YAML file
   * @param yaml_path Path to positions.yaml file
   * @return true if loaded successfully
   */
  bool loadFromYAML(const std::string & yaml_path);

  /**
   * @brief Save positions to YAML file
   * @param yaml_path Path to positions.yaml file
   * @return true if saved successfully
   */
  bool saveToYAML(const std::string & yaml_path) const;

  /**
   * @brief Add or update a position
   * @param position_id Position ID (e.g., "position_A")
   * @param x X coordinate
   * @param y Y coordinate
   * @param theta Orientation angle in radians
   * @param description Optional description
   * @return true if added successfully
   */
  bool addPosition(const std::string & position_id, double x, double y, double theta, const std::string & description = "");

  /**
   * @brief Remove a position
   * @param position_id Position ID to remove
   * @return true if removed successfully
   */
  bool removePosition(const std::string & position_id);

  /**
   * @brief Get position by ID
   * @param position_id Position ID (e.g., "position_A")
   * @param pose Output pose (in map frame)
   * @return true if position found
   */
  bool getPosition(const std::string & position_id, geometry_msgs::msg::PoseStamped & pose) const;

  /**
   * @brief Check if position exists
   * @param position_id Position ID
   * @return true if exists
   */
  bool hasPosition(const std::string & position_id) const;

  /**
   * @brief Get all position IDs
   * @return Vector of position IDs
   */
  std::vector<std::string> getAllPositionIds() const;

  /**
   * @brief Get number of positions
   * @return Number of positions
   */
  size_t getPositionCount() const;

private:
  std::map<std::string, Position> positions_;
  std::string map_frame_id_{"map"};
};

}  // namespace aehub_navigation

