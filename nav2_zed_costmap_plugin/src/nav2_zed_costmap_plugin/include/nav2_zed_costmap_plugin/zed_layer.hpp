#ifndef ZED_LAYER_HPP_
#define ZED_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "message_filters/subscriber.h"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_zed_costmap_plugin
{

class ZedLayer : public nav2_costmap_2d::Layer
{
public:
  ZedLayer();

  virtual void onInitialize();

  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset() {return;}

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}

protected:
  void getParameters();

  void incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map);

private:
  // Occupancy Grid parameters
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  // Parameters
  std::string map_topic_;

  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  // Indicates that the entire gradient should be recalculated next time.
  bool need_recalculation_;

  // Size of gradient in cells
  int GRADIENT_SIZE = 20;
  // Step of increasing cost per one cell in gradient
  int GRADIENT_FACTOR = 10;
};

}  // namespace nav2_zed_costmap_plugin

#endif  // ZED_LAYER_HPP_