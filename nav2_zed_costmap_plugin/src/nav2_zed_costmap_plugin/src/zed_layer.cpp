/**
Incomplete file
Tried to create new layer type before finding out static layer can be updated
 */
#include "nav2_zed_costmap_plugin/zed_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using rcl_interfaces::msg::ParameterType;

namespace nav2_zed_costmap_plugin
{

ZedLayer::ZedLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

void
ZedLayer::onInitialize()
{
  RCLCPP_INFO(
    logger_,
    "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
  
  getParameters();

  // Map QoS
  rclcpp::QoS map_qos(10);
  map_qos.transient_local();
  map_qos.reliable();
  map_qos.keep_last(1);

  // Display info
  RCLCPP_INFO(
    logger_,
    "Subscribing to the map topic (%s) with transient local durability",
    map_topic_.c_str());

  // Lock node for topic subscription
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Create subscription, which runs 'incomingUpdate' method when message is received
  // 'incomingUpdate' is run rather than 'incomingMap' because zed produces
  map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, 
    map_qos,
    std::bind(&ZedLayer::incomingMap, this, std::placeholders::_1));
  need_recalculation_ = false;
  current_ = true;
}

void 
ZedLayer::getParameters()
{
  declareParameter("map_topic", rclcpp::ParameterValue("/zed"));

  auto node = node_.lock(); 
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "map_topic", map_topic_);
}

void
ZedLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map)
{
  if (new_map != nullptr) {
    RCLCPP_INFO(
    logger_,
    "received message from /zed");
  } else {
    RCLCPP_WARN(
      logger_,
      "Null message received from topic");
  }
}

void
ZedLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

void
ZedLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "ZedLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

void
ZedLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  int gradient_index;
  for (int j = min_j; j < max_j; j++) {
    gradient_index = 0;
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);
      unsigned char cost = (LETHAL_OBSTACLE - gradient_index*GRADIENT_FACTOR)%255;
      if (gradient_index <= GRADIENT_SIZE) {
        gradient_index++;
      } else {
        gradient_index = 0;
      }
      master_array[index] = cost;
    }
  }
}

}  // namespace nav2_zed_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_zed_costmap_plugin::ZedLayer, nav2_costmap_2d::Layer)