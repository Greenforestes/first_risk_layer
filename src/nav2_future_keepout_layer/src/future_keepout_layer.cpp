#include "nav2_future_keepout_layer/future_keepout_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace nav2_future_keepout_layer
{

FutureKeepoutLayer::FutureKeepoutLayer()
: enabled_(true), keepout_cost_(253), keepout_radius_(0.20), keepout_points_(8), has_path_(false)
{
}

void FutureKeepoutLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in FutureKeepoutLayer::onInitialize");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("keepout_cost", rclcpp::ParameterValue(253));
  declareParameter("keepout_radius", rclcpp::ParameterValue(0.20));
  declareParameter("keepout_points", rclcpp::ParameterValue(8));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "keepout_cost", keepout_cost_);
  node->get_parameter(name_ + "." + "keepout_radius", keepout_radius_);
  node->get_parameter(name_ + "." + "keepout_points", keepout_points_);

  keepout_cost_ = std::clamp(keepout_cost_, 0, 254);
  keepout_points_ = std::max(1, keepout_points_);

  future_path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    "/dynamic_agents/future_path",
    rclcpp::QoS(10),
    std::bind(&FutureKeepoutLayer::futurePathCallback, this, std::placeholders::_1));

  current_ = true;

  RCLCPP_INFO(node->get_logger(), "FutureKeepoutLayer initialized");
  RCLCPP_INFO(node->get_logger(), "FutureKeepoutLayer subscribed to /dynamic_agents/future_path");
  RCLCPP_INFO(node->get_logger(), "FutureKeepoutLayer keepout_cost = %d", keepout_cost_);
  RCLCPP_INFO(node->get_logger(), "FutureKeepoutLayer keepout_radius = %.3f m", keepout_radius_);
  RCLCPP_INFO(node->get_logger(), "FutureKeepoutLayer keepout_points = %d", keepout_points_);
}

void FutureKeepoutLayer::futurePathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  latest_path_ = *msg;
  has_path_ = !latest_path_.poses.empty();
}

void FutureKeepoutLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(path_mutex_);
  if (!has_path_) {
    return;
  }

  const int n = std::min(static_cast<int>(latest_path_.poses.size()), keepout_points_);
  for (int i = 0; i < n; ++i) {
    const double x = latest_path_.poses[i].pose.position.x;
    const double y = latest_path_.poses[i].pose.position.y;

    *min_x = std::min(*min_x, x - keepout_radius_);
    *min_y = std::min(*min_y, y - keepout_radius_);
    *max_x = std::max(*max_x, x + keepout_radius_);
    *max_y = std::max(*max_y, y + keepout_radius_);
  }
}

void FutureKeepoutLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(path_mutex_);
  if (!has_path_) {
    return;
  }

  const double resolution = master_grid.getResolution();
  const int cell_radius = std::max(1, static_cast<int>(std::ceil(keepout_radius_ / resolution)));
  const int n = std::min(static_cast<int>(latest_path_.poses.size()), keepout_points_);

  unsigned int mx, my;
  for (int idx = 0; idx < n; ++idx) {
    const double wx = latest_path_.poses[idx].pose.position.x;
    const double wy = latest_path_.poses[idx].pose.position.y;

    if (!master_grid.worldToMap(wx, wy, mx, my)) {
      continue;
    }

    const int cx = static_cast<int>(mx);
    const int cy = static_cast<int>(my);

    for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
      for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        if (dx * dx + dy * dy > cell_radius * cell_radius) {
          continue;
        }

        const int nx = cx + dx;
        const int ny = cy + dy;

        if (nx < min_i || nx >= max_i || ny < min_j || ny >= max_j) {
          continue;
        }

        if (nx < 0 || ny < 0 ||
            nx >= static_cast<int>(master_grid.getSizeInCellsX()) ||
            ny >= static_cast<int>(master_grid.getSizeInCellsY())) {
          continue;
        }

        const unsigned char old_cost = master_grid.getCost(nx, ny);
        const unsigned char new_cost = static_cast<unsigned char>(keepout_cost_);
        master_grid.setCost(nx, ny, std::max(old_cost, new_cost));
      }
    }
  }
}

void FutureKeepoutLayer::reset()
{
  matchSize();
}

}  // namespace nav2_future_keepout_layer

PLUGINLIB_EXPORT_CLASS(
  nav2_future_keepout_layer::FutureKeepoutLayer,
  nav2_costmap_2d::Layer)