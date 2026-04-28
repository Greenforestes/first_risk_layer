#include "nav2_current_actor_layer/current_actor_layer.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace nav2_current_actor_layer
{

CurrentActorLayer::CurrentActorLayer()
: enabled_(true), current_cost_(254), current_radius_(0.20), has_pose_(false)
{
}

void CurrentActorLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in CurrentActorLayer::onInitialize");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("current_cost", rclcpp::ParameterValue(254));
  declareParameter("current_radius", rclcpp::ParameterValue(0.20));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "current_cost", current_cost_);
  node->get_parameter(name_ + "." + "current_radius", current_radius_);

  current_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/dynamic_agents/current_pose",
    rclcpp::QoS(10),
    std::bind(&CurrentActorLayer::currentPoseCallback, this, std::placeholders::_1));

  current_ = true;

  RCLCPP_INFO(node->get_logger(), "CurrentActorLayer initialized");
  RCLCPP_INFO(node->get_logger(), "CurrentActorLayer subscribed to /dynamic_agents/current_pose");
  RCLCPP_INFO(node->get_logger(), "CurrentActorLayer current_cost = %d", current_cost_);
  RCLCPP_INFO(node->get_logger(), "CurrentActorLayer current_radius = %.3f m", current_radius_);
}

void CurrentActorLayer::currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  latest_pose_ = *msg;
  has_pose_ = true;
}

void CurrentActorLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (!has_pose_) {
    return;
  }

  const double x = latest_pose_.pose.position.x;
  const double y = latest_pose_.pose.position.y;

  *min_x = std::min(*min_x, x - current_radius_);
  *min_y = std::min(*min_y, y - current_radius_);
  *max_x = std::max(*max_x, x + current_radius_);
  *max_y = std::max(*max_y, y + current_radius_);
}

void CurrentActorLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (!has_pose_) {
    return;
  }

  unsigned int mx, my;
  const double wx = latest_pose_.pose.position.x;
  const double wy = latest_pose_.pose.position.y;

  if (!master_grid.worldToMap(wx, wy, mx, my)) {
    return;
  }

  const double resolution = master_grid.getResolution();
  const int cell_radius = std::max(1, static_cast<int>(std::ceil(current_radius_ / resolution)));

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
      const unsigned char new_cost = static_cast<unsigned char>(current_cost_);
      master_grid.setCost(nx, ny, std::max(old_cost, new_cost));
    }
  }
}

void CurrentActorLayer::reset()
{
  matchSize();
}

}  // namespace nav2_current_actor_layer

PLUGINLIB_EXPORT_CLASS(nav2_current_actor_layer::CurrentActorLayer, nav2_costmap_2d::Layer)