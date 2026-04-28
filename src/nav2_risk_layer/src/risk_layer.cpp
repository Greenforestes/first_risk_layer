#include "nav2_risk_layer/risk_layer.hpp"

#include <algorithm>
#include <string>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"

namespace nav2_risk_layer
{

RiskLayer::RiskLayer()
: enabled_(true), risk_cost_(200), min_risk_cost_(80), risk_radius_(0.20), has_path_(false)
{
}

void RiskLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in RiskLayer::onInitialize");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("risk_cost", rclcpp::ParameterValue(200));
  declareParameter("min_risk_cost", rclcpp::ParameterValue(80));
  declareParameter("risk_radius", rclcpp::ParameterValue(0.20));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "risk_cost", risk_cost_);
  node->get_parameter(name_ + "." + "min_risk_cost", min_risk_cost_);
  node->get_parameter(name_ + "." + "risk_radius", risk_radius_);

  min_risk_cost_ = std::min(min_risk_cost_, risk_cost_);
  min_risk_cost_ = std::max(min_risk_cost_, 0);

  future_path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    "/dynamic_agents/future_path",
    rclcpp::QoS(10),
    std::bind(&RiskLayer::futurePathCallback, this, std::placeholders::_1));

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  debug_markers_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/risk_layer/debug_markers", qos);

  current_ = true;

  RCLCPP_INFO(node->get_logger(), "RiskLayer initialized");
  RCLCPP_INFO(node->get_logger(), "RiskLayer subscribed to /dynamic_agents/future_path");
  RCLCPP_INFO(node->get_logger(), "RiskLayer risk_cost = %d", risk_cost_);
  RCLCPP_INFO(node->get_logger(), "RiskLayer min_risk_cost = %d", min_risk_cost_);
  RCLCPP_INFO(node->get_logger(), "RiskLayer risk_radius = %.3f m", risk_radius_);
  RCLCPP_INFO(node->get_logger(), "RiskLayer debug topic = /risk_layer/debug_markers");
}

void RiskLayer::futurePathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    latest_path_ = *msg;
    has_path_ = !latest_path_.poses.empty();
  }

  publishDebugMarkers();
}

void RiskLayer::publishDebugMarkers()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  nav_msgs::msg::Path path_copy;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!has_path_) {
      return;
    }
    path_copy = latest_path_;
  }

  visualization_msgs::msg::MarkerArray marker_array;

  // 删除旧 marker
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header.frame_id = path_copy.header.frame_id;
  delete_marker.header.stamp = node->now();
  delete_marker.ns = "risk_debug";
  delete_marker.id = 0;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  // 1) future centerline
  visualization_msgs::msg::Marker line_marker;
  line_marker.header.frame_id = path_copy.header.frame_id;
  line_marker.header.stamp = node->now();
  line_marker.ns = "risk_debug_line";
  line_marker.id = 1;
  line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::msg::Marker::ADD;
  line_marker.pose.orientation.w = 1.0;
  line_marker.scale.x = 0.03;
  line_marker.color.a = 1.0;
  line_marker.color.r = 0.1;
  line_marker.color.g = 1.0;
  line_marker.color.b = 0.1;

  // 2) risk discs
  visualization_msgs::msg::Marker discs_marker;
  discs_marker.header.frame_id = path_copy.header.frame_id;
  discs_marker.header.stamp = node->now();
  discs_marker.ns = "risk_debug_discs";
  discs_marker.id = 2;
  discs_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  discs_marker.action = visualization_msgs::msg::Marker::ADD;
  discs_marker.pose.orientation.w = 1.0;
  discs_marker.scale.x = risk_radius_ * 2.0;
  discs_marker.scale.y = risk_radius_ * 2.0;
  discs_marker.scale.z = 0.03;

  const size_t n = path_copy.poses.size();

  for (size_t i = 0; i < n; ++i) {
    const auto & ps = path_copy.poses[i];

    geometry_msgs::msg::Point p;
    p.x = ps.pose.position.x;
    p.y = ps.pose.position.y;
    p.z = 0.03;

    line_marker.points.push_back(p);
    discs_marker.points.push_back(p);

    double ratio = 0.0;
    if (n > 1) {
      ratio = static_cast<double>(i) / static_cast<double>(n - 1);
    }

    const int point_cost = static_cast<int>(
      std::round(
        static_cast<double>(risk_cost_) -
        ratio * static_cast<double>(risk_cost_ - min_risk_cost_)
      )
    );

    const double norm = static_cast<double>(point_cost) / static_cast<double>(risk_cost_);

    std_msgs::msg::ColorRGBA c;
    c.r = 1.0f;
    c.g = static_cast<float>(0.15 + 0.70 * (1.0 - norm));
    c.b = 0.1f;
    c.a = static_cast<float>(0.25 + 0.65 * norm);

    discs_marker.colors.push_back(c);
  }

  marker_array.markers.push_back(line_marker);
  marker_array.markers.push_back(discs_marker);

  debug_markers_pub_->publish(marker_array);
}

void RiskLayer::updateBounds(
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

  for (const auto & pose_stamped : latest_path_.poses) {
    const double x = pose_stamped.pose.position.x;
    const double y = pose_stamped.pose.position.y;

    *min_x = std::min(*min_x, x - risk_radius_);
    *min_y = std::min(*min_y, y - risk_radius_);
    *max_x = std::max(*max_x, x + risk_radius_);
    *max_y = std::max(*max_y, y + risk_radius_);
  }
}

void RiskLayer::updateCosts(
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
  const int cell_radius = std::max(1, static_cast<int>(std::ceil(risk_radius_ / resolution)));

  const size_t path_size = latest_path_.poses.size();
  if (path_size == 0) {
    return;
  }

  unsigned int mx, my;
  for (size_t idx = 0; idx < path_size; ++idx) {
    const auto & pose_stamped = latest_path_.poses[idx];
    const double wx = pose_stamped.pose.position.x;
    const double wy = pose_stamped.pose.position.y;

    if (!master_grid.worldToMap(wx, wy, mx, my)) {
      continue;
    }

    double ratio = 0.0;
    if (path_size > 1) {
      ratio = static_cast<double>(idx) / static_cast<double>(path_size - 1);
    }

    const int point_cost = static_cast<int>(
      std::round(
        static_cast<double>(risk_cost_) -
        ratio * static_cast<double>(risk_cost_ - min_risk_cost_)
      )
    );

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
        const unsigned char new_cost = static_cast<unsigned char>(point_cost);
        master_grid.setCost(nx, ny, std::max(old_cost, new_cost));
      }
    }
  }
}

void RiskLayer::reset()
{
  matchSize();
}

}  // namespace nav2_risk_layer

PLUGINLIB_EXPORT_CLASS(nav2_risk_layer::RiskLayer, nav2_costmap_2d::Layer)