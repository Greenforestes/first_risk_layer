#ifndef NAV2_RISK_LAYER__RISK_LAYER_HPP_
#define NAV2_RISK_LAYER__RISK_LAYER_HPP_

#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_risk_layer
{

class RiskLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  RiskLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);
  virtual void reset();
  virtual bool isClearable() {return false;}

private:
  void futurePathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void publishDebugMarkers();

  bool enabled_;
  int risk_cost_;
  int min_risk_cost_;
  double risk_radius_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr future_path_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;

  nav_msgs::msg::Path latest_path_;
  bool has_path_;
  std::mutex path_mutex_;
};

}  // namespace nav2_risk_layer

#endif  // NAV2_RISK_LAYER__RISK_LAYER_HPP_