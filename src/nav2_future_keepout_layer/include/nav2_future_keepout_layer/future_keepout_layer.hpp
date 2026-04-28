#ifndef NAV2_FUTURE_KEEPOUT_LAYER__FUTURE_KEEPOUT_LAYER_HPP_
#define NAV2_FUTURE_KEEPOUT_LAYER__FUTURE_KEEPOUT_LAYER_HPP_

#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_future_keepout_layer
{

class FutureKeepoutLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  FutureKeepoutLayer();

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}

private:
  void futurePathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  bool enabled_;
  int keepout_cost_;
  double keepout_radius_;
  int keepout_points_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr future_path_sub_;

  nav_msgs::msg::Path latest_path_;
  bool has_path_;
  std::mutex path_mutex_;
};

}  // namespace nav2_future_keepout_layer

#endif  // NAV2_FUTURE_KEEPOUT_LAYER__FUTURE_KEEPOUT_LAYER_HPP_