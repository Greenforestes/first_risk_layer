#ifndef NAV2_CURRENT_ACTOR_LAYER__CURRENT_ACTOR_LAYER_HPP_
#define NAV2_CURRENT_ACTOR_LAYER__CURRENT_ACTOR_LAYER_HPP_

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_current_actor_layer
{

class CurrentActorLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  CurrentActorLayer();

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
  void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  bool enabled_;
  int current_cost_;
  double current_radius_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;

  geometry_msgs::msg::PoseStamped latest_pose_;
  bool has_pose_;
  std::mutex pose_mutex_;
};

}  // namespace nav2_current_actor_layer

#endif