#ifndef INTPC_GLOBAL_PLANNER_ROS_H_
#define INTPC_GLOBAL_PLANNER_ROS_H_

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

// Navigation2 global planner base class and utilities
#include <nav2_core/global_planner.hpp>

// message types
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// transforms
#include <tf2_ros/transform_listener.h>

// costmap
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

#include <nav2_util/lifecycle_node.hpp>
#include <nav2_util/node_utils.hpp>

// Intpc core planner
#include <intpc_local_planner/intpc_local_planner.h>

// Eigen for matrix operations
#include <Eigen/Dense>

// Standard library
#include <vector>
#include <queue>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <algorithm>

namespace intpc_local_planner
{

using TFBufferPtr = std::shared_ptr<tf2_ros::Buffer>;
using CostmapROSPtr = std::shared_ptr<nav2_costmap_2d::Costmap2DROS>;

struct Node {
  int x, y;
  double g, h, f;
  Node* parent;
  
  Node(int x_, int y_, double g_, double h_, Node* parent_ = nullptr)
    : x(x_), y(y_), g(g_), h(h_), f(g_ + h_), parent(parent_) {}
  
  bool operator>(const Node& other) const {
    return f > other.f;
  }
};

struct NodeHash {
  size_t operator()(const std::pair<int, int>& p) const {
    return p.first * 31 + p.second;
  }
};

class IntpcGlobalPlannerROS : public nav2_core::GlobalPlanner
{
public:
  IntpcGlobalPlannerROS();
  ~IntpcGlobalPlannerROS();

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  std::unique_ptr<IntpcLocalPlanner> planner_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("IntpcGlobalPlannerROS")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;

  double path_resolution_;
  double max_planning_time_;
  double waypoint_separation_;
  double obstacle_inflation_radius_;
  double smoothing_factor_;
  bool use_smoother_;
  bool use_astar_;

  bool isCollisionFree(double x, double y);
  bool isPathCollisionFree(const std::vector<std::pair<int, int>>& path);
  nav_msgs::msg::Path aStarPlanning(double start_x, double start_y, double goal_x, double goal_y);
  nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& path);
  double heuristic(int x1, int y1, int x2, int y2);
  std::vector<std::pair<int, int>> getNeighbors(int x, int y, int width, int height);
};

}  // namespace intpc_local_planner

#endif  // INPC_GLOBAL_PLANNER_ROS_H_
