#include <intpc_local_planner/intpc_global_planner_ros.h>
#include <intpc_local_planner/intpc_local_planner.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav2_util/node_utils.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <vector>
#include <set>
#include <limits>

namespace intpc_local_planner
{

IntpcGlobalPlannerROS::IntpcGlobalPlannerROS()
{
  planner_ = std::make_unique<IntpcLocalPlanner>();
}

IntpcGlobalPlannerROS::~IntpcGlobalPlannerROS()
{
}

void IntpcGlobalPlannerROS::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in configure");
  }
  
  RCLCPP_INFO(logger_, "Configuring IntpcGlobalPlannerROS");
  clock_ = node->get_clock();
  name_ = name;
  
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".path_resolution", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".max_planning_time", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".waypoint_separation", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".obstacle_inflation_radius", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".smoothing_factor", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".use_smoother", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".use_astar", rclcpp::ParameterValue(false));
  
  node->get_parameter(name_ + ".path_resolution", path_resolution_);
  node->get_parameter(name_ + ".max_planning_time", max_planning_time_);
  node->get_parameter(name_ + ".waypoint_separation", waypoint_separation_);
  node->get_parameter(name_ + ".obstacle_inflation_radius", obstacle_inflation_radius_);
  node->get_parameter(name_ + ".smoothing_factor", smoothing_factor_);
  node->get_parameter(name_ + ".use_smoother", use_smoother_);
  node->get_parameter(name_ + ".use_astar", use_astar_);
  
  RCLCPP_INFO(logger_, "Intpc global planner parameters: path_resolution=%.2f, max_planning_time=%.2f, waypoint_separation=%.2f", 
              path_resolution_, max_planning_time_, waypoint_separation_);
  RCLCPP_INFO(logger_, "Obstacle inflation radius: %.2f, Smoothing factor: %.2f, Use smoother: %s, Use A*: %s", 
              obstacle_inflation_radius_, smoothing_factor_, use_smoother_ ? "true" : "false", use_astar_ ? "true" : "false");
}

void IntpcGlobalPlannerROS::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up IntpcGlobalPlannerROS");
}

void IntpcGlobalPlannerROS::activate()
{
  RCLCPP_INFO(logger_, "Activating IntpcGlobalPlannerROS");
}

void IntpcGlobalPlannerROS::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating IntpcGlobalPlannerROS");
}

bool IntpcGlobalPlannerROS::isCollisionFree(double x, double y)
{
  if (!costmap_ros_) {
    return false;
  }
  
  nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    return false;
  }
  
  unsigned int map_x, map_y;
  if (!costmap->worldToMap(x, y, map_x, map_y)) {
    return false;
  }
  
  unsigned char cost = costmap->getCost(map_x, map_y);
  return cost < nav2_costmap_2d::LETHAL_OBSTACLE;
}

bool IntpcGlobalPlannerROS::isPathCollisionFree(const std::vector<std::pair<int, int>>& path)
{
  if (!costmap_ros_) {
    return false;
  }
  
  nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    return false;
  }
  
  int inflation_cells = static_cast<int>(obstacle_inflation_radius_ / costmap->getResolution());
  
  for (const auto& point : path) {
    for (int dx = -inflation_cells; dx <= inflation_cells; dx++) {
      for (int dy = -inflation_cells; dy <= inflation_cells; dy++) {
        int check_x = point.first + dx;
        int check_y = point.second + dy;
        
        if (check_x < 0 || check_x >= static_cast<int>(costmap->getSizeInCellsX()) ||
            check_y < 0 || check_y >= static_cast<int>(costmap->getSizeInCellsY())) {
          continue;
        }
        
        unsigned char cost = costmap->getCost(check_x, check_y);
        if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
          return false;
        }
      }
    }
  }
  
  return true;
}

double IntpcGlobalPlannerROS::heuristic(int x1, int y1, int x2, int y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

std::vector<std::pair<int, int>> IntpcGlobalPlannerROS::getNeighbors(int x, int y, int width, int height)
{
  std::vector<std::pair<int, int>> neighbors;
  
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      
      int nx = x + dx;
      int ny = y + dy;
      
      if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
        neighbors.push_back({nx, ny});
      }
    }
  }
  
  return neighbors;
}

nav_msgs::msg::Path IntpcGlobalPlannerROS::aStarPlanning(double start_x, double start_y, double goal_x, double goal_y)
{
  nav_msgs::msg::Path plan;
  plan.header.stamp = clock_->now();
  
  if (!costmap_ros_) {
    RCLCPP_ERROR(logger_, "Costmap not available");
    return plan;
  }
  
  nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    RCLCPP_ERROR(logger_, "Costmap pointer is null");
    return plan;
  }
  
  unsigned int start_map_x, start_map_y, goal_map_x, goal_map_y;
  if (!costmap->worldToMap(start_x, start_y, start_map_x, start_map_y)) {
    RCLCPP_ERROR(logger_, "Start position outside costmap");
    return plan;
  }
  
  if (!costmap->worldToMap(goal_x, goal_y, goal_map_x, goal_map_y)) {
    RCLCPP_ERROR(logger_, "Goal position outside costmap");
    return plan;
  }
  
  if (costmap->getCost(goal_map_x, goal_map_y) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
    RCLCPP_WARN(logger_, "Goal is in collision, attempting to find nearest free cell");
    
    bool found_free = false;
    int search_radius = 10;
    for (int r = 1; r <= search_radius && !found_free; r++) {
      for (int dx = -r; dx <= r && !found_free; dx++) {
        for (int dy = -r; dy <= r && !found_free; dy++) {
          int nx = static_cast<int>(goal_map_x) + dx;
          int ny = static_cast<int>(goal_map_y) + dy;
          
          if (nx >= 0 && nx < static_cast<int>(costmap->getSizeInCellsX()) &&
              ny >= 0 && ny < static_cast<int>(costmap->getSizeInCellsY())) {
            if (costmap->getCost(nx, ny) < nav2_costmap_2d::LETHAL_OBSTACLE) {
              goal_map_x = nx;
              goal_map_y = ny;
              found_free = true;
            }
          }
        }
      }
    }
    
    if (!found_free) {
      RCLCPP_ERROR(logger_, "Could not find free cell near goal");
      return plan;
    }
  }
  
  int width = costmap->getSizeInCellsX();
  int height = costmap->getSizeInCellsY();
  
  std::priority_queue<Node*, std::vector<Node*>, std::function<bool(Node*, Node*)>> open_set(
    [](Node* a, Node* b) { return a->f > b->f; });
  
  std::unordered_map<std::pair<int, int>, Node*, NodeHash> all_nodes;
  
  Node* start_node = new Node(start_map_x, start_map_y, 0.0, 
                              heuristic(start_map_x, start_map_y, goal_map_x, goal_map_y));
  open_set.push(start_node);
  all_nodes[{start_map_x, start_map_y}] = start_node;
  
  std::set<std::pair<int, int>> closed_set;
  
  int inflation_cells = static_cast<int>(obstacle_inflation_radius_ / costmap->getResolution());
  
  auto start_time = clock_->now();
  
  while (!open_set.empty()) {
    auto current_time = clock_->now();
    if ((current_time - start_time).seconds() > max_planning_time_) {
      RCLCPP_WARN(logger_, "Planning timeout after %.2f seconds", max_planning_time_);
      break;
    }
    
    Node* current = open_set.top();
    open_set.pop();
    
    if (current->x == static_cast<int>(goal_map_x) && current->y == static_cast<int>(goal_map_y)) {
      std::vector<std::pair<int, int>> path;
      Node* node = current;
      while (node != nullptr) {
        path.push_back({node->x, node->y});
        node = node->parent;
      }
      std::reverse(path.begin(), path.end());
      
      if (isPathCollisionFree(path)) {
        plan.header.frame_id = costmap_ros_->getGlobalFrameID();
        
        for (size_t i = 0; i < path.size(); i++) {
          double wx, wy;
          costmap->mapToWorld(path[i].first, path[i].second, wx, wy);
          
          geometry_msgs::msg::PoseStamped pose;
          pose.header = plan.header;
          pose.pose.position.x = wx;
          pose.pose.position.y = wy;
          pose.pose.position.z = 0.0;
          
          double yaw = 0.0;
          if (i < path.size() - 1) {
            double next_wx, next_wy;
            costmap->mapToWorld(path[i+1].first, path[i+1].second, next_wx, next_wy);
            yaw = std::atan2(next_wy - wy, next_wx - wx);
          }
          
          tf2::Quaternion q;
          q.setRPY(0, 0, yaw);
          pose.pose.orientation = tf2::toMsg(q);
          
          plan.poses.push_back(pose);
        }
        
        RCLCPP_INFO(logger_, "A* planning successful: %zu waypoints", plan.poses.size());
      } else {
        RCLCPP_WARN(logger_, "Path found but not collision-free");
      }
      
      for (auto& pair : all_nodes) {
        delete pair.second;
      }
      return plan;
    }
    
    closed_set.insert({current->x, current->y});
    
    for (const auto& neighbor : getNeighbors(current->x, current->y, width, height)) {
      if (closed_set.count(neighbor)) {
        continue;
      }
      
      bool is_collision = false;
      for (int dx = -inflation_cells; dx <= inflation_cells && !is_collision; dx++) {
        for (int dy = -inflation_cells; dy <= inflation_cells && !is_collision; dy++) {
          int check_x = neighbor.first + dx;
          int check_y = neighbor.second + dy;
          
          if (check_x >= 0 && check_x < width && check_y >= 0 && check_y < height) {
            if (costmap->getCost(check_x, check_y) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
              is_collision = true;
            }
          }
        }
      }
      
      if (is_collision) {
        continue;
      }
      
      double move_cost = std::hypot(neighbor.first - current->x, neighbor.second - current->y);
      double tentative_g = current->g + move_cost;
      
      Node* neighbor_node = nullptr;
      auto it = all_nodes.find(neighbor);
      if (it != all_nodes.end()) {
        neighbor_node = it->second;
      } else {
        neighbor_node = new Node(neighbor.first, neighbor.second, tentative_g,
                                heuristic(neighbor.first, neighbor.second, goal_map_x, goal_map_y));
        all_nodes[neighbor] = neighbor_node;
      }
      
      if (tentative_g < neighbor_node->g) {
        neighbor_node->g = tentative_g;
        neighbor_node->f = neighbor_node->g + neighbor_node->h;
        neighbor_node->parent = current;
        open_set.push(neighbor_node);
      }
    }
  }
  
  RCLCPP_WARN(logger_, "A* planning failed: no path found");
  
  for (auto& pair : all_nodes) {
    delete pair.second;
  }
  
  return plan;
}

nav_msgs::msg::Path IntpcGlobalPlannerROS::smoothPath(const nav_msgs::msg::Path& path)
{
  if (path.poses.size() < 3) {
    return path;
  }
  
  nav_msgs::msg::Path smoothed_path = path;
  
  int iterations = 10;
  for (int iter = 0; iter < iterations; iter++) {
    for (size_t i = 1; i < smoothed_path.poses.size() - 1; i++) {
      double prev_x = smoothed_path.poses[i-1].pose.position.x;
      double prev_y = smoothed_path.poses[i-1].pose.position.y;
      double curr_x = smoothed_path.poses[i].pose.position.x;
      double curr_y = smoothed_path.poses[i].pose.position.y;
      double next_x = smoothed_path.poses[i+1].pose.position.x;
      double next_y = smoothed_path.poses[i+1].pose.position.y;
      
      double new_x = (1.0 - smoothing_factor_) * curr_x + smoothing_factor_ * (prev_x + next_x) / 2.0;
      double new_y = (1.0 - smoothing_factor_) * curr_y + smoothing_factor_ * (prev_y + next_y) / 2.0;
      
      if (isCollisionFree(new_x, new_y)) {
        smoothed_path.poses[i].pose.position.x = new_x;
        smoothed_path.poses[i].pose.position.y = new_y;
        
        double yaw = std::atan2(next_y - new_y, next_x - new_x);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        smoothed_path.poses[i].pose.orientation = tf2::toMsg(q);
      }
    }
  }
  
  RCLCPP_INFO(logger_, "Path smoothing completed");
  return smoothed_path;
}

nav_msgs::msg::Path IntpcGlobalPlannerROS::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path plan;
  plan.header.stamp = clock_->now();
  plan.header.frame_id = start.header.frame_id;
  
  if (start.header.frame_id != goal.header.frame_id) {
    RCLCPP_ERROR(logger_, "Start and goal poses are in different frames");
    return plan;
  }
  
  double start_x = start.pose.position.x;
  double start_y = start.pose.position.y;
  double goal_x = goal.pose.position.x;
  double goal_y = goal.pose.position.y;
  
  if (std::isnan(start_x) || std::isnan(start_y) || std::isnan(goal_x) || std::isnan(goal_y)) {
    RCLCPP_ERROR(logger_, "Start or goal position contains NaN values");
    return plan;
  }
  
  if (!isCollisionFree(start_x, start_y)) {
    RCLCPP_ERROR(logger_, "Start position is in collision");
    return plan;
  }
  
  planner_->initializePathParams(goal_x, goal_y);
  
  RCLCPP_INFO(logger_, "Using Intpc Fourier path fitting for global planning");
  
  plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  
  double dx = goal_x - start_x;
  double dy = goal_y - start_y;
  int harmonic = 10;
  
  std::vector<double> path_x, path_y, theta;
  int num_waypoints = 20;
  for (int i = 0; i < num_waypoints; i++) {
    double t = static_cast<double>(i) / (num_waypoints - 1);
    double x = start_x + t * dx;
    double y = start_y + t * dy;
    path_x.push_back(x);
    path_y.push_back(y);
    theta.push_back(2.0 * M_PI * t);
  }
  
  std::vector<double> px, py;
  planner_->fourierFit(path_x, path_y, theta, harmonic, px, py);
  
  for (double t = 0.0; t <= 2.0 * M_PI; t += 0.1) {
    double x, y;
    planner_->fourierPathPoint(t, px, py, x, y);
    
    if (isCollisionFree(x, y)) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = plan.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;
      
      double next_x, next_y;
      planner_->fourierPathPoint(t + 0.01, px, py, next_x, next_y);
      double yaw = std::atan2(next_y - y, next_x - x);
      
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.pose.orientation = tf2::toMsg(q);
      
      plan.poses.push_back(pose);
    }
  }
  
  if (plan.poses.empty()) {
    RCLCPP_WARN(logger_, "Fourier path planning failed, returning empty path");
    return plan;
  }
  
  RCLCPP_INFO(logger_, "Fourier path planning successful: %zu waypoints", plan.poses.size());
  
  if (use_smoother_) {
    plan = smoothPath(plan);
  }
  
  std::vector<geometry_msgs::msg::PoseStamped> resampled_poses;
  double accumulated_dist = 0.0;
  
  if (!plan.poses.empty()) {
    resampled_poses.push_back(plan.poses[0]);
  }
  
  for (size_t i = 1; i < plan.poses.size(); i++) {
    double dx = plan.poses[i].pose.position.x - plan.poses[i-1].pose.position.x;
    double dy = plan.poses[i].pose.position.y - plan.poses[i-1].pose.position.y;
    double dist = std::hypot(dx, dy);
    
    if (accumulated_dist + dist >= waypoint_separation_) {
      double ratio = (waypoint_separation_ - accumulated_dist) / dist;
      geometry_msgs::msg::PoseStamped new_pose = plan.poses[i-1];
      new_pose.pose.position.x += dx * ratio;
      new_pose.pose.position.y += dy * ratio;
      
      double yaw = std::atan2(dy, dx);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      new_pose.pose.orientation = tf2::toMsg(q);
      
      resampled_poses.push_back(new_pose);
      accumulated_dist = 0.0;
    } else {
      accumulated_dist += dist;
    }
  }
  
  if (!resampled_poses.empty() && !plan.poses.empty() &&
      (resampled_poses.back().pose.position.x != plan.poses.back().pose.position.x ||
       resampled_poses.back().pose.position.y != plan.poses.back().pose.position.y)) {
    resampled_poses.push_back(plan.poses.back());
  }
  
  plan.poses = resampled_poses;
  
  RCLCPP_INFO(logger_, "Generated global plan with %zu waypoints from (%.2f, %.2f) to (%.2f, %.2f) using Fourier path representation", 
              plan.poses.size(), start_x, start_y, goal_x, goal_y);
  
  return plan;
}

}  // namespace intpc_local_planner

PLUGINLIB_EXPORT_CLASS(intpc_local_planner::IntpcGlobalPlannerROS, nav2_core::GlobalPlanner)
