# IntPC全局规划器实现文档

## 概述

IntPC全局规划器（IntpcGlobalPlannerROS）是基于Navigation2框架的自定义全局规划器，结合了A*路径搜索算法和傅里叶路径表示，能够生成平滑、无碰撞的全局路径。

## 核心特性

- A*路径搜索：基于代价地图的全局路径搜索
- 傅里叶路径表示：使用傅里叶级数对路径进行平滑处理
- 碰撞检测：确保路径上的所有点都是无碰撞的
- 路径重采样：按照固定间隔生成路径点
- 与Navigation2无缝集成：实现nav2_core::GlobalPlanner接口

## 类定义

```cpp
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
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;

  double path_resolution_;
  double max_planning_time_;
  double waypoint_separation_;
  double obstacle_inflation_radius_;
  double smoothing_factor_;
  bool use_smoother_;

  bool isCollisionFree(double x, double y);
  bool isPathCollisionFree(const std::vector<std::pair<int, int>>& path);
  nav_msgs::msg::Path aStarPlanning(double start_x, double start_y, double goal_x, double goal_y);
  nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& path);
  double heuristic(int x1, int y1, int x2, int y2);
  std::vector<std::pair<int, int>> getNeighbors(int x, int y, int width, int height);
};
```

## 核心算法

### A*路径搜索

使用A*算法在代价地图上搜索最优路径，考虑障碍物膨胀半径，确保路径安全性。

### 傅里叶路径拟合

```cpp
void IntpcLocalPlanner::fourierFit(const std::vector<double>& path_x, 
                                   const std::vector<double>& path_y,
                                   const std::vector<double>& theta, 
                                   int harmonic,
                                   std::vector<double>& px, 
                                   std::vector<double>& py);
```

使用傅里叶级数对A*生成的路径进行平滑处理，生成傅里叶系数。

### 路径点计算

```cpp
void IntpcLocalPlanner::fourierPathPoint(double theta, const std::vector<double>& px,
                                         const std::vector<double>& py, double& x, double& y);
```

使用傅里叶系数计算路径上的点，生成平滑的连续路径。

### 碰撞检测

```cpp
bool IntpcGlobalPlannerROS::isCollisionFree(double x, double y);
```

检查指定位置是否无碰撞，确保路径安全性。

## 路径生成流程

1. 输入验证：检查起点和终点位置是否有效
2. A*路径搜索：在代价地图上使用A*算法搜索最优路径
3. 傅里叶拟合：使用傅里叶级数对路径进行平滑处理
4. 碰撞检测：验证路径上的所有点都是无碰撞的
5. 路径重采样：按照固定间隔生成路径点
6. 输出路径：返回平滑的全局路径

## 配置参数

| 参数名称 | 类型 | 默认值 | 描述 |
|---------|------|-------|------|
| tolerance | double | 0.5 | 目标点容差（米） |
| use_astar | bool | false | 是否使用A*算法 |
| allow_unknown | bool | true | 是否允许穿过未知区域 |
| path_resolution | double | 0.05 | 路径分辨率（米） |
| max_planning_time | double | 5.0 | 最大规划时间（秒） |
| waypoint_separation | double | 0.2 | 航点间距（米） |
| obstacle_inflation_radius | double | 0.25 | 障碍物膨胀半径（米） |
| smoothing_factor | double | 0.3 | 平滑因子 |
| use_smoother | bool | true | 是否使用路径平滑器 |

## 使用方法

### 启动命令

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py planner_type:=intpc_global_dwb_local
```

### 验证插件加载

```bash
ros2 param get /planner_server GridBased/plugin
```

## 代码位置

```
Intpc_local_planner/
├── include/intpc_local_planner/
│   └── intpc_global_planner_ros.h      # 全局规划器头文件
├── src/
│   └── intpc_global_planner_ros.cpp     # 全局规划器实现
└── intpc_local_planner_plugin.xml      # 插件注册文件
```

## 性能优化建议

1. **path_resolution**：增加分辨率可以生成更精细的路径，但会增加计算时间
2. **max_planning_time**：增加最大规划时间可以生成更优路径，但会降低实时性
3. **waypoint_separation**：调整航点间距可以平衡路径平滑度和计算效率
4. **obstacle_inflation_radius**：增加膨胀半径可以提高安全性，但可能限制路径选择
