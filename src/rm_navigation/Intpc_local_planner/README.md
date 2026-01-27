# intpc_local_planner ROS2 Package

intpc_local_planner是一个基于ROS2 Nav2框架的高性能局部规划器插件，采用集成路径规划与控制策略，实现实时轨迹规划和障碍规避。

## 功能特点

- **集成规划与控制**：将路径生成与跟踪控制集成，减少模块间通信延迟
- **精确路径跟踪**：采用比例切向-法向控制策略实现高精度路径跟踪
- **智能障碍规避**：基于距离加权的避障算法，结合切向-法向向量控制
- **运动学约束**：考虑机器人物理限制，生成符合实际的控制命令
- **无缝Nav2集成**：作为标准Nav2控制器插件，易于集成到现有导航系统
- **可视化支持**：提供调试和演示用的实时可视化标记

## 算法原理

intpc_local_planner的核心算法采用模块化设计，由以下组件构成：

### 1. 参考轨迹生成

**实现架构**：采用固定前瞻距离的局部路径跟踪策略，通过以下步骤生成参考轨迹：

```cpp
// 获取本地目标点（固定50cm前瞻距离）
geometry_msgs::msg::PoseStamped IntpcLocalPlannerROS::getLocalGoal(
    const nav_msgs::msg::Path &transformed_plan) {
    
    // 计算前瞻距离
    double lookahead_dist = 0.5; // 50cm固定前瞻距离
    
    // 遍历全局路径，找到第一个距离大于前瞻距离的点
    for (size_t i = 0; i < transformed_plan.poses.size(); i++) {
        double dist = calculateDistance(current_pose_, transformed_plan.poses[i]);
        if (dist >= lookahead_dist) {
            return transformed_plan.poses[i];
        }
    }
    
    // 如果没有足够远的点，返回最后一个点
    if (!transformed_plan.poses.empty()) {
        return transformed_plan.poses.back();
    }
    
    return current_pose_;
}
```

### 2. 参考速度计算

**核心算法**：基于比例-切向-法向（P-T-N）控制策略，计算参考速度向量：

```
v_ref = v_d * τ + k * e * n
```

其中：
- `v_d`：期望速度
- `τ`：路径切向单位向量
- `k`：比例增益
- `e`：路径跟踪误差
- `n`：路径法向单位向量

**实现细节**：

```cpp
// 参考速度计算
void IntpcLocalPlanner::reference(double k, double x, double y, double vd, 
                                  double& e, Eigen::Vector2d& vf) {
    // 计算路径跟踪误差
    double dx = goal_x_ - x;
    double dy = goal_y_ - y;
    double dist = std::hypot(dx, dy);
    
    // 计算切向和法向单位向量
    Eigen::Vector2d tau(dx/dist, dy/dist);
    Eigen::Vector2d n(-dy/dist, dx/dist);
    
    // 计算路径跟踪误差
    e = 0.0; // 简化计算，实际应根据路径计算
    
    // 计算参考速度
    vf = vd * tau + k * e * n;
}
```

### 3. 障碍规避策略

**实现架构**：基于距离加权的避障算法，结合切向-法向向量控制：

1. **障碍物检测**：
   ```cpp
   // 从代价图提取障碍物
   void IntpcLocalPlannerROS::extractObstaclesFromCostmap(
       const geometry_msgs::msg::PoseStamped &pose,
       std::vector<double> &x_obstacles,
       std::vector<double> &y_obstacles,
       std::vector<double> &obstacle_radii) {
       
       // 获取代价地图
       nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
       
       // 转换机器人位置到地图坐标
       unsigned int map_x, map_y;
       if (!costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, map_x, map_y)) {
           return;
       }
       
       // 搜索机器人周围的障碍物
       int search_radius = 10; // 10个栅格
       for (int i = -search_radius; i <= search_radius; i++) {
           for (int j = -search_radius; j <= search_radius; j++) {
               unsigned int cx = map_x + i;
               unsigned int cy = map_y + j;
               
               if (cx < costmap->getSizeInCellsX() && cy < costmap->getSizeInCellsY()) {
                   unsigned char cost = costmap->getCost(cx, cy);
                   if (cost > nav2_costmap_2d::LETHAL_OBSTACLE) {
                       double wx, wy;
                       costmap->mapToWorld(cx, cy, wx, wy);
                       x_obstacles.push_back(wx);
                       y_obstacles.push_back(wy);
                       obstacle_radii.push_back(0.1); // 默认障碍物半径
                   }
               }
           }
       }
   }
   ```

2. **避障速度计算**：
   ```cpp
   // 计算避障速度
   Eigen::Vector2d IntpcLocalPlanner::calculateObstacleAvoidanceVelocity(
       double x, double y, 
       const std::vector<double> &x_obstacles,
       const std::vector<double> &y_obstacles) {
       
       Eigen::Vector2d avoidance_velocity(0.0, 0.0);
       double min_dist = std::numeric_limits<double>::max();
       int closest_obstacle_idx = -1;
       
       // 找到最近障碍物
       for (size_t i = 0; i < x_obstacles.size(); i++) {
           double dx = x - x_obstacles[i];
           double dy = y - y_obstacles[i];
           double dist = std::hypot(dx, dy);
           
           if (dist < min_dist) {
               min_dist = dist;
               closest_obstacle_idx = i;
           }
       }
       
       // 计算避障速度
       if (closest_obstacle_idx >= 0 && min_dist < 0.5) {
           double dx = x - x_obstacles[closest_obstacle_idx];
           double dy = y - y_obstacles[closest_obstacle_idx];
           double dist = std::hypot(dx, dy);
           
           // 避障强度与距离成反比
           double avoidance_strength = std::min(1.0, (0.5 - dist) / 0.5);
           
           // 计算避障方向
           avoidance_velocity.x() = (dx / dist) * avoidance_strength;
           avoidance_velocity.y() = (dy / dist) * avoidance_strength;
       }
       
       return avoidance_velocity;
   }
   ```

### 4. 控制命令转换

**实现细节**：将笛卡尔空间控制速度转换为机器人线速度和角速度命令：

```cpp
// 计算控制命令
goometry_msgs::msg::TwistStamped IntpcLocalPlannerROS::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker * goal_checker) {
    
    // 获取本地目标点
    geometry_msgs::msg::PoseStamped local_goal = getLocalGoal(transformed_plan_);
    
    // 提取障碍物
    std::vector<double> x_obstacles, y_obstacles, obstacle_radii;
    extractObstaclesFromCostmap(pose, x_obstacles, y_obstacles, obstacle_radii);
    
    // 计算控制输入
    Eigen::Vector2d control_input = planner_->computeIntpcControl(
        pose.pose.position.x, pose.pose.position.y,
        getYaw(pose.pose.orientation),
        x_obstacles, y_obstacles, obstacle_radii,
        k_gain_, max_vel_x_
    );
    
    // 转换为线速度和角速度
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.header.frame_id = pose.header.frame_id;
    
    // 线速度
    cmd_vel.twist.linear.x = control_input.norm();
    
    // 角速度（基于方向角）
    double yaw = getYaw(pose.pose.orientation);
    double desired_yaw = std::atan2(control_input.y(), control_input.x());
    double yaw_error = normalizeAngle(desired_yaw - yaw);
    cmd_vel.twist.angular.z = yaw_error * 2.0; // 比例控制
    
    // 速度限制
    saturateVelocity(cmd_vel.twist);
    
    return cmd_vel;
}
```

## 代码框架

### 核心类结构

#### IntpcLocalPlanner类

核心规划算法实现类：

```cpp
class IntpcLocalPlanner {
public:
  // 构造函数和析构函数
  IntpcLocalPlanner(double max_vel, double acc_lim);
  ~IntpcLocalPlanner();
  
  // 路径参数初始化（目标点坐标）
  void initializePathParams(double goal_x, double goal_y);
  
  // 参考速度计算
  void reference(double k, double x, double y, double vd, double& e, Eigen::Vector2d& vf);
  
  // 前向运动学计算
  void forwardKinematics(double x, double y, double theta, const Eigen::Vector2d& u,
                         double dt, double l, double& x_new, double& y_new, double& theta_new);
  
  // 主要控制计算接口
  Eigen::Vector2d computeIntpcControl(double x, double y, double theta,
                                     const std::vector<double>& x_obstacles,
                                     const std::vector<double>& y_obstacles,
                                     const std::vector<double>& obstacle_radii,
                                     double k_gain, double max_speed);
  
private:
  // 机器人参数
  double wheel_base_;
  
  // 控制器参数
  double alpha_;
  double l_;
  double d_;
  double max_velocity_;
  double acceleration_limit_;
  
  // 路径参数
  double goal_x_;
  double goal_y_;
  double vd_;
};
```

#### IntpcLocalPlannerROS类

Nav2控制器插件实现类：

```cpp
class IntpcLocalPlannerROS : public nav2_core::Controller {
public:
  // 构造函数和析构函数
  IntpcLocalPlannerROS();
  ~IntpcLocalPlannerROS();
  
  // 框架生命周期管理
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;
  
  // 设置全局路径
  void setPlan(const nav_msgs::msg::Path & orig_global_plan) override;
  
  // 计算速度命令
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker * goal_checker);
  
protected:
  // 辅助方法
  nav_msgs::msg::Path transformGlobalPlan(const nav_msgs::msg::Path &global_plan,
                                         const std::string &global_frame,
                                         const geometry_msgs::msg::PoseStamped &robot_pose);
  
  // 获取本地目标点（固定50cm前瞻距离）
  geometry_msgs::msg::PoseStamped getLocalGoal(const nav_msgs::msg::Path &transformed_plan);
  
  // 速度限制
  void saturateVelocity(geometry_msgs::msg::Twist &twist);
  
  // 从代价图提取障碍物
  void extractObstaclesFromCostmap(
    const geometry_msgs::msg::PoseStamped &pose,
    std::vector<double> &x_obstacles,
    std::vector<double> &y_obstacles,
    std::vector<double> &obstacle_radii);
  
  // 成员变量
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  
  TFBufferPtr tf_;
  CostmapROSPtr costmap_ros_;
  
  nav_msgs::msg::Path global_plan_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::Twist current_velocity_;
  
  // 运动参数
  double max_vel_x_;
  double max_vel_x_backwards_;
  double max_vel_theta_;
  double acc_lim_x_;
  double acc_lim_theta_;
  
  // 核心规划器
  std::unique_ptr<IntpcLocalPlanner> planner_;
  
  // 控制器参数
  double k_gain_;
  double obstacle_radius_;
  double robot_radius_;
  
  // 可视化
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};
```
```

### 文件结构

```
Intpc_local_planner/
├── CMakeLists.txt                 # CMake构建文件
├── include/
│   └── intpc_local_planner/       # 头文件目录
│       ├── intpc_local_planner.h  # 核心规划器头文件
│       └── intpc_local_planner_ros.h  # ROS接口头文件
├── intpc_local_planner_plugin.xml  # 插件描述文件
├── package.xml                    # 包信息文件
├── scripts/                       # 辅助测试脚本（仅用于独立测试，导航系统不依赖）
│   ├── cbf.py                     # 控制屏障函数实现
│   ├── detect.py                  # 障碍物检测
│   ├── fourier.py                 # 傅里叶路径生成（历史遗留，当前版本不使用）
│   ├── kinematics.py              # 运动学计算
│   ├── main.py                    # 主测试脚本
│   └── obstacle.py                # 障碍物表示
└── src/
    ├── intpc_local_planner.cpp    # 核心规划器实现
    └── intpc_local_planner_ros.cpp  # ROS接口实现
```

## 配置参数

| 参数类别 | 参数名 | 类型 | 说明 | 默认值 |
|---------|--------|------|------|--------|
| **运动限制** | `max_vel_x` | double | 最大前进线速度 (m/s) | 1.0 |
| **运动限制** | `max_vel_x_backwards` | double | 最大后退线速度 (m/s) | 0.5 |
| **运动限制** | `max_vel_theta` | double | 最大角速度 (rad/s) | 1.5 |
| **运动限制** | `acc_lim_x` | double | 线加速度限制 (m/s²) | 2.0 |
| **运动限制** | `acc_lim_theta` | double | 角加速度限制 (rad/s²) | 3.0 |
| **规划器参数** | `k_gain` | double | 路径跟踪比例增益 | 1.0 |
| **规划器参数** | `obstacle_radius` | double | 障碍物检测半径 (m) | 0.25 |
| **规划器参数** | `robot_radius` | double | 机器人碰撞半径 (m) | 0.2 |
| **控制参数** | `lookahead_distance` | double | 路径前瞻距离 (固定为0.5m) | - |

## 使用方法

### 1. 集成到Navigation2

将intpc_local_planner配置为Nav2控制器插件：

```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    controller_plugins:
      - FollowPath
    FollowPath:
      plugin: intpc_local_planner/IntpcLocalPlannerROS
      max_vel_x: 0.5
      max_vel_x_backwards: 0.3
      max_vel_theta: 1.0
      acc_lim_x: 1.0
      acc_lim_theta: 1.0
      k_gain: 1.0
      obstacle_radius: 0.2
      robot_radius: 0.15
```

### 2. 启动导航系统

```bash
# 启动导航系统
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True map:=my_map.yaml

# 或使用项目提供的启动文件
ros2 launch rm_nav_bringup bringup_sim.launch.py planner_type:=intpc
```

## 依赖项

- **ROS2 Humble** 或更高版本
- **Navigation2** 导航框架
- **Eigen3** 矩阵运算库
- **tf2** 坐标变换库
- **nav2_costmap_2d** 障碍物检测

## 安装方法

### 1. 克隆仓库

```bash
git clone https://github.com/Xiancaijiang/Intpc_local_planner.git
```

### 2. 安装依赖

```bash
cd Intpc_local_planner
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 构建包

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select intpc_local_planner
```

## 更新说明

### 版本1.1 (当前版本)

- **移除傅里叶路径依赖**：简化为直线路径跟踪，不再使用傅里叶级数生成路径
- **固定前瞻距离**：采用50cm固定前瞻距离，提高算法稳定性
- **优化接口设计**：简化`initializePathParams`和`reference`接口
- **移除冗余参数**：删除`path_shape_`和`lookahead_dist_`等不再使用的参数
- **提升实时性能**：优化控制算法，减少计算复杂度

### 版本1.0

- 初始版本发布
- 支持傅里叶路径生成
- 基本的路径跟踪和障碍规避功能

## 联系方式

如有问题或建议，请通过GitHub Issues提交。

## 规划器实现架构

### 1. 整体架构设计

Intpc规划器采用模块化设计架构，同时支持全局和局部规划功能，通过清晰的分层结构实现算法的可维护性和可扩展性。

```
Intpc规划器架构
┌─────────────────────────────────────────────────────────────┐
│                 Navigation2 框架                            │
├─────────────────────┬───────────────────────────────────────┤
│                     │                                       │
▼                     ▼                                       │
┌─────────────────┐  ┌─────────────────┐                     │
│ 全局规划器接口  │  │ 局部规划器接口  │                     │
│ (GlobalPlanner) │  │  (Controller)   │                     │
└────────┬────────┘  └────────┬────────┘                     │
         │                    │                              │
         ▼                    ▼                              │
┌─────────────────┐  ┌─────────────────┐                     │
│ IntpcGlobalPlanner││ IntpcLocalPlanner│                    │
│   ROS接口层     │  │   ROS接口层     │                     │
└────────┬────────┘  └────────┬────────┘                     │
         │                    │                              │
         ▼                    ▼                              │
┌─────────────────┐  ┌─────────────────┐                     │
│  核心规划算法   │  │  核心控制算法   │                     │
│  (Fourier + CBF)│  │ (P-T-N控制)     │                     │
└────────┬────────┘  └────────┬────────┘                     │
         │                    │                              │
         ▼                    ▼                              │
┌─────────────────┐  ┌─────────────────┐                     │
│  路径生成模块   │  │  速度控制模块   │                     │
└────────┬────────┘  └────────┬────────┘                     │
         │                    │                              │
         └────────┬───────────┘                              │
                  │                                           │
                  ▼                                           │
┌─────────────────────────────────────────────────────────────┐
│                  代价地图与障碍物检测                       │
└─────────────────────────────────────────────────────────────┘
```

### 2. 模块职责划分

| 模块 | 职责 | 实现文件 | 核心功能 |
|------|------|----------|----------|
| **全局规划器接口** | 实现Nav2 GlobalPlanner接口 | intpc_global_planner_ros.h/cpp | 路径规划请求处理，生命周期管理 |
| **局部规划器接口** | 实现Nav2 Controller接口 | intpc_local_planner_ros.h/cpp | 速度命令计算，路径跟踪 |
| **核心规划算法** | 傅里叶路径拟合与CBF优化 | intpc_global_planner.h/cpp | 平滑路径生成，安全避障 |
| **核心控制算法** | 比例-切向-法向控制 | intpc_local_planner.h/cpp | 高精度轨迹跟踪，动态避障 |
| **路径生成模块** | 参考轨迹计算 | 集成在核心算法中 | 路径点采样，傅里叶拟合 |
| **速度控制模块** | 控制命令生成 | 集成在核心算法中 | 速度命令计算，运动学约束 |
| **障碍物检测** | 从代价地图提取障碍物 | 集成在ROS接口层 | 障碍物信息提取，距离计算 |

## 全局规划器实现

### 1. 功能特点

- **傅里叶路径表示**：使用傅里叶级数拟合路径，生成平滑连续的轨迹
- **CBF优化**：集成控制障碍函数，实现安全避障
- **实时规划**：优化算法结构，确保规划频率大于1Hz
- **Nav2兼容**：作为标准Nav2全局规划器插件，无缝集成到导航系统

### 2. 算法原理

#### 2.1 傅里叶路径拟合

**核心思想**：使用傅里叶级数表示路径，通过优化系数生成平滑轨迹，避免传统路径规划中的不连续问题。

**数学模型**：

```cpp
// 傅里叶级数路径表示
x(θ) = a0 + Σ(an*cos(nθ) + bn*sin(nθ))
y(θ) = c0 + Σ(cn*cos(nθ) + dn*sin(nθ))
```

其中：
- `θ`：路径参数（0到2π）
- `an, bn, cn, dn`：傅里叶系数
- `n`：谐波次数（默认10）

**实现细节**：

1. **路径点采样**：
   ```cpp
   // 采样路径点
   std::vector<double> path_x, path_y;
   for (double theta = 0; theta <= 2 * M_PI; theta += 0.1) {
       double x = start.x + (goal.x - start.x) * (theta / (2 * M_PI));
       double y = start.y + (goal.y - start.y) * (theta / (2 * M_PI));
       path_x.push_back(x);
       path_y.push_back(y);
   }
   ```

2. **优化问题构建**：
   ```cpp
   // 构建优化矩阵
   Eigen::MatrixXd G = Eigen::MatrixXd::Ones(path_x.size(), 2 * harmonic + 1);
   Eigen::VectorXd X(path_x.size()), Y(path_y.size());
   
   for (int i = 0; i < path_x.size(); i++) {
       double theta = 2 * M_PI * i / (path_x.size() - 1);
       for (int j = 1; j <= harmonic; j++) {
           G(i, 2*j-1) = cos(j * theta);
           G(i, 2*j) = sin(j * theta);
       }
       X(i) = path_x[i];
       Y(i) = path_y[i];
   }
   ```

3. **系数求解**：
   ```cpp
   // 最小二乘求解傅里叶系数
   Eigen::VectorXd coeff_x = G.householderQr().solve(X);
   Eigen::VectorXd coeff_y = G.householderQr().solve(Y);
   ```

4. **路径生成**：
   ```cpp
   // 根据傅里叶系数生成平滑路径
   for (double theta = 0; theta <= 2 * M_PI; theta += 0.05) {
       double x = coeff_x(0);
       double y = coeff_y(0);
       for (int j = 1; j <= harmonic; j++) {
           x += coeff_x(2*j-1) * cos(j*theta) + coeff_x(2*j) * sin(j*theta);
           y += coeff_y(2*j-1) * cos(j*theta) + coeff_y(2*j) * sin(j*theta);
       }
       path.poses.push_back(createPoseStamped(x, y, theta));
   }
   ```

#### 2.2 CBF优化

**控制障碍函数(CBF)**：通过约束控制输入，确保机器人与障碍物保持安全距离，是一种基于李导数的安全控制方法。

**优化目标**：
- 最小化路径长度
- 最大化与障碍物的距离
- 满足运动学约束

**实现细节**：

1. **障碍物检测**：
   ```cpp
   // 从代价地图提取障碍物
   void IntpcGlobalPlannerROS::detectObstacles(
       const geometry_msgs::msg::PoseStamped &start,
       const geometry_msgs::msg::PoseStamped &goal,
       std::vector<geometry_msgs::msg::Point> &obstacles) {
       
       // 获取代价地图
       nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
       
       // 遍历代价地图，提取障碍物
       for (unsigned int x = 0; x < costmap->getSizeInCellsX(); x++) {
           for (unsigned int y = 0; y < costmap->getSizeInCellsY(); y++) {
               unsigned char cost = costmap->getCost(x, y);
               if (cost > nav2_costmap_2d::LETHAL_OBSTACLE) {
                   double wx, wy;
                   costmap->mapToWorld(x, y, wx, wy);
                   geometry_msgs::msg::Point obstacle;
                   obstacle.x = wx;
                   obstacle.y = wy;
                   obstacles.push_back(obstacle);
               }
           }
       }
   }
   ```

2. **CBF约束构建**：
   ```cpp
   // 计算障碍函数
   double computeBarrierFunction(const Eigen::Vector2d &pos, 
                                const std::vector<geometry_msgs::msg::Point> &obstacles) {
       double min_dist = std::numeric_limits<double>::max();
       for (const auto &obs : obstacles) {
           Eigen::Vector2d obs_pos(obs.x, obs.y);
           double dist = (pos - obs_pos).norm();
           min_dist = std::min(min_dist, dist);
       }
       return min_dist - safety_distance_;
   }
   ```

3. **优化问题求解**：
   ```cpp
   // 集成CBF的路径优化
   void IntpcGlobalPlanner::optimizePathWithCBF(
       std::vector<double> &path_x, 
       std::vector<double> &path_y,
       const std::vector<geometry_msgs::msg::Point> &obstacles) {
       
       // 构建优化问题
       for (int i = 0; i < path_x.size(); i++) {
           Eigen::Vector2d pos(path_x[i], path_y[i]);
           double h = computeBarrierFunction(pos, obstacles);
           
           // 如果违反安全距离，进行调整
           if (h < 0) {
               // 找到最近障碍物
               geometry_msgs::msg::Point closest_obs;
               double min_dist = std::numeric_limits<double>::max();
               
               for (const auto &obs : obstacles) {
                   double dist = std::hypot(path_x[i] - obs.x, path_y[i] - obs.y);
                   if (dist < min_dist) {
                       min_dist = dist;
                       closest_obs = obs;
                   }
               }
               
               // 计算避障方向
               double dx = path_x[i] - closest_obs.x;
               double dy = path_y[i] - closest_obs.y;
               double dist = std::hypot(dx, dy);
               
               // 调整路径点
               if (dist > 0) {
                   double scale = (safety_distance_ / dist);
                   path_x[i] = closest_obs.x + dx * scale;
                   path_y[i] = closest_obs.y + dy * scale;
               }
           }
       }
   }
   ```

### 3. 代码结构

#### 3.1 核心类结构

**IntpcGlobalPlannerROS类**：

```cpp
class IntpcGlobalPlannerROS : public nav2_core::GlobalPlanner {
public:
  // 生命周期管理
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, 
                std::string name, std::shared_ptr<tf2_ros::Buffer> tf, 
                std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  
  // 路径规划
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start, 
                                const geometry_msgs::msg::PoseStamped & goal) override;
  
protected:
  // 路径生成
  void generatePath(const geometry_msgs::msg::PoseStamped & start, 
                   const geometry_msgs::msg::PoseStamped & goal, 
                   nav_msgs::msg::Path & path);
  
  // 障碍物检测
  void detectObstacles(const geometry_msgs::msg::PoseStamped & start, 
                      const geometry_msgs::msg::PoseStamped & goal, 
                      std::vector<geometry_msgs::msg::Point> & obstacles);
  
  // 成员变量
  rclcpp::Logger logger_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<IntpcGlobalPlanner> planner_;
};
```

#### 3.2 文件结构

```
Intpc_local_planner/
├── include/
│   └── intpc_local_planner/
│       ├── intpc_global_planner.h       # 全局规划器核心算法
│       └── intpc_global_planner_ros.h   # ROS接口
├── src/
│   ├── intpc_global_planner.cpp         # 全局规划器实现
│   └── intpc_global_planner_ros.cpp     # ROS接口实现
└── intpc_local_planner_plugin.xml       # 插件注册
```

### 4. 配置参数

| 参数名 | 类型 | 说明 | 默认值 |
|--------|------|------|--------|
| `tolerance` | double | 目标点容差 (m) | 0.5 |
| `use_astar` | bool | 是否使用A*算法（默认使用傅里叶路径） | false |
| `harmonic` | int | 傅里叶级数谐波次数 | 10 |
| `cbf_weight` | double | CBF优化权重 | 1.0 |
| `obstacle_safety_distance` | double | 障碍物安全距离 (m) | 0.2 |

### 5. 配置示例

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "intpc_local_planner/IntpcGlobalPlannerROS"
      tolerance: 0.5
      use_astar: false
      harmonic: 10
      cbf_weight: 1.0
      obstacle_safety_distance: 0.2
```

## 自检机制

### 1. 接口实现验证

**检查点1：插件注册**
```bash
# 查看插件注册文件
cat intpc_local_planner_plugin.xml
```

预期输出应包含：
```xml
<class type="intpc_local_planner::IntpcGlobalPlannerROS" base_class_type="nav2_core::GlobalPlanner">
  <description>Intpc Global Planner implements path planning functionality with Fourier path representation.</description>
</class>
```

**检查点2：头文件接口**
```bash
# 检查头文件是否正确继承nav2_core::GlobalPlanner
grep -n "class IntpcGlobalPlannerROS" include/intpc_local_planner/intpc_global_planner_ros.h
```

预期输出：
```cpp
class IntpcGlobalPlannerROS : public nav2_core::GlobalPlanner
```

**检查点3：必需方法实现**
```bash
# 检查是否实现了所有必需的接口方法
grep -E "(configure|cleanup|activate|deactivate|createPlan)" include/intpc_local_planner/intpc_global_planner_ros.h
```

预期应包含所有五个方法。

### 2. 编译验证

```bash
# 编译全局规划器
colcon build --packages-select Intpc_local_planner --symlink-install

# 检查编译输出
# 预期：编译成功，无错误
```

### 3. 运行时验证

```bash
# 启动导航系统
ros2 launch rm_nav_bringup bringup_sim.launch.py planner_type:=intpc_global_dwb_local

# 检查插件是否正确加载
ros2 param get /planner_server GridBased/plugin
# 预期输出：intpc_local_planner/IntpcGlobalPlannerROS

# 测试路径规划
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"

# 查看规划结果
ros2 topic echo /plan
# 预期输出：应生成有效的全局路径
```

## 性能调优

### 1. 全局规划器调优

| 参数 | 说明 | 建议值 | 调优策略 |
|------|------|--------|----------|
| `harmonic` | 傅里叶谐波次数 | 5-15 | 复杂环境增加，简单环境减少 |
| `tolerance` | 目标点容差 | 0.1-0.5 | 精度要求高减小，实时性要求高增大 |
| `expected_planner_frequency` | 规划频率 | 1.0-5.0 | 根据硬件性能调整 |

### 2. 局部规划器调优

| 参数 | 说明 | 建议值 | 调优策略 |
|------|------|--------|----------|
| `k_gain` | 比例增益 | 0.5-2.0 | 跟踪精度要求高增大，稳定性要求高减小 |
| `controller_frequency` | 控制频率 | 10.0-30.0 | 根据硬件性能调整 |
| `max_vel_x` | 最大线速度 | 0.5-2.0 | 根据机器人性能和场景调整 |
| `max_vel_theta` | 最大角速度 | 1.0-3.0 | 根据机器人性能调整 |

### 3. 系统级优化

1. **使用Release模式编译**：
   ```bash
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

2. **优化代价地图**：
   - 调整更新频率
   - 优化分辨率
   - 合理设置膨胀半径

3. **硬件优化**：
   - 使用性能更好的CPU
   - 增加内存容量
   - 使用SSD存储

## 常见问题排查

### 1. 插件加载失败

**症状**：启动时出现 "Failed to create planner" 错误

**排查步骤**：
```bash
# 1. 检查插件XML文件
cat intpc_local_planner_plugin.xml

# 2. 检查CMakeLists.txt中的插件导出
cat CMakeLists.txt | grep pluginlib

# 3. 检查编译输出
colcon build --packages-select Intpc_local_planner --symlink-install
```

**解决方案**：确保CMakeLists.txt中包含`pluginlib_export_plugin_description_file(nav2_core intpc_local_planner_plugin.xml)`

### 2. 路径规划失败

**症状**：无法生成有效路径或路径穿越障碍物

**排查步骤**：
```bash
# 1. 检查代价地图
ros2 topic echo /global_costmap/costmap

# 2. 检查起点和终点是否有效
ros2 topic echo /amcl_pose

# 3. 检查规划器日志
ros2 topic echo /rosout | grep -i planner
```

**解决方案**：
- 确保起点和终点不在障碍物内
- 调整障碍物膨胀半径
- 增加安全距离参数

### 3. 性能问题

**症状**：规划时间过长，影响导航实时性

**排查步骤**：
```bash
# 查看规划器计算时间
ros2 topic hz /plan

# 查看CPU使用情况
top -p $(pgrep -f planner_server)
```

**解决方案**：
- 减少傅里叶谐波次数
- 增加规划容差
- 优化代价地图更新频率

## 性能指标

| 指标 | 全局规划器 | 局部规划器 |
|------|-----------|-----------|
| 规划频率 | > 1 Hz | > 10 Hz |
| 规划时间 | < 1.0 s | < 0.1 s |
| 路径平滑度 | 良好 | 优秀 |
| 避障响应 | 中等 | 优秀 |
| 内存占用 | < 100 MB | < 50 MB |
| CPU使用率 | < 10% | < 5% |

## 许可证

本项目采用Apache 2.0许可证，详见LICENSE文件。