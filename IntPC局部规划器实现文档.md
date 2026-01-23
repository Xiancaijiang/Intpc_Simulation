# IntPC局部规划器实现文档

## 概述

IntPC（Integrated Path Control）局部规划器是一个基于Navigation2框架的路径跟踪控制器，实现了从全局路径到机器人运动控制的转换。

## 核心特性

- 傅里叶路径表示：使用傅里叶级数表示平滑路径
- 比例-切向-法向控制：基于路径几何特性的跟踪控制
- CBF安全约束：使用控制屏障函数实现实时避障
- QP优化求解：二次规划优化生成最优控制命令
- 动态障碍物响应：实时响应环境中的障碍物变化

## 类定义

### ROS接口类

```cpp
class IntpcLocalPlannerROS : public nav2_core::Controller
{
public:
  IntpcLocalPlannerROS();
  ~IntpcLocalPlannerROS();

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  std::unique_ptr<IntpcLocalPlanner> planner_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("IntpcLocalPlannerROS")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
  std::string global_frame_;
  std::string robot_base_frame_;

  nav_msgs::msg::Path global_plan_;
  double max_vel_x_;
  double max_vel_x_backwards_;
  double max_vel_theta_;
  double acc_lim_x_;
  double acc_lim_theta_;
  double k_gain_;
  double obstacle_radius_;
  double alpha_;
  double l_;
  double d_;
  double max_speed_;
  double goal_tolerance_;

  std::vector<double> px_;
  std::vector<double> py_;
  double vd_;
  double distance_;
  double x_c_;
  double y_c_;
  int harmonic_;

  geometry_msgs::msg::PoseStamped current_pose_;
};
```

### 核心算法类

```cpp
class IntpcLocalPlanner
{
public:
  IntpcLocalPlanner();
  ~IntpcLocalPlanner();

  void initialize(double max_vel_x, double max_vel_theta, 
                  double acc_lim_x, double acc_lim_theta,
                  double k_gain, double obstacle_radius,
                  double alpha, double l, double d, double max_speed);

  void setPathParams(const std::vector<double>& px, const std::vector<double>& py,
                     double vd, double distance, double x_c, double y_c, int harmonic);

  void setCurrentPose(const geometry_msgs::msg::PoseStamped& pose);

  void computeControl(geometry_msgs::msg::Twist& cmd_vel);

  void fourierFit(const std::vector<double>& path_x, const std::vector<double>& path_y,
                   const std::vector<double>& theta, int harmonic,
                   std::vector<double>& px, std::vector<double>& py);

  void fourierPathPoint(double theta, const std::vector<double>& px,
                         const std::vector<double>& py, double& x, double& y);

  void reference(double k, double x, double y, const std::vector<double>& px,
                 const std::vector<double>& py, double vd, double distance,
                 double x_c, double y_c, int harmonic, double& e, Eigen::Vector2d& vf);

  Eigen::Vector2d qp(const std::vector<Eigen::Vector2d>& obstacles,
                     const Eigen::Vector2d& v_ref, double safety_distance);

protected:
  double max_vel_x_;
  double max_vel_theta_;
  double acc_lim_x_;
  double acc_lim_theta_;
  double k_gain_;
  double obstacle_radius_;
  double alpha_;
  double l_;
  double d_;
  double max_speed_;

  std::vector<double> px_;
  std::vector<double> py_;
  double vd_;
  double distance_;
  double x_c_;
  double y_c_;
  int harmonic_;

  geometry_msgs::msg::PoseStamped current_pose_;
};
```

## 核心算法

### 傅里叶路径拟合

```cpp
void IntpcLocalPlanner::fourierFit(const std::vector<double>& path_x, 
                                   const std::vector<double>& path_y,
                                   const std::vector<double>& theta, 
                                   int harmonic,
                                   std::vector<double>& px, 
                                   std::vector<double>& py);
```

使用傅里叶级数对路径进行平滑处理，生成傅里叶系数。

### 路径点计算

```cpp
void IntpcLocalPlanner::fourierPathPoint(double theta, const std::vector<double>& px,
                                         const std::vector<double>& py, double& x, double& y);
```

使用傅里叶系数计算路径上的点，生成平滑的连续路径。

### 参考向量计算

```cpp
void IntpcLocalPlanner::reference(double k, double x, double y, 
                                  const std::vector<double>& px,
                                  const std::vector<double>& py, 
                                  double vd, double distance,
                                  double x_c, double y_c, 
                                  int harmonic, double& e, 
                                  Eigen::Vector2d& vf);
```

基于路径几何特性生成期望速度，包含切向和法向控制分量。

### CBF优化（QP求解）

```cpp
Eigen::Vector2d IntpcLocalPlanner::qp(const std::vector<Eigen::Vector2d>& obstacles,
                                       const Eigen::Vector2d& v_ref,
                                       double safety_distance);
```

使用控制屏障函数确保机器人不会与障碍物碰撞，在安全约束下优化速度。

## 算法流程

1. 接收全局路径：从全局规划器获取路径
2. 傅里叶拟合：将路径转换为傅里叶表示
3. 当前位置获取：获取机器人当前位姿
4. 障碍物检测：提取局部障碍物信息
5. 参考向量计算：基于路径几何计算期望速度
6. CBF优化：在安全约束下优化速度
7. 速度限制：应用物理约束
8. 输出控制：生成最终速度命令

## CBF（控制屏障函数）

### CBF的作用

1. 安全性保障：确保机器人不会与障碍物碰撞
2. 实时避障：响应动态环境中的障碍物变化
3. 优化整合：将安全约束整合到QP优化框架中
4. 理论保证：提供数学上的安全性证明

### CBF数学原理

CBF将障碍物避障转化为以下约束：

```
h(x) = distance(robot, obstacle)^2 - safety_radius^2
ḣ(x) ≥ -α h(x)
```

其中：
- h(x) 是距离障碍物的安全余量
- ḣ(x) 是安全余量的变化率
- α 是控制参数，决定避障的激进程度

### CBF与QP的结合

IntPC将路径跟踪目标和CBF安全约束统一到一个二次规划问题中：

```
min_u ||u - u_d||^2
s.t. ḣ(x) ≥ -α h(x)
```

其中 u_d 是期望的控制输入（来自路径跟踪）。

## 配置参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| max_vel_x | 最大线速度 | 0.5 m/s |
| max_vel_theta | 最大角速度 | 1.0 rad/s |
| acc_lim_x | 线加速度限制 | 2.5 m/s² |
| acc_lim_theta | 角加速度限制 | 3.2 rad/s² |
| k_gain | 比例增益 | 1.0 |
| obstacle_radius | 障碍物安全半径 | 0.25 m |
| alpha | CBF控制参数 | 0.1 |
| l | 机器人参数 | 0.1 |
| d | 机器人参数 | 0.05 |
| max_speed | 最大速度 | 0.5 m/s |
| harmonic | 傅里叶级数阶数 | 10 |

## 使用方法

### 启动局部规划器

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py planner_type:=intpc
```

### 发送导航目标

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"
```

### 查看规划器状态

```bash
ros2 param get /controller_server FollowPath/plugin
```

### 调整规划器参数

```bash
ros2 param set /controller_server FollowPath.max_vel_x 0.8
```

## 代码位置

```
Intpc_local_planner/
├── include/intpc_local_planner/
│   ├── intpc_local_planner.h           # 核心算法头文件
│   └── intpc_local_planner_ros.h       # 局部规划器ROS接口
├── src/
│   ├── intpc_local_planner.cpp         # 核心算法实现
│   └── intpc_local_planner_ros.cpp     # 局部规划器ROS接口实现
└── scripts/                            # 辅助脚本（Python参考实现）
    ├── cbf.py                          # 控制屏障函数实现
    ├── fourier.py                      # 傅里叶变换路径表示
    ├── obstacle.py                     # 障碍物检测
    └── vector.py                       # 参考向量计算
```

## 性能优化建议

1. **k_gain**：增加比例增益可以提高路径跟踪精度，但可能导致震荡
2. **alpha**：增加CBF参数可以使避障更保守，但可能降低路径跟踪性能
3. **harmonic**：增加傅里叶阶数可以生成更平滑的路径，但会增加计算量
4. **obstacle_radius**：增加安全半径可以提高安全性，但可能限制路径选择
