# Intpc_local_planner

## 一. 项目介绍

本项目是一个基于ROS2 Nav2框架的高性能导航解决方案，专为RoboMaster等机器人竞赛和工业场景设计。支持多种局部规划器（包括TEB和自主研发的IntPC规划器）的集成与切换，具备实时定位、建图和导航能力。

### 项目特点

- **多规划器支持**：内置TEB和IntPC两种局部规划器，支持用户自定义扩展
- **多传感器融合**：支持LiDAR（如Livox Mid360）的高精度定位与建图
- **实时性能优异**：支持100+Hz的里程计输出，确保导航实时性
- **仿真与实车兼容**：统一的接口设计，支持Gazebo仿真和真实环境部署
- **灵活配置**：丰富的参数配置，适应不同场景需求

## 二. 环境配置

### 支持环境
- **操作系统**：Ubuntu 22.04
- **ROS版本**：ROS2 Humble
- **仿真环境**：Gazebo Classic 11.10.0

### 安装步骤

1. **克隆仓库（包含子模块）**
    ```sh
    # 方法1：直接克隆时包含子模块（推荐）
c

    # 方法2：先克隆主仓库，然后初始化子模块
    git clone https://github.com/Xiancaijiang/Intpc_local_planner.git
    cd Intpc_local_planner
    git submodule update --init --recursive
    ```

2. **安装Livox SDK2**（用于LiDAR支持）
    ```sh
    sudo apt install cmake
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd Livox-SDK2 && mkdir build && cd build
    cmake .. && make -j && sudo make install
    ```

3. **安装项目依赖**
    ```sh
    cd Intpc_local_planner
    rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

4. **编译项目**
    ```sh
    colcon build --symlink-install
    ```

## 三. 运行

### 3.1 核心参数说明

| 参数名 | 可选值 | 说明 |
|--------|--------|------|
| `world` | `RMUL`/`RMUC`/自定义 | 仿真模式下选择场地，真实环境下为地图文件名 |
| `mode` | `mapping`/`nav` | `mapping`：边建图边导航<br>`nav`：已知地图导航 |
| `lio` | `fastlio`/`pointlio` | 激光里程计算法<br>`fastlio`：~10Hz，资源占用低<br>`pointlio`：~100Hz，定位更平滑 |
| `localization` | `slam_toolbox`/`amcl`/`icp` | 定位算法（仅`mode:=nav`有效） |
| `lio_rviz` | `True`/`False` | 是否可视化LiDAR点云 |
| `nav_rviz` | `True`/`False` | 是否可视化导航信息 |
| `planner_type` | `dwb`/`teb`/`intpc`/`intpc_global_dwb_local`/`<planner_config>` | 规划器配置选择<br>`dwb`：全局NavfnPlanner + 局部DWB<br>`teb`：全局NavfnPlanner + 局部TEB<br>`intpc`：全局NavfnPlanner + 局部Intpc<br>`intpc_global_dwb_local`：全局Intpc + 局部DWB（默认）<br>`<planner_config>`：自定义规划器配置 |

### 定位算法说明

- **slam_toolbox**：动态场景下定位效果好，支持回环检测
- **amcl**：经典概率定位算法，启动需手动给定初始位姿
- **icp**：基于点云配准的定位，仅初始定位时使用，长时间运行可能有累积误差

### 规划器配置说明

本项目支持四种规划器配置，每种配置使用不同的全局和局部规划器组合：

| 配置类型 | 全局规划器 | 局部规划器 | 特点 |
|---------|-----------|-----------|------|
| `dwb` | NavfnPlanner | DWB | 基于Dijkstra算法的全局规划 + 动态窗口法的局部规划 |
| `teb` | NavfnPlanner | TEB | 基于Dijkstra算法的全局规划 + 时间弹性带算法的局部规划，适合复杂环境动态避障 |
| `intpc` | NavfnPlanner | Intpc | 基于Dijkstra算法的全局规划 + 自主研发的集成规划与控制算法的局部规划 |
| `intpc_global_dwb_local` | Intpc | DWB | 默认配置，使用Intpc作为全局规划器（基于傅里叶路径表示和优化算法）+ DWB作为局部规划器 |

**全局规划器说明**：
- **NavfnPlanner**：Navigation2默认的全局规划器，基于Dijkstra算法，快速生成从起点到目标点的全局路径
- **Intpc全局规划器**：自主研发的全局规划器，使用傅里叶路径表示和优化算法，生成更平滑的全局路径

**局部规划器说明**：
- **DWB**：Navigation2默认的局部规划器，基于动态窗口法，在全局路径附近生成满足运动学约束的局部轨迹
- **TEB**：时间弹性带算法，通过优化时间参数化的路径来生成平滑的轨迹，适合复杂环境下的动态避障
- **Intpc**：自主研发的集成规划与控制算法，使用控制障碍函数（CBF）和二次规划优化，实现更精确的轨迹跟踪

### 3.2 仿真模式示例
    ```sh
    source install/setup.bash
    ```
#### 边建图边导航
```sh
ros2 launch rm_nav_bringup bringup_sim.launch.py \
world:=RMUL \
mode:=mapping \  
lio:=fastlio \
planner_type:=dwb \
lio_rviz:=False \
nav_rviz:=True
```

#### 已知地图导航
```sh
ros2 launch rm_nav_bringup bringup_sim.launch.py \
world:=RMUL \
mode:=nav \
lio:=fastlio \
localization:=slam_toolbox \
planner_type:=dwb \
lio_rviz:=False \
nav_rviz:=True
```

#### 使用Intpc全局规划器 + DWB局部规划器导航
```sh
ros2 launch rm_nav_bringup bringup_sim.launch.py \
world:=RMUL \
mode:=nav \
lio:=fastlio \
localization:=slam_toolbox \
planner_type:=intpc_global_dwb_local \
lio_rviz:=False \
nav_rviz:=True
```

### 3.3 真实环境示例

#### 边建图边导航
```sh
ros2 launch rm_nav_bringup bringup_real.launch.py \
world:=YOUR_WORLD_NAME \
mode:=mapping  \
lio:=fastlio \
planner_type:=dwb \
lio_rviz:=False \
nav_rviz:=True
```

**地图保存说明**：
1. **保存点云地图**：修改 `src/rm_nav_bringup/config/reality/fastlio_mid360_real.yaml` 中的 `pcd_save_en` 为 `true`，设置保存路径后运行：
   ```sh
   ros2 service call /map_save std_srvs/srv/Trigger
   ```
2. **保存栅格地图**：参考 [保存 .pgm 和 .posegraph 地图](https://gitee.com/SMBU-POLARBEAR/pb_rmsimulation/issues/I9427I)，地图名需与 `YOUR_WORLD_NAME` 一致。

#### 已知地图导航
```sh
ros2 launch rm_nav_bringup bringup_real.launch.py \
world:=YOUR_WORLD_NAME \
mode:=nav \
lio:=fastlio \
localization:=slam_toolbox \
planner_type:=dwb \
lio_rviz:=False \
nav_rviz:=True
```

### 3.4 辅助工具

#### 键盘控制
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### 地图保存脚本
```sh
# 保存点云地图
./save_pcd.sh

# 保存栅格地图
./save_grid_map.sh
```

## 四. 规划器构建与集成指南

本项目支持将自定义规划器作为算法可选项集成到系统中，包括全局规划器和局部规划器。

### 4.1 规划器架构概览

系统采用 Navigation2 插件架构，支持以下规划器类型：

| 规划器类型 | 角色 | 实现位置 | 核心算法 |
|-----------|------|----------|----------|
| NavfnPlanner | 全局规划器 | Nav2 默认 | Dijkstra 算法 |
| IntpcGlobalPlanner | 全局规划器 | `Intpc_local_planner` 包 | 傅里叶路径表示 + CBF 优化 |
| DWB | 局部规划器 | Nav2 默认 | 动态窗口法 |
| TEB | 局部规划器 | `teb_local_planner` 包 | 时间弹性带算法 |
| IntpcLocalPlanner | 局部规划器 | `Intpc_local_planner` 包 | 傅里叶路径表示 + CBF 优化 |

### 4.2 构建自定义规划器

#### 步骤1：创建规划器包结构

```bash
# 在 src/rm_navigation/ 目录下创建规划器包
cd src/rm_navigation/
ros2 pkg create --build-type ament_cmake <your_planner_package>
```

#### 步骤2：实现规划器接口

创建以下文件结构：

```
<your_planner_package>/
├── include/
│   └── <your_planner_package>/
│       ├── <your_global_planner>.h  # 全局规划器接口（可选）
│       └── <your_local_planner>.h   # 局部规划器接口（可选）
├── src/
│   ├── <your_global_planner>.cpp    # 全局规划器实现（可选）
│   └── <your_local_planner>.cpp     # 局部规划器实现（可选）
├── <your_planner_package>_plugin.xml  # 插件注册文件
├── CMakeLists.txt
├── package.xml
└── README.md
```

#### 步骤3：实现规划器核心功能

**全局规划器接口**：

```cpp
class <YourGlobalPlanner> : public nav2_core::GlobalPlanner {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                 std::string name,
                 const std::shared_ptr<tf2_ros::Buffer> &tf,
                 const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros) override;
  
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                                const geometry_msgs::msg::PoseStamped &goal) override;
};
```

**局部规划器接口**：

```cpp
class <YourLocalPlanner> : public nav2_core::Controller {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                 std::string name,
                 const std::shared_ptr<tf2_ros::Buffer> &tf,
                 const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros) override;
  
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                                                           const geometry_msgs::msg::Twist &velocity,
                                                           nav2_core::GoalChecker *goal_checker) override;
  
  void setPlan(const nav_msgs::msg::Path &path) override;
  void setSpeedLimit(const double &speed_limit, const bool &percentage) override;
};
```

#### 步骤4：注册插件

创建 `<your_planner_package>_plugin.xml` 文件：

```xml
<library path="<your_planner_package>_plugin">
  <!-- 全局规划器插件（可选） -->
  <class type="<your_planner_package>::<YourGlobalPlanner>" base_class_type="nav2_core::GlobalPlanner">
    <description>Your Global Planner Description</description>
  </class>
  
  <!-- 局部规划器插件（可选） -->
  <class type="<your_planner_package>::<YourLocalPlanner>" base_class_type="nav2_core::Controller">
    <description>Your Local Planner Description</description>
  </class>
</library>
```

#### 步骤5：配置 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(<your_planner_package>)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# 构建插件库
add_library(${PROJECT_NAME}_plugin SHARED
  src/<your_global_planner>.cpp  # 全局规划器实现（可选）
  src/<your_local_planner>.cpp   # 局部规划器实现（可选）
)

ament_target_dependencies(${PROJECT_NAME}_plugin
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# 导出插件
target_compile_definitions(${PROJECT_NAME}_plugin PRIVATE "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core <your_planner_package>_plugin.xml)

install(TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(FILES <your_planner_package>_plugin.xml
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```

#### 步骤6：配置 package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name><your_planner_package></name>
  <version>0.0.0</version>
  <description>Your Planner Package Description</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>rclcpp</build_depend>
  <build_depend>rclcpp_lifecycle</build_depend>
  <build_depend>nav2_core</build_depend>
  <build_depend>nav2_costmap_2d</build_depend>
  <build_depend>pluginlib</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>nav_msgs</build_depend>
  <build_depend>tf2</build_depend>
  <build_depend>tf2_ros</build_depend>

  <exec_depend>rclcpp</exec_depend>
  <exec_depend>rclcpp_lifecycle</exec_depend>
  <exec_depend>nav2_core</exec_depend>
  <exec_depend>nav2_costmap_2d</exec_depend>
  <exec_depend>pluginlib</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf2</exec_depend>
  <exec_depend>tf2_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 4.3 集成规划器到系统

#### 步骤1：创建配置文件

在 `src/rm_nav_bringup/config/simulation/` 目录创建 `nav2_params_<planner_config>.yaml` 配置文件，示例：

**全局规划器配置**：

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 5.0
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "<your_planner_package>/<YourGlobalPlanner>"
      # 全局规划器特定参数
      param1: value1
      param2: value2
```

**局部规划器配置**：

```yaml
controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "<your_planner_package>/<YourLocalPlanner>"
      # 局部规划器特定参数
      param1: value1
      param2: value2
```

#### 步骤2：修改启动文件

编辑 `src/rm_nav_bringup/launch/bringup_sim.launch.py`，添加新的规划器配置选项：

```python
start_navigation2_<planner_config> = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'bringup_rm_navigation.py')),
    condition=LaunchConfigurationEquals('planner_type', '<planner_config>'),
    launch_arguments={
        'use_sim_time': use_sim_time,
        'map': nav2_map_dir,
        'params_file': os.path.join(rm_nav_bringup_dir, 'config', 'simulation', 'nav2_params_<planner_config>.yaml'),
        'nav_rviz': use_nav_rviz}.items()
)
```

#### 步骤3：更新参数说明

在 README.md 的参数说明部分添加新规划器配置：

```markdown
| `planner_type` | `dwb`/`teb`/`intpc`/`intpc_global_dwb_local`/`<planner_config>` | 规划器配置选择 |
```

### 4.4 构建与测试

#### 步骤1：编译项目

```bash
cd Intpc_Simulation
colcon build --symlink-install --packages-select <your_planner_package>
```

#### 步骤2：测试规划器

```bash
source install/setup.bash
ros2 launch rm_nav_bringup bringup_sim.launch.py planner_type:=<planner_config>
```

#### 步骤3：验证插件加载

```bash
# 检查全局规划器插件（如果实现了）
ros2 param get /planner_server GridBased/plugin

# 检查局部规划器插件（如果实现了）
ros2 param get /controller_server FollowPath/plugin
```

#### 步骤4：测试路径规划

```bash
# 发送导航目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"

# 查看规划结果
ros2 topic echo /plan

# 查看速度命令
ros2 topic echo /cmd_vel
```

### 4.5 规划器性能调优

#### 全局规划器调优

| 参数 | 说明 | 建议值 |
|------|------|--------|
| `expected_planner_frequency` | 规划器期望频率 | 1.0-5.0 Hz |
| `tolerance` | 目标点容差 | 0.1-0.5 m |
| `allow_unknown` | 是否允许未知区域 | true/false |

#### 局部规划器调优

| 参数 | 说明 | 建议值 |
|------|------|--------|
| `controller_frequency` | 控制器频率 | 10.0-30.0 Hz |
| `max_vel_x` | 最大线速度 | 0.5-2.0 m/s |
| `max_vel_theta` | 最大角速度 | 1.0-3.0 rad/s |
| `min_vel_x` | 最小线速度 | 0.0-0.1 m/s |
| `min_vel_theta` | 最小角速度 | -3.0-3.0 rad/s |
| `acc_lim_x` | 线加速度限制 | 0.5-2.0 m/s² |
| `acc_lim_theta` | 角加速度限制 | 1.0-5.0 rad/s² |

### 4.6 示例：集成Intpc规划器

Intpc规划器提供了全局和局部规划能力，详细实现和配置信息请参考 **Intpc规划器实现文档** (`src/rm_navigation/Intpc_local_planner/README.md`)。

**核心特性**：
- **Intpc全局规划器**：使用傅里叶路径表示，生成平滑路径，集成CBF优化实现安全避障
- **Intpc局部规划器**：基于比例-切向-法向控制，实现高精度轨迹跟踪和动态避障

**配置示例**：
- 全局规划器配置：使用 `intpc_local_planner/IntpcGlobalPlannerROS` 插件
- 局部规划器配置：使用 `intpc_local_planner/IntpcLocalPlannerROS` 插件

详细的参数配置、算法原理和实现细节请查阅 Intpc 规划器实现文档。

### 4.7 规划器选择指南

| 场景 | 推荐规划器配置 | 优势 |
|------|---------------|------|
| 简单环境，快速部署 | `dwb`（全局NavfnPlanner + 局部DWB） | 配置简单，可靠性高 |
| 复杂环境，动态避障 | `teb`（全局NavfnPlanner + 局部TEB） | 避障能力强，轨迹平滑 |
| 高精度轨迹跟踪 | `intpc`（全局NavfnPlanner + 局部Intpc） | 跟踪精度高，控制平稳 |
| 全局路径优化 | `intpc_global_dwb_local`（全局Intpc + 局部DWB） | 全局路径平滑，局部反应迅速 |
| 自定义需求 | 自定义规划器配置 | 完全满足特定场景需求 |

### 4.8 常见问题排查

#### 插件加载失败

**症状**：启动时出现 "Failed to create planner" 错误

**解决方案**：
1. 检查插件XML文件路径和内容是否正确
2. 确保CMakeLists.txt中正确导出插件
3. 验证编译是否成功，无错误
4. 检查环境变量是否正确设置：`source install/setup.bash`

#### 规划器参数无效

**症状**：规划器行为不符合预期

**解决方案**：
1. 检查配置文件格式是否正确
2. 验证参数名称是否与规划器实现匹配
3. 使用 `ros2 param dump` 查看当前参数值
4. 调整参数值，重启导航系统测试

#### 路径规划失败

**症状**：无法生成有效路径或路径穿越障碍物

**解决方案**：
1. 检查代价地图是否正确生成：`ros2 topic echo /global_costmap/costmap`
2. 验证起点和终点是否在可行区域内
3. 调整规划器参数，如增加障碍物膨胀半径
4. 检查传感器数据是否正确传入代价地图

### 4.9 性能监控与分析

#### 监控规划器性能

```bash
# 查看规划器计算时间
ros2 topic hz /plan

# 查看速度命令频率
ros2 topic hz /cmd_vel

# 查看CPU使用情况
top -p $(pgrep -f planner_server) $(pgrep -f controller_server)

# 查看内存使用情况
ps aux | grep -E "planner_server|controller_server" | awk '{print $4, $11}'
```

#### 性能优化建议

1. **全局规划器**：
   - 降低规划频率（expected_planner_frequency）
   - 增加规划容差（tolerance）
   - 使用更简单的路径表示方法

2. **局部规划器**：
   - 调整控制器频率（controller_frequency）
   - 优化障碍物检测和避障算法
   - 使用更高效的优化方法（如快速QP求解器）

3. **系统级优化**：
   - 使用Release模式编译：`colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`
   - 关闭不必要的日志输出
   - 优化代价地图更新频率和分辨率

## 五. 实车适配指南

### 1. 雷达配置
- **IP设置**：修改 `src/rm_nav_bringup/config/reality/MID360_config.json` 中的 `lidar_configs.ip`
- **点云旋转**：雷达倾斜放置时，在 `extrinsic_parameter` 中设置旋转角度

### 2. 坐标参数
- **雷达安装位置**：测量底盘中心到雷达的相对坐标（x, y 影响定位精度），填入 `src/rm_nav_bringup/config/reality/measurement_params_real.yaml`
- **雷达高度**：测量雷达与地面的垂直距离，填入 `src/rm_nav_bringup/config/reality/segmentation_real.yaml` 中的 `sensor_height`

### 3. 导航参数
关键参数包括：
- `robot_radius`：机器人碰撞半径
- 速度限制参数：`max_vel_x`、`max_vel_theta` 等
- 加速度限制参数：`acc_lim_x`、`acc_lim_theta` 等

详见 [Nav2官方文档](https://docs.nav2.org/) 了解更多参数配置。

## 六. 项目结构

```
├── src/
│   ├── rm_driver/         # 传感器驱动（Livox雷达等）
│   ├── rm_localization/   # 定位算法（FAST_LIO、Point_LIO、ICP等）
│   ├── rm_nav_bringup/    # 启动文件和配置
│   ├── rm_navigation/     # 导航核心功能
│   │   ├── Intpc_local_planner/   # 自主研发的IntPC局部规划器
│   │   ├── costmap_converter/     # 代价图转换工具
│   │   ├── fake_vel_transform/    # 速度转换工具
│   │   ├── rm_navigation/         # Nav2配置和启动文件
│   │   └── teb_local_planner/     # TEB局部规划器
│   ├── rm_perception/     # 感知模块（地面分割等）
│   └── rm_simulation/     # 仿真相关功能
├── build.sh               # 一键编译脚本
├── save_grid_map.sh       # 栅格地图保存脚本
├── save_pcd.sh            # 点云地图保存脚本
├── README.md              # 项目说明文档
└── LICENSE                # 许可证
```

## 八. 自检机制

### 8.1 全局规划器自检

#### 接口实现验证

**检查点1：插件注册**
```bash
# 查看插件注册文件
cat src/rm_navigation/Intpc_local_planner/intpc_local_planner_plugin.xml
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
grep -n "class IntpcGlobalPlannerROS" src/rm_navigation/Intpc_local_planner/include/intpc_local_planner/intpc_global_planner_ros.h
```

预期输出：
```cpp
class IntpcGlobalPlannerROS : public nav2_core::GlobalPlanner
```

**检查点3：必需方法实现**
```bash
# 检查是否实现了所有必需的接口方法
grep -E "(configure|cleanup|activate|deactivate|createPlan)" src/rm_navigation/Intpc_local_planner/include/intpc_local_planner/intpc_global_planner_ros.h
```

预期应包含所有五个方法。

#### 配置验证

**检查点4：配置文件设置**
```bash
# 查看全局规划器配置
cat src/rm_navigation/rm_navigation/params/nav2_params_intpc_global_dwb_local.yaml | grep -A 10 "planner_server:"
```

预期输出应包含：
```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "intpc_local_planner/IntpcGlobalPlannerROS"
```

#### 编译验证

**检查点5：编译成功**
```bash
colcon build --packages-select Intpc_local_planner --symlink-install
```

预期输出：编译成功，无错误。

#### 运行时验证

**检查点6：插件加载**
```bash
# 启动导航系统
ros2 launch rm_nav_bringup bringup_sim.launch.py planner_type:=intpc_global_dwb_local

# 在另一个终端检查插件是否正确加载
ros2 param get /planner_server GridBased/plugin
```

预期输出：`intpc_local_planner/IntpcGlobalPlannerROS`

**检查点7：路径规划功能**
```bash
# 发送导航目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"

# 查看规划结果
ros2 topic echo /plan
```

预期输出：应生成有效的全局路径。

### 8.2 局部规划器自检

#### 接口实现验证

**检查点1：插件注册**
```bash
# 查看插件注册文件
cat src/rm_navigation/Intpc_local_planner/intpc_local_planner_plugin.xml
```

预期输出应包含：
```xml
<class type="intpc_local_planner::IntpcLocalPlannerROS" base_class_type="nav2_core::Controller">
  <description>Intpc Local Planner implements Fourier path representation, proportional-tangential-normal control, CBF, and quadratic programming optimization.</description>
</class>
```

**检查点2：头文件接口**
```bash
# 检查头文件是否正确继承nav2_core::Controller
grep -n "class IntpcLocalPlannerROS" src/rm_navigation/Intpc_local_planner/include/intpc_local_planner/intpc_local_planner_ros.h
```

预期输出：
```cpp
class IntpcLocalPlannerROS : public nav2_core::Controller
```

**检查点3：必需方法实现**
```bash
# 检查是否实现了所有必需的接口方法
grep -E "(configure|cleanup|activate|deactivate|computeVelocityCommands|setPlan|setSpeedLimit)" src/rm_navigation/Intpc_local_planner/include/intpc_local_planner/intpc_local_planner_ros.h
```

预期应包含所有七个方法。

#### 配置验证

**检查点4：配置文件设置**
```bash
# 查看局部规划器配置（使用Intpc作为局部规划器时）
cat src/rm_navigation/rm_navigation/params/nav2_params_intpc.yaml | grep -A 20 "controller_server:"
```

预期输出应包含：
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "intpc_local_planner/IntpcLocalPlannerROS"
```

#### 编译验证

**检查点5：编译成功**
```bash
colcon build --packages-select Intpc_local_planner --symlink-install
```

预期输出：编译成功，无错误。

#### 运行时验证

**检查点6：插件加载**
```bash
# 启动导航系统
ros2 launch rm_nav_bringup bringup_sim.launch.py planner_type:=intpc

# 在另一个终端检查插件是否正确加载
ros2 param get /controller_server FollowPath/plugin
```

预期输出：`intpc_local_planner/IntpcLocalPlannerROS`

**检查点7：速度控制功能**
```bash
# 发送导航目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"

# 查看速度命令
ros2 topic echo /cmd_vel
```

预期输出：应生成有效的速度命令。

### 8.3 常见问题排查

#### 问题1：插件未正确加载

**症状**：启动时出现插件加载错误

**排查步骤**：
```bash
# 1. 检查插件XML文件
cat src/rm_navigation/Intpc_local_planner/intpc_local_planner_plugin.xml

# 2. 检查CMakeLists.txt中的插件导出
cat src/rm_navigation/Intpc_local_planner/CMakeLists.txt | grep pluginlib

# 3. 检查编译输出
colcon build --packages-select Intpc_local_planner --symlink-install
```

**解决方案**：确保CMakeLists.txt中包含`pluginlib_export_plugin_description_file(nav2_core intpc_local_planner_plugin.xml)`

#### 问题2：路径规划失败

**症状**：无法生成有效路径

**排查步骤**：
```bash
# 1. 检查代价地图
ros2 topic echo /global_costmap/costmap

# 2. 检查起点和终点是否有效
ros2 topic echo /amcl_pose

# 3. 检查规划器日志
ros2 topic echo /rosout | grep -i planner
```

**解决方案**：确保起点和终点不在障碍物内，检查代价地图配置。

#### 问题3：速度命令异常

**症状**：机器人运动异常或不动

**排查步骤**：
```bash
# 1. 检查速度命令
ros2 topic echo /cmd_vel

# 2. 检查规划器参数
ros2 param dump /controller_server

# 3. 检查障碍物信息
ros2 topic echo /local_costmap/obstacles
```

**解决方案**：调整规划器参数（如max_vel_x、k_gain等），检查障碍物检测。

### 8.4 性能监控

#### 监控规划器性能

```bash
# 查看规划器计算时间
ros2 topic hz /plan

# 查看速度命令频率
ros2 topic hz /cmd_vel

# 查看CPU使用情况
top -p $(pgrep -f planner_server)
```

#### 性能指标

| 指标 | 全局规划器 | 局部规划器 |
|------|-----------|-----------|
| 规划频率 | > 1 Hz | > 10 Hz |
| 规划时间 | < 1.0 s | < 0.1 s |
| 路径平滑度 | 良好 | 优秀 |
| 避障响应 | 中等 | 优秀 |

## 九. 致谢与参考

### 算法参考
- IntPC算法思想来源于北航机器人平台相关工作
- LiDAR点云仿真参考：`livox_laser_simulation`、`livox_laser_simulation_RO2`

### 代码基础
- 基于[深圳北理莫斯科大学 北极熊战队 导航系统](https://github.com/LihanChen2004/PB_RMSimulation.git)、[中南大学 FYT 战队 RM 哨兵上位机算法](https://github.com/baiyeweiguang/CSU-RM-Sentry) 开发

### 开源依赖
- [Navigation2](https://navigation.ros.org/)
- [FAST_LIO](https://github.com/hku-mars/FAST_LIO)
- [Point-LIO](https://github.com/hku-mars/Point-LIO)
- [TEB Local Planner](http://wiki.ros.org/teb_local_planner)

感谢所有为开源导航社区做出贡献的开发者们！