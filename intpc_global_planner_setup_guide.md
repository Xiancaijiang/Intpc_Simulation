# Intpc全局规划器设置指南

## 一、未使用Intpc全局规划器时的默认配置

### 1. 规划器服务器配置
```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

### 2. 控制器服务器配置
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      # DWB局部规划器参数...
```

### 3. 代价地图配置
```yaml
# 全局代价地图
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 5.0
      width: 100.0
      height: 100.0
      resolution: 0.1
      robot_base_frame: "base_link"
      global_frame: "map"
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

# 局部代价地图
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 20.0
      publish_frequency: 20.0
      width: 3.0
      height: 3.0
      resolution: 0.05
      robot_base_frame: "base_link"
      global_frame: "odom"
      rolling_window: true
      plugins: ["obstacle_layer", "inflation_layer"]
```

## 二、启用Intpc全局规划器的步骤

### 1. 修改规划器服务器配置
```yaml
planner_server:
  ros__parameters:
    # 将GridBased替换为IntpcGlobalPlanner
    planner_plugins: ["IntpcGlobalPlanner"]
    
    # 启用Intpc全局规划器
    IntpcGlobalPlanner:
      plugin: "intpc_local_planner::IntpcGlobalPlannerROS"
      path_resolution: 0.1         # 路径点分辨率
      max_planning_time: 1.0       # 最大规划时间
      waypoint_separation: 0.5     # 航点间距
```

### 2. 保持或更新控制器配置
可以继续使用Intpc局部规划器：
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["IntpcController"]
    
    IntpcController:
      plugin: "intpc_local_planner::IntpcLocalPlannerROS"
      max_vel_x: 0.5
      max_vel_theta: 1.0
      # 其他Intpc局部规划器参数...
```

## 三、localmap接收不到的问题解决方案

### 可能原因
1. 代价地图参数配置错误
2. TF变换问题
3. 传感器数据缺失
4. 话题名称不匹配

### 解决方案

#### 1. 检查代价地图参数
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      robot_base_frame: "base_link"  # 确保与机器人实际坐标系匹配
      global_frame: "odom"          # 确保全局坐标系正确
      rolling_window: true           # 启用滚动窗口
      width: 3.0                     # 适当的地图宽度
      height: 3.0                    # 适当的地图高度
      resolution: 0.05               # 适当的分辨率
```

#### 2. 检查TF变换
确保机器人坐标系与全局坐标系之间有正确的TF变换：
```bash
# 检查TF变换
ros2 run tf2_ros tf2_echo odom base_link
```

#### 3. 检查传感器数据
确保激光雷达或深度相机等传感器正常发布数据：
```bash
# 检查激光数据
ros2 topic list | grep laser
ros2 topic echo /scan

# 检查点云数据
ros2 topic list | grep pointcloud
ros2 topic echo /pointcloud
```

#### 4. 检查话题名称
确保传感器话题与代价地图配置匹配：
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins:
        - name: obstacle_layer
          type: "nav2_costmap_2d::ObstacleLayer"
          obstacle_layer:
            observation_sources: scan
            scan:
              topic: /scan          # 确保与实际传感器话题匹配
              sensor_frame: lidar   # 确保与传感器坐标系匹配
              data_type: LaserScan
              marking: true
              clearing: true
              obstacle_max_range: 3.0
              obstacle_min_range: 0.0
```

## 四、构建和运行

### 1. 构建项目
```bash
cd g:\桌面\Intpc_Simulation
colcon build --symlink-install
```

### 2. 运行Navigation2
```bash
# 激活环境
source install/local_setup.bash

# 运行Navigation2（使用默认配置，未启用Intpc全局规划器）
ros2 launch nav2_bringup navigation_launch.py map:=map.yaml

# 或使用自定义配置文件
hros2 launch nav2_bringup navigation_launch.py params_file:=g:\桌面\Intpc_Simulation\intpc_global_planner_config_example.yaml
```

### 3. 切换到Intpc全局规划器
编辑配置文件，将规划器插件从`nav2_navfn_planner/NavfnPlanner`切换为`intpc_local_planner::IntpcGlobalPlannerROS`，然后重新启动Navigation2。

## 五、调试技巧

1. **检查节点状态**：
   ```bash
   ros2 node list
   ros2 node info /planner_server
   ```

2. **检查插件加载情况**：
   ```bash
   ros2 param get /planner_server planner_plugins
   ```

3. **查看日志信息**：
   ```bash
   ros2 logs --ros-args --name /planner_server
   ```

4. **使用RViz可视化**：
   ```bash
   ros2 run rviz2 rviz2 -d install/nav2_bringup/share/nav2_bringup/rviz/nav2_default_view.rviz
   ```

## 六、常见问题与解答

### Q1: 如何验证Intpc全局规划器是否正常工作？
A: 可以通过以下方式验证：
- 查看RViz中的规划路径
- 检查日志中是否有Intpc相关信息
- 使用`ros2 service call /compute_path_to_pose`服务测试规划功能

### Q2: Intpc全局规划器与其他规划器有什么区别？
A: Intpc全局规划器使用傅里叶路径表示和优化算法，具有以下特点：
- 平滑的路径生成
- 与Intpc局部规划器的无缝集成
- 支持自定义参数调整

### Q3: 如何优化Intpc全局规划器的性能？
A: 可以通过调整以下参数优化性能：
- `path_resolution`: 增加分辨率可以生成更精细的路径，但会增加计算时间
- `max_planning_time`: 增加最大规划时间可以生成更优路径，但会降低实时性
- `waypoint_separation`: 调整航点间距可以平衡路径平滑度和计算效率

## 七、总结

在未使用Intpc全局规划器时，Navigation2默认使用`nav2_navfn_planner/NavfnPlanner`作为全局规划器。要使用Intpc全局规划器，需要：

1. 确保已构建包含Intpc全局规划器的项目
2. 修改规划器服务器配置，启用Intpc全局规划器插件
3. 调整Intpc全局规划器的参数以满足特定需求
4. 解决可能的localmap接收问题

通过本指南，您可以顺利完成Intpc全局规划器的设置和调试。