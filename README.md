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
| `planner_type` | `dwb`/`teb`/`intpc`/`intpc_global_dwb_local` | 规划器配置选择<br>`dwb`：全局NavfnPlanner + 局部DWB<br>`teb`：全局NavfnPlanner + 局部TEB<br>`intpc`：全局NavfnPlanner + 局部Intpc<br>`intpc_global_dwb_local`：全局Intpc + 局部DWB（默认） |

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

## 四. 扩展新的局部规划器

本项目支持轻松集成自定义局部规划器，只需遵循以下步骤：

### 1. 准备规划器代码
将规划器代码添加到 `src/rm_navigation/` 目录，确保实现 Nav2 局部规划器接口。

### 2. 创建配置文件
在 `src/rm_navigation/rm_navigation/config/` 目录创建 `nav2_params_<planner_name>.yaml` 配置文件，示例：

```yaml
controller_server:
  ros__parameters:
    controller_plugins:
      - FollowPath
    FollowPath:
      plugin: <your_planner_package>/<your_planner_class>
      # 规划器特定参数
      param1: value1
      param2: value2
```

### 3. 修改启动文件
编辑 `src/rm_navigation/rm_navigation/launch/navigation_launch.py`：

1. 添加规划器类型参数选项：
```python
declare_planner_type_cmd = DeclareLaunchArgument(
    'planner_type',
    default_value='teb',
    description='Choose local planner: teb, intpc or <your_planner_name>')
```

2. 添加配置文件选择逻辑：
```python
elif planner_type == '<your_planner_name>':
    params_file = os.path.join(
        get_package_share_directory('rm_navigation'),
        'config',
        'nav2_params_<your_planner_name>.yaml')
```

### 4. 更新文档
在 README.md 的参数说明部分添加新规划器的介绍。

### 5. 编译测试
```sh
colcon build --symlink-install
ros2 launch rm_nav_bringup bringup_sim.launch.py planner_type:=<your_planner_name>
```

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