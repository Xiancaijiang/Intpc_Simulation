/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024,
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#ifndef INCLUDE_INSPC_LOCAL_PLANNER_INSPC_LOCAL_PLANNER_H_  // 修正了头文件保护宏的拼写
#define INCLUDE_INSPC_LOCAL_PLANNER_INSPC_LOCAL_PLANNER_H_

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace intpc_local_planner {

/**
 * @class IntpcLocalPlanner
 * @brief 核心规划算法实现类，包含参考速度计算、障碍物避障和运动学转换
 */
class IntpcLocalPlanner {
public:
  /**
   * @brief 构造函数
   */
  IntpcLocalPlanner();

  /**
   * @brief 析构函数
   */
  ~IntpcLocalPlanner();

  /**
   * @brief 计算参考速度向量
   * @param k 比例增益
   * @param x 当前x坐标
   * @param y 当前y坐标
   * @param vd 期望速度
   * @param e 输出参数：距离误差
   * @param vf 输出参数：参考速度向量
   */
  void reference(double k, double x, double y, double vd, double& e, Eigen::Vector2d& vf);

  /**
   * @brief 求解QP优化问题，生成考虑障碍物的控制速度
   * @param ud 期望控制输入
   * @param x 当前x坐标
   * @param y 当前y坐标
   * @param theta 当前角度
   * @param x_o 障碍物x坐标列表
   * @param y_o 障碍物y坐标列表
   * @param r_o 障碍物半径列表
   * @param alpha 避障参数
   * @param l 机器人参数
   * @param d 机器人参数
   * @param max_speed 最大速度限制
   * @return 优化后的控制输入
   */
  Eigen::Vector2d qp(const Eigen::Vector2d& ud, double x, double y, double theta,
                     const std::vector<double>& x_o, const std::vector<double>& y_o,
                     const std::vector<double>& r_o, double alpha, double l, double d,
                     double max_speed);

  /**
   * @brief 前向运动学计算
   * @param x 当前x坐标
   * @param y 当前y坐标
   * @param theta 当前角度
   * @param u 控制输入
   * @param dt 时间步长
   * @param l 机器人参数
   * @param x_new 输出参数：新的x坐标
   * @param y_new 输出参数：新的y坐标
   * @param theta_new 输出参数：新的角度
   */
  void forwardKinematics(double x, double y, double theta, const Eigen::Vector2d& u,
                         double dt, double l, double& x_new, double& y_new, double& theta_new);

  /**
   * @brief Initialize path parameters
   * @param goal_x Goal x position
   * @param goal_y Goal y position
   */
  void initializePathParams(double goal_x, double goal_y);

  /**
   * @brief 主要的控制计算接口，整合所有算法
   * @param x 当前x坐标
   * @param y 当前y坐标
   * @param theta 当前角度
   * @param x_obstacles 障碍物x坐标列表
   * @param y_obstacles 障碍物y坐标列表
   * @param obstacle_radii 障碍物半径列表
   * @param k_gain 比例增益
   * @param max_speed 最大速度
   * @return 控制速度（线速度和角速度）
   */
  Eigen::Vector2d computeIntpcControl(double x, double y, double theta,
                                     const std::vector<double>& x_obstacles,
                                     const std::vector<double>& y_obstacles,
                                     const std::vector<double>& obstacle_radii,
                                     double k_gain, double max_speed);

  /**
   * @brief 傅里叶路径拟合
   * @param path_x 路径x坐标列表
   * @param path_y 路径y坐标列表
   * @param theta 角度参数列表
   * @param harmonic 傅里叶级数的阶数
   * @param px 输出：傅里叶系数x
   * @param py 输出：傅里叶系数y
   */
  void fourierFit(const std::vector<double>& path_x, const std::vector<double>& path_y,
                  const std::vector<double>& theta, int harmonic,
                  std::vector<double>& px, std::vector<double>& py);

  /**
   * @brief 使用傅里叶系数计算路径点
   * @param theta 角度参数
   * @param px 傅里叶系数x
   * @param py 傅里叶系数y
   * @param x 输出：x坐标
   * @param y 输出：y坐标
   */
  void fourierPathPoint(double theta, const std::vector<double>& px,
                       const std::vector<double>& py, double& x, double& y);

  /**
   * @brief 计算参考向量（模仿vector.py的reference函数）
   * @param k 比例增益
   * @param x 当前x坐标
   * @param y 当前y坐标
   * @param px 傅里叶系数x
   * @param py 傅里叶系数y
   * @param vd 期望速度
   * @param distance 距离偏移
   * @param x_c 路径中心x坐标
   * @param y_c 路径中心y坐标
   * @param harmonic 傅里叶阶数
   * @param e 输出：距离误差
   * @param vf 输出：参考速度向量
   */
  void reference(double k, double x, double y, const std::vector<double>& px,
                 const std::vector<double>& py, double vd, double distance,
                 double x_c, double y_c, int harmonic, double& e, Eigen::Vector2d& vf);

private:
  // 机器人参数
  double wheel_base_;  // 轴距
  
  // 控制器参数
  double alpha_;       // CBF参数
  double l_;           // 机器人参数
  double d_;           // 机器人参数
  
  // 路径参数
  double vd_;          // 期望速度
  double goal_x_;      // 目标x坐标
  double goal_y_;      // 目标y坐标
};

}  // namespace intpc_local_planner

#endif  // INCLUDE_INSPC_LOCAL_PLANNER_INSPC_LOCAL_PLANNER_H_
