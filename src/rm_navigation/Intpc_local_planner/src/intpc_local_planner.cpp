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

#include <intpc_local_planner/intpc_local_planner.h>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace intpc_local_planner {

IntpcLocalPlanner::IntpcLocalPlanner() {
  // 初始化参数
  wheel_base_ = 0.1;  // 默认轴距（机器人两个轮子之间的距离）
  vd_ = 0.5;          // 默认期望速度（机器人的目标线速度）
  goal_x_ = 0.0;      // 默认目标x坐标
  goal_y_ = 0.0;      // 默认目标y坐标
  
  // 控制器参数
  alpha_ = 0.1;       // CBF参数
  l_ = 0.1;           // 机器人参数
  d_ = 0.05;          // 机器人参数
}

IntpcLocalPlanner::~IntpcLocalPlanner() {
  // 析构函数（无需特殊清理）
}

/**
 * @brief 初始化路径参数
 * @param goal_x 目标点x坐标
 * @param goal_y 目标点y坐标
 */
void IntpcLocalPlanner::initializePathParams(double goal_x, double goal_y) {
  goal_x_ = goal_x;        // 存储目标点x坐标
  goal_y_ = goal_y;        // 存储目标点y坐标
}

/**
 * @brief 简化的参考速度计算函数，基于切向和法向向量
 * @param k 比例增益系数
 * @param x 机器人当前x坐标
 * @param y 机器人当前y坐标
 * @param vd 期望速度
 * @param e 输出：路径跟踪误差
 * @param vf 输出：参考速度向量（包含x和y方向速度分量）
 */
void IntpcLocalPlanner::reference(double k, double x, double y, double vd, double& e, Eigen::Vector2d& vf) {
  // 计算到目标的向量
  double dx = goal_x_ - x;            // x方向距离差
  double dy = goal_y_ - y;            // y方向距离差
  double dist_to_goal = std::hypot(dx, dy);  // 计算距离目标的直线距离
  
  // 计算切向向量（指向目标）
  Eigen::Vector2d tau;                // 切向向量，代表期望运动方向
  if (dist_to_goal > 1e-6) {          // 确保机器人不在目标点上
    tau(0) = dx / dist_to_goal;       // 归一化x分量
    tau(1) = dy / dist_to_goal;       // 归一化y分量
  } else {
    // 已经很接近目标，使用小的默认方向
    tau << 0.01, 0.0;
  }
  
  // 计算法向向量（逆时针90度旋转切向向量）
  Eigen::Vector2d n(-tau(1), tau(0));  // 法向向量，用于路径纠偏
  
  // 计算路径跟踪误差（简化为到目标的距离）
  e = dist_to_goal;
  
  // 计算参考速度：切向速度 + 比例控制的法向速度
  // 切向速度使机器人朝目标移动，法向速度用于路径纠偏
  vf = tau * vd;
  
  // 归一化并设置速度大小
  double norm = vf.norm();
  if (norm > 1e-6) {
    vf = vd * vf / norm;  // 保持速度大小为vd，同时保持方向
  } else {
    // 避免除零，使用默认方向
    vf << vd, 0.0;
  }
}

/**
 * @brief 前向运动学计算
 * @param x 当前x坐标
 * @param y 当前y坐标
 * @param theta 当前航向角
 * @param u 控制输入向量（线速度，角速度）
 * @param dt 时间步长
 * @param l 轴距
 * @param x_new 输出：新的x坐标
 * @param y_new 输出：新的y坐标
 * @param theta_new 输出：新的航向角
 */
void IntpcLocalPlanner::forwardKinematics(double x, double y, double theta, const Eigen::Vector2d& u,
                                         double dt, double l, double& x_new, double& y_new, double& theta_new) {
  double v = u(0);           // 线速度
  double omega = u(1);       // 角速度
  
  // 计算速度分量
  double dx = v * std::cos(theta);     // x方向速度分量
  double dy = v * std::sin(theta);     // y方向速度分量
  double dtheta = omega;               // 角速度分量
  
  // 更新位姿（简单的欧拉积分）
  x_new = x + dx * dt;                 // 新的x坐标
  y_new = y + dy * dt;                 // 新的y坐标
  theta_new = theta + dtheta * dt;     // 新的航向角
  
  // 归一化角度到[-pi, pi]范围
  while (theta_new > M_PI) theta_new -= 2.0 * M_PI;
  while (theta_new < -M_PI) theta_new += 2.0 * M_PI;
}

/**
 * @brief 基于切向与法向向量的简单避障算法
 * @param x 机器人当前x坐标
 * @param y 机器人当前y坐标
 * @param theta 机器人当前航向角
 * @param x_obstacles 障碍物x坐标数组
 * @param y_obstacles 障碍物y坐标数组
 * @param obstacle_radii 障碍物半径数组
 * @param k_gain 比例增益系数
 * @param max_vel 最大允许速度
 * @return 控制速度向量（线性速度，角速度）
 */
Eigen::Vector2d IntpcLocalPlanner::computeIntpcControl(
  double x, double y, double theta,
  const std::vector<double>& x_obstacles, const std::vector<double>& y_obstacles,
  const std::vector<double>& obstacle_radii,
  double k_gain, double max_vel) {
  
  // 计算参考速度（基于路径跟踪的理想速度）
  double e;
  Eigen::Vector2d ref_vel;
  reference(k_gain, x, y, vd_, e, ref_vel);
  
  // 初始控制速度为参考速度（无障碍物时使用）
  Eigen::Vector2d control_vel = ref_vel;
  
  // 如果有障碍物，实现基于切向与法向向量的避障
  if (!x_obstacles.empty()) {
    // 找到最近的障碍物
    double min_dist = std::numeric_limits<double>::max();
    size_t closest_obstacle = 0;
    
    for (size_t i = 0; i < x_obstacles.size(); i++) {
      // 计算到障碍物的距离（减去障碍物半径）
      double dist = std::sqrt(
        (x - x_obstacles[i]) * (x - x_obstacles[i]) + 
        (y - y_obstacles[i]) * (y - y_obstacles[i])
      ) - obstacle_radii[i];
      
      if (dist < min_dist) {
        min_dist = dist;
        closest_obstacle = i;
      }
    }
    
    // 计算避障向量（从障碍物指向机器人，即远离障碍物的方向）
    double obs_dx = x - x_obstacles[closest_obstacle];
    double obs_dy = y - y_obstacles[closest_obstacle];
    double obs_dist = std::hypot(obs_dx, obs_dy);
    
    // 归一化避障方向
    Eigen::Vector2d avoid_dir;
    if (obs_dist > 1e-6) {
      avoid_dir(0) = obs_dx / obs_dist;  // 归一化x分量
      avoid_dir(1) = obs_dy / obs_dist;  // 归一化y分量
    } else {
      // 避免除零，使用默认避障方向
      avoid_dir << 1.0, 0.0;
    }
    
    // 计算避障权重：距离障碍物越近，避障权重越大
    const double safe_dist = 0.5; // 安全距离（50cm）
    double avoid_weight = std::max(0.0, (safe_dist - min_dist) / safe_dist);
    avoid_weight = std::min(avoid_weight, 1.0); // 限制最大权重为1
    
    // 计算最终控制速度：结合参考速度和避障速度
    // 距离障碍物越近，避障方向的权重越大
    control_vel = (1.0 - avoid_weight) * ref_vel + avoid_weight * avoid_dir * vd_;
    
    // 距离障碍物过近时（小于30cm），降低速度以增加安全性
    if (min_dist < 0.3) {
      double speed_factor = min_dist / 0.3; // 线性降低速度
      control_vel *= speed_factor;
    }
  }
  
  // 计算线性速度（控制向量的大小），并限制在最大速度范围内
  double linear_vel = std::min(control_vel.norm(), max_vel);
  
  // 计算角速度：基于控制向量的方向和机器人当前朝向
  double desired_theta = std::atan2(control_vel(1), control_vel(0)); // 期望航向角
  double theta_error = desired_theta - theta;                      // 航向角误差
  
  // 归一化角度误差到[-pi, pi]范围，确保最短旋转方向
  while (theta_error > M_PI) theta_error -= 2*M_PI;
  while (theta_error < -M_PI) theta_error += 2*M_PI;
  
  // 比例控制计算角速度（误差越大，旋转越快）
  double angular_vel = k_gain * theta_error;
  
  // 角速度限制，防止旋转过快
  double max_angular_vel = 2.0;  // 最大角速度（弧度/秒）
  if (angular_vel > max_angular_vel) angular_vel = max_angular_vel;
  if (angular_vel < -max_angular_vel) angular_vel = -max_angular_vel;
  
  // 返回控制速度（线速度和角速度），用于生成最终的cmd_vel命令
  return Eigen::Vector2d(linear_vel, angular_vel);
}

/**
 * @brief 傅里叶路径拟合
 * @param path_x 路径x坐标列表
 * @param path_y 路径y坐标列表
 * @param theta 角度参数列表
 * @param harmonic 傅里叶级数的阶数
 * @param px 输出：傅里叶系数x
 * @param py 输出：傅里叶系数y
 */
void IntpcLocalPlanner::fourierFit(const std::vector<double>& path_x, const std::vector<double>& path_y,
                                    const std::vector<double>& theta, int harmonic,
                                    std::vector<double>& px, std::vector<double>& py) {
  int number = path_x.size();
  
  // 构造G矩阵
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(2 * number, 4 * harmonic);
  for (int i = 0; i < 2 * number; i++) {
    for (int j = 0; j < 4 * harmonic; j++) {
      double theta_val = theta[i / 2];
      if (i % 2 == 0) {
        if (j % 4 == 0) {
          G(i, j) = std::cos((j / 4 + 1) * theta_val);
        } else if (j % 4 == 1) {
          G(i, j) = std::sin((j / 4 + 1) * theta_val);
        } else {
          G(i, j) = 0.0;
        }
      } else {
        if (j % 4 == 2) {
          G(i, j) = std::cos((j / 4 + 1) * theta_val);
        } else if (j % 4 == 3) {
          G(i, j) = std::sin((j / 4 + 1) * theta_val);
        } else {
          G(i, j) = 0.0;
        }
      }
    }
  }
  
  // 构造I矩阵
  Eigen::MatrixXd I = Eigen::MatrixXd::Zero(2 * number, 2);
  for (int i = 0; i < number; i++) {
    I(2 * i, 0) = 1.0;
    I(2 * i + 1, 1) = 1.0;
  }
  
  // 构造H矩阵
  Eigen::MatrixXd H(2 * number, 4 * harmonic + 2);
  H << G, I;
  
  // 构造L向量
  Eigen::VectorXd L = Eigen::VectorXd::Zero(2 * number);
  for (int i = 0; i < 2 * number; i++) {
    if (i % 2 == 0) {
      L(i) = path_x[i / 2];
    } else {
      L(i) = path_y[i / 2];
    }
  }
  
  // 求解最小二乘问题
  Eigen::VectorXd parameter = (H.transpose() * H).inverse() * H.transpose() * L;
  
  // 分离px和py系数
  px.clear();
  py.clear();
  for (int i = 0; i < parameter.size() - 2; i++) {
    if ((i / 2) % 2 == 0) {
      px.push_back(parameter(i));
    } else {
      py.push_back(parameter(i));
    }
  }
  px.push_back(parameter(parameter.size() - 2));
  py.push_back(parameter(parameter.size() - 1));
}

/**
 * @brief 使用傅里叶系数计算路径点
 * @param theta 角度参数
 * @param px 傅里叶系数x
 * @param py 傅里叶系数y
 * @param x 输出：x坐标
 * @param y 输出：y坐标
 */
void IntpcLocalPlanner::fourierPathPoint(double theta, const std::vector<double>& px,
                                          const std::vector<double>& py, double& x, double& y) {
  int harmonic = px.size() / 2;
  
  // 计算x坐标
  x = px.back();
  for (int i = 0; i < harmonic; i++) {
    x += px[2 * i] * std::cos((i + 1) * theta) + px[2 * i + 1] * std::sin((i + 1) * theta);
  }
  
  // 计算y坐标
  y = py.back();
  for (int i = 0; i < harmonic; i++) {
    y += py[2 * i] * std::cos((i + 1) * theta) + py[2 * i + 1] * std::sin((i + 1) * theta);
  }
}

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
Eigen::Vector2d IntpcLocalPlanner::qp(const Eigen::Vector2d& ud, double x, double y, double theta,
                                      const std::vector<double>& x_o, const std::vector<double>& y_o,
                                      const std::vector<double>& r_o, double alpha, double l, double d,
                                      double max_speed) {
  // 构造R_bar矩阵
  Eigen::Matrix2d R_bar;
  R_bar(0, 0) = std::cos(theta);
  R_bar(0, 1) = -l * std::sin(theta);
  R_bar(1, 0) = std::sin(theta);
  R_bar(1, 1) = l * std::cos(theta);
  
  double delta = 0.5;
  
  // 构造H矩阵（目标函数的二次项）
  Eigen::Matrix2d H;
  H(0, 0) = R_bar(0, 0) * R_bar(0, 0) + R_bar(1, 0) * R_bar(1, 0) + delta;
  H(0, 1) = R_bar(0, 0) * R_bar(0, 1) + R_bar(1, 0) * R_bar(1, 1);
  H(1, 0) = H(0, 1);
  H(1, 1) = R_bar(0, 1) * R_bar(0, 1) + R_bar(1, 1) * R_bar(1, 1);
  
  // 构造f向量（目标函数的一次项）
  Eigen::Vector2d f;
  f(0) = -(ud(0) * R_bar(0, 0) + ud(1) * R_bar(1, 0) + delta * 0.02);
  f(1) = -(ud(0) * R_bar(0, 1) + ud(1) * R_bar(1, 1));
  
  // 构造约束矩阵A和向量b
  int num_constraints = x_o.size() + 1 + 4;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_constraints, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(num_constraints);
  
  // 障碍物约束
  for (size_t i = 0; i < x_o.size(); i++) {
    A(i, 0) = -2.0 * (R_bar(0, 0) * (x - x_o[i]) + R_bar(1, 0) * (y - y_o[i]));
    A(i, 1) = -2.0 * (R_bar(0, 1) * (x - x_o[i]) + R_bar(1, 1) * (y - y_o[i]));
    double h = (x - x_o[i]) * (x - x_o[i]) + (y - y_o[i]) * (y - y_o[i]) - r_o[i] * r_o[i];
    b(i) = alpha * h;
  }
  
  // 速度约束
  int idx = x_o.size();
  A(idx, 0) = 1.0;
  A(idx, 1) = d;
  A(idx + 1, 0) = 1.0;
  A(idx + 1, 1) = -d;
  A(idx + 2, 0) = -1.0;
  A(idx + 2, 1) = -d;
  A(idx + 3, 0) = -1.0;
  A(idx + 3, 1) = d;
  
  b(idx) = max_speed;
  b(idx + 1) = max_speed;
  b(idx + 2) = max_speed;
  b(idx + 3) = max_speed;
  
  // 使用梯度投影法求解QP问题
  Eigen::Vector2d u = ud;
  
  // 简化求解：直接使用投影梯度下降
  double learning_rate = 0.01;
  int max_iterations = 100;
  
  for (int iter = 0; iter < max_iterations; iter++) {
    // 计算梯度
    Eigen::Vector2d grad = H * u + f;
    
    // 梯度下降更新
    u = u - learning_rate * grad;
    
    // 投影到约束区域
    for (int i = 0; i < num_constraints; i++) {
      double constraint_val = A.row(i) * u - b(i);
      if (constraint_val > 0) {
        // 违反约束，进行投影
        double norm = A.row(i).squaredNorm();
        if (norm > 1e-6) {
          u = u - (constraint_val / norm) * A.row(i).transpose();
        }
      }
    }
  }
  
  return u;
}

}  // namespace intpc_local_planner
