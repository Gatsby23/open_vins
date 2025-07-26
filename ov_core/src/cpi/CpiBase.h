#ifndef CPI_BASE_H
#define CPI_BASE_H

/*
 * MIT License
 * Copyright (c) 2018 Kevin Eckenhoff
 * Copyright (c) 2018 Patrick Geneva
 * Copyright (c) 2018 Guoquan Huang
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "utils/quat_ops.h"
#include <Eigen/Dense>

namespace ov_core {

/**
 * @brief 预积分的基础类.
 * 具体的实现在CpiV1和CpiV2两个类中.
 * 理论细节在@cite Eckenhoff2019IJRR :
 * > Continuous Preintegration Theory for Graph-based Visual-Inertial Navigation
 * > Authors: Kevin Eckenhoff, Patrick Geneva, and Guoquan Huang
 * > http://udel.edu/~ghuang/papers/tr_cpi.pdf
 *
 * 使用该预积分的方式如下三步：
 * 1. 调用setLinearizationPoints()来设置bias/线性化点
 * 2. 调用feed_IMU()传入所有IMU测量值
 * 3. 获取估计出的均值、雅可比矩阵和测量协方差
 */
class CpiBase {
public:
  /**
   * @brief 默认构造函数
   * @param sigma_w 陀螺仪白噪声密度 (rad/s/sqrt(hz))
   * @param sigma_wb 陀螺仪随机游走 (rad/s^2/sqrt(hz))
   * @param sigma_a 加速度计白噪声密度 (m/s^2/sqrt(hz))
   * @param sigma_ab 加速度计随机游走 (m/s^3/sqrt(hz))
   * @param imu_avg_ 是否对IMU测量信息进行平均化处理(IJRR paper did not do this)。
   */
  CpiBase(double sigma_w, double sigma_wb, double sigma_a, double sigma_ab, bool imu_avg_ = false) {
    // Calculate our covariance matrix
    Q_c.block(0, 0, 3, 3) = std::pow(sigma_w, 2) * eye3;
    Q_c.block(3, 3, 3, 3) = std::pow(sigma_wb, 2) * eye3;
    Q_c.block(6, 6, 3, 3) = std::pow(sigma_a, 2) * eye3;
    Q_c.block(9, 9, 3, 3) = std::pow(sigma_ab, 2) * eye3;
    imu_avg = imu_avg_;
    /// 计算单位向量及其斜对称矩阵(用于后续的雅可比计算)
    e_1 << 1, 0, 0;
    e_2 << 0, 1, 0;
    e_3 << 0, 0, 1;
    e_1x = skew_x(e_1);
    e_2x = skew_x(e_2);
    e_3x = skew_x(e_3);
  }

  virtual ~CpiBase() {}

  /**
   * @brief Set linearization points of the integration.
   * @param[in] b_w_lin_ gyroscope bias linearization point
   * @param[in] b_a_lin_ accelerometer bias linearization point
   * @param[in] q_k_lin_ orientation linearization point (only model 2 uses)
   * @param[in] grav_ global gravity at the current timestep
   *
   * This function sets the linearization points we are to preintegrate about.
   * For model 2 we will also pass the q_GtoK and current gravity estimate.
   */
  void setLinearizationPoints(Eigen::Matrix<double, 3, 1> b_w_lin_, Eigen::Matrix<double, 3, 1> b_a_lin_,
                              Eigen::Matrix<double, 4, 1> q_k_lin_ = Eigen::Matrix<double, 4, 1>::Zero(),
                              Eigen::Matrix<double, 3, 1> grav_ = Eigen::Matrix<double, 3, 1>::Zero()) {
    b_w_lin = b_w_lin_;
    b_a_lin = b_a_lin_;
    q_k_lin = q_k_lin_;
    grav = grav_;
  }

  /**
   * @brief Main function that will sequentially compute the preintegration measurement.
   * @param[in] t_0 first IMU timestamp
   * @param[in] t_1 second IMU timestamp
   * @param[in] w_m_0 first imu gyroscope measurement
   * @param[in] a_m_0 first imu acceleration measurement
   * @param[in] w_m_1 second imu gyroscope measurement
   * @param[in] a_m_1 second imu acceleration measurement
   *
   * This new IMU messages and will precompound our measurements, jacobians, and measurement covariance.
   * Please see both CpiV1 and CpiV2 classes for implementation details on how this works.
   */
  virtual void feed_IMU(double t_0, double t_1, Eigen::Matrix<double, 3, 1> w_m_0, Eigen::Matrix<double, 3, 1> a_m_0,
                        Eigen::Matrix<double, 3, 1> w_m_1 = Eigen::Matrix<double, 3, 1>::Zero(),
                        Eigen::Matrix<double, 3, 1> a_m_1 = Eigen::Matrix<double, 3, 1>::Zero()) = 0;

  // 是否进行IMU测量值平均化处理.
  // For version 1 we should average the measurement
  // For version 2 we average the local true
  bool imu_avg = false;

  // 测量信息.
  double DT = 0;                                                                 ///< measurement integration time
  Eigen::Matrix<double, 3, 1> alpha_tau = Eigen::Matrix<double, 3, 1>::Zero();   ///< alpha measurement mean
  Eigen::Matrix<double, 3, 1> beta_tau = Eigen::Matrix<double, 3, 1>::Zero();    ///< beta measurement mean
  Eigen::Matrix<double, 4, 1> q_k2tau;                                           ///< orientation measurement mean
  Eigen::Matrix<double, 3, 3> R_k2tau = Eigen::Matrix<double, 3, 3>::Identity(); ///< orientation measurement mean

  // 雅可比矩阵.
  Eigen::Matrix<double, 3, 3> J_q = Eigen::Matrix<double, 3, 3>::Zero(); ///< orientation Jacobian wrt b_w
  Eigen::Matrix<double, 3, 3> J_a = Eigen::Matrix<double, 3, 3>::Zero(); ///< alpha Jacobian wrt b_w
  Eigen::Matrix<double, 3, 3> J_b = Eigen::Matrix<double, 3, 3>::Zero(); ///< beta Jacobian wrt b_w
  Eigen::Matrix<double, 3, 3> H_a = Eigen::Matrix<double, 3, 3>::Zero(); ///< alpha Jacobian wrt b_a
  Eigen::Matrix<double, 3, 3> H_b = Eigen::Matrix<double, 3, 3>::Zero(); ///< beta Jacobian wrt b_a

  // 线性化点.
  Eigen::Matrix<double, 3, 1> b_w_lin; ///< b_w linearization point (gyroscope)
  Eigen::Matrix<double, 3, 1> b_a_lin; ///< b_a linearization point (accelerometer)
  Eigen::Matrix<double, 4, 1> q_k_lin; ///< q_k linearization point (only model 2 uses)

  /// 全局重力变量.
  Eigen::Matrix<double, 3, 1> grav = Eigen::Matrix<double, 3, 1>::Zero();

  /// 连续时间测量噪声矩阵(从构造函数中的噪声值计算得出)
  Eigen::Matrix<double, 12, 12> Q_c = Eigen::Matrix<double, 12, 12>::Zero();

  ///最后的测量协方差.
  Eigen::Matrix<double, 15, 15> P_meas = Eigen::Matrix<double, 15, 15>::Zero();

  //==========================================================================
  // HELPER VARIABLES
  //==========================================================================

  // 3x3 identity matrix
  Eigen::Matrix<double, 3, 3> eye3 = Eigen::Matrix<double, 3, 3>::Identity();

  // Simple unit vectors (used in bias jacobian calculations)
  Eigen::Matrix<double, 3, 1> e_1; // = Eigen::Matrix<double,3,1>::Constant(1,0,0);
  Eigen::Matrix<double, 3, 1> e_2; // = Eigen::Matrix<double,3,1>::Constant(0,1,0);
  Eigen::Matrix<double, 3, 1> e_3; // = Eigen::Matrix<double,3,1>::Constant(0,0,1);

  // Calculate the skew-symetric of our unit vectors
  Eigen::Matrix<double, 3, 3> e_1x; // = skew_x(e_1);
  Eigen::Matrix<double, 3, 3> e_2x; // = skew_x(e_2);
  Eigen::Matrix<double, 3, 3> e_3x; // = skew_x(e_3);
};

} // namespace ov_core

#endif /* CPI_BASE_H */