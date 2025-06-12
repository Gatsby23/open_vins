/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_INIT_STATICINITIALIZER_H
#define OV_INIT_STATICINITIALIZER_H

#include "init/InertialInitializerOptions.h"

namespace ov_core {
class FeatureDatabase;
struct ImuData;
} // namespace ov_core
namespace ov_type {
class Type;
class IMU;
} // namespace ov_type

namespace ov_init {

/**
 * @brief 视觉惯性系统的静态初始化器，用于VIO系统的静态初始化方法.
 * 该实现假设IMU从静止状态开始。
 * 1. 收集所有惯性测量数据
 * 2. 检查在最后一个时间窗口内是否存在加速度跳变
 * 3. 如果跳变超过阈值则进行初始化(即系统开始运动)
 * 4. 使用前一个时间窗口(此时系统应该处于静止状态)来初始化姿态
 * 5. 返回与重力和偏置对齐的横滚角和俯仰角
 *
 */
class StaticInitializer {

public:
  /******************************************************
   * @brief 默认构造函数
   * @param params_ 从ROS或命令行加载的参数
   * @param db 包含所有特征的特征追踪器数据库
   * @param imu_data_ 指向我们IMU历史信息向量的共享指针
   ******************************************************/
   // 有了explicit关键字，防止隐式类型转换.
  explicit StaticInitializer(InertialInitializerOptions &params_, std::shared_ptr<ov_core::FeatureDatabase> db,
                             std::shared_ptr<std::vector<ov_core::ImuData>> imu_data_)
      : params(params_), _db(db), imu_data(imu_data_) {}

  /**
   * @brief 尝试仅使用IMU数据初始化系统
   *
   * 该函数会检查加速度是否发生了足够大的跳变。
   * 如果发生跳变，我们将使用跳变前的时间段来初始化状态。
   * 这假设我们的IMU处于静止状态且没有移动（因此如果我们经历恒定的加速度，这将失败）。
   *
   * 如果我们不等待跳变（即`wait_for_jerk`为false），则系统将尝试尽快初始化。
   * 这仅在启用了零速度更新来处理静止情况时才建议使用。
   * 在这种情况下进行初始化，我们需要平均角速度方差低于设定的阈值（即我们需要处于静止状态）。
   *
   * @param[out] timestamp 我们初始化状态时的时间戳
   * @param[out] covariance 返回状态的计算协方差
   * @param[out] order 协方差矩阵的顺序
   * @param[out] t_imu 我们的IMU类型元素
   * @param wait_for_jerk 如果为true，我们将等待"跳变"
   * @return 如果我们成功初始化了系统，则返回True
   */
  bool initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order,
                  std::shared_ptr<ov_type::IMU> t_imu, bool wait_for_jerk = true);

private:
  /// 初始化参数.
  InertialInitializerOptions params;

  /// 追踪特征数据库.
  std::shared_ptr<ov_core::FeatureDatabase> _db;

  /// 历史的IMU信息数据.
  std::shared_ptr<std::vector<ov_core::ImuData>> imu_data;
};

} // namespace ov_init

#endif // OV_INIT_STATICINITIALIZER_H
