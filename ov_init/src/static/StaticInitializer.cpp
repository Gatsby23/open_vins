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

#include "StaticInitializer.h"

#include "utils/helper.h"

#include "feat/FeatureHelper.h"
#include "types/IMU.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_init;

bool StaticInitializer::initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<Type>> &order,
                                   std::shared_ptr<IMU> t_imu, bool wait_for_jerk) {
  // 如果没有测量数据则返回
  if (imu_data->size() < 2) {
    return false;
  }

  // 最新和最老的IMU时间戳.
  double newesttime = imu_data->at(imu_data->size() - 1).timestamp;
  double oldesttime = imu_data->at(0).timestamp;

  // 如果时间窗太短，则直接返回.
  if (newesttime - oldesttime < params.init_window_time) {
    PRINT_INFO(YELLOW "[init-s]: unable to select window of IMU readings, not enough readings\n" RESET);
    return false;
  }

  // 1. 收集出IMU数据窗口,并将收集到的数据划分成两个时间窗口.
  std::vector<ImuData> window_1to0, window_2to1;
  for (const ImuData &data : *imu_data) {
    if (data.timestamp > newesttime - 0.5 * params.init_window_time && data.timestamp <= newesttime - 0.0 * params.init_window_time) {
      window_1to0.push_back(data);
    }
    if (data.timestamp > newesttime - 1.0 * params.init_window_time && data.timestamp <= newesttime - 0.5 * params.init_window_time) {
      window_2to1.push_back(data);
    }
  }

  // 2. 检查两个时间窗口，如果它们各自中的数量小于2，则返回false.
  if (window_1to0.size() < 2 || window_2to1.size() < 2) {
    PRINT_INFO(YELLOW "[init-s]: unable to select window of IMU readings, not enough readings\n" RESET);
    return false;
  }

  // 3. 计算第一个窗口（0->1）中的平均加速度和方差值.
  Eigen::Vector3d a_avg_1to0 = Eigen::Vector3d::Zero();
  for (const ImuData &data : window_1to0) {
    a_avg_1to0 += data.am;
  }
  a_avg_1to0 /= (int)window_1to0.size();
  double a_var_1to0 = 0;
  for (const ImuData &data : window_1to0) {
    a_var_1to0 += (data.am - a_avg_1to0).dot(data.am - a_avg_1to0);
  }
  a_var_1to0 = std::sqrt(a_var_1to0 / ((int)window_1to0.size() - 1));


  // 4. 计算第二个窗口（1->2）中的平均加速度以及其方差和平均角速度及其方差.
  Eigen::Vector3d a_avg_2to1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_avg_2to1 = Eigen::Vector3d::Zero();
  for (const ImuData &data : window_2to1) {
    a_avg_2to1 += data.am;
    w_avg_2to1 += data.wm;
  }
  a_avg_2to1 = a_avg_2to1 / window_2to1.size();
  w_avg_2to1 = w_avg_2to1 / window_2to1.size();
  double a_var_2to1 = 0;
  for (const ImuData &data : window_2to1) {
    a_var_2to1 += (data.am - a_avg_2to1).dot(data.am - a_avg_2to1);
  }
  a_var_2to1 = std::sqrt(a_var_2to1 / ((int)window_2to1.size() - 1));
  PRINT_DEBUG(YELLOW "[init-s]: IMU excitation stats: %.3f,%.3f\n" RESET, a_var_2to1, a_var_1to0);

  // 5. 如果加速度的协方差低于阈值，则一直等到有冲击的情况下再初始化.
  if (a_var_1to0 < params.init_imu_thresh && wait_for_jerk) {
    PRINT_INFO(YELLOW "[init-s]: no IMU excitation, below threshold %.3f < %.3f\n" RESET, a_var_1to0, params.init_imu_thresh);
    return false;
  }

  // 6. 检查旧状态是否低于阈值！这种情况发生在我们开始移动时，因此我们需要等待一段时间的静止运动.
  //    如果加速度的协方差高于阈值，并且在等待冲击的情况下则返回false，代表初始化失败.
  if (a_var_2to1 > params.init_imu_thresh && wait_for_jerk) {
    PRINT_INFO(YELLOW "[init-s]: to much IMU excitation, above threshold %.3f > %.3f\n" RESET, a_var_2to1, params.init_imu_thresh);
    return false;
  }

  // 7. If it is above the threshold and we are not waiting for a jerk
  // Then we are not stationary (i.e. moving) so we should wait till we are
  if ((a_var_1to0 > params.init_imu_thresh || a_var_2to1 > params.init_imu_thresh) && !wait_for_jerk) {
    PRINT_INFO(YELLOW "[init-s]: to much IMU excitation, above threshold %.3f,%.3f > %.3f\n" RESET, a_var_2to1, a_var_1to0,
               params.init_imu_thresh);
    return false;
  }

  // 8. 通过z轴对齐，获得旋转矩阵.
  Eigen::Vector3d z_axis = a_avg_2to1 / a_avg_2to1.norm();
  Eigen::Matrix3d Ro;
  InitializerHelper::gram_schmidt(z_axis, Ro);
  Eigen::Vector4d q_GtoI = rot_2_quat(Ro);

  // 8. 依据结果计算出bg和ba(正常为0或重力值).
  Eigen::Vector3d gravity_inG;
  gravity_inG << 0.0, 0.0, params.gravity_mag;
  Eigen::Vector3d bg = w_avg_2to1;
  Eigen::Vector3d ba = a_avg_2to1 - quat_2_Rot(q_GtoI) * gravity_inG;

  // 10. 设置出对应的状态.
  timestamp = window_2to1.at(window_2to1.size() - 1).timestamp;
  Eigen::VectorXd imu_state = Eigen::VectorXd::Zero(16);
  imu_state.block(0, 0, 4, 1) = q_GtoI;
  imu_state.block(10, 0, 3, 1) = bg;
  imu_state.block(13, 0, 3, 1) = ba;
  assert(t_imu != nullptr);
  t_imu->set_value(imu_state);
  t_imu->set_fej(imu_state);

  // 设置对应的协方差.
  order.clear();
  order.push_back(t_imu);
  covariance = std::pow(0.02, 2) * Eigen::MatrixXd::Identity(t_imu->size(), t_imu->size());
  covariance.block(0, 0, 3, 3) = std::pow(0.02, 2) * Eigen::Matrix3d::Identity(); // q
  covariance.block(3, 3, 3, 3) = std::pow(0.05, 2) * Eigen::Matrix3d::Identity(); // p
  covariance.block(6, 6, 3, 3) = std::pow(0.01, 2) * Eigen::Matrix3d::Identity(); // v (static)

  // A VIO system has 4dof unobservable directions which can be arbitrarily picked.
  // This means that on startup, we can fix the yaw and position to be 100 percent known.
  // TODO: why can't we set these to zero and get a good NEES realworld result?
  // Thus, after determining the global to current IMU orientation after initialization, we can propagate the global error
  // into the new IMU pose. In this case the position is directly equivalent, but the orientation needs to be propagated.
  // We propagate the global orientation into the current local IMU frame
  // R_GtoI = R_GtoI*R_GtoG -> H = R_GtoI
  // Eigen::Matrix3d R_GtoI = quat_2_Rot(q_GtoI);
  // covariance(2, 2) = std::pow(1e-4, 2);
  // covariance.block(0, 0, 3, 3) = R_GtoI * covariance.block(0, 0, 3, 3) * R_GtoI.transpose();
  // covariance.block(3, 3, 3, 3) = std::pow(1e-3, 2) * Eigen::Matrix3d::Identity();

  // Return :D
  return true;
}
