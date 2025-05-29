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

#ifndef OV_MSCKF_STATE_H
#define OV_MSCKF_STATE_H

#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "StateOptions.h"
#include "cam/CamBase.h"
#include "types/IMU.h"
#include "types/Landmark.h"
#include "types/PoseJPL.h"
#include "types/Type.h"
#include "types/Vec.h"

namespace ov_msckf {

/**
 * @brief State of our filter
 *
 * This state has all the current estimates for the filter.
 * This system is modeled after the MSCKF filter, thus we have a sliding window of clones.
 * We additionally have more parameters for online estimation of calibration and SLAM features.
 * We also have the covariance of the system, which should be managed using the StateHelper class.
 */
class State {

public:
  /**
   * @brief Default Constructor (will initialize variables to defaults)
   * @param options_ Options structure containing filter options
   */
  State(StateOptions &options_);

  ~State() {}

    /**
     * @brief 返回下一个将被边缘化的时间戳->即最老的时间戳。
     * 当前实现使用滑动窗口机制，因此这里返回的是最早的克隆状态（oldest clone）。
     * 但如果采用关键帧系统，也可以选择性地边缘化某些克隆状态。
     * @return 将被边缘化的克隆状态的时间戳
     */
  double margtimestep() {
    std::lock_guard<std::mutex> lock(_mutex_state);
    double time = INFINITY;
    for (const auto &clone_imu : _clones_IMU) {
      if (clone_imu.first < time) {
        time = clone_imu.first;
      }
    }
    return time;
  }

   /**
    * @brief 说明现在协方差矩阵的大小.
    * @return 返回协方差矩阵的大小。
    */
  int max_covariance_size() { return (int)_Cov.rows(); }

  /**
   * @brief Gyroscope and accelerometer intrinsic matrix (scale imperfection and axis misalignment)
   *
   * If kalibr model, lower triangular of the matrix is used
   * If rpng model, upper triangular of the matrix is used
   *
   * @return 3x3 matrix of current imu gyroscope / accelerometer intrinsics
   */
  static Eigen::Matrix3d Dm(StateOptions::ImuModel imu_model, const Eigen::MatrixXd &vec) {
    assert(vec.rows() == 6);
    assert(vec.cols() == 1);
    Eigen::Matrix3d D_matrix = Eigen::Matrix3d::Identity();
    if (imu_model == StateOptions::ImuModel::KALIBR) {
      D_matrix << vec(0), 0, 0, vec(1), vec(3), 0, vec(2), vec(4), vec(5);
    } else {
      D_matrix << vec(0), vec(1), vec(3), 0, vec(2), vec(4), 0, 0, vec(5);
    }
    return D_matrix;
  }

  /**
   * @brief 重力敏感性.
   * 无论是Kalibr还是RNG的IMU内参模型，该矩阵都是一个按列填充的矩阵.
   * @return 一个3x3大小的重力敏感性矩阵.
   */
  static Eigen::Matrix3d Tg(const Eigen::MatrixXd &vec) {
    assert(vec.rows() == 9);
    assert(vec.cols() == 1);
    Eigen::Matrix3d Tg = Eigen::Matrix3d::Zero();
    Tg << vec(0), vec(3), vec(6), vec(1), vec(4), vec(7), vec(2), vec(5), vec(8);
    return Tg;
  }

  /**
   * @brief Calculates the error state size for imu intrinsics.
   *
   * This is used to construct our state transition which depends on if we are estimating calibration.
   * 15 if doing intrinsics, another +9 if doing grav sensitivity
   *
   * @return size of error state
   */
  int imu_intrinsic_size() const {
    int sz = 0;
    if (_options.do_calib_imu_intrinsics) {
      sz += 15;
      if (_options.do_calib_imu_g_sensitivity) {
        sz += 9;
      }
    }
    return sz;
  }

  /// 控制state访问的锁.
  std::mutex _mutex_state;

  /// 当前时间戳.
  double _timestamp = -1;

  /// 滤波器的整体配置.
  StateOptions _options;

  /// 指向IMU状态的指针.
  std::shared_ptr<ov_type::IMU> _imu;

  /// 成像时间与克隆的IMU状态 ->
  /// 状态克隆即是把相机那一时刻下的IMU状态给映射出来.
  std::map<double, std::shared_ptr<ov_type::PoseJPL>> _clones_IMU;

  /// 当前SLAM特征的集合.
  std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> _features_SLAM;

  /// IMU和相机之间的时间差[Cam->IMU].
  std::shared_ptr<ov_type::Vec> _calib_dt_CAMtoIMU;

  /// IMU->相机的外参.
  std::unordered_map<size_t, std::shared_ptr<ov_type::PoseJPL>> _calib_IMUtoCAM;

  /// 相机内参.
  std::unordered_map<size_t, std::shared_ptr<ov_type::Vec>> _cam_intrinsics;

  /// 基于IMU内参的相机实体.
  std::unordered_map<size_t, std::shared_ptr<ov_core::CamBase>> _cam_intrinsics_cameras;

  /// Gyroscope IMU intrinsics (scale imperfection and axis misalignment)
  std::shared_ptr<ov_type::Vec> _calib_imu_dw;

  /// Accelerometer IMU intrinsics (scale imperfection and axis misalignment)
  std::shared_ptr<ov_type::Vec> _calib_imu_da;

  /// Gyroscope gravity sensitivity
  std::shared_ptr<ov_type::Vec> _calib_imu_tg;

  /// Rotation from gyroscope frame to the "IMU" accelerometer frame (kalibr model)
  /// 从GYRO->ACC的外参转化(Kalibr模型)
  std::shared_ptr<ov_type::JPLQuat> _calib_imu_GYROtoIMU;

  /// Rotation from accelerometer to the "IMU" gyroscope frame frame (rpng model)
  /// RNG模型，从IMU加速度->IMU陀螺仪的旋转转换.
  std::shared_ptr<ov_type::JPLQuat> _calib_imu_ACCtoIMU;

private:
  // Define that the state helper is a friend class of this class
  // This will allow it to access the below functions which should normally not be called
  // This prevents a developer from thinking that the "insert clone" will actually correctly add it to the covariance
  friend class StateHelper;

  /// Covariance of all active variables
  /// 激活变量的协方差.
  Eigen::MatrixXd _Cov;

  /// Vector of variables
  /// 变量的集合.
  std::vector<std::shared_ptr<ov_type::Type>> _variables;
};

} // namespace ov_msckf

#endif // OV_MSCKF_STATE_H