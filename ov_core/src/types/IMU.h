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

#ifndef OV_TYPE_TYPE_IMU_H
#define OV_TYPE_TYPE_IMU_H

#include "PoseJPL.h"
#include "utils/quat_ops.h"

namespace ov_type {

/**
 * @brief Type的派生类IMU，用于表示当前IMU状态。
 * 该状态中包含一个PoseJPL、速度Vec、陀螺仪零偏Vec和加速度计零偏Vec。
 * 与标准的MSCKF状态相比，该状态仅在变量的排列顺序不同。
 * 姿态在前，依次是速度、陀螺仪零偏和加速度计零偏。
 */
class IMU : public Type {

public:
  IMU() : Type(15) {

    // Create all the sub-variables
    _pose = std::shared_ptr<PoseJPL>(new PoseJPL());
    _v = std::shared_ptr<Vec>(new Vec(3));
    _bg = std::shared_ptr<Vec>(new Vec(3));
    _ba = std::shared_ptr<Vec>(new Vec(3));

    // Set our default state value
    Eigen::VectorXd imu0 = Eigen::VectorXd::Zero(16, 1);
    imu0(3) = 1.0;
    set_value_internal(imu0);
    set_fej_internal(imu0);
  }

  ~IMU() {}

  /**
   * @brief 设置id，用于跟踪滤波中变量对应的协方差。
   * 同时更新子变量。
   * @param new_id 变量的位置。
   */
  void set_local_id(int new_id) override {
    _id = new_id;
    _pose->set_local_id(new_id);
    _v->set_local_id(_pose->id() + ((new_id != -1) ? _pose->size() : 0));
    _bg->set_local_id(_v->id() + ((new_id != -1) ? _v->size() : 0));
    _ba->set_local_id(_bg->id() + ((new_id != -1) ? _bg->size() : 0));
  }

  /**
   * @brief Performs update operation using JPLQuat update for orientation, then vector updates for
   * position, velocity, gyro bias, and accel bias (in that order).
   *
   * @param dx 15 DOF vector encoding update using the following order (q, p, v, bg, ba)
   */
  void update(const Eigen::VectorXd &dx) override {

    assert(dx.rows() == _size);

    Eigen::Matrix<double, 16, 1> newX = _value;

    Eigen::Matrix<double, 4, 1> dq;
    dq << .5 * dx.block(0, 0, 3, 1), 1.0;
    dq = ov_core::quatnorm(dq);

    newX.block(0, 0, 4, 1) = ov_core::quat_multiply(dq, quat());
    newX.block(4, 0, 3, 1) += dx.block(3, 0, 3, 1);

    newX.block(7, 0, 3, 1) += dx.block(6, 0, 3, 1);
    newX.block(10, 0, 3, 1) += dx.block(9, 0, 3, 1);
    newX.block(13, 0, 3, 1) += dx.block(12, 0, 3, 1);

    set_value(newX);
  }

  /**
   * @brief Sets the value of the estimate
   * @param new_value New value we should set
   */
  void set_value(const Eigen::MatrixXd &new_value) override { set_value_internal(new_value); }

  /**
   * @brief Sets the value of the first estimate
   * @param new_value New value we should set
   */
  void set_fej(const Eigen::MatrixXd &new_value) override { set_fej_internal(new_value); }

  // 克隆当前IMU状态，返回一个IMU状态的智能指针。
  std::shared_ptr<Type> clone() override {
    auto Clone = std::shared_ptr<Type>(new IMU());
    Clone->set_value(value());
    Clone->set_fej(fej());
    return Clone;
  }

  // 检查当前参数变量是否是当前状态中的子变量。
  std::shared_ptr<Type> check_if_subvariable(const std::shared_ptr<Type> check) override {
    if (check == _pose) {
      return _pose;
    } else if (check == _pose->check_if_subvariable(check)) {
      return _pose->check_if_subvariable(check);
    } else if (check == _v) {
      return _v;
    } else if (check == _bg) {
      return _bg;
    } else if (check == _ba) {
      return _ba;
    }
    return nullptr;
  }

  /// 访问状态中的旋转变量。
  Eigen::Matrix<double, 3, 3> Rot() const { return _pose->Rot(); }

  /// 访问状态中旋转变量对应的FEJ.
  Eigen::Matrix<double, 3, 3> Rot_fej() const { return _pose->Rot_fej(); }

  /// 访问状态中的旋转部分。
  Eigen::Matrix<double, 4, 1> quat() const { return _pose->quat(); }

  /// FEJ Rotation access quaternion
  Eigen::Matrix<double, 4, 1> quat_fej() const { return _pose->quat_fej(); }

  /// Position access
  Eigen::Matrix<double, 3, 1> pos() const { return _pose->pos(); }

  /// FEJ position access
  Eigen::Matrix<double, 3, 1> pos_fej() const { return _pose->pos_fej(); }

  /// Velocity access
  Eigen::Matrix<double, 3, 1> vel() const { return _v->value(); }

  // FEJ velocity access
  Eigen::Matrix<double, 3, 1> vel_fej() const { return _v->fej(); }

  /// Gyro bias access
  Eigen::Matrix<double, 3, 1> bias_g() const { return _bg->value(); }

  /// FEJ gyro bias access
  Eigen::Matrix<double, 3, 1> bias_g_fej() const { return _bg->fej(); }

  /// Accel bias access
  Eigen::Matrix<double, 3, 1> bias_a() const { return _ba->value(); }

  // FEJ accel bias access
  Eigen::Matrix<double, 3, 1> bias_a_fej() const { return _ba->fej(); }

  /// Pose type access
  std::shared_ptr<PoseJPL> pose() { return _pose; }

  /// Quaternion type access
  std::shared_ptr<JPLQuat> q() { return _pose->q(); }

  /// Position type access
  std::shared_ptr<Vec> p() { return _pose->p(); }

  /// Velocity type access
  std::shared_ptr<Vec> v() { return _v; }

  /// Gyroscope bias access
  std::shared_ptr<Vec> bg() { return _bg; }

  /// Acceleration bias access
  std::shared_ptr<Vec> ba() { return _ba; }

protected:
  /// Pose subvariable
  std::shared_ptr<PoseJPL> _pose;

  /// Velocity subvariable
  std::shared_ptr<Vec> _v;

  /// Gyroscope bias subvariable
  std::shared_ptr<Vec> _bg;

  /// Acceleration bias subvariable
  std::shared_ptr<Vec> _ba;

  /**
   * @brief Sets the value of the estimate
   * @param new_value New value we should set
   */
  void set_value_internal(const Eigen::MatrixXd &new_value) {

    assert(new_value.rows() == 16);
    assert(new_value.cols() == 1);

    _pose->set_value(new_value.block(0, 0, 7, 1));
    _v->set_value(new_value.block(7, 0, 3, 1));
    _bg->set_value(new_value.block(10, 0, 3, 1));
    _ba->set_value(new_value.block(13, 0, 3, 1));

    _value = new_value;
  }

  /**
   * @brief Sets the value of the first estimate
   * @param new_value New value we should set
   */
  void set_fej_internal(const Eigen::MatrixXd &new_value) {

    assert(new_value.rows() == 16);
    assert(new_value.cols() == 1);

    _pose->set_fej(new_value.block(0, 0, 7, 1));
    _v->set_fej(new_value.block(7, 0, 3, 1));
    _bg->set_fej(new_value.block(10, 0, 3, 1));
    _ba->set_fej(new_value.block(13, 0, 3, 1));

    _fej = new_value;
  }
};

} // namespace ov_type

#endif // OV_TYPE_TYPE_IMU_H
