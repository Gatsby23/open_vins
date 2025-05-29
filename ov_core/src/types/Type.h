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

#ifndef OV_TYPE_TYPE_BASE_H
#define OV_TYPE_TYPE_BASE_H

#include <Eigen/Eigen>
#include <memory>

namespace ov_type {

/**
 * @brief Base class for estimated variables.
 *        待估计状态的基类.
 * This class is used how variables are represented or updated (e.g., vectors or quaternions).
 * Each variable is defined by its error state size and its location in the covariance matrix.
 * We additionally require all sub-types to have a update procedure.
 */
class Type {

public:
  /**
   * @brief 自定义类型基类的默认构造函数
   * @param size_ 代表变量的自由度数量（即误差状态的维度大小）
   */
  Type(int size_) { _size = size_; }

  virtual ~Type(){};

  /**
   * @brief 用于跟踪该变量在滤波器协方差矩阵中的位置。
   * 注意，最小的 ID 值为 -1，表示该状态未包含在协方差矩阵中。
   * 若 ID 大于 -1，则表示其在协方差矩阵中的索引位置。
   *
   * @param new_id 对应于该变量在滤波器协方差矩阵中的条目索引
   */
  virtual void set_local_id(int new_id) { _id = new_id; }

  /**
   * @brief 获取该变量在协方差矩阵中的位置ID
   */
  int id() { return _id; }

  /**
   * @brief 获取当前变量的大小（通常指error state）.
   */
  int size() { return _size; }

  /**
   * @brief 依据误差对当前状态进行扰动更新.
   * @param dx 这里的扰动是通过之前预先定义的运算法则来执行.
   */
  virtual void update(const Eigen::VectorXd &dx) = 0;

  /**
   * @brief 获取当前变量的预测值.
   */
  virtual const Eigen::MatrixXd &value() const { return _value; }

  /**
   * @brief 获取当前变量的FEJ.
   */
  virtual const Eigen::MatrixXd &fej() const { return _fej; }

  /**
   * @brief 设置当前估计的新值.
   * @param new_value 设置的新值.
   */
  virtual void set_value(const Eigen::MatrixXd &new_value) {
    assert(_value.rows() == new_value.rows());
    assert(_value.cols() == new_value.cols());
    _value = new_value;
  }

  /**
   * @brief 设置当前FEJ的值.
   * @param new_value 当前FEJ的设置值.
   */
  virtual void set_fej(const Eigen::MatrixXd &new_value) {
    assert(_fej.rows() == new_value.rows());
    assert(_fej.cols() == new_value.cols());
    _fej = new_value;
  }

  /**
   * @brief 对当前状态进行克隆.
   */
  virtual std::shared_ptr<Type> clone() = 0;

  /**
   * @brief Determine if pass variable is a sub-variable
   *        判断心传递的状态值是不是状态的子变量.
   * If the passed variable is a sub-variable or the current variable this will return it.
   * Otherwise it will return a nullptr, meaning that it was unable to be found.
   *
   * @param check Type pointer to compare our subvariables to
   */
  virtual std::shared_ptr<Type> check_if_subvariable(const std::shared_ptr<Type> check) { return nullptr; }

protected:
  /// FEJ
  Eigen::MatrixXd _fej;

  /// 当前的最优估计.
  Eigen::MatrixXd _value;

  /// 当前误差状态在协方差中的位置.
  int _id = -1;

  /// 当前误差状态的维度.
  int _size = -1;
};

} // namespace ov_type

#endif // OV_TYPE_TYPE_BASE_H