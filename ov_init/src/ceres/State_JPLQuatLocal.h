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

#ifndef OV_INIT_CERES_JPLQUATLOCAL_H
#define OV_INIT_CERES_JPLQUATLOCAL_H

#include <ceres/ceres.h>

namespace ov_init {

/**
 * @brief 针对JPL形式的四元数状态参数化
 */
class State_JPLQuatLocal : public ceres::LocalParameterization {
public:
  /**
   * @brief JPL更新函数。
   * 通过将当前四元数与一个由小轴角扰动构建的四元数相乘来实现左乘操作，对当前状态进行更新。
   * @f[
   * \bar{q}=norm\Big(\begin{bmatrix} 0.5*\mathbf{\theta_{dx}} \\ 1 \end{bmatrix}\Big) \hat{\bar{q}}
   * @f]
   */
  bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;

  /**
   * @brief 针对局部参数化形式的Jacobian求解计算.
   * CERES默认的做法是先对全局参数求导，再对局部参数求导，从而得到雅可比矩阵。即
   * dr/dlocal= dr/dglobal * dglobal/dlocal
   * 但我们是直接对局部参数求导得到Jacobian矩阵，即:
   * dr/dlocal= [ dr/dlocal, 0] * [I; 0]= dr/dlocal.
   * 因此这里直接将全局对局部的导数定义为 dglobal/dlocal= [I; 0]
   */
  bool ComputeJacobian(const double *x, double *jacobian) const override;

  int GlobalSize() const override { return 4; };

  int LocalSize() const override { return 3; };
};

} // namespace ov_init

#endif // OV_INIT_CERES_JPLQUATLOCAL_H