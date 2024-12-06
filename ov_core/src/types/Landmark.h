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

#ifndef OV_TYPE_TYPE_LANDMARK_H
#define OV_TYPE_TYPE_LANDMARK_H

#include "LandmarkRepresentation.h"
#include "Vec.h"
#include "utils/colors.h"
#include "utils/print.h"

namespace ov_type {

/**
 * @brief Type that implements a persistent SLAM feature.
 *
 * We store the feature ID that should match the IDs in the trackers.
 * Additionally if this is an anchored representation we store what clone timestamp this is anchored from and what camera.
 * If this features should be marginalized its flag can be set and during cleanup it will be removed.
 */
class Landmark : public Vec {

public:
  /// Default constructor (feature is a Vec of size 3 or Vec of size 1)
  Landmark(int dim) : Vec(dim) {}

  /// Feature ID of this landmark (corresponds to frontend id)
  /// 该变量用于表示地标的特征ID，通常与前端模块中使用的ID相对应。
  size_t _featid;

  /// What unique camera stream this slam feature was observed from
  /// 该变量用于表示观察到此 SLAM 特征的唯一相机流。它用于跟踪和识别特征在不同相机视角下的观测信息
  int _unique_camera_id = -1;

  /// What camera ID our pose is anchored in!! By default the first measurement is the anchor.
  /// 该变量用于表示位姿锚定的相机ID。默认情况下，第一个测量值即为锚点。
  int _anchor_cam_id = -1;

  /// Timestamp of anchor clone
  /// 特征在被锚定时的具体时间点
  double _anchor_clone_timestamp = -1;

  /// Boolean if this landmark has had at least one anchor change
  /// 该变量用于表示该地标是否经历过至少一次锚定的变化。
  bool has_had_anchor_change = false;

  /// Boolean if this landmark should be marginalized out
  /// 该变量用于表示该地标是否应在处理过程中被边缘化。
  bool should_marg = false;

  /// Number of times the update has failed for this feature (we should remove if it fails a couple times!)
  /// 该变量用于记录此特征更新失败的次数，如果失败次数达到一定阈值，则应考虑将该特征移除，以保持系统的稳定性和准确性。
  int update_fail_count = 0;

  /// First normalized uv coordinate bearing of this measurement (used for single depth representation)
  /// 该变量用于表示此测量的第一个归一化UV坐标方向，通常用于单一深度表示，通过记录此坐标，可以在深度估计和特征跟踪中保持对方向的准确性。
  Eigen::Vector3d uv_norm_zero;

  /// First estimate normalized uv coordinate bearing of this measurement (used for single depth representation)
  /// 该变量用于表示此测量的第一个估计归一化 UV 坐标方向，通常用于单一深度表示，通过记录此坐标，可以在深度估计和特征跟踪中保持对方向的准确性。
  Eigen::Vector3d uv_norm_zero_fej;

  /// What feature representation this feature currently has
  /// 该变量用于指示此特征当前采用的表示形式
  LandmarkRepresentation::Representation _feat_representation;

  /**
   * @brief Overrides the default vector update rule
   * We want to selectively update the FEJ value if we are using an anchored representation.
   * @param dx Additive error state correction
   */
  void update(const Eigen::VectorXd &dx) override {
    // Update estimate
    assert(dx.rows() == _size);
    set_value(_value + dx);
    // Ensure we are not near zero in the z-direction
    // if (LandmarkRepresentation::is_relative_representation(_feat_representation) && _value(_value.rows() - 1) < 1e-8) {
    //  PRINT_DEBUG(YELLOW "WARNING DEPTH %.8f BECAME CLOSE TO ZERO IN UPDATE!!!\n" RESET, _value(_value.rows() - 1));
    //  should_marg = true;
    // }
  }

  /**
   * @brief Will return the position of the feature in the global frame of reference.
   * @param getfej Set to true to get the landmark FEJ value
   * @return Position of feature either in global or anchor frame
   * @return 特征的位置，返回的是在全局框架或锚定框架中的位置.
   */
  Eigen::Matrix<double, 3, 1> get_xyz(bool getfej) const;

  /**
   * @brief Will set the current value based on the representation.
   * @brief 根据表示对当前值进行设置.
   * @param p_FinG Position of the feature either in global or anchor frame
   * @param p_FinG 可以是相对于全局参考框架或锚定框架的坐标。
   * @param isfej Set to true to set the landmark FEJ value
   */
  void set_from_xyz(Eigen::Matrix<double, 3, 1> p_FinG, bool isfej);
};
} // namespace ov_type

#endif // OV_TYPE_TYPE_LANDMARK_H
