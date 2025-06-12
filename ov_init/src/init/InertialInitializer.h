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

#ifndef OV_INIT_INERTIALINITIALIZER_H
#define OV_INIT_INERTIALINITIALIZER_H

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

class StaticInitializer;
class DynamicInitializer;

/**
 * @brief 视觉惯性系统的初始化器
 *
 * 这个初始化器会尝试对系统状态进行动态或静态初始化。
 * 用户可以选择等待IMU读数出现跳变(比如设备被拿起)或者尽快进行初始化。
 * 对于静态初始化,用户需要提前指定标定参数,否则将始终使用动态初始化。
 *
 * 初始化的逻辑如下:
 * 1. 尝试对状态元素进行动态初始化
 * 2. 如果动态初始化失败且我们有标定参数,则尝试进行静态初始化
 * 3. 如果设备静止且我们在等待跳变,则返回,否则直接初始化状态!
 *
 * 动态初始化系统基于论文《Estimator initialization in vision-aided inertial navigation with unknown camera-IMU calibration》
 * 的实现和扩展。该论文通过以下步骤解决初始化问题:
 * 首先创建线性系统恢复相机到IMU的旋转,然后恢复速度、重力和特征点位置,
 * 最后进行完整优化以获得协方差。
 *
 * 另一篇值得参考的论文是《An Analytical Solution to the IMU Initialization Problem for Visual-Inertial Systems》,
 * 该论文对尺度恢复和加速度计偏置进行了详细的实验。
 */
class InertialInitializer {

public:
  /**
   * @brief Default constructor
   *        默认构造函数.
   * @param params_ Parameters loaded from either ROS or CMDLINE
   *        从ROS或命令行读取配置文件.
   * @param db Feature tracker database with all features in it
   *        用于存放特征的数据库.
   */
  explicit InertialInitializer(InertialInitializerOptions &params_, std::shared_ptr<ov_core::FeatureDatabase> db);

  /**
   * @brief Feed function for inertial data
   *        用于输入惯性数据的接口，但图像数据的接口在哪里呢？
   * @param message Contains our timestamp and inertial information
   *        包含时间戳和惯性数据信息.
   * @param oldest_time Time that we can discard measurements before
   *        可以抛弃之前多少的数据.
   * 这里只推入IMU数据.
   */
  void feed_imu(const ov_core::ImuData &message, double oldest_time = -1);

  /**
   * @brief Try to get the initialized system ->用来得到初始化好的系统.
   * @m_class{m-note m-warning} ->这里主要用于doxygen生成网页用的.
   * @par Processing Cost
   * This is a serial process that can take on orders of seconds to complete.
   * If you are a real-time application then you will likely want to call this from
   * a async thread which allows for this to process in the background.
   * The features used are cloned from the feature database thus should be thread-safe
   * to continue to append new feature tracks to the database.
   *
   * 这是一个串行的过程，可能需要几秒钟才能完成这个过程.
   *
   * @param[out] timestamp Timestamp we have initialized the state at
   *                       我们用来初始化的时间戳.
   * @param[out] covariance Calculated covariance of the returned state
   *                       初始化后的状态协方差.
   * @param[out] order Order of the covariance matrix
   *                       协方差的顺序.
   * @param[out] t_imu Our imu type (need to have correct ids)
   *                       我们IMU的类型?
   * @param wait_for_jerk If true we will wait for a "jerk"
   *                       等待跳变.
   * @return True if we have successfully initialized our system
   *              如果我们成功初始化自己的系统，则返回True.
   */
  bool initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order,
                  std::shared_ptr<ov_type::IMU> t_imu, bool wait_for_jerk = true);

protected:
  /// Initialization parameters
  /// 初始化参数.
  InertialInitializerOptions params;

  /// Feature tracker database with all features in it
  /// 特征追踪数据库.
  std::shared_ptr<ov_core::FeatureDatabase> _db;

  /// Our history of IMU messages (time, angular, linear)
  /// 历史IMU数据.
  std::shared_ptr<std::vector<ov_core::ImuData>> imu_data;

  /*****************************************************************************************************
   * @brief 这里可以看到，也没必要一定抽象化类出来->
   *      可以通过包含的子类来看，然后看那个指针不是空指针，就代表用了某类方法.
   *****************************************************************************************************/

  /// 静态初始化相关类.
  std::shared_ptr<StaticInitializer> init_static;

  /// 动态初始化相关类.
  std::shared_ptr<DynamicInitializer> init_dynamic;
};

} // namespace ov_init

#endif // OV_INIT_INERTIALINITIALIZER_H
