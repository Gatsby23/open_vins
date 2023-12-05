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
 * @brief Initializer for visual-inertial system.
 *        VIO系统的初始化模块.
 * This will try to do both dynamic and state initialization of the state.
 * The user can request to wait for a jump in our IMU readings (i.e. device is picked up) or to initialize as soon as possible.
 * For state initialization, the user needs to specify the calibration beforehand, otherwise dynamic is always used.
 * 这里将尝试对系统进行动态初始化，来初始化我们的状态。一般来说，我们会先进行动态初始化。如果用户指定了IMU数据跳变（如把设备拿起等），否则就进行静态太初始化.
 * The logic is as follows:
 * 1. Try to perform dynamic initialization of state elements.
 * 2. If this fails and we have calibration then we can try to do static initialization
 * 3. If the unit is stationary and we are waiting for a jerk, just return, otherwise initialize the state!
 *  如果什么跳变都没有的话，则直接进行初始化.
 * The dynamic system is based on an implementation and extension of the work [Estimator initialization in vision-aided inertial navigation
 * with unknown camera-IMU calibration](https://ieeexplore.ieee.org/document/6386235) @cite Dong2012IROS which solves the initialization
 * problem by first creating a linear system for recovering the camera to IMU rotation, then for velocity, gravity, and feature positions,
 * and finally a full optimization to allow for covariance recovery.
 * 这里动态初始化主要是基于论文《Estimator initialization in vision-aided inertial navigation with unknown camera-IMU calibration》来实现，
 * Dong创建了第一个线性系统来恢复出相机和IMU之间的旋转，然后是速度、重力和特征点位置，最后是优化出整体对应的协方差矩阵.
 * Another paper which might be of interest to the reader is [An Analytical Solution to the IMU Initialization
 * Problem for Visual-Inertial Systems](https://ieeexplore.ieee.org/abstract/document/9462400) which has some detailed
 * experiments on scale recovery and the accelerometer bias.
 * 另一篇文章更关注恢复尺度和加速度计bias.
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
   * @brief 这里可以看到，也没必要一定抽象化类出来->可以通过包含的子类来看，然后看那个指针不是空指针，就代表用了某类方法.
   *****************************************************************************************************/

  /// Static initialization helper class
  /// 静态初始化相关.
  std::shared_ptr<StaticInitializer> init_static;

  /// Dynamic initialization helper class
  /// 动态初始化相关.
  std::shared_ptr<DynamicInitializer> init_dynamic;
};

} // namespace ov_init

#endif // OV_INIT_INERTIALINITIALIZER_H
