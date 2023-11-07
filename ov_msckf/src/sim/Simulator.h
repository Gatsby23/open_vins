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

#ifndef OV_MSCKF_SIMULATOR_H
#define OV_MSCKF_SIMULATOR_H

#include <Eigen/Eigen>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>

#include "core/VioManagerOptions.h"

namespace ov_core {
class BsplineSE3;
} // namespace ov_core

namespace ov_msckf {

/**
 * @brief Master simulator class that generated visual-inertial measurements
 *
 * Given a trajectory this will generate a SE(3) @ref ov_core::BsplineSE3 for that trajectory.
 * This allows us to get the inertial measurement information at each timestep during this trajectory.
 * After creating the bspline we will generate an environmental feature map which will be used as our feature measurements.
 * This map will be projected into the frame at each timestep to get our "raw" uv measurements.
 * We inject bias and white noises into our inertial readings while adding our white noise to the uv measurements also.
 * The user should specify the sensor rates that they desire along with the seeds of the random number generators.
 *
 */
/*****************************************************************
 * @brief 这里仿真器主要作用是给定轨迹，生成对应的观测特征+某一时刻的IMU测量.
 * 主要生成了3D空间中的3D点特征，以及这些点特征在图像上的投影观测.
******************************************************************/
class Simulator {

public:
  /**
   * @brief Default constructor, will load all configuration variables
   * @param params_ VioManager parameters. Should have already been loaded from cmd.
   */
  /**
   * @brief 默认构造函数，加载所有的配置文件.
   * @param params_ VIO管理参数，可以通过终端加载得到.
  */
  Simulator(VioManagerOptions &params_);

  /**
   * @brief Will get a set of perturbed parameters
   *        这里我们将得到扰动的参数.
   * @param gen_state Random number gen to use
   *        这里是用来生成状态的随机数种子.
   * @param params_ Parameters we will perturb
   *        这里是与扰动相关的参数.
   **/
  static void perturb_parameters(std::mt19937 gen_state, VioManagerOptions &params_);

  /**
   * @brief Returns if we are actively simulating -> 判断我们当前是否在仿真数据.
   * @return True if we still have simulation data
   */
  bool ok() { return is_running; }

  /**
   * @brief Gets the timestamp we have simulated up too
   * @return Timestamp
   */
  double current_timestamp() { return timestamp; }

  /**
   * @brief Get the simulation state at a specified timestep
   * @param desired_time Timestamp we want to get the state at
   * @param imustate State in the MSCKF ordering: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
   * @return True if we have a state
   */
  /*********************************
   * @brief 生成某一时刻下的仿真IMU状态.
   * @param 需要的时间戳.
   * @param 生成的IMU状态序列是：[time(sec), 旋转，位置，速度，陀螺仪，加速度计]
   * @return 生成成功则返回true.
  **********************************/
  bool get_state(double desired_time, Eigen::Matrix<double, 17, 1> &imustate);

  /**
   * @brief Gets the next inertial reading if we have one.
   * @param time_imu Time that this measurement occured at
   * @param wm Angular velocity measurement in the inertial frame
   * @param am Linear velocity in the inertial frame
   * @return True if we have a measurement
   */
  /**
   * @brief 生成下一时刻的IMU读数.
  */
  bool get_next_imu(double &time_imu, Eigen::Vector3d &wm, Eigen::Vector3d &am);

  /**
   * @brief Gets the next camera reading if we have one.
   * @param time_cam Time that this measurement occured at
   * @param camids Camera ids that the corresponding vectors match
   * @param feats Noisy uv measurements and ids for the returned time
   * @return True if we have a measurement
   */
  /**
   * @brief 生成下一时刻的相机读数.
  */
  bool get_next_cam(double &time_cam, std::vector<int> &camids, std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &feats);

  /// Returns the true 3d map of features
  /// 返回3D特征点地图
  std::unordered_map<size_t, Eigen::Vector3d> get_map() { return featmap; }

  /// Returns the true 3d map of features
  std::vector<Eigen::Vector3d> get_map_vec() {
    std::vector<Eigen::Vector3d> feats;
    for (auto const &feat : featmap)
      feats.push_back(feat.second);
    return feats;
  }

  /// Access function to get the true parameters (i.e. calibration and settings)
  /// 获得真实的VIO参数.
  VioManagerOptions get_true_parameters() { return params; }

protected:
  /**
   * @brief Projects the passed map features into the desired camera frame.
   *        将相关的3D电晕投射到某一帧相机上进行观测.
   * @param R_GtoI Orientation of the IMU pose
   * @param p_IinG Position of the IMU pose
   * @param camid Camera id of the camera sensor we want to project into
   * @param feats Our set of 3d features
   * @return True distorted raw image measurements and their ids for the specified camera
   */
  std::vector<std::pair<size_t, Eigen::VectorXf>> project_pointcloud(const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG,
                                                                     int camid, const std::unordered_map<size_t, Eigen::Vector3d> &feats);

  /**
   * @brief Will generate points in the fov of the specified camera
   *        由某一帧的观测生成对应的电晕.
   * @param R_GtoI Orientation of the IMU pose
   * @param p_IinG Position of the IMU pose
   * @param camid Camera id of the camera sensor we want to project into
   * @param[out] feats Map we will append new features to
   * @param numpts Number of points we should generate
   */
  void generate_points(const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG, int camid,
                       std::unordered_map<size_t, Eigen::Vector3d> &feats, int numpts);

  //===================================================================
  // Configuration variables
  //===================================================================

  /// True vio manager params (a copy of the parsed ones)
  VioManagerOptions params;

  //===================================================================
  // State related variables
  //===================================================================

  /// Our loaded trajectory data (timestamp(s), q_GtoI, p_IinG)
  /// 读入的轨迹数据.
  std::vector<Eigen::VectorXd> traj_data;

  /// Our b-spline trajectory
  /// 插值得到的spline数据
  std::shared_ptr<ov_core::BsplineSE3> spline;

  /// Our map of 3d features
  size_t id_map = 0;
  std::unordered_map<size_t, Eigen::Vector3d> featmap;

  /// Mersenne twister PRNG for measurements (IMU)
  /// 用于生成IMU随机数的算法.
  std::mt19937 gen_meas_imu;

  /// Mersenne twister PRNG for measurements (CAMERAS)
  /// 用于生成相机观测的随机数算法.
  std::vector<std::mt19937> gen_meas_cams;

  /// Mersenne twister PRNG for state initialization
  /// 用于生成状态初始化的算法.
  std::mt19937 gen_state_init;

  /// Mersenne twister PRNG for state perturbations
  /// 用于生成扰动状态. 
  std::mt19937 gen_state_perturb;

  /// If our simulation is running
  bool is_running;

  //===================================================================
  // Simulation specific variables
  //===================================================================

  /// Current timestamp of the system
  /// 当前系统的时间戳.
  double timestamp;

  /// Last time we had an IMU reading
  /// 最后一次IMU读数.
  double timestamp_last_imu;

  /// Last time we had an CAMERA reading
  double timestamp_last_cam;

  /// Our running acceleration bias
  /// 运行的加速度计 bias.
  Eigen::Vector3d true_bias_accel = Eigen::Vector3d::Zero();

  /// Our running gyroscope bias
  /// 运行的陀螺仪计 bias.
  Eigen::Vector3d true_bias_gyro = Eigen::Vector3d::Zero();

  // Our history of true biases
  // 真实bias记录.
  bool has_skipped_first_bias = false;
  std::vector<double> hist_true_bias_time;
  std::vector<Eigen::Vector3d> hist_true_bias_accel;
  std::vector<Eigen::Vector3d> hist_true_bias_gyro;
};

} // namespace ov_msckf

#endif // OV_MSCKF_SIMULATOR_H
