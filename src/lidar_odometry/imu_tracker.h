/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
#define CARTOGRAPHER_MAPPING_IMU_TRACKER_H_

#include "Eigen/Geometry"
#include "Eigen/Eigen"
// Keeps track of the orientation using angular velocities and linear
// accelerations from an IMU. Because averaged linear acceleration (assuming
// slow movement) is a direct measurement of gravity, roll/pitch does not drift,
// though yaw does.
class ImuTracker
{
public:
  ImuTracker(const double imu_gravity_time_constant, const double time);

  // Advances to the given 'time' and updates the orientation to reflect this.
  void Advance(const double time);

  // Updates from an IMU reading (in the IMU frame).
  void AddImuLinearAccelerationObservation(
      const Eigen::Vector3d& imu_linear_acceleration);
  void
  AddImuAngularVelocityObservation(const Eigen::Vector3d& imu_angular_velocity);

  // Query the current time.
  double time() const { return time_; }

  // Query the current orientation estimate.
  Eigen::Quaterniond orientation() const { return orientation_; }

  template <typename T>
  Eigen::Quaternion<T>
  AngleAxisVectorToRotationQuaternion(const Eigen::Matrix<T, 3, 1>& angle_axis)
  {
    T scale = T(0.5);
    T w = T(1.);
    constexpr double kCutoffAngle = 1e-8; // We linearize below this angle.
    if (angle_axis.squaredNorm() > kCutoffAngle)
    {
      const T norm = angle_axis.norm();
      scale = sin(norm / 2.) / norm;
      w = cos(norm / 2.);
    }
    const Eigen::Matrix<T, 3, 1> quaternion_xyz = scale * angle_axis;
    return Eigen::Quaternion<T>(w, quaternion_xyz.x(), quaternion_xyz.y(),
                                quaternion_xyz.z());
  }

private:
  const double imu_gravity_time_constant_;
  double time_;
  double last_linear_acceleration_time_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d gravity_vector_;
  Eigen::Vector3d imu_angular_velocity_;
};
#endif // CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
