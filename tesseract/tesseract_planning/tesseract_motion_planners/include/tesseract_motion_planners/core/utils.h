/**
 * @file utils.h
 * @brief Planner utility functions.
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_MOTION_PLANNERS_UTILS_H
#define TESSERACT_MOTION_PLANNERS_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/types.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_motion_planners/core/waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/core/waypoint.h>

namespace tesseract_motion_planners
{
/**
 * @brief Inerpolate between two transforms return a vector of Eigen::Isometry transforms.
 * @param start The Start Transform
 * @param stop The Stop/End Transform
 * @param steps The number of step
 * @return A vector of Eigen::Isometry with a length = steps + 1
 */
inline tesseract_common::VectorIsometry3d interpolate(const Eigen::Isometry3d& start,
                                                      const Eigen::Isometry3d& stop,
                                                      int steps)
{
  // Required position change
  Eigen::Vector3d delta_translation = (stop.translation() - start.translation());
  Eigen::Vector3d start_pos = start.translation();
  Eigen::Affine3d stop_prime = start.inverse() * stop;
  Eigen::AngleAxisd delta_rotation(stop_prime.rotation());

  // Step size
  Eigen::Vector3d step = delta_translation / steps;

  // Orientation interpolation
  Eigen::Quaterniond start_q(start.rotation());
  Eigen::Quaterniond stop_q(stop.rotation());
  double slerp_ratio = 1.0 / steps;

  tesseract_common::VectorIsometry3d result;
  Eigen::Vector3d trans;
  Eigen::Quaterniond q;
  Eigen::Isometry3d pose;
  result.reserve(static_cast<size_t>(steps) + 1);
  for (unsigned i = 0; i <= static_cast<unsigned>(steps); ++i)
  {
    trans = start_pos + step * i;
    q = start_q.slerp(slerp_ratio * i, stop_q);
    pose = (Eigen::Translation3d(trans) * q);
    result.push_back(pose);
  }
  return result;
}

/**
 * @brief Inerpolate between two Eigen::VectorXd and return a Matrix
 * @param start The Start State
 * @param stop The Stop/End State
 * @param steps The number of step
 * @return A matrix where columns = steps + 1
 */
inline Eigen::MatrixXd interpolate(const Eigen::VectorXd& start, const Eigen::VectorXd& stop, int steps)
{
  assert(start.size() == stop.size());

  Eigen::MatrixXd result(start.size(), steps + 1);

  for (int i = 0; i < start.size(); ++i)
    result.row(i) = Eigen::VectorXd::LinSpaced(steps + 1, start(i), stop(i));

  return result;
}

/**
 * @brief Inerpolate between two waypoints return a vector of waypoints.
 * @param start The Start Waypoint
 * @param stop The Stop/End Waypoint
 * @param steps The number of step
 * @return A vector of waypoints with a length = steps + 1
 */
 inline std::vector<tesseract_motion_planners::Waypoint::Ptr>
 interpolate(const tesseract_motion_planners::Waypoint& start,
            const tesseract_motion_planners::Waypoint& stop,
            int steps)
{
  switch (start.getType())
  {
    case WaypointType::CARTESIAN_WAYPOINT:
    {
      const auto& w1 = static_cast<const CartesianWaypoint&>(start);
      const auto& w2 = static_cast<const CartesianWaypoint&>(stop);
      tesseract_common::VectorIsometry3d eigen_poses = interpolate(w1.getTransform(), w2.getTransform(), steps);

      std::vector<Waypoint::Ptr> result;
      result.reserve(eigen_poses.size());
      for (auto& eigen_pose : eigen_poses)
      {
        CartesianWaypoint::Ptr new_waypoint =
            std::make_shared<tesseract_motion_planners::CartesianWaypoint>(eigen_pose, w1.getParentLinkName());
        new_waypoint->setCoefficients(start.getCoefficients());
        new_waypoint->setIsCritical(start.isCritical());
        result.push_back(new_waypoint);
      }

      return result;
    }
    default:
    {
      CONSOLE_BRIDGE_logError("Interpolator for Waypoint type %d is currently not support!", start.getType());
      return std::vector<Waypoint::Ptr>();
    }
  }
}

/**
 * @brief Converts a waypoint to a cartesian waypoint
 * @param input Input waypoint. May be either a cartesian or joint waypoint
 * @param kin Forward Kinematics used to convert joint waypoint to cartesian
 * @return Cartesian waypoint
 */
inline tesseract_planning::CartesianWaypoint
toCartesianWaypoint(const tesseract_planning::Waypoint& input, const tesseract_kinematics::ForwardKinematics::Ptr& kin)
{
  if (input.getType() == static_cast<int>(tesseract_planning::WaypointType::CARTESIAN_WAYPOINT))
  {
    return *input.cast_const<tesseract_planning::CartesianWaypoint>();
  }
  else if (input.getType() == static_cast<int>(tesseract_planning::WaypointType::JOINT_WAYPOINT))
  {
    Eigen::Isometry3d solution;
    kin->calcFwdKin(solution, *input.cast_const<tesseract_planning::JointWaypoint>());
    return tesseract_planning::CartesianWaypoint(solution);
  }

  return tesseract_planning::CartesianWaypoint();
}

/**
 * @brief Converts a waypoint to a joint waypoint
 * @param input Input waypoint. May be either a cartesian or joint waypoint
 * @param kin Inverse kinematics used to convert a cartesian waypoint to joint
 * @param seed Seed position used for inverse kinematics
 * @return Joint Waypoint
 */
inline tesseract_planning::JointWaypoint toJointWaypoint(const tesseract_planning::Waypoint& input,
                                                         const tesseract_kinematics::InverseKinematics::Ptr& kin,
                                                         const tesseract_planning::JointWaypoint& seed)
{
  if (input.getType() == static_cast<int>(tesseract_planning::WaypointType::JOINT_WAYPOINT))
  {
    return *input.cast_const<tesseract_planning::JointWaypoint>();
  }
  else if (input.getType() == static_cast<int>(tesseract_planning::WaypointType::CARTESIAN_WAYPOINT))
  {
    Eigen::VectorXd solution;
    kin->calcInvKin(solution, *input.cast_const<tesseract_planning::CartesianWaypoint>(), seed);
    return tesseract_planning::JointWaypoint(solution.topRows(seed.size()));
  }
}
}  // namespace tesseract_motion_planners

#endif  // TESSERACT_PLANNING_UTILS_H
