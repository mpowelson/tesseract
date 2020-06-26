/**
 * @file descartes_robot_positioner_sampler.h
 * @brief Tesseract Descartes robot on a positioner sampler
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_ROBOT_POSITIONER_SAMPLER_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_ROBOT_POSITIONER_SAMPLER_H

#include <tesseract/tesseract.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/interface/position_sampler.h>
#include <descartes_light/interface/collision_interface.h>
#include <descartes_light/utils.h>
#include <Eigen/Dense>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_environment/core/types.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_motion_planners/descartes/types.h>

namespace tesseract_planning
{
template <typename FloatType>
class DescartesRobotPositionerSampler : public descartes_light::PositionSampler<FloatType>
{
public:
  /**
   * @brief This is a descartes sampler for a positioner system + robot.
   *
   * The tip link name of the positioner kinematics should match the base link name of the robot kinematics
   *
   * @param target_pose The target pose to sample
   * @param target_pose_sampler The target pose sampler function to be used
   * @param positioner_kinematics The forward kinematics object for the positioner system the robot is attached to
   * @param robot_kinematics The robot inverse kinematics object
   * @param collision The collision interface
   * @param curret_state The currect state of the system
   * @param positioner_sample_resolution The positioner system sampling resolution
   * @param robot_tcp The robot tcp to be used.
   * @param robot_reach The reach of the robot. Used to filter rail samples.
   * @param allow_collision If true and no valid solution was found it will return the best of the worst
   * @param is_valid This is a user defined function to filter out solution
   */
  DescartesRobotPositionerSampler(const Eigen::Isometry3d& target_pose,
                                  PoseSamplerFn target_pose_sampler,
                                  tesseract_kinematics::ForwardKinematics::ConstPtr positioner_kinematics,
                                  tesseract_kinematics::InverseKinematics::ConstPtr robot_kinematics,
                                  typename descartes_light::CollisionInterface<FloatType>::Ptr collision,
                                  const tesseract_environment::EnvState::ConstPtr& current_state,
                                  const Eigen::Ref<const Eigen::VectorXd>& positioner_sample_resolution,
                                  const Eigen::Isometry3d& robot_tcp,
                                  double robot_reach,
                                  bool allow_collision,
                                  DescartesIsValidFn<FloatType> is_valid);

  bool sample(std::vector<FloatType>& solution_set) override;

private:
  Eigen::Isometry3d target_pose_;                                           /**< @brief The target pose to sample */
  PoseSamplerFn target_pose_sampler_;                                       /**< @brief Target pose sampler function */
  tesseract_kinematics::ForwardKinematics::ConstPtr positioner_kinematics_; /**< @brief The robot base positioner
                                                                               kinematics */
  tesseract_kinematics::InverseKinematics::ConstPtr robot_kinematics_;      /**< @brief The robot inverse kinematics */
  typename descartes_light::CollisionInterface<FloatType>::Ptr collision_;  /**< @brief The collision interface */
  Eigen::Isometry3d world_to_positioner_base_;   /**< @brief The transform from world to the base of the base positioner
                                                    kinematic */
  Eigen::MatrixX2d positioner_limits_;           /**< @brief The joint limits for the base positioner kinematics */
  Eigen::VectorXd positioner_sample_resolution_; /**< @brief The joint sampling resolution for the base positioner
                                                    kinematics */
  Eigen::Isometry3d robot_tcp_;                  /**< @brief The robot tool center point */
  double robot_reach_;                           /**< @brief The reach of the robot used to quickly filter samples */
  bool allow_collision_;    /**< @brief If true and no valid solution was found it will return the best of the worst */
  int dof_;                 /**< @brief The number of joints in the system base positioner + robot */
  Eigen::VectorXd ik_seed_; /**< @brief The seed for inverse kinematics which is zeros */
  DescartesIsValidFn<FloatType> is_valid_; /**< @brief This is a user defined function to filter out solution */

  /**
   * @brief Check if a solution is passes collision test
   * @param vertex The joint solution to check
   * @return True if collision test passed, otherwise false
   */
  bool isCollisionFree(const FloatType* vertex);

  /**
   * @brief This will return the best of the worst solution
   * @param solution_set The solution to populate
   * @param target_poses The target pose to sample
   * @param dof_range The sampling ranges for the individual joints
   * @return True if a solution was found, otherwise false
   */
  bool getBestSolution(std::vector<FloatType>& solution_set,
                       const tesseract_common::VectorIsometry3d& target_poses,
                       const std::vector<Eigen::VectorXd>& dof_range);

  /**
   * @brief Perform a nested IK solve, given that the number joints being sampled can vary
   * @param solution_set The joint solution to be returned
   * @param loop_level The current loop level
   * @param dof_range The sampling ranges for the individual joints
   * @param target_pose The target pose to sample
   * @param sample_pose The joint sampled pose
   * @param get_best_solution Enable if best solution should be returned
   * @param distance The current best distance
   */
  void nested_ik(std::vector<FloatType>& solution_set,
                 int loop_level,
                 const std::vector<Eigen::VectorXd>& dof_range,
                 const Eigen::Isometry3d& target_pose,
                 Eigen::Ref<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> sample_pose,
                 bool get_best_solution,
                 double& distance);

  /**
   * @brief Solve inverse kinematics for the target pose
   * @param solution_set The soltuion to return
   * @param target_pose The target pose to solve inverse kinematics
   * @param positoner_pose the positioner pose
   * @param get_best_solution Enable if best solution should be returned
   * @param distance The current best distance
   * @return True if a solution was found, otherwise false
   */
  bool ikAt(std::vector<FloatType>& solution_set,
            const Eigen::Isometry3d& target_pose,
            const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& positioner_pose,
            bool get_best_solution,
            double& distance);
};

using DescartesRobotPositionerSamplerF = DescartesRobotPositionerSampler<float>;
using DescartesRobotPositionerSamplerD = DescartesRobotPositionerSampler<double>;

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_ROBOT_POSITIONER_SAMPLER_H
