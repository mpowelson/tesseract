/**
 * @file Interpolator_planner_config.h
 * @brief Tesseract Interpolator planner config.
 *
 * @author Levi Armstrong
 * @date January 22, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in cInterpolatoriance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_MOTION_PLANNERS_INTERPOLATOR_PLANNER_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_INTERPOLATOR_PLANNER_CONFIG_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_command_language/composite_instruction.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>

namespace tesseract_motion_planners
{
/** @brief The InterpolatorPlannerConfig struct */
struct InterpolatorPlannerConfig
{
  using Ptr = std::shared_ptr<InterpolatorPlannerConfig>;
  using ConstPtr = std::shared_ptr<const InterpolatorPlannerConfig>;

  //  explicit InterpolatorPlannerConfig(tesseract::Tesseract::ConstPtr tesseract, std::string manipulator);

  //  explicit InterpolatorPlannerConfig(tesseract::Tesseract::ConstPtr tesseract,
  //                             std::string manipulator,
  //                             std::vector<InterpolatorPlannerConfigurator::ConstPtr> planners);

  //  explicit InterpolatorPlannerConfig(tesseract::Tesseract::ConstPtr tesseract,
  //                             std::string manipulator,
  //                             std::vector<InterpolatorPlannerConfigurator::ConstPtr> planners,
  //                             Interpolator::geometric::SimpleSetupPtr simple_setup);

  virtual ~InterpolatorPlannerConfig() = default;
  InterpolatorPlannerConfig(const InterpolatorPlannerConfig&) = default;
  InterpolatorPlannerConfig& operator=(const InterpolatorPlannerConfig&) = default;
  InterpolatorPlannerConfig(InterpolatorPlannerConfig&&) noexcept = default;
  InterpolatorPlannerConfig& operator=(InterpolatorPlannerConfig&&) noexcept = default;

  /**
   * @brief Generates the Interpolator problem and saves the result internally
   * @return True on success, false on failure
   */
  virtual bool generate();

  /**
   * @brief Convert the path stored in simple_setup to tesseract trajectory
   * This is required because the motion planner is not aware of the state space type.
   * @return Tesseract Trajectory
   */
  tesseract_common::TrajArray getTrajectory() const;

  /** @brief Tesseract object. ***REQUIRED*** */
  tesseract::Tesseract::ConstPtr tesseract;

  /**
   * @brief The program instruction
   * This must contain a minimum of two move instruction the first move instruction is the start state
   */
  tesseract_planning::CompositeInstruction instructions;

  /**
   * @brief This should be a one to one match with the instructions where the PlanInstruction is replaced with a
   * composite instruction of MoveInstructions.
   */
  tesseract_planning::CompositeInstruction seed;

  // TODO: This might be able to go away

  /** @brief If true, collision checking will be enabled. Default: true*/
  bool collision_check = true;
  /** @brief If true, use continuous collision checking */
  bool collision_continuous = true;
  /** @brief Max distance over which collisions are checked */
  double collision_safety_margin = 0.025;
};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_Interpolator_PLANNER_CONFIG_H
