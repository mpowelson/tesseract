/**
 * @file Interpolator_motion_planner.h
 * @brief Tesseract Interpolator motion planner.
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
#ifndef TESSERACT_MOTION_PLANNERS_INTERPOLATOR_MOTION_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_INTERPOLATOR_MOTION_PLANNER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <utility>
#include <type_traits>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/core/waypoint.h>
#include <tesseract_motion_planners/interpolator/config/interpolator_planner_config.h>
#include <tesseract_motion_planners/interpolator/interpolator_motion_planner_status_category.h>

namespace tesseract_planning
{
/**
 * @brief This planner is intended to provide an easy to use interface
 */
class InterpolatorMotionPlanner : public MotionPlanner
{
public:
  /** @brief Construct a planner */
  InterpolatorMotionPlanner(std::string name = "Interpolator");

  /**
   * @brief Set the configuration for the planner
   *
   * This must be called prior to calling solve.
   *
   * @param config The planners configuration
   * @return True if successful otherwise false
   */
  bool setConfiguration(InterpolatorPlannerConfig::Ptr config);

  /**
   * @brief Sets up the Interpolator problem then solves. It is intended to simplify setting up
   * and solving freespace motion problems.
   * @param response The results of Interpolator.
   * @param check_type The type of validation check to be performed on the planned trajectory
   * @param verbose Flag for printing more detailed planning information
   * @return true if valid solution was found
   */
  tesseract_common::StatusCode solve(PlannerResponse& response,
                                     PostPlanCheckType check_type = PostPlanCheckType::DISCRETE_CONTINUOUS_COLLISION,
                                     bool verbose = false) override;

  bool terminate() override;

  void clear() override;

  /**
   * @brief checks whether the planner is properly configure for solving a motion plan
   * @return True when it is configured correctly, false otherwise
   */
  tesseract_common::StatusCode isConfigured() const override;

protected:
  PostPlanCheckType check_type_;

  /** @brief The Interpolator planner planner */
  typename InterpolatorPlannerConfig::Ptr config_;

  /** @brief The planners status codes */
  std::shared_ptr<const InterpolatorMotionPlannerStatusCategory> status_category_;
};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_Interpolator_MOTION_PLANNER_H
