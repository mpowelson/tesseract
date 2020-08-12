﻿/**
 * @file default_freespace_processes.h
 * @brief Default processes for the freespace process manager
 *
 * @author Matthew Powelson
 * @date July 15. 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_PROCESS_MANAGER_DEFAULT_FREESPACE_PROCESSES_H
#define TESSERACT_PROCESS_MANAGER_DEFAULT_FREESPACE_PROCESSES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_generators/motion_planner_process_generator.h>
#include <tesseract_process_managers/process_generators/continuous_contact_check_process_generator.h>
#include <tesseract_process_managers/process_generators/iterative_spline_parameterization_process_generator.h>
#include <tesseract_process_managers/taskflow_generators/sequential_taskflow.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_default_plan_profile.h>

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>

namespace tesseract_planning
{
inline SequentialProcesses defaultFreespaceProcesses()
{
  SequentialProcesses sp;

  // Setup Interpolator
  auto interpolator = std::make_shared<SimpleMotionPlanner>("INTERPOLATOR");
  interpolator->plan_profiles["FREESPACE"] = std::make_shared<SimplePlannerDefaultPlanProfile>();
  auto interpolator_generator = std::make_unique<MotionPlannerProcessGenerator>(interpolator);
  sp.emplace_back(std::move(interpolator_generator), SequentialTaskType::CONDITIONAL_EXIT_ON_FAILURE);

  // Setup OMPL
  auto ompl_planner = std::make_shared<OMPLMotionPlanner>();
  ompl_planner->problem_generator = &DefaultOMPLProblemGenerator;
  ompl_planner->plan_profiles["FREESPACE"] = std::make_shared<OMPLDefaultPlanProfile>();
  auto ompl_generator = std::make_unique<MotionPlannerProcessGenerator>(ompl_planner);
  sp.emplace_back(std::move(ompl_generator), SequentialTaskType::CONDITIONAL_EXIT_ON_FAILURE);

  // Setup TrajOpt
  auto trajopt_planner = std::make_shared<TrajOptMotionPlanner>();
  trajopt_planner->problem_generator = &DefaultTrajoptProblemGenerator;
  trajopt_planner->plan_profiles["FREESPACE"] = std::make_shared<TrajOptDefaultPlanProfile>();
  trajopt_planner->composite_profiles["FREESPACE"] = std::make_shared<TrajOptDefaultCompositeProfile>();
  auto trajopt_generator = std::make_unique<MotionPlannerProcessGenerator>(trajopt_planner);
  sp.emplace_back(std::move(trajopt_generator), SequentialTaskType::CONDITIONAL_EXIT_ON_FAILURE);

  // Add Final Continuous Contact Check of trajectory
  auto contact_check_generator = std::make_unique<ContinuousContactCheckProcessGenerator>();
  sp.emplace_back(std::move(contact_check_generator), SequentialTaskType::CONDITIONAL_EXIT_ON_FAILURE);

  // Time parameterization trajectory
  auto time_parameterization_generator = std::make_unique<IterativeSplineParameterizationProcessGenerator>();
  sp.emplace_back(std::move(time_parameterization_generator), SequentialTaskType::CONDITIONAL_EXIT_ON_FAILURE);

  return sp;
}

}  // namespace tesseract_planning

#endif
