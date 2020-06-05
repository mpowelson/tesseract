/**
 * @file Interpolator_motion_planner.cpp
 * @brief Tesseract Interpolator motion planner
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/utils.h>
#include <tesseract_motion_planners/interpolator/interpolator_motion_planner.h>
#include <tesseract_motion_planners/interpolator/interpolator_motion_planner_status_category.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>

// using namespace tesseract_motion_planners;
using namespace tesseract_planning;

/** @brief Construct a basic planner */
tesseract_motion_planners::InterpolatorMotionPlanner::InterpolatorMotionPlanner(std::string name)
  : MotionPlanner(std::move(name))
  , config_(nullptr)
  , status_category_(std::make_shared<const InterpolatorMotionPlannerStatusCategory>(name_))
{
}

bool tesseract_motion_planners::InterpolatorMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of planner is not implemented yet");
  return false;
}

tesseract_common::StatusCode
tesseract_motion_planners::InterpolatorMotionPlanner::solve(tesseract_motion_planners::PlannerResponse& response,
                                                            tesseract_motion_planners::PostPlanCheckType check_type,
                                                            bool verbose)
{
  tesseract_common::StatusCode config_status = isConfigured();
  if (!config_status)
  {
    response.status = config_status;
    CONSOLE_BRIDGE_logError("Planner %s is not configured", name_.c_str());
    return config_status;
  }

  // If the verbose set the log level to debug.
  if (verbose)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  tesseract_planning::CompositeInstruction flat_instr = config_->instructions.flatten();
  tesseract_planning::CompositeInstruction flat_seed = config_->seed.flatten();

  if (flat_instr.size() != flat_seed.size())
  {
    CONSOLE_BRIDGE_logError(
        "Invalid Planner Input. Instruction size: %s , Seed size: %s", flat_instr.size(), flat_seed.size());
    return tesseract_common::StatusCode(InterpolatorMotionPlannerStatusCategory::ErrorFailedToParseConfig,
                                        status_category_);
  }

  for (std::size_t i = 0; i < flat_instr.size() - 1; i++)
  {
    assert(flat_instr[i].getType() == static_cast<int>(InstructionType::PLAN_INSTRUCTION));
    assert(flat_instr[i + 1].getType() == static_cast<int>(InstructionType::PLAN_INSTRUCTION));

    const auto* plan_instruction_1 = flat_instr[i].cast_const<PlanInstruction>();
    const tesseract_planning::Waypoint& wp_1 = plan_instruction_1->getWaypoint();
    const auto* plan_instruction_2 = flat_instr[i + 1].cast_const<PlanInstruction>();
    const tesseract_planning::Waypoint& wp_2 = plan_instruction_2->getWaypoint();

    if (wp_1.getType() == static_cast<int>(tesseract_planning::WaypointType::CARTESIAN_WAYPOINT) &&
        wp_2.getType() == static_cast<int>(tesseract_planning::WaypointType::CARTESIAN_WAYPOINT))
      int t = 5;

    auto cart_1 = tesseract_motion_planners::toCartesianWaypoint(
        wp_1, config_->tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator"));
    auto cart_2 = tesseract_motion_planners::toCartesianWaypoint(
        wp_2, config_->tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator"));

    // MoveJ
    if (plan_instruction_2->isFreespace())
    {
    }
    // MoveL
    else if (plan_instruction_2->isLinear())
    {
      // Convert the waypoints to cartesian waypoints and interpolate
      tesseract_common::VectorIsometry3d eigen_vec;
      if (wp_1.getType() == static_cast<int>(tesseract_planning::WaypointType::CARTESIAN_WAYPOINT) &&
          wp_2.getType() == static_cast<int>(tesseract_planning::WaypointType::CARTESIAN_WAYPOINT))
      {
        auto cart_1 = wp_1.cast_const<tesseract_planning::CartesianWaypoint>();
        auto cart_2 = wp_2.cast_const<tesseract_planning::CartesianWaypoint>();
        int size = static_cast<int>(flat_seed[i + 1].cast<CompositeInstruction>()->size());
        eigen_vec = interpolate(*cart_1, *cart_2, size);
      }
      else if (wp_1.getType() == static_cast<int>(tesseract_planning::WaypointType::JOINT_WAYPOINT) &&
               wp_2.getType() == static_cast<int>(tesseract_planning::WaypointType::CARTESIAN_WAYPOINT))
      {
        auto cart_1 = tesseract_motion_planners::toCartesianWaypoint(
            wp_1, config_->tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator"));
        auto cart_2 = wp_2.cast_const<tesseract_planning::CartesianWaypoint>();
        int size = static_cast<int>(flat_seed[i + 1].cast<CompositeInstruction>()->size());
        eigen_vec = interpolate(cart_1, *cart_2, size);
      }
      else if (wp_1.getType() == static_cast<int>(tesseract_planning::WaypointType::CARTESIAN_WAYPOINT) &&
               wp_2.getType() == static_cast<int>(tesseract_planning::WaypointType::JOINT_WAYPOINT))
      {
        auto cart_1 = wp_1.cast_const<tesseract_planning::CartesianWaypoint>();
        auto cart_2 = tesseract_motion_planners::toCartesianWaypoint(
            wp_2, config_->tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator"));
        int size = static_cast<int>(flat_seed[i + 1].cast<CompositeInstruction>()->size());
        eigen_vec = interpolate(*cart_1, cart_2, size);
      }
      else if (wp_1.getType() == static_cast<int>(tesseract_planning::WaypointType::JOINT_WAYPOINT) &&
               wp_2.getType() == static_cast<int>(tesseract_planning::WaypointType::JOINT_WAYPOINT))
      {
        auto cart_1 = tesseract_motion_planners::toCartesianWaypoint(
            wp_1, config_->tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator"));
        auto cart_2 = tesseract_motion_planners::toCartesianWaypoint(
            wp_2, config_->tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator"));
        int size = static_cast<int>(flat_seed[i + 1].cast<CompositeInstruction>()->size());
        eigen_vec = interpolate(cart_1, cart_2, size);
      }
      else
      {
        CONSOLE_BRIDGE_logWarn("Unsupported waypoint type");
      }

      // Write the results into the seed
      for (std::size_t j = 0; j < eigen_vec.size(); j++)
      {
        tesseract_planners::CartesianWaypoint move_wp(eigen_vec[j]);
        tesseract_planning::MoveInstruction move_instruction(move_wp, MoveInstructionType::LINEAR);
        flat_seed[i + 1].cast<CompositeInstruction>()->at(j) = move_instruction;
      }


    }
    else
    {
      CONSOLE_BRIDGE_logWarn("Unsupported Plan Type. Must be linear or freespace");
    }
  }

  //  Interpolator::base::PlannerStatus status;
  //  if (!config_->optimize)
  //  {
  //    // Solve problem. Results are stored in the response
  //    // Disabling hybridization because there is a bug which will return a trajectory that starts at the end state
  //    // and finishes at the end state.
  //    status = parallel_plan_->solve(config_->planning_time, 1, static_cast<unsigned>(config_->max_solutions), false);
  //  }
  //  else
  //  {
  //    Interpolator::time::point end = Interpolator::time::now() + Interpolator::time::seconds(config_->planning_time);
  //    const Interpolator::base::ProblemDefinitionPtr& pdef = config_->simple_setup->getProblemDefinition();
  //    while (Interpolator::time::now() < end)
  //    {
  //      // Solve problem. Results are stored in the response
  //      // Disabling hybridization because there is a bug which will return a trajectory that starts at the end state
  //      // and finishes at the end state.
  //      Interpolator::base::PlannerStatus localResult =
  //          parallel_plan_->solve(std::max(Interpolator::time::seconds(end - Interpolator::time::now()), 0.0),
  //                                1,
  //                                static_cast<unsigned>(config_->max_solutions),
  //                                false);
  //      if (localResult)
  //      {
  //        if (status != Interpolator::base::PlannerStatus::EXACT_SOLUTION)
  //          status = localResult;

  //        if (!pdef->hasOptimizationObjective())
  //        {
  //          CONSOLE_BRIDGE_logDebug("Terminating early since there is no optimization objective specified");
  //          break;
  //        }

  //        Interpolator::base::Cost obj_cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective());
  //        CONSOLE_BRIDGE_logDebug("Motion Objective Cost: %f", obj_cost.value());

  //        if (pdef->getOptimizationObjective()->isSatisfied(obj_cost))
  //        {
  //          CONSOLE_BRIDGE_logDebug("Terminating early since solution path satisfies the optimization objective");
  //          break;
  //        }

  //        if (pdef->getSolutionCount() >= static_cast<std::size_t>(config_->max_solutions))
  //        {
  //          CONSOLE_BRIDGE_logDebug("Terminating early since %u solutions were generated", config_->max_solutions);
  //          break;
  //        }
  //      }
  //    }
  //  }

  //  if (status != Interpolator::base::PlannerStatus::EXACT_SOLUTION)
  //  {
  //    response.status =
  //        tesseract_common::StatusCode(InterpolatorMotionPlannerStatusCategory::ErrorFailedToFindValidSolution,
  //        status_category_);
  //    return response.status;
  //  }

  //  if (config_->simplify)
  //  {
  //    config_->simple_setup->simplifySolution();
  //  }
  //  else
  //  {
  //    // Interpolate the path if it shouldn't be simplified and there are currently fewer states than requested
  //    auto num_output_states = static_cast<unsigned>(config_->n_output_states);
  //    if (config_->simple_setup->getSolutionPath().getStateCount() < num_output_states)
  //    {
  //      config_->simple_setup->getSolutionPath().interpolate(num_output_states);
  //    }
  //    else
  //    {
  //      // Now try to simplify the trajectory to get it under the requested number of output states
  //      // The interpolate function only executes if the current number of states is less than the requested
  //      config_->simple_setup->simplifySolution();
  //      if (config_->simple_setup->getSolutionPath().getStateCount() < num_output_states)
  //        config_->simple_setup->getSolutionPath().interpolate(num_output_states);
  //    }
  //  }

  //  tesseract_common::TrajArray traj = config_->getTrajectory();

  //  assert(config_->simple_setup->getProblemDefinition()->getStartStateCount() == 1);
  //  assert(config_->extractor(config_->simple_setup->getProblemDefinition()->getStartState(0))
  //             .transpose()
  //             .isApprox(traj.row(0), 1e-5));
  //  assert(
  //      config_
  //          ->extractor(config_->simple_setup->getProblemDefinition()->getGoal()->as<Interpolator::base::GoalState>()->getState())
  //          .transpose()
  //          .isApprox(traj.bottomRows(1), 1e-5));

  //  // Check and report collisions
  //  continuous_contact_manager_->setContactDistanceThreshold(0.0);

  //  bool valid = true;
  //  {
  //    auto env = config_->tesseract->getEnvironmentConst();
  //    auto adj_map = std::make_shared<tesseract_environment::AdjacencyMap>(
  //        env->getSceneGraph(), kin_->getActiveLinkNames(), env->getCurrentState()->link_transforms);
  //    auto discrete_contact_manager = env->getDiscreteContactManager();
  //    discrete_contact_manager->setActiveCollisionObjects(adj_map->getActiveLinkNames());
  //    discrete_contact_manager->setContactDistanceThreshold(0.0);

  //    tesseract_environment::StateSolver::Ptr state_solver = env->getStateSolver();

  //    validator_ = std::make_shared<TrajectoryValidator>(
  //        continuous_contact_manager_, discrete_contact_manager, config_->longest_valid_segment_length, verbose);
  //    valid = validator_->trajectoryValid(traj, check_type, *state_solver, kin_->getJointNames());
  //  }

  //  // Set the contact distance back to original incase solve was called again.
  //  continuous_contact_manager_->setContactDistanceThreshold(config_->collision_safety_margin);

  //  // Send response
  //  response.joint_trajectory.trajectory = traj;
  //  response.joint_trajectory.joint_names = kin_->getJointNames();
  //  if (!valid)
  //  {
  //    response.status =
  //    tesseract_common::StatusCode(InterpolatorMotionPlannerStatusCategory::ErrorFoundValidSolutionInCollision,
  //                                                   status_category_);
  //  }
  //  else
  //  {
  //    response.status = tesseract_common::StatusCode(InterpolatorMotionPlannerStatusCategory::SolutionFound,
  //    status_category_); CONSOLE_BRIDGE_logInform("%s, final trajectory is collision free", name_.c_str());
  //  }

  //  return response.status;
}

void tesseract_motion_planners::InterpolatorMotionPlanner::clear()
{
  //  request_ = PlannerRequest();
  config_ = nullptr;
}

tesseract_common::StatusCode tesseract_motion_planners::InterpolatorMotionPlanner::isConfigured() const
{
  if (config_ == nullptr)
    return tesseract_common::StatusCode(InterpolatorMotionPlannerStatusCategory::ErrorIsNotConfigured,
                                        status_category_);

  return tesseract_common::StatusCode(InterpolatorMotionPlannerStatusCategory::IsConfigured, status_category_);
}

bool tesseract_motion_planners::InterpolatorMotionPlanner::setConfiguration(
    tesseract_motion_planners::InterpolatorPlannerConfig::Ptr config)
{
  // Reset state
  clear();

  if (!config)
    return false;

  config_ = std::move(config);
  return true;
}
