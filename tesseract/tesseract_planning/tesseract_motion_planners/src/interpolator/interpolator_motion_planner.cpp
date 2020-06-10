/**
 * @file Interpolator_motion_planner.cpp
 * @brief Tesseract Interpolator motion planner
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 *
 * @date June 10, 2020
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/utils.h>
#include <tesseract_motion_planners/interpolator/interpolator_motion_planner.h>
#include <tesseract_motion_planners/interpolator/interpolator_motion_planner_status_category.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/core/trajectory_validator.h>

#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>

using namespace tesseract_planning;

// void FlattenHelper(std::vector<std::reference_wrapper<Instruction>>& flattened, CompositeInstruction& composite);

void FlattenHelper(std::vector<std::reference_wrapper<Instruction>>& flattened, CompositeInstruction& composite)
{
  for (auto& i : composite)
  {
    if (i.isComposite())
      FlattenHelper(flattened, *(i.cast<CompositeInstruction>()));
    else
      flattened.push_back(i);
  }
}

std::vector<std::reference_wrapper<Instruction>> Flatten(CompositeInstruction& instruction)
{
  std::vector<std::reference_wrapper<Instruction>> flattened;
  FlattenHelper(flattened, instruction);
  return flattened;
}

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
                                                            PostPlanCheckType check_type,
                                                            bool verbose)
{
  // Check that the planner is configured
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

  // this planner interpolates b/n each instruction, so flatten them for convenience
  const std::vector<std::reference_wrapper<Instruction>> flat_instr = Flatten(config_->instructions);
  std::vector<std::reference_wrapper<Instruction>> flat_seed = Flatten(config_->seed);
  std::vector<std::reference_wrapper<Instruction>> flat_seed_copy = flat_seed;

  if (flat_instr.size() != flat_seed_copy.size())
  {
    CONSOLE_BRIDGE_logError(
        "Invalid Planner Input. Instruction size: %s , Seed size: %s", flat_instr.size(), flat_seed_copy.size());
    return tesseract_common::StatusCode(InterpolatorMotionPlannerStatusCategory::ErrorFailedToParseConfig,
                                        status_category_);
  }

  // Loop over all instructions and interpolate
  for (std::size_t i = 0; i < flat_instr.size() - 1; i++)
  {
    // Eventually we might want to be able to skip other instructions, but for now just error
    assert(flat_instr[i].get().getType() == static_cast<int>(InstructionType::PLAN_INSTRUCTION));
    assert(flat_instr[i + 1].get().getType() == static_cast<int>(InstructionType::PLAN_INSTRUCTION));

    // Convert to plan instructions
    const auto* plan_instruction_1 = flat_instr[i].get().cast_const<PlanInstruction>();
    const tesseract_planning::Waypoint& wp_1 = plan_instruction_1->getWaypoint();
    const auto* plan_instruction_2 = flat_instr[i + 1].get().cast_const<PlanInstruction>();
    const tesseract_planning::Waypoint& wp_2 = plan_instruction_2->getWaypoint();

    // MoveJ
    if (plan_instruction_2->isFreespace())
    {
      CONSOLE_BRIDGE_logWarn("Freespace moves are not yet supported");
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
        int size = static_cast<int>(flat_seed_copy[i + 1].get().cast<CompositeInstruction>()->size());
        eigen_vec = interpolate(*cart_1, *cart_2, size);
      }
      else if (wp_1.getType() == static_cast<int>(tesseract_planning::WaypointType::JOINT_WAYPOINT) &&
               wp_2.getType() == static_cast<int>(tesseract_planning::WaypointType::CARTESIAN_WAYPOINT))
      {
        auto cart_1 = tesseract_motion_planners::toCartesianWaypoint(
            wp_1, config_->tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator"));
        auto cart_2 = wp_2.cast_const<tesseract_planning::CartesianWaypoint>();
        int size = static_cast<int>(flat_seed_copy[i + 1].get().cast<CompositeInstruction>()->size());
        eigen_vec = interpolate(cart_1, *cart_2, size);
      }
      else if (wp_1.getType() == static_cast<int>(tesseract_planning::WaypointType::CARTESIAN_WAYPOINT) &&
               wp_2.getType() == static_cast<int>(tesseract_planning::WaypointType::JOINT_WAYPOINT))
      {
        auto cart_1 = wp_1.cast_const<tesseract_planning::CartesianWaypoint>();
        auto cart_2 = tesseract_motion_planners::toCartesianWaypoint(
            wp_2, config_->tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator"));
        int size = static_cast<int>(flat_seed_copy[i + 1].get().cast<CompositeInstruction>()->size());
        eigen_vec = interpolate(*cart_1, cart_2, size);
      }
      else if (wp_1.getType() == static_cast<int>(tesseract_planning::WaypointType::JOINT_WAYPOINT) &&
               wp_2.getType() == static_cast<int>(tesseract_planning::WaypointType::JOINT_WAYPOINT))
      {
        auto cart_1 = tesseract_motion_planners::toCartesianWaypoint(
            wp_1, config_->tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator"));
        auto cart_2 = tesseract_motion_planners::toCartesianWaypoint(
            wp_2, config_->tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator"));
        int size = static_cast<int>(flat_seed_copy[i + 1].get().cast<CompositeInstruction>()->size());
        eigen_vec = interpolate(cart_1, cart_2, size);
      }
      else
      {
        CONSOLE_BRIDGE_logWarn("Unsupported waypoint type");
      }

      // Write the results into the seed
      for (std::size_t j = 0; j < eigen_vec.size(); j++)
      {
        tesseract_planning::CartesianWaypoint move_wp(eigen_vec[j]);
        tesseract_planning::MoveInstruction move_instruction(move_wp, MoveInstructionType::LINEAR);
        flat_seed_copy[i + 1].get().cast<CompositeInstruction>()->at(j) = move_instruction;
      }
    }
    // Not MoveJ or MoveL
    else
    {
      CONSOLE_BRIDGE_logWarn("Unsupported Plan Type. Must be linear or freespace");
    }
  }

  bool valid = true;
  // Check if the results are valid
  if (config_->collision_check)
  {
    std::vector<tesseract_collision::ContactResultMap> collisions;
    tesseract_environment::StateSolver::Ptr state_solver = config_->tesseract->getEnvironmentConst()->getStateSolver();
    tesseract_collision::ContinuousContactManager::Ptr continuous_manager =
        config_->tesseract->getEnvironmentConst()->getContinuousContactManager();
    tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
        config_->tesseract->getEnvironmentConst()->getSceneGraph(),
        config_->tesseract->getEnvironmentConst()->getActiveLinkNames(),
        config_->tesseract->getEnvironmentConst()->getCurrentState()->link_transforms);

    validator_ =
        std::make_shared<TrajectoryValidator>(config_->tesseract->getEnvironmentConst()->getContinuousContactManager(),
                                              config_->tesseract->getEnvironmentConst()->getDiscreteContactManager(),
                                              config_->longest_valid_segment_length_,
                                              verbose);

    // Convert to TrajArray
    tesseract_common::TrajArray traj(flat_seed_copy.size(),
                                     flat_seed_copy[0].get().cast_const<MoveInstruction>()->getPosition().size());
    for (std::size_t index = 0; index < flat_seed_copy.size(); index++)
    {
      traj.row(static_cast<Eigen::Index>(index)) =
          flat_seed_copy[index].get().cast_const<MoveInstruction>()->getPosition().transpose();
    }

    valid = validator_->trajectoryValid(
        traj,
        check_type,
        *state_solver,
        config_->tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator")->getJointNames());
  }

  // If valid, store the results and return true
  if (!valid)
  {
    response.status = tesseract_common::StatusCode(
        InterpolatorMotionPlannerStatusCategory::ErrorFoundValidSolutionInCollision, status_category_);
  }
  else
  {
    response.status =
        tesseract_common::StatusCode(InterpolatorMotionPlannerStatusCategory::SolutionFound, status_category_);
    CONSOLE_BRIDGE_logInform("Final trajectory is collision free");
    // Copy results into the original seed
    for (std::size_t index = 0; index < flat_seed.size(); index++)
      flat_seed[index] = flat_seed_copy[index];
  }

  return response.status;
}

void tesseract_motion_planners::InterpolatorMotionPlanner::clear() { config_ = nullptr; }

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
