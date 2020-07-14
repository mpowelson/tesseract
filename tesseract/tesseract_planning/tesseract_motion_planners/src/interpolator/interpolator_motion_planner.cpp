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

/** @brief Construct a basic planner */
InterpolatorMotionPlanner::InterpolatorMotionPlanner(std::string name)
  : MotionPlanner(std::move(name))
  , status_category_(std::make_shared<const InterpolatorMotionPlannerStatusCategory>(name_))
{
}

bool InterpolatorMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of planner is not implemented yet");
  return false;
}

tesseract_common::StatusCode
InterpolatorMotionPlanner::solve(const PlannerRequest& request,
                                 PlannerResponse& response,
                                 bool verbose) const
{
  // Check the planner input
  if (!checkUserInput(request))
  {
    response.status = tesseract_common::StatusCode(InterpolatorMotionPlannerStatusCategory::InvalidInput, status_category_);
    CONSOLE_BRIDGE_logError("Planner %s is not configured", name_.c_str());
    return response.status;
  }

  // If the verbose set the log level to debug.
  if (verbose)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // Setup (may go in problem generator)``
  tesseract_kinematics::InverseKinematics::Ptr inv_kin;
  tesseract_kinematics::ForwardKinematics::Ptr fwd_kin = request.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(request.manipulator);
  if (request.manipulator_ik_solver.empty())
    inv_kin = request.tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(request.manipulator);
  else
    inv_kin = request.tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(
        request.manipulator, request.manipulator_ik_solver);
  if (!fwd_kin)
  {
    response.status = tesseract_common::StatusCode(InterpolatorMotionPlannerStatusCategory::InvalidInput, status_category_);
    CONSOLE_BRIDGE_logError("No Forward Kinematics solver found");
    return response.status;
  }
  if (!inv_kin)
  {
    response.status = tesseract_common::StatusCode(InterpolatorMotionPlannerStatusCategory::InvalidInput, status_category_);
    CONSOLE_BRIDGE_logError("No Inverse Kinematics solver found");
    return response.status;
  }


  // Begin planning


  // This planner interpolates b/n each instruction, so flatten them for convenience
  const std::vector<std::reference_wrapper<const Instruction>> flat_instr = Flatten(request.instructions);
  response.results = request.seed;
  std::vector<std::reference_wrapper<Instruction>> flat_results = FlattenToPattern(response.results, request.instructions);




  //TODO: make interpolateComposite helper. Then call that recursively (to handle the start waypoint below)

  int cartesian_segments = 10; // TODO Replace with size of seed
  int freespace_segments = 10;


  CompositeInstruction seed;
  Eigen::VectorXd current_jv = request.env_state->getJointValues(fwd_kin->getJointNames());
  Eigen::Isometry3d world_to_base = request.env_state->link_transforms.at(fwd_kin->getBaseLinkName());

  Waypoint start_waypoint = NullWaypoint();
  if (request.instructions.hasStartWaypoint())
  {
    start_waypoint = request.instructions.getStartWaypoint();
  }
  else
  {
    JointWaypoint temp(current_jv);
    temp.joint_names = fwd_kin->getJointNames();
    start_waypoint = temp;
  }

  bool found_plan_instruction = false;
  for (const auto& instruction : request.instructions)
  {
    if (instruction.isPlan())
    {
      const auto* plan_instruction = instruction.cast_const<PlanInstruction>();
      if (plan_instruction->isLinear())
      {
        CompositeInstruction composite;

        bool is_cwp1 = isCartesianWaypoint(start_waypoint.getType());
        bool is_jwp1 = isJointWaypoint(start_waypoint.getType());
        bool is_cwp2 = isCartesianWaypoint(plan_instruction->getWaypoint().getType());
        bool is_jwp2 = isJointWaypoint(plan_instruction->getWaypoint().getType());
        if (!found_plan_instruction)
        {
          tesseract_planning::MoveInstruction move_instruction(start_waypoint, MoveInstructionType::LINEAR);

          if (is_jwp1)
            move_instruction.setPosition(*(start_waypoint.cast_const<JointWaypoint>()));
          else
            move_instruction.setPosition(current_jv);

          move_instruction.setTCP(plan_instruction->getTCP());
          move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
          move_instruction.setDescription(plan_instruction->getDescription());
          composite.push_back(move_instruction);
        }

        assert(is_cwp1 || is_jwp1);
        assert(is_cwp2 || is_jwp2);

        if (is_cwp1 && is_cwp2)
        {
          // If both are cartesian it will cartesian interpolate and use the current state as the seed.
          const auto* pre_cwp = start_waypoint.cast_const<CartesianWaypoint>();
          const auto* cur_cwp = plan_instruction->getWaypoint().cast_const<CartesianWaypoint>();

          tesseract_common::VectorIsometry3d poses = interpolate(*pre_cwp, *cur_cwp, cartesian_segments);
          for (std::size_t p = 1; p < poses.size(); ++p)
          {
            tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]),
                                                                 MoveInstructionType::LINEAR);
            move_instruction.setPosition(current_jv);
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else if (is_cwp1 && is_jwp2)
        {
          // If one is cartesian and the other is a joint waypoint it will calculate the forward kinematics
          // then cartesian interpolate and set the seed as the provided joint_waypoint.
          const auto* pre_cwp = start_waypoint.cast_const<CartesianWaypoint>();
          const auto* cur_jwp = plan_instruction->getWaypoint().cast_const<JointWaypoint>();

          Eigen::Isometry3d p2 = Eigen::Isometry3d::Identity();
          if (!fwd_kin->calcFwdKin(p2, *cur_jwp))
            throw std::runtime_error("tesseract_planning::generateSeed: failed to find forward kinematics solution!");

          p2 = world_to_base * p2 * plan_instruction->getTCP();
          tesseract_common::VectorIsometry3d poses = interpolate(*pre_cwp, p2, cartesian_segments);
          for (std::size_t p = 1; p < poses.size(); ++p)
          {
            tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]),
                                                                 MoveInstructionType::LINEAR);
            move_instruction.setPosition(*cur_jwp);
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else if (is_cwp2 && is_jwp1)
        {
          // If one is cartesian and the other is a joint waypoint it will calculate the forward kinematics
          // then cartesian interpolate and set the seed as the provided joint_waypoint.
          const auto* pre_jwp = start_waypoint.cast_const<JointWaypoint>();
          const auto* cur_cwp = plan_instruction->getWaypoint().cast_const<CartesianWaypoint>();

          Eigen::Isometry3d p1 = Eigen::Isometry3d::Identity();
          if (!fwd_kin->calcFwdKin(p1, *pre_jwp))
            throw std::runtime_error("tesseract_planning::generateSeed: failed to find forward kinematics solution!");

          p1 = world_to_base * p1 * plan_instruction->getTCP();
          tesseract_common::VectorIsometry3d poses = interpolate(p1, *cur_cwp, cartesian_segments);
          for (std::size_t p = 1; p < poses.size(); ++p)
          {
            tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]),
                                                                 MoveInstructionType::LINEAR);
            move_instruction.setPosition(*pre_jwp);
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else if (is_jwp1 && is_jwp2)
        {
          // If both are joint waypoints it will calculate the forward kinematics for both then cartesian interpolate
          // and set the seed using joint interpolation between the two.
          const auto* pre_jwp = start_waypoint.cast_const<JointWaypoint>();
          const auto* cur_jwp = plan_instruction->getWaypoint().cast_const<JointWaypoint>();

          Eigen::Isometry3d p1 = Eigen::Isometry3d::Identity();
          if (!fwd_kin->calcFwdKin(p1, *pre_jwp))
            throw std::runtime_error("tesseract_planning::generateSeed: failed to find forward kinematics solution!");

          p1 = world_to_base * p1 * plan_instruction->getTCP();

          Eigen::Isometry3d p2 = Eigen::Isometry3d::Identity();
          if (!fwd_kin->calcFwdKin(p2, *cur_jwp))
            throw std::runtime_error("tesseract_planning::generateSeed: failed to find forward kinematics solution!");

          p2 = world_to_base * p2 * plan_instruction->getTCP();
          tesseract_common::VectorIsometry3d poses = interpolate(p1, p2, cartesian_segments);
          Eigen::MatrixXd joint_poses = interpolate(*pre_jwp, *cur_jwp, cartesian_segments);
          for (std::size_t p = 1; p < poses.size(); ++p)
          {
            tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]),
                                                                 MoveInstructionType::LINEAR);
            move_instruction.setPosition(joint_poses.col(static_cast<long>(p)));
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else
        {
          throw std::runtime_error("tesseract_planning::InterpolatorPlanner: unsupported waypoints provided!");
        }

        seed.push_back(composite);
      }
      else if (plan_instruction->isFreespace())
      {
        CompositeInstruction composite;

        bool is_cwp1 = isCartesianWaypoint(start_waypoint.getType());
        bool is_jwp1 = isJointWaypoint(start_waypoint.getType());
        bool is_cwp2 = isCartesianWaypoint(plan_instruction->getWaypoint().getType());
        bool is_jwp2 = isJointWaypoint(plan_instruction->getWaypoint().getType());
        if (!found_plan_instruction)
        {
          tesseract_planning::MoveInstruction move_instruction(start_waypoint, MoveInstructionType::FREESPACE);

          if (is_jwp1)
            move_instruction.setPosition(*(start_waypoint.cast_const<JointWaypoint>()));
          else
            move_instruction.setPosition(current_jv);

          move_instruction.setTCP(plan_instruction->getTCP());
          move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
          move_instruction.setDescription(plan_instruction->getDescription());
          composite.push_back(move_instruction);
        }

        assert(is_cwp1 || is_jwp1);
        assert(is_cwp2 || is_jwp2);

        if (is_jwp1 && is_jwp2)
        {
          const auto* pre_cwp = start_waypoint.cast_const<JointWaypoint>();
          const auto* cur_cwp = plan_instruction->getWaypoint().cast_const<JointWaypoint>();

          Eigen::MatrixXd states = interpolate(*pre_cwp, *cur_cwp, freespace_segments);
          for (long i = 1; i < states.cols(); ++i)
          {
            tesseract_planning::MoveInstruction move_instruction(JointWaypoint(states.col(i)),
                                                                 MoveInstructionType::FREESPACE);
            move_instruction.setPosition(states.col(i));
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else if (is_cwp1 && is_jwp2)
        {
          const auto* cur_jwp = plan_instruction->getWaypoint().cast_const<JointWaypoint>();

          for (long i = 1; i < freespace_segments + 1; ++i)
          {
            tesseract_planning::MoveInstruction move_instruction(*cur_jwp, MoveInstructionType::FREESPACE);
            move_instruction.setPosition(*cur_jwp);
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else if (is_cwp2 && is_jwp1)
        {
          const auto* pre_jwp = start_waypoint.cast_const<JointWaypoint>();

          for (long i = 1; i < freespace_segments + 1; ++i)
          {
            tesseract_planning::MoveInstruction move_instruction(*pre_jwp, MoveInstructionType::FREESPACE);
            move_instruction.setPosition(*pre_jwp);
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else if (is_cwp1 && is_cwp2)
        {
          for (long i = 1; i < freespace_segments + 1; ++i)
          {
            tesseract_planning::MoveInstruction move_instruction(JointWaypoint(current_jv),
                                                                 MoveInstructionType::FREESPACE);
            move_instruction.setPosition(current_jv);
            move_instruction.setTCP(plan_instruction->getTCP());
            move_instruction.setWorkingFrame(plan_instruction->getWorkingFrame());
            move_instruction.setDescription(plan_instruction->getDescription());
            composite.push_back(move_instruction);
          }
        }
        else
        {
          throw std::runtime_error("tesseract_planning::generateSeed: unsupported waypoints provided!");
        }

        seed.push_back(composite);
      }
      else
      {
        throw std::runtime_error("Unsupported!");
      }

      found_plan_instruction = true;
      start_waypoint = plan_instruction->getWaypoint();
    }
    else
    {
      seed.push_back(instruction);
    }
  }







  return response.status;
}

void InterpolatorMotionPlanner::clear() {}


bool InterpolatorMotionPlanner::checkUserInput(const PlannerRequest& request) const
{
  // Check that parameters are valid
  if (request.tesseract == nullptr)
  {
    CONSOLE_BRIDGE_logError("In InterpolatorPlanner: tesseract is a required parameter and has not been set");
    return false;
  }

  // Check that parameters are valid
  auto manipulators = request.tesseract->getFwdKinematicsManagerConst()->getAvailableFwdKinematicsManipulators();
  if (std::find(manipulators.begin(), manipulators.end(), request.manipulator) == manipulators.end())
  {
    CONSOLE_BRIDGE_logError("In InterpolatorPlanner: manipulator is a required parameter and is not found in "
                            "the list of available manipulators");
    return false;
  }

  if (request.instructions.empty())
  {
    CONSOLE_BRIDGE_logError("InterpolatorPlanner requires at least 2 instructions");
    return false;
  }

  return true;
}
