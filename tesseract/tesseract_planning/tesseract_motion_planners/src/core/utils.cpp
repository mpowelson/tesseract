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
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/command_language_utils.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
//CompositeInstruction SeedGenerator::generateSeed(const CompositeInstruction& instructions)
//{
//  current_jv = current_state->getJointValues(fwd_kin->getJointNames());
//  world_to_base = current_state->link_transforms.at(fwd_kin->getBaseLinkName());

//  CompositeInstruction seed;

//  // Get the start waypoint/instruction
//  start_waypoint = NullWaypoint();
//  MoveInstruction seed_start(start_waypoint, MoveInstructionType::START);
//  if (instructions.hasStartInstruction())
//  {
//    assert(isMoveInstruction(instructions.getStartInstruction()));
//    const auto* start_instruction = instructions.getStartInstruction().cast_const<MoveInstruction>();
//    assert(start_instruction->isStart());
//    start_waypoint = start_instruction->getWaypoint();

//    if (isJointWaypoint(start_waypoint))
//    {
//      StateWaypoint state_waypoint(*(start_waypoint.cast<JointWaypoint>()));
//      seed_start.setWaypoint(start_waypoint);
//    }
//    else if (isCartesianWaypoint(start_waypoint))
//    {
//      StateWaypoint state_waypoint(current_jv);
//      seed_start.setWaypoint(start_waypoint);
//    }
//    else
//      throw std::runtime_error("Generate Seed: Unsupported waypoint type!");
//  }
//  else
//  {
//    JointWaypoint temp(current_jv);
//    temp.joint_names = fwd_kin->getJointNames();
//    start_waypoint = temp;

//    StateWaypoint state_waypoint(current_jv);
//    seed_start.setWaypoint(start_waypoint);
//  }
//  // Process the seed
//  seed = processCompositeInstruction(instructions);

//  // Set start instruction
//  seed.setStartInstruction(seed_start);

//  return seed;
//}

};  // namespace tesseract_planning
