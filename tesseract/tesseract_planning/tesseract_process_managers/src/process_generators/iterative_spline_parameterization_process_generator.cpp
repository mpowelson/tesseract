/**
 * @file iterative_spline_parameterization_process_generator.cpp
 * @brief Perform iterative spline time parameterization
 *
 * @author Levi Armstrong
 * @date August 11. 2020
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/process_generators/iterative_spline_parameterization_process_generator.h>
#include <tesseract_command_language/utils/utils.h>

namespace tesseract_planning
{
inline const std::vector<std::vector<std::reference_wrapper<Instruction>>>
splitAtProfileChange(const std::vector<std::reference_wrapper<Instruction>>& trajectory,
                     const IterativeSplineParameterizationPlanProfileMap& map)
{
  std::vector<std::vector<std::reference_wrapper<Instruction>>> results;
  std::vector<std::reference_wrapper<Instruction>> current_split;
  current_split.reserve(trajectory.size());

  std::string previous_profile;
  for (const auto& instruction : trajectory)
  {
    // trajectory should be only planInstructions (use the planFilter)
    assert(isPlanInstruction(instruction));
    auto plan_instruction = instruction.get().cast_const<PlanInstruction>();

    // The profile is in the map set the current to that.
    std::string current_profile;
    if (map.find(plan_instruction->getProfile()) != map.end())
    {
      current_profile = plan_instruction->getProfile();
    }

    // Continue to append to the vector if the profile has not changed.
    if (current_profile == previous_profile)
    {
      current_split.push_back(instruction);
    }
    else
    {
      // Otherwise split it
      if (!current_split.empty())
        results.push_back(std::move(current_split));

      // And start a new vector
      current_split.clear();
      current_split.reserve(trajectory.size());
      current_split.push_back(instruction);
      previous_profile = current_profile;
    }
  }
  // Add the last split
  if (!current_split.empty())
    results.push_back(std::move(current_split));
  return results;
}

IterativeSplineParameterizationProcessGenerator::IterativeSplineParameterizationProcessGenerator(bool add_points,
                                                                                                 std::string name)
  : name_(std::move(name)), solver_(add_points)
{
  // Register default profile
}

const std::string& IterativeSplineParameterizationProcessGenerator::getName() const { return name_; }

std::function<void()> IterativeSplineParameterizationProcessGenerator::generateTask(ProcessInput input)
{
  return [=]() { process(input); };
}

std::function<int()> IterativeSplineParameterizationProcessGenerator::generateConditionalTask(ProcessInput input)
{
  return [=]() { return conditionalProcess(input); };
}

int IterativeSplineParameterizationProcessGenerator::conditionalProcess(ProcessInput input) const
{
  if (abort_)
    return 0;

  // --------------------
  // Check that inputs are valid
  // --------------------
  if (!isCompositeInstruction(*(input.results)))
  {
    CONSOLE_BRIDGE_logError("Input results to iterative spline parameterization must be a composite instruction");
    return 0;
  }

  auto* ci = input.results->cast<CompositeInstruction>();
  const ManipulatorInfo& manip_info = ci->getManipulatorInfo();
  const auto fwd_kin = input.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manip_info.manipulator);

  // Flatten and split all instructions based on plan profile
  auto flattened = flatten(*ci, planFilter);
  auto split_instructions = splitAtProfileChange(flattened, plan_profiles);

  for (auto& split : split_instructions)
  {
    // Get Plan Profile for ths split
    std::string profile = split[0].get().cast_const<PlanInstruction>()->getProfile();
    if (profile.empty())
      profile = "DEFAULT";

    // Check for remapping of profile
    auto remap = input.plan_profile_remapping.find(name_);
    if (remap != input.plan_profile_remapping.end())
    {
      auto p = remap->second.find(profile);
      if (p != remap->second.end())
        profile = p->second;
    }

    // Get the profile associated with this split
    typename IterativeSplineParameterizationPlanProfile::Ptr cur_plan_profile{ nullptr };
    auto it = plan_profiles.find(profile);
    if (it == plan_profiles.end())
      cur_plan_profile = std::make_shared<IterativeSplineParameterizationPlanProfile>();
    else
      cur_plan_profile = it->second;

    // Solve using plan profile parameters
    if (!solver_.compute(split,
                         fwd_kin->getLimits().velocity_limits,
                         fwd_kin->getLimits().acceleration_limits,
                         cur_plan_profile->max_velocity_scaling_factor,
                         cur_plan_profile->max_acceleration_scaling_factor))

    {
      CONSOLE_BRIDGE_logInform("Failed to perform iterative spline time parameterization!");
      return 0;
    }
  }

  CONSOLE_BRIDGE_logDebug("Iterative spline time parameterization succeeded");
  return 1;
}

void IterativeSplineParameterizationProcessGenerator::process(ProcessInput input) const { conditionalProcess(input); }

bool IterativeSplineParameterizationProcessGenerator::getAbort() const { return abort_; }
void IterativeSplineParameterizationProcessGenerator::setAbort(bool abort) { abort_ = abort; }

}  // namespace tesseract_planning
