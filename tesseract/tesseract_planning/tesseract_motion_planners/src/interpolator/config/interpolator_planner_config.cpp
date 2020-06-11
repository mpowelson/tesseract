/**
 * @file Interpolator_planner_config.cpp
 * @brief Tesseract Interpolator planner config implementation.
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

#include <tesseract_motion_planners/interpolator/config/interpolator_planner_config.h>
#include <tesseract/tesseract.h>

namespace tesseract_motion_planners
{
InterpolatorPlannerConfig::InterpolatorPlannerConfig(tesseract::Tesseract::ConstPtr tesseract)
  : tesseract(std::move(tesseract))
{
}

tesseract_common::TrajArray InterpolatorPlannerConfig::getTrajectory() const
{
  //  assert(extractor != nullptr);
  //  return toTrajArray(this->simple_setup->getSolutionPath(), extractor);

//  tesseract_common::TrajArray traj(flat_seed_copy.size(),
//                                   flat_seed_copy[0].get().cast_const<MoveInstruction>()->getPosition().size());
//  for (std::size_t index = 0; index < flat_seed_copy.size(); index++)
//  {
//    traj.row(static_cast<Eigen::Index>(index)) =
//        flat_seed_copy[index].get().cast_const<MoveInstruction>()->getPosition().transpose();
//  }
  return tesseract_common::TrajArray();
}

}  // namespace tesseract_motion_planners
