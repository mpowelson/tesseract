/**
 * @file kdl_env.cpp
 * @brief Tesseract environment kdl implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#include "tesseract_environment/kdl/kdl_env.h"
#include "tesseract_environment/kdl/kdl_state_solver.h"

namespace tesseract_environment
{
bool KDLEnv::init(tesseract_scene_graph::SceneGraph::Ptr scene_graph)
{
  return Environment::create<KDLStateSolver>(scene_graph);
}

Environment::Ptr KDLEnv::clone() const
{
  auto cloned_env = std::make_shared<KDLEnv>();

  // Bring cloned environment up to current state using command history
  cloned_env->init(scene_graph_);
  cloned_env->applyCommands(commands_);

  // Register contact managers
  tesseract_collision::DiscreteContactManager::Ptr discrete_manager = getDiscreteContactManager();
  if (discrete_manager)
    cloned_env->registerDiscreteContactManager(discrete_manager_name_,
                                               [discrete_manager]() { return discrete_manager; });

  tesseract_collision::ContinuousContactManager::Ptr continuous_manager = getContinuousContactManager();
  if (continuous_manager)
    cloned_env->registerContinuousContactManager(continuous_manager_name_,
                                                 [continuous_manager]() { return continuous_manager; });

  return cloned_env;
}

}  // namespace tesseract_environment
