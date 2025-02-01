/**
 * @file fcl_collision_geometry_cache.h
 * @brief This is a static cache mapping tesseract geometry shared pointers to fcl collision geometry to avoid
 * recreating the same collision object
 *
 * @author Levi Armstrong
 * @date January 25, 2025
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2025, Levi Armstrong
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
#ifndef TESSERACT_COLLISION_FCL_COLLISION_GEOMETRY_CACHE_H
#define TESSERACT_COLLISION_FCL_COLLISION_GEOMETRY_CACHE_H

#include <map>
#include <memory>
#include <mutex>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <fcl/geometry/collision_geometry.h>
#include <boost/uuid/uuid.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/fwd.h>

namespace tesseract_collision::tesseract_collision_fcl
{
class FCLCollisionGeometryCache
{
public:
  /**
   * @brief Insert a new entry into the cache
   * @param key The cache key
   * @param value The value to store
   */
  static void insert(const std::shared_ptr<const tesseract_geometry::Geometry>& key,
                     const std::shared_ptr<fcl::CollisionGeometryd>& value);

  /**
   * @brief Retrieve the cache entry by key
   * @param key The cache key
   * @return If key exists the entry is returned, otherwise a nullptr is returned
   */
  static std::shared_ptr<fcl::CollisionGeometryd> get(const std::shared_ptr<const tesseract_geometry::Geometry>& key);

  /** @brief Remove any entries which are no longer valid */
  static void prune();

private:
  /** @brief The static cache */
  static std::map<boost::uuids::uuid, std::weak_ptr<fcl::CollisionGeometryd>> cache_;
  /** @brief The shared mutex for thread safety */
  static std::mutex mutex_;
};
}  // namespace tesseract_collision::tesseract_collision_fcl

#endif  // TESSERACT_COLLISION_FCL_COLLISION_GEOMETRY_CACHE_H
