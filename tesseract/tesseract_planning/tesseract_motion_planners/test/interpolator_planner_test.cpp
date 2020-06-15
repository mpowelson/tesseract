/**
 * @file interpolator_planner_test.cpp
 * @brief
 *
 * @author Matthew Powelson
 * @date June 15, 2020
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
#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <boost/filesystem.hpp>

TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract/tesseract.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_command_language/plan_instruction.h>

using namespace tesseract;
using namespace tesseract_planning;

bool DEBUG = false;

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

class TesseractPlanningInterpolatorPlannerUnit : public ::testing::Test
{
protected:
  Tesseract::Ptr tesseract_ptr_;

  void SetUp() override
  {
    tesseract_scene_graph::ResourceLocator::Ptr locator =
        std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
    Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
    boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
    boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
    EXPECT_TRUE(tesseract->init(urdf_path, srdf_path, locator));
    tesseract_ptr_ = tesseract;
  }
};

TEST_F(TesseractPlanningInterpolatorPlannerUnit, JointJoint)  // NOLINT
{
  CompositeInstruction program;

  Waypoint wp1 = JointWaypoint(Eigen::VectorXd::Zero(6));
  Waypoint wp2 = JointWaypoint(Eigen::VectorXd::Ones(6));

  // Define raster move instruction
  PlanInstruction plan_c1(wp3, PlanInstructionType::LINEAR);
  plan_c1.addConstraint(FixedComponentInfo());
  program.push_back(plan_c1);

  PlanInstruction plan_c2(wp4, PlanInstructionType::LINEAR);
  plan_c2.addConstraint(FixedComponentInfo());
  program.push_back(plan_f0);

  PlanInstruction plan_f1(wp2, PlanInstructionType::FREESPACE);
  plan_f1.setDescription("freespace");
  plan_f1.addCost(FixedComponentInfo());
  program.push_back(plan_f1);

  CompositeInstruction raster1;
  raster1.setDescription("raster");
  raster1.push_back(plan_c1);
  raster1.push_back(plan_c2);
  raster1.push_back(plan_c3);
  raster1.push_back(plan_c4);
  raster1.push_back(plan_c5);
  raster1.addCost(VelocitySmoothingComponentInfo());
  program.push_back(raster1);





  EXPECT_TRUE(true);
}

TEST_F(TesseractPlanningInterpolatorPlannerUnit, toJointWaypoint)  // NOLINT
{
  CompositeInstruction program;
  // Start Joint Position for the program
  Waypoint wp1 = JointWaypoint(Eigen::VectorXd::Ones(6));

  // Define raster poses
  Waypoint wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -1, 1));
  Waypoint wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -0.4, 1));
  Waypoint wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -0.2, 1));
  Waypoint wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.0, 1));
  Waypoint wp6 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.2, 1));
  Waypoint wp7 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.4, 1));

  // Define raster move instruction
  PlanInstruction plan_c1(wp3, PlanInstructionType::LINEAR);
  plan_c1.addConstraint(FixedComponentInfo());
  PlanInstruction plan_c2(wp4, PlanInstructionType::LINEAR);
  plan_c2.addConstraint(FixedComponentInfo());
  PlanInstruction plan_c3(wp5, PlanInstructionType::LINEAR);
  plan_c3.addConstraint(FixedComponentInfo());
  PlanInstruction plan_c4(wp6, PlanInstructionType::LINEAR);
  plan_c4.addConstraint(FixedComponentInfo());
  PlanInstruction plan_c5(wp7, PlanInstructionType::LINEAR);
  plan_c5.addConstraint(FixedComponentInfo());

//  PlanInstruction plan_f0(wp1, PlanInstructionType::FREESPACE);
  PlanInstruction plan_f0(wp2, PlanInstructionType::FREESPACE);
//  plan_f0.addConstraint(FixedComponentInfo());
  plan_f0.setDescription("to_start");
  program.push_back(plan_f0);

  PlanInstruction plan_f1(wp2, PlanInstructionType::FREESPACE);
  plan_f1.setDescription("freespace");
  plan_f1.addCost(FixedComponentInfo());
  program.push_back(plan_f1);

  CompositeInstruction raster1;
  raster1.setDescription("raster");
  raster1.push_back(plan_c1);
  raster1.push_back(plan_c2);
  raster1.push_back(plan_c3);
  raster1.push_back(plan_c4);
  raster1.push_back(plan_c5);
  raster1.addCost(VelocitySmoothingComponentInfo());
  program.push_back(raster1);


  EXPECT_TRUE(true);
}

TEST_F(TesseractPlanningInterpolatorPlannerUnit, Flatten)  // NOLINT
{
  EXPECT_TRUE(true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
