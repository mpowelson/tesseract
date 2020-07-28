/**
 * @file simple_planner_fixed_size_interpolation.cpp
 * @brief
 *
 * @author Matthew Powelson
 * @date July 23, 2020
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
#include <boost/filesystem.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/step_generators/fixed_size_interpolation.h>

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

class TesseractPlanningSimplePlannerFixedSizeInterpolationUnit : public ::testing::Test
{
protected:
  Tesseract::Ptr tesseract_ptr_;
  ManipulatorInfo manip_info_;

  void SetUp() override
  {
    tesseract_scene_graph::ResourceLocator::Ptr locator =
        std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
    Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
    boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
    boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
    EXPECT_TRUE(tesseract->init(urdf_path, srdf_path, locator));
    tesseract_ptr_ = tesseract;

    manip_info_.manipulator = "manipulator";
  }
};

TEST_F(TesseractPlanningSimplePlannerFixedSizeInterpolationUnit, JointJoint_JointInterpolation)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  JointWaypoint wp1 = Eigen::VectorXd::Zero(7);
  JointWaypoint wp2 = Eigen::VectorXd::Ones(7);
  PlanInstruction instr(wp1, PlanInstructionType::FREESPACE, "DEFAULT", manip_info_);

  auto composite = fixedSizeJointInterpolation(wp1, wp2, instr, request, ManipulatorInfo(), 10);
  EXPECT_EQ(composite.size(), 10);
  // TODO: Test correctness
}

TEST_F(TesseractPlanningSimplePlannerFixedSizeInterpolationUnit, JointCart_JointInterpolation)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  JointWaypoint wp1 = Eigen::VectorXd::Zero(7);
  CartesianWaypoint wp2 = Eigen::Isometry3d::Identity();
  PlanInstruction instr(wp1, PlanInstructionType::FREESPACE, "DEFAULT", manip_info_);

  auto composite = fixedSizeJointInterpolation(wp1, wp2, instr, request, ManipulatorInfo(), 10);
  EXPECT_EQ(composite.size(), 0);  // TODO: Implement this function
}

TEST_F(TesseractPlanningSimplePlannerFixedSizeInterpolationUnit, CartJoint_JointInterpolation)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  CartesianWaypoint wp1 = Eigen::Isometry3d::Identity();
  JointWaypoint wp2 = Eigen::VectorXd::Zero(7);
  PlanInstruction instr(wp1, PlanInstructionType::FREESPACE, "DEFAULT", manip_info_);

  auto composite = fixedSizeJointInterpolation(wp1, wp2, instr, request, ManipulatorInfo(), 10);
  EXPECT_EQ(composite.size(), 0);  // TODO: Implement this function
}

TEST_F(TesseractPlanningSimplePlannerFixedSizeInterpolationUnit, CartCart_JointInterpolation)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  CartesianWaypoint wp1 = Eigen::Isometry3d::Identity();
  CartesianWaypoint wp2 = Eigen::Isometry3d::Identity();
  PlanInstruction instr(wp1, PlanInstructionType::FREESPACE, "DEFAULT", manip_info_);

  auto composite = fixedSizeJointInterpolation(wp1, wp2, instr, request, ManipulatorInfo(), 10);
  EXPECT_EQ(composite.size(), 0);  // TODO: Implement this function
}

TEST_F(TesseractPlanningSimplePlannerFixedSizeInterpolationUnit, JointJoint_CartesianInterpolation)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();
  JointWaypoint wp1 = Eigen::VectorXd::Zero(7);
  JointWaypoint wp2 = Eigen::VectorXd::Ones(7);
  PlanInstruction instr(wp1, PlanInstructionType::FREESPACE, "DEFAULT", manip_info_);

  auto composite = fixedSizeCartesianInterpolation(wp1, wp2, instr, request, ManipulatorInfo(), 10);
  EXPECT_EQ(composite.size(), 10);
  // TODO: Test correctness
}

TEST_F(TesseractPlanningSimplePlannerFixedSizeInterpolationUnit, JointCart_CartesianInterpolation)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  JointWaypoint wp1 = Eigen::VectorXd::Zero(7);
  CartesianWaypoint wp2 = Eigen::Isometry3d::Identity();
  PlanInstruction instr(wp1, PlanInstructionType::FREESPACE, "DEFAULT", manip_info_);

  auto composite = fixedSizeCartesianInterpolation(wp1, wp2, instr, request, ManipulatorInfo(), 10);
  EXPECT_EQ(composite.size(), 0);  // TODO: Implement this function
}

TEST_F(TesseractPlanningSimplePlannerFixedSizeInterpolationUnit, CartJoint_CartesianInterpolation)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  CartesianWaypoint wp1 = Eigen::Isometry3d::Identity();
  JointWaypoint wp2 = Eigen::VectorXd::Zero(7);
  PlanInstruction instr(wp1, PlanInstructionType::FREESPACE, "DEFAULT", manip_info_);

  auto composite = fixedSizeCartesianInterpolation(wp1, wp2, instr, request, ManipulatorInfo(), 10);
  EXPECT_EQ(composite.size(), 0);  // TODO: Implement this function
}

TEST_F(TesseractPlanningSimplePlannerFixedSizeInterpolationUnit, CartCart_CartesianInterpolation)  // NOLINT
{
  PlannerRequest request;
  request.tesseract = tesseract_ptr_;
  CartesianWaypoint wp1 = Eigen::Isometry3d::Identity();
  CartesianWaypoint wp2 = Eigen::Isometry3d::Identity();
  PlanInstruction instr(wp1, PlanInstructionType::FREESPACE, "DEFAULT", manip_info_);

  auto composite = fixedSizeCartesianInterpolation(wp1, wp2, instr, request, ManipulatorInfo(), 10);
  EXPECT_EQ(composite.size(), 10);
  // TODO: Test correctness
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
