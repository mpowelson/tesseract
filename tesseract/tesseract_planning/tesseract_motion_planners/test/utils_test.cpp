/**
  These tests test the TrajOptArrayPlanner and the TrajOptFreespacePlanner. They primarily check that the correct types
  of costs and constraints are added when the flags like smooth_velocity are specified. However they are not foolproof.
  They only check that at least one term of the correct type is in the cost or constraint vector. If there should be
  more than one, then it might not be caught. This could be improved in the future, but it is better than nothing.

  Additional features that could be tested in the future
  * Configuration costs added correctly
  * Intermediate waypoints added correctly to freespace
  * coeffs set correctly
  * init info is set correctly
  * Seed trajectory is set correctly
  * callbacks are added correctly
  * Number of steps are obeyed for freespace
  * continuous collision checking flag set correctly

  Last updated: July 15, 2019
  Matthew Powelson
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

class TesseractPlanningUtilsUnit : public ::testing::Test
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

TEST_F(TesseractPlanningUtilsUnit, toCartesianWaypoint)  // NOLINT
{
  EXPECT_TRUE(true);
}

TEST_F(TesseractPlanningUtilsUnit, toJointWaypoint)  // NOLINT
{
  EXPECT_TRUE(true);
}

TEST_F(TesseractPlanningUtilsUnit, Flatten)  // NOLINT
{
  // Create a composite
  CompositeInstruction composite;
  std::size_t i_max = 4;
  std::size_t j_max = 3;
  std::size_t k_max = 2;

  for (std::size_t i = 0; i < i_max; i++)
  {
    CompositeInstruction sub_composite;
    sub_composite.setDescription("sub_composite_" + std::to_string(i));
    for (std::size_t j = 0; j < j_max; j++)
    {
      CompositeInstruction sub_sub_composite;
      sub_composite.setDescription("sub_sub_composite_" + std::to_string(j));
      for (std::size_t k = 0; k < k_max; k++)
      {
        Waypoint wp = CartesianWaypoint(Eigen::Isometry3d::Identity());
        PlanInstruction instruction(wp, PlanInstructionType::LINEAR);
        instruction.setDescription("instruction_" + std::to_string(i) + "_" + std::to_string(j) + "_" +
                                   std::to_string(k));
        sub_sub_composite.push_back(instruction);
      }
      sub_composite.push_back(sub_sub_composite);
    }
    composite.push_back(sub_composite);
  }
  if (DEBUG)
    composite.print();

  // Flatten the composite
  std::vector<std::reference_wrapper<Instruction>> flattened = Flatten(composite);
  EXPECT_EQ(flattened.size(), i_max * j_max * k_max);

  // Now change something in the flattened composite
  for (std::size_t i = 0; i < flattened.size(); i++)
  {
    flattened[i].get().setDescription("test_" + std::to_string(i));
  }

  if (DEBUG)
    composite.print();

  // Now make sure the original changed
  std::size_t cumulative = 0;
  for (std::size_t i = 0; i < i_max; i++)
  {
    for (std::size_t j = 0; j < j_max; j++)
    {
      for (std::size_t k = 0; k < k_max; k++)
      {
        EXPECT_EQ(
            "test_" + std::to_string(cumulative),
            composite[i].cast<CompositeInstruction>()->at(j).cast<CompositeInstruction>()->at(k).getDescription());
        cumulative++;
      }
    }
  }

  EXPECT_TRUE(true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
