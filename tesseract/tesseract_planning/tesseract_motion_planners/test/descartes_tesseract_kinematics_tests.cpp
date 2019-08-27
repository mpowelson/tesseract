#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <fstream>
#include <tesseract_scene_graph/parser/urdf_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <urdf_parser/urdf_parser.h>
#include <tesseract/tesseract.h>

#include <tesseract_motion_planners/descartes/impl/descartes_tesseract_kinematics.hpp>

using namespace tesseract;
using namespace tesseract_scene_graph;
using namespace tesseract_motion_planners;

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find("/");
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

    mod_url = package_path + mod_url;  // "file://" + package_path + mod_url;
  }

  return mod_url;
}

template <typename FloatType>
inline bool isNotValid(const FloatType* vertex)
{
  return false;
}

template <typename FloatType>
inline bool isCompletelyValid(const FloatType* vertex)
{
  return true;
}

class DescartesTesseractKinematicsUnit : public ::testing::Test
{
protected:
  Tesseract::Ptr tesseract_ptr_;
  tesseract_motion_planners::DescartesTesseractKinematics<double>::Ptr descartes_tesseract_kinematics_d_;
  tesseract_motion_planners::DescartesTesseractKinematics<double>::Ptr descartes_tesseract_kinematics_f_;

  tesseract_kinematics::ForwardKinematics::Ptr kdl_fk_;
  tesseract_kinematics::InverseKinematics::Ptr kdl_ik_;

  void SetUp() override
  {
    // Set up the Tesseract
    ResourceLocatorFn locator = locateResource;
    Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
    boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
    boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
    ASSERT_TRUE(tesseract->init(urdf_path, srdf_path, locator));
    tesseract_ptr_ = tesseract;

    // Set up the kinematics objects
    kdl_fk_ = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
    kdl_ik_ = tesseract_ptr_->getInvKinematicsManagerConst()->getInvKinematicSolver("manipulator");

    descartes_tesseract_kinematics_d_ = std::make_shared<tesseract_motion_planners::DescartesTesseractKinematics<double>>(kdl_fk_, kdl_ik_);
//    descartes_tesseract_kinematics_f_ = std::make_shared<tesseract_motion_planners::DescartesTesseractKinematics<float>>(kdl_fk_, kdl_ik_);

//    DescartesTesseractKinematics(const tesseract_kinematics::ForwardKinematics::ConstPtr tesseract_fk,
//                                 const tesseract_kinematics::InverseKinematics::ConstPtr tesseract_ik,
//                                 const descartes_light::IsValidFn<FloatType>& is_valid_fn,
//                                 const descartes_light::GetRedundantSolutionsFn<FloatType>& redundant_sol_fn)
  }
};


TEST_F(DescartesTesseractKinematicsUnit, ConstructorTest)
{
  // Test that the constructors all work
}

TEST_F(DescartesTesseractKinematicsUnit, IKTest)
{
  // Test the IK

  // Test the redundant solutions work

  // Test the isvalid works
}

TEST_F(DescartesTesseractKinematicsUnit, FKTest)
{
  // Test the FK
}

TEST_F(DescartesTesseractKinematicsUnit, DOFTest)
{
  // Sanity Check that urdf has 7 joints
  EXPECT_EQ(kdl_fk_->numJoints(), 7);
  // Actual Check
  EXPECT_EQ(descartes_tesseract_kinematics_d_->dof(), 7);
//  EXPECT_EQ(descartes_tesseract_kinematics_f_->dof(), 7);
}

TEST_F(DescartesTesseractKinematicsUnit, AnalyzeIKTest)
{
  // Check that this doesn't crash
  descartes_tesseract_kinematics_d_->analyzeIK(Eigen::Isometry3d::Identity());
//  descartes_tesseract_kinematics_f_->analyzeIK();
  EXPECT_TRUE(true);
}

TEST_F(DescartesTesseractKinematicsUnit, SetIKSeedTest)
{
  // TODO Create test class that checks that seed gets set
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
