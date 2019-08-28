#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <fstream>
//#include <tesseract_scene_graph/parser/urdf_parser.h>
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

template <typename FloatType>
inline std::vector<FloatType> noRedundantSolutions(const FloatType* sol, unsigned int& dof)
{
  std::vector<FloatType> redundant_sols(sol, sol + dof);
  return redundant_sols;
}

class DescartesTesseractKinematicsUnit : public ::testing::Test
{
protected:
  Tesseract::Ptr tesseract_ptr_;
  tesseract_motion_planners::DescartesTesseractKinematics<double>::Ptr descartes_tesseract_kinematics_d_;
  tesseract_motion_planners::DescartesTesseractKinematics<float>::Ptr descartes_tesseract_kinematics_f_;

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

    descartes_tesseract_kinematics_d_ =
        std::make_shared<tesseract_motion_planners::DescartesTesseractKinematics<double>>(kdl_fk_, kdl_ik_);
    descartes_tesseract_kinematics_f_ =
        std::make_shared<tesseract_motion_planners::DescartesTesseractKinematics<float>>(kdl_fk_, kdl_ik_);
  }
};

/** @brief This is used to test that private members are set correctly */
template <typename FloatType>
class DescartesTesseractKinematicsTest : public tesseract_motion_planners::DescartesTesseractKinematics<FloatType>
{
public:
  using tesseract_motion_planners::DescartesTesseractKinematics<FloatType>::DescartesTesseractKinematics;

  Eigen::VectorXd getIKSeed() { return this->ik_seed_; }
};

TEST_F(DescartesTesseractKinematicsUnit, IKTest)
{
  unsigned int dof = kdl_fk_->numJoints();
  {
    tesseract_motion_planners::DescartesTesseractKinematics<double> kin(
        kdl_fk_,
        kdl_ik_,
        nullptr,
        nullptr);
    // TODO: Test IK results
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<float> kin(
        kdl_fk_,
        kdl_ik_,
        nullptr,
        nullptr);
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<double> kin(
        kdl_fk_,
        kdl_ik_,
        &isNotValid<double>,
        nullptr);
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<float> kin(
        kdl_fk_,
        kdl_ik_,
        &isNotValid<float>,
        nullptr);
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<double> kin(
        kdl_fk_,
        kdl_ik_,
        nullptr,
        std::bind(&noRedundantSolutions<double>, std::placeholders::_1, dof));
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<float> kin(
        kdl_fk_,
        kdl_ik_,
        nullptr,
        std::bind(&noRedundantSolutions<float>, std::placeholders::_1, dof));
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<double> kin(
        kdl_fk_,
        kdl_ik_,
        &isNotValid<double>,
        std::bind(&noRedundantSolutions<double>, std::placeholders::_1, dof));
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<float> kin(
        kdl_fk_,
        kdl_ik_,
        &isNotValid<float>,
        std::bind(&noRedundantSolutions<float>, std::placeholders::_1, dof));
  }
}

/** @brief This checks fk() against calling the tesseract kinematics object directly */
TEST_F(DescartesTesseractKinematicsUnit, FKTest)
{
  Eigen::VectorXd joints_d(7);
  Eigen::VectorXf joints_f(7);
  joints_d << 0., 0.25, 0.5, 0.75, 1.0, 1.25, 1.5;
  joints_f << 0., 0.25, 0.5, 0.75, 1.0, 1.25, 1.5;
  Eigen::Isometry3d kdl_result_d = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d result_d = Eigen::Isometry3d::Identity();
  Eigen::Isometry3f result_f = Eigen::Isometry3f::Identity();

  // Test the fk against the tesseract object directly
  kdl_fk_->calcFwdKin(kdl_result_d, joints_d);
  EXPECT_TRUE(descartes_tesseract_kinematics_d_->fk(joints_d.data(), result_d));
  EXPECT_TRUE(descartes_tesseract_kinematics_f_->fk(joints_f.data(), result_f));

  EXPECT_TRUE(result_d.isApprox(kdl_result_d, 0.00001));
  EXPECT_TRUE(result_f.isApprox(kdl_result_d.cast<float>(), 0.00001));
}

/** @brief This checks that that dof() returns the correct value*/
TEST_F(DescartesTesseractKinematicsUnit, DOFTest)
{
  // Sanity Check that urdf has 7 joints
  EXPECT_EQ(kdl_fk_->numJoints(), 7);
  // Actual Check
  EXPECT_EQ(descartes_tesseract_kinematics_d_->dof(), 7);
  EXPECT_EQ(descartes_tesseract_kinematics_f_->dof(), 7);
}

/** @brief Since analyzeIK() does not have any results, this just checkst that it doesn't crash */
TEST_F(DescartesTesseractKinematicsUnit, AnalyzeIKTest)
{
  // Check that this doesn't crash
  descartes_tesseract_kinematics_d_->analyzeIK(Eigen::Isometry3d::Identity());
  descartes_tesseract_kinematics_f_->analyzeIK(Eigen::Isometry3f::Identity());
  EXPECT_TRUE(true);
}

/** @brief This tests that the ik seed is set correctly */
TEST_F(DescartesTesseractKinematicsUnit, SetIKSeedTest)
{
  auto kin_d = DescartesTesseractKinematicsTest<double>(kdl_fk_, kdl_ik_);
  auto kin_f = DescartesTesseractKinematicsTest<float>(kdl_fk_, kdl_ik_);

  std::vector<double> double_vec(7, 1.234567);
  std::vector<float> float_vec(7, 1.234567f);
  Eigen::Map<Eigen::VectorXd> double_eigen(double_vec.data(), 7);
  Eigen::Map<Eigen::VectorXf> float_eigen(float_vec.data(), 7);

  // Check when using std::vector
  kin_d.setIKSeed(double_vec);
  EXPECT_TRUE(kin_d.getIKSeed().isApprox(double_eigen, 0.00001));
  kin_f.setIKSeed(float_vec);
  EXPECT_TRUE(kin_f.getIKSeed().isApprox(double_eigen, 0.00001));

  // Check when using Eigen::VectorX
  kin_d.setIKSeed(double_eigen);
  EXPECT_TRUE(kin_d.getIKSeed().isApprox(double_eigen, 0.00001));
  kin_f.setIKSeed(float_eigen);
  EXPECT_TRUE(kin_f.getIKSeed().isApprox(double_eigen, 0.00001));  // Note that the seed is stored as a VectorXd
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
