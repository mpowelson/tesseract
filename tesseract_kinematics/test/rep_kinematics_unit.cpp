#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <fstream>
#include <tesseract_urdf/urdf_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "kinematics_test_utils.h"

#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/core/rep_inv_kin.h>
#include <tesseract_kinematics/opw/opw_inv_kin.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_state_solver/kdl/kdl_state_solver.h>
#include <opw_kinematics/opw_parameters.h>

using namespace tesseract_kinematics::test_suite;
using namespace tesseract_kinematics;

inline opw_kinematics::Parameters<double> getOPWKinematicsParamABB()
{
  opw_kinematics::Parameters<double> opw_params;
  opw_params.a1 = (0.100);
  opw_params.a2 = (-0.135);
  opw_params.b = (0.000);
  opw_params.c1 = (0.615);
  opw_params.c2 = (0.705);
  opw_params.c3 = (0.755);
  opw_params.c4 = (0.085);

  opw_params.offsets[2] = -M_PI / 2.0;

  return opw_params;
}

ForwardKinematics::UPtr getRobotFwdKinematics(const tesseract_scene_graph::SceneGraph& scene_graph)
{
  return std::make_unique<KDLFwdKinChain>(scene_graph, "base_link", "tool0");
}

ForwardKinematics::UPtr getFullFwdKinematics(const tesseract_scene_graph::SceneGraph& scene_graph)
{
  return std::make_unique<KDLFwdKinChain>(scene_graph, "positioner_tool0", "tool0");
}

ForwardKinematics::UPtr getPositionerFwdKinematics(const tesseract_scene_graph::SceneGraph& scene_graph)
{
  return std::make_unique<KDLFwdKinChain>(scene_graph, "positioner_base_link", "positioner_tool0");
}

InverseKinematics::UPtr getFullInvKinematics(const tesseract_scene_graph::SceneGraph& scene_graph)
{
  auto robot_fwd_kin = getRobotFwdKinematics(scene_graph);

  tesseract_scene_graph::KDLStateSolver state_solver(scene_graph);
  tesseract_scene_graph::SceneState scene_state = state_solver.getState();

  opw_kinematics::Parameters<double> opw_params = getOPWKinematicsParamABB();

  auto opw_kin = std::make_unique<OPWInvKin>(opw_params,
                                             robot_fwd_kin->getBaseLinkName(),
                                             robot_fwd_kin->getTipLinkNames()[0],
                                             robot_fwd_kin->getJointNames());

  auto positioner_kin = getPositionerFwdKinematics(scene_graph);
  Eigen::VectorXd positioner_resolution = Eigen::VectorXd::Constant(2, 1, 0.1);
  auto rep_inv_kin = std::make_unique<REPInvKin>(
      scene_graph, scene_state, opw_kin->clone(), 2.5, positioner_kin->clone(), positioner_resolution);

  {  // Test failure
    auto scene_graph_empty = std::make_shared<tesseract_scene_graph::SceneGraph>();
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(std::make_shared<REPInvKin>(
        *scene_graph_empty, scene_state, opw_kin->clone(), 2.5, positioner_kin->clone(), positioner_resolution));

    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(std::make_shared<REPInvKin>(
        scene_graph, scene_state, nullptr, 2.5, positioner_kin->clone(), positioner_resolution));

    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(std::make_shared<REPInvKin>(
        scene_graph, scene_state, opw_kin->clone(), -2.5, positioner_kin->clone(), positioner_resolution));

    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(
        std::make_shared<REPInvKin>(scene_graph, scene_state, opw_kin->clone(), 2.5, nullptr, positioner_resolution));

    positioner_resolution = Eigen::VectorXd();
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(std::make_shared<REPInvKin>(
        scene_graph, scene_state, opw_kin->clone(), 2.5, positioner_kin->clone(), positioner_resolution));

    positioner_resolution = Eigen::VectorXd::Constant(2, -0.1);
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(std::make_shared<REPInvKin>(
        scene_graph, scene_state, opw_kin->clone(), 2.5, positioner_kin->clone(), positioner_resolution));
  }

  return rep_inv_kin;
}

TEST(TesseractKinematicsUnit, RobotWithExternalPositionerInverseKinematicUnit)  // NOLINT
{
  auto scene_graph = getSceneGraphABBExternalPositioner();

  tesseract_scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract_scene_graph::SceneState scene_state = state_solver.getState();

  std::string manip_name = "robot_external_positioner";
  std::string base_link_name = "base_link";
  std::string working_frame = "positioner_tool0";
  std::string tip_link_name = "tool0";
  std::vector<std::string> joint_names{
    "positioner_joint_1", "positioner_joint_2", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
  };
  tesseract_common::KinematicLimits target_limits = getTargetLimits(*scene_graph, joint_names);

  auto fwd_kin = getFullFwdKinematics(*scene_graph);
  auto inv_kin = getFullInvKinematics(*scene_graph);
  auto inv_kin2 = inv_kin->clone();

  std::vector<std::string> fwd_joint_names = fwd_kin->getJointNames();
  std::vector<std::string> inv_joint_names = inv_kin->getJointNames();

  EXPECT_TRUE(tesseract_common::isIdentical(fwd_joint_names, inv_joint_names, false));

  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 0;
  pose.translation()[1] = 0;
  pose.translation()[2] = 0.1;

  Eigen::VectorXd seed = Eigen::VectorXd::Zero(fwd_kin->numJoints());

  EXPECT_TRUE(inv_kin != nullptr);
  EXPECT_EQ(inv_kin->getSolverName(), DEFAULT_REP_INV_KIN_SOLVER_NAME);
  EXPECT_EQ(inv_kin->numJoints(), 8);
  EXPECT_EQ(inv_kin->getBaseLinkName(), base_link_name);
  EXPECT_EQ(inv_kin->getWorkingFrame(), working_frame);
  EXPECT_EQ(inv_kin->getTipLinkNames().size(), 1);
  EXPECT_EQ(inv_kin->getTipLinkNames()[0], tip_link_name);
  EXPECT_EQ(inv_kin->getJointNames(), joint_names);

  KinematicGroup kin_group(manip_name, std::move(inv_kin), *scene_graph, scene_state);
  KinematicGroup kin_group_copy(kin_group);

  EXPECT_EQ(kin_group.getBaseLinkName(), scene_graph->getRoot());
  runInvKinTest(kin_group, pose, working_frame, tip_link_name, seed);
  runKinGroupJacobianABBExternalPositionerTest(kin_group);
  runActiveLinkNamesABBExternalPositionerTest(kin_group);
  runKinJointLimitsTest(kin_group.getLimits(), target_limits);
  runKinSetJointLimitsTest(kin_group);
  EXPECT_EQ(kin_group.getName(), manip_name);
  EXPECT_EQ(kin_group.getJointNames(), joint_names);
  EXPECT_EQ(kin_group.getAllPossibleTipLinkNames().size(), 1);
  EXPECT_EQ(kin_group.getAllPossibleTipLinkNames()[0], tip_link_name);
  EXPECT_EQ(kin_group.getAllValidWorkingFrames().size(), 1);
  EXPECT_EQ(kin_group.getAllValidWorkingFrames()[0], working_frame);

  // Check KinematicGroup copy
  EXPECT_EQ(kin_group_copy.getBaseLinkName(), scene_graph->getRoot());
  runInvKinTest(kin_group_copy, pose, working_frame, tip_link_name, seed);
  runKinGroupJacobianABBExternalPositionerTest(kin_group_copy);
  runActiveLinkNamesABBExternalPositionerTest(kin_group_copy);
  runKinJointLimitsTest(kin_group_copy.getLimits(), target_limits);
  runKinSetJointLimitsTest(kin_group_copy);
  EXPECT_EQ(kin_group_copy.getName(), manip_name);
  EXPECT_EQ(kin_group_copy.getJointNames(), joint_names);
  EXPECT_EQ(kin_group_copy.getAllPossibleTipLinkNames().size(), 1);
  EXPECT_EQ(kin_group_copy.getAllPossibleTipLinkNames()[0], tip_link_name);
  EXPECT_EQ(kin_group_copy.getAllValidWorkingFrames().size(), 1);
  EXPECT_EQ(kin_group_copy.getAllValidWorkingFrames()[0], working_frame);

  // Check cloned
  EXPECT_TRUE(inv_kin2 != nullptr);
  EXPECT_EQ(inv_kin2->getSolverName(), DEFAULT_REP_INV_KIN_SOLVER_NAME);
  EXPECT_EQ(inv_kin2->numJoints(), 8);
  EXPECT_EQ(inv_kin2->getBaseLinkName(), base_link_name);
  EXPECT_EQ(inv_kin2->getWorkingFrame(), working_frame);
  EXPECT_EQ(inv_kin2->getTipLinkNames().size(), 1);
  EXPECT_EQ(inv_kin2->getTipLinkNames()[0], tip_link_name);
  EXPECT_EQ(inv_kin2->getJointNames(), joint_names);

  KinematicGroup kin_group2(manip_name, std::move(inv_kin2), *scene_graph, scene_state);
  EXPECT_EQ(kin_group2.getBaseLinkName(), scene_graph->getRoot());
  runInvKinTest(kin_group2, pose, working_frame, tip_link_name, seed);
  runKinGroupJacobianABBExternalPositionerTest(kin_group2);
  runActiveLinkNamesABBExternalPositionerTest(kin_group2);
  runKinJointLimitsTest(kin_group2.getLimits(), target_limits);
  runKinSetJointLimitsTest(kin_group2);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
