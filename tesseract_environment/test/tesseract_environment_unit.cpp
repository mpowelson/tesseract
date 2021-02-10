#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <vector>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/fcl/fcl_discrete_managers.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_geometry/impl/box.h>
#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/types.h>
#include <tesseract_environment/core/commands.h>
#include <tesseract_environment/kdl/kdl_state_solver.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_environment/core/environment.h>

using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_environment;

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

SceneGraph::Ptr getSceneGraph()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  return tesseract_urdf::parseURDFFile(path, locator);
}

tesseract_scene_graph::SRDFModel::Ptr getSRDFModel(const SceneGraph::Ptr& scene_graph)
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf";

  tesseract_scene_graph::SRDFModel::Ptr srdf = std::make_shared<tesseract_scene_graph::SRDFModel>();
  srdf->initFile(*scene_graph, path);

  return srdf;
}

template <typename S>
Environment::Ptr getEnvironment()
{
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraph();
  EXPECT_TRUE(scene_graph != nullptr);

  // Check to make sure all links are enabled
  for (const auto& link : scene_graph->getLinks())
  {
    EXPECT_TRUE(scene_graph->getLinkCollisionEnabled(link->getName()));
    EXPECT_TRUE(scene_graph->getLinkVisibility(link->getName()));
  }

  auto srdf = getSRDFModel(scene_graph);
  EXPECT_TRUE(srdf != nullptr);

  auto env = std::make_shared<Environment>(false);
  EXPECT_TRUE(env != nullptr);
  EXPECT_EQ(0, env->getRevision());

  bool success = env->init<S>(*scene_graph, srdf);
  EXPECT_TRUE(success);
  EXPECT_EQ(2, env->getRevision());

  // Check to make sure all links are enabled
  for (const auto& link : env->getSceneGraph()->getLinks())
  {
    EXPECT_TRUE(env->getSceneGraph()->getLinkCollisionEnabled(link->getName()));
    EXPECT_TRUE(env->getSceneGraph()->getLinkVisibility(link->getName()));
  }

  // Register contact manager
  EXPECT_TRUE(env->registerDiscreteContactManager(tesseract_collision_bullet::BulletDiscreteBVHManager::name(),
                                                  &tesseract_collision_bullet::BulletDiscreteBVHManager::create));
  EXPECT_TRUE(env->registerContinuousContactManager(tesseract_collision_bullet::BulletCastBVHManager::name(),
                                                    &tesseract_collision_bullet::BulletCastBVHManager::create));

  // Set Active contact manager
  EXPECT_TRUE(env->setActiveDiscreteContactManager(tesseract_collision_bullet::BulletDiscreteBVHManager::name()));
  EXPECT_TRUE(env->setActiveContinuousContactManager(tesseract_collision_bullet::BulletCastBVHManager::name()));

  return env;
}

template <typename S>
void runContactManagerCloneTest()
{
  // Get the environment
  auto env = getEnvironment<S>();

  // Test after clone if active list correct
  tesseract_collision::DiscreteContactManager::Ptr discrete_manager = env->getDiscreteContactManager();
  const std::vector<std::string>& e_active_list = env->getActiveLinkNames();
  const std::vector<std::string>& d_active_list = discrete_manager->getActiveCollisionObjects();
  EXPECT_TRUE(std::equal(e_active_list.begin(), e_active_list.end(), d_active_list.begin()));

  tesseract_collision::ContinuousContactManager::Ptr cast_manager = env->getContinuousContactManager();
  const std::vector<std::string>& c_active_list = cast_manager->getActiveCollisionObjects();
  EXPECT_TRUE(std::equal(e_active_list.begin(), e_active_list.end(), c_active_list.begin()));
}

template <typename S>
void runAddandRemoveAllowedCollisionCommandTest()
{
  // Get the environment
  auto env = getEnvironment<S>();

  std::string l1 = "link_1";
  std::string l2 = "link_6";
  std::string r = "Unit Test";

  AllowedCollisionMatrix::ConstPtr acm = env->getAllowedCollisionMatrix();
  EXPECT_TRUE(acm->isCollisionAllowed(l1, "base_link"));
  EXPECT_TRUE(acm->isCollisionAllowed(l1, "link_2"));
  EXPECT_TRUE(acm->isCollisionAllowed(l1, "link_3"));
  EXPECT_TRUE(acm->isCollisionAllowed(l1, "link_4"));
  EXPECT_TRUE(acm->isCollisionAllowed(l1, "link_5"));
  EXPECT_TRUE(acm->isCollisionAllowed(l1, "link_6"));
  EXPECT_TRUE(acm->isCollisionAllowed(l1, "link_7"));
  EXPECT_EQ(env->getRevision(), 2);
  EXPECT_EQ(env->getCommandHistory().size(), 2);

  // Remove allowed collision
  auto cmd_remove = std::make_shared<RemoveAllowedCollisionCommand>(l1, l2);
  EXPECT_EQ(cmd_remove->getType(), CommandType::REMOVE_ALLOWED_COLLISION);
  EXPECT_EQ(cmd_remove->getLinkName1(), l1);
  EXPECT_EQ(cmd_remove->getLinkName2(), l2);

  EXPECT_TRUE(env->applyCommand(cmd_remove));

  EXPECT_FALSE(acm->isCollisionAllowed(l1, l2));
  EXPECT_EQ(env->getRevision(), 3);
  EXPECT_EQ(env->getCommandHistory().size(), 3);
  EXPECT_EQ(env->getCommandHistory().back(), cmd_remove);

  // Add allowed collision back
  auto cmd_add = std::make_shared<AddAllowedCollisionCommand>(l1, l2, r);
  EXPECT_EQ(cmd_add->getType(), CommandType::ADD_ALLOWED_COLLISION);
  EXPECT_EQ(cmd_add->getLinkName1(), l1);
  EXPECT_EQ(cmd_add->getLinkName2(), l2);
  EXPECT_EQ(cmd_add->getReason(), r);

  EXPECT_TRUE(env->applyCommand(cmd_add));

  EXPECT_TRUE(acm->isCollisionAllowed(l1, l2));
  EXPECT_EQ(env->getRevision(), 4);
  EXPECT_EQ(env->getCommandHistory().size(), 4);
  EXPECT_EQ(env->getCommandHistory().back(), cmd_add);

  // Remove allowed collision
  auto cmd_remove_link = std::make_shared<RemoveAllowedCollisionLinkCommand>(l1);
  EXPECT_EQ(cmd_remove_link->getType(), CommandType::REMOVE_ALLOWED_COLLISION_LINK);
  EXPECT_EQ(cmd_remove_link->getLinkName(), l1);

  EXPECT_TRUE(env->applyCommand(cmd_remove_link));

  EXPECT_FALSE(acm->isCollisionAllowed(l1, "base_link"));
  EXPECT_FALSE(acm->isCollisionAllowed(l1, "link_2"));
  EXPECT_FALSE(acm->isCollisionAllowed(l1, "link_3"));
  EXPECT_FALSE(acm->isCollisionAllowed(l1, "link_4"));
  EXPECT_FALSE(acm->isCollisionAllowed(l1, "link_5"));
  EXPECT_FALSE(acm->isCollisionAllowed(l1, "link_6"));
  EXPECT_FALSE(acm->isCollisionAllowed(l1, "link_7"));
  EXPECT_EQ(env->getRevision(), 5);
  EXPECT_EQ(env->getCommandHistory().size(), 5);
  EXPECT_EQ(env->getCommandHistory().back(), cmd_remove_link);
}

template <typename S>
void runAddandRemoveLinkTest(bool use_command = false)
{
  // Get the environment
  auto env = getEnvironment<S>();

  auto visual = std::make_shared<Visual>();
  visual->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  auto collision = std::make_shared<Collision>();
  collision->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);

  const std::string link_name1 = "link_n1";
  const std::string link_name2 = "link_n2";
  const std::string joint_name1 = "joint_n1";
  Link link_1(link_name1);
  link_1.visual.push_back(visual);
  link_1.collision.push_back(collision);
  Link link_2(link_name2);

  Joint joint_1(joint_name1);
  joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1.parent_link_name = link_name1;
  joint_1.child_link_name = link_name2;
  joint_1.type = JointType::FIXED;

  if (use_command)
  {
    auto cmd = std::make_shared<AddCommand>(link_1);
    EXPECT_TRUE(cmd != nullptr);
    EXPECT_EQ(cmd->getType(), CommandType::ADD);
    EXPECT_TRUE(cmd->getLink() != nullptr);
    EXPECT_TRUE(cmd->getJoint() == nullptr);
    EXPECT_TRUE(env->applyCommand(cmd));
  }
  else
  {
    EXPECT_TRUE(env->addLink(link_1));
  }
  EXPECT_EQ(env->getRevision(), 3);
  EXPECT_EQ(env->getCommandHistory().size(), 3);
  EXPECT_TRUE(env->getDiscreteContactManager()->hasCollisionObject(link_name1));
  EXPECT_FALSE(env->getDiscreteContactManager()->hasCollisionObject(link_name2));
  EXPECT_TRUE(env->getContinuousContactManager()->hasCollisionObject(link_name1));
  EXPECT_FALSE(env->getContinuousContactManager()->hasCollisionObject(link_name2));

  std::vector<std::string> link_names = env->getLinkNames();
  std::vector<std::string> joint_names = env->getJointNames();
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name1) != link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), "joint_" + link_name1) != joint_names.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name1) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find("joint_" + link_name1) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find("joint_" + link_name1) == env->getCurrentState()->joints.end());

  if (use_command)
  {
    auto cmd = std::make_shared<AddCommand>(link_2, joint_1);
    EXPECT_TRUE(cmd != nullptr);
    EXPECT_EQ(cmd->getType(), CommandType::ADD);
    EXPECT_TRUE(cmd->getLink() != nullptr);
    EXPECT_TRUE(cmd->getJoint() != nullptr);
    EXPECT_TRUE(env->applyCommand(cmd));
  }
  else
  {
    env->addLink(link_2, joint_1);
  }
  EXPECT_EQ(env->getRevision(), 4);
  EXPECT_EQ(env->getCommandHistory().size(), 4);

  link_names = env->getLinkNames();
  joint_names = env->getJointNames();
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name2) != link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) != joint_names.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name2) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name1) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name1) == env->getCurrentState()->joints.end());

  env->getSceneGraph()->saveDOT(tesseract_common::getTempPath() + "before_remove_link_unit.dot");

  if (use_command)
  {
    auto cmd = std::make_shared<RemoveLinkCommand>(link_name1);
    EXPECT_TRUE(cmd != nullptr);
    EXPECT_EQ(cmd->getType(), CommandType::REMOVE_LINK);
    EXPECT_EQ(cmd->getLinkName(), link_name1);
    EXPECT_TRUE(env->applyCommand(cmd));
  }
  else
  {
    env->removeLink(link_name1);
  }
  EXPECT_EQ(env->getRevision(), 5);
  EXPECT_EQ(env->getCommandHistory().size(), 5);
  EXPECT_FALSE(env->getDiscreteContactManager()->hasCollisionObject(link_name1));
  EXPECT_FALSE(env->getDiscreteContactManager()->hasCollisionObject(link_name2));
  EXPECT_FALSE(env->getContinuousContactManager()->hasCollisionObject(link_name1));
  EXPECT_FALSE(env->getContinuousContactManager()->hasCollisionObject(link_name2));

  link_names = env->getLinkNames();
  joint_names = env->getJointNames();
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name1) == link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), "joint_" + link_name1) == joint_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name2) == link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name1) ==
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find("joint_" + link_name1) ==
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find("joint_" + link_name1) == env->getCurrentState()->joints.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name2) ==
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name1) ==
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name1) == env->getCurrentState()->joints.end());

  env->getSceneGraph()->saveDOT(tesseract_common::getTempPath() + "after_remove_link_unit.dot");

  // Test against double removing
  if (use_command)
  {
    auto cmd = std::make_shared<RemoveLinkCommand>(link_name1);
    EXPECT_TRUE(cmd != nullptr);
    EXPECT_EQ(cmd->getType(), CommandType::REMOVE_LINK);
    EXPECT_EQ(cmd->getLinkName(), link_name1);
    EXPECT_FALSE(env->applyCommand(cmd));
    EXPECT_EQ(env->getRevision(), 5);
    EXPECT_EQ(env->getCommandHistory().size(), 5);

    cmd = std::make_shared<RemoveLinkCommand>(link_name2);
    EXPECT_TRUE(cmd != nullptr);
    EXPECT_EQ(cmd->getType(), CommandType::REMOVE_LINK);
    EXPECT_EQ(cmd->getLinkName(), link_name2);
    EXPECT_FALSE(env->applyCommand(cmd));
    EXPECT_EQ(env->getRevision(), 5);
    EXPECT_EQ(env->getCommandHistory().size(), 5);

    auto cmd2 = std::make_shared<RemoveJointCommand>(joint_name1);
    EXPECT_TRUE(cmd2 != nullptr);
    EXPECT_EQ(cmd2->getType(), CommandType::REMOVE_JOINT);
    EXPECT_EQ(cmd2->getJointName(), joint_name1);
    EXPECT_FALSE(env->applyCommand(cmd2));
    EXPECT_EQ(env->getRevision(), 5);
    EXPECT_EQ(env->getCommandHistory().size(), 5);

    cmd2 = std::make_shared<RemoveJointCommand>("joint_" + link_name1);
    EXPECT_TRUE(cmd2 != nullptr);
    EXPECT_EQ(cmd2->getType(), CommandType::REMOVE_JOINT);
    EXPECT_EQ(cmd2->getJointName(), "joint_" + link_name1);
    EXPECT_FALSE(env->applyCommand(cmd2));
    EXPECT_EQ(env->getRevision(), 5);
    EXPECT_EQ(env->getCommandHistory().size(), 5);
  }
  else
  {
    EXPECT_FALSE(env->removeLink(link_name1));
    EXPECT_EQ(env->getRevision(), 5);
    EXPECT_EQ(env->getCommandHistory().size(), 5);

    EXPECT_FALSE(env->removeLink(link_name2));
    EXPECT_EQ(env->getRevision(), 5);
    EXPECT_EQ(env->getCommandHistory().size(), 5);

    EXPECT_FALSE(env->removeJoint(joint_name1));
    EXPECT_EQ(env->getRevision(), 5);
    EXPECT_EQ(env->getCommandHistory().size(), 5);

    EXPECT_FALSE(env->removeJoint("joint_" + link_name1));
    EXPECT_EQ(env->getRevision(), 5);
    EXPECT_EQ(env->getCommandHistory().size(), 5);
  }
}

template <typename S>
void runAddKinematicsInformationCommandTest(bool use_command = false)
{
  // Get the environment
  auto env = getEnvironment<S>();

  KinematicsInformation kin_info;

  if (use_command)
  {
    auto cmd = std::make_shared<AddKinematicsInformationCommand>(kin_info);

    EXPECT_TRUE(cmd != nullptr);
    EXPECT_EQ(cmd->getType(), CommandType::ADD_KINEMATICS_INFORMATION);
    KinematicsInformation kin_info2 = cmd->getKinematicsInformation();
    EXPECT_TRUE(kin_info2.group_names == kin_info.group_names);
    EXPECT_TRUE(kin_info2.group_names == kin_info.group_names);
    EXPECT_TRUE(kin_info2.chain_groups == kin_info.chain_groups);
    EXPECT_TRUE(kin_info2.joint_groups == kin_info.joint_groups);
    EXPECT_TRUE(kin_info2.link_groups == kin_info.link_groups);
    //    EXPECT_TRUE(kin_info2.group_rop_kinematics == kin_info.group_rop_kinematics);
    //    EXPECT_TRUE(kin_info2.group_rep_kinematics == kin_info.group_rep_kinematics);
    EXPECT_TRUE(kin_info2.group_states == kin_info.group_states);
    //    EXPECT_TRUE(kin_info2.group_tcps == kin_info.group_tcps);
    //    EXPECT_TRUE(kin_info2.group_opw_kinematics == kin_info.group_opw_kinematics);
    EXPECT_TRUE(env->applyCommand(cmd));
  }
  else
  {
    EXPECT_TRUE(env->addKinematicsInformation(kin_info));
  }
  EXPECT_EQ(env->getRevision(), 3);
  EXPECT_EQ(env->getCommandHistory().size(), 3);
}

template <typename S>
void runAddSceneGraphCommandTest(bool use_command = false)
{
  // Get the environment
  auto env = getEnvironment<S>();

  SceneGraph::Ptr subgraph = std::make_shared<SceneGraph>();
  subgraph->setName("subgraph");

  // Adding an empty scene graph which should fail
  if (use_command)
  {
    auto cmd = std::make_shared<AddSceneGraphCommand>(*subgraph);
    EXPECT_TRUE(cmd != nullptr);
    EXPECT_EQ(cmd->getType(), CommandType::ADD_SCENE_GRAPH);
    EXPECT_TRUE(cmd->getSceneGraph() != nullptr);
    EXPECT_FALSE(env->applyCommand(cmd));
  }
  else
  {
    EXPECT_FALSE(env->addSceneGraph(*subgraph));
  }
  EXPECT_EQ(env->getRevision(), 2);
  EXPECT_EQ(env->getCommandHistory().size(), 2);

  // Now add a link to empty environment
  auto visual = std::make_shared<Visual>();
  visual->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  auto collision = std::make_shared<Collision>();
  collision->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);

  const std::string link_name1 = "subgraph_base_link";
  const std::string link_name2 = "subgraph_link_1";
  const std::string joint_name1 = "subgraph_joint1";
  Link link_1(link_name1);
  link_1.visual.push_back(visual);
  link_1.collision.push_back(collision);
  Link link_2(link_name2);

  Joint joint_1(joint_name1);
  joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1.parent_link_name = link_name1;
  joint_1.child_link_name = link_name2;
  joint_1.type = JointType::FIXED;

  subgraph->addLink(link_1);
  subgraph->addLink(link_2);
  subgraph->addJoint(joint_1);

  if (use_command)
  {
    auto cmd = std::make_shared<AddSceneGraphCommand>(*subgraph);
    EXPECT_TRUE(cmd != nullptr);
    EXPECT_EQ(cmd->getType(), CommandType::ADD_SCENE_GRAPH);
    EXPECT_TRUE(cmd->getSceneGraph() != nullptr);
    EXPECT_TRUE(env->applyCommand(cmd));
  }
  else
  {
    EXPECT_TRUE(env->addSceneGraph(*subgraph));
  }
  EXPECT_EQ(env->getRevision(), 3);
  EXPECT_EQ(env->getCommandHistory().size(), 3);
  EXPECT_TRUE(env->getDiscreteContactManager()->hasCollisionObject(link_name1));
  EXPECT_FALSE(env->getDiscreteContactManager()->hasCollisionObject(link_name2));
  EXPECT_TRUE(env->getContinuousContactManager()->hasCollisionObject(link_name1));
  EXPECT_FALSE(env->getContinuousContactManager()->hasCollisionObject(link_name2));

  EXPECT_TRUE(env->getJoint("subgraph_joint") != nullptr);
  EXPECT_TRUE(env->getLink(link_name1) != nullptr);
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name1) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find("subgraph_joint") !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find("subgraph_joint") == env->getCurrentState()->joints.end());

  // Adding twice with the same name should fail
  if (use_command)
  {
    auto cmd = std::make_shared<AddSceneGraphCommand>(*subgraph);
    EXPECT_TRUE(cmd != nullptr);
    EXPECT_EQ(cmd->getType(), CommandType::ADD_SCENE_GRAPH);
    EXPECT_TRUE(cmd->getSceneGraph() != nullptr);
    EXPECT_FALSE(env->applyCommand(cmd));
  }
  else
  {
    EXPECT_FALSE(env->addSceneGraph(*subgraph));
  }
  EXPECT_EQ(env->getRevision(), 3);
  EXPECT_EQ(env->getCommandHistory().size(), 3);

  if (use_command)
  {
    auto cmd = std::make_shared<AddSceneGraphCommand>(*subgraph, "prefix_");
    EXPECT_TRUE(cmd != nullptr);
    EXPECT_EQ(cmd->getType(), CommandType::ADD_SCENE_GRAPH);
    EXPECT_TRUE(cmd->getSceneGraph() != nullptr);
    EXPECT_TRUE(env->applyCommand(cmd));
  }
  else
  {
    EXPECT_TRUE(env->addSceneGraph(*subgraph, "prefix_"));
  }
  EXPECT_EQ(env->getRevision(), 4);
  EXPECT_EQ(env->getCommandHistory().size(), 4);

  EXPECT_TRUE(env->getJoint("prefix_subgraph_joint") != nullptr);
  EXPECT_TRUE(env->getLink("prefix_subgraph_base_link") != nullptr);
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name1) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find("subgraph_joint") !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find("subgraph_joint") == env->getCurrentState()->joints.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find("prefix_subgraph_base_link") !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find("prefix_subgraph_joint") !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find("prefix_subgraph_joint") == env->getCurrentState()->joints.end());
}

template <typename S>
void runChangeDefaultContactMarginCommandTest()
{
  // Get the environment
  auto env = getEnvironment<S>();
  EXPECT_EQ(env->getRevision(), 2);
  EXPECT_EQ(env->getCommandHistory().size(), 2);
  EXPECT_NEAR(env->getDiscreteContactManager()->getCollisionMarginData().getDefaultCollisionMarginData(), 0.0, 1e-6);
  EXPECT_NEAR(env->getContinuousContactManager()->getCollisionMarginData().getDefaultCollisionMarginData(), 0.0, 1e-6);

  auto cmd = std::make_shared<ChangeDefaultContactMarginCommand>(0.1);
  EXPECT_TRUE(cmd != nullptr);
  EXPECT_EQ(cmd->getType(), CommandType::CHANGE_DEFAULT_CONTACT_MARGIN);
  EXPECT_NEAR(cmd->getDefaultCollisionMargin(), 0.1, 1e-6);
  EXPECT_TRUE(env->applyCommand(cmd));

  EXPECT_EQ(env->getRevision(), 3);
  EXPECT_EQ(env->getCommandHistory().size(), 3);
  EXPECT_EQ(env->getCommandHistory().back(), cmd);
  EXPECT_NEAR(env->getDiscreteContactManager()->getCollisionMarginData().getDefaultCollisionMarginData(), 0.1, 1e-6);
  EXPECT_NEAR(env->getContinuousContactManager()->getCollisionMarginData().getDefaultCollisionMarginData(), 0.1, 1e-6);
}

template <typename S>
void runChangeJointLimitsCommandTest(bool use_command = false)
{
  // Get the environment
  auto env = getEnvironment<S>();
  EXPECT_EQ(env->getRevision(), 2);
  EXPECT_EQ(env->getCommandHistory().size(), 2);

  {
    JointLimits::ConstPtr limits = env->getJointLimits("not_in_graph");
    EXPECT_TRUE(limits == nullptr);
  }

  {  // Note that this will fail artificially if the urdf is changed for some reason
    JointLimits::ConstPtr limits = env->getJointLimits("joint_a1");
    EXPECT_NEAR(limits->lower, -2.9668, 1e-5);
    EXPECT_NEAR(limits->upper, 2.9668, 1e-5);
    EXPECT_NEAR(limits->velocity, 1.4834, 1e-5);
    EXPECT_NEAR(limits->effort, 0, 1e-5);
  }

  {
    double new_lower = 1.0;
    double new_upper = 2.0;
    double new_velocity = 3.0;
    double new_acceleration = 4.0;

    int revision = env->getRevision();
    if (use_command)
    {
      auto cmd_jpl = std::make_shared<ChangeJointPositionLimitsCommand>("joint_a1", new_lower, new_upper);
      EXPECT_EQ(cmd_jpl->getType(), CommandType::CHANGE_JOINT_POSITION_LIMITS);
      EXPECT_EQ(cmd_jpl->getLimits().size(), 1);
      auto it_jpl = cmd_jpl->getLimits().find("joint_a1");
      EXPECT_TRUE(it_jpl != cmd_jpl->getLimits().end());
      EXPECT_EQ(it_jpl->first, "joint_a1");
      EXPECT_NEAR(it_jpl->second.first, new_lower, 1e-6);
      EXPECT_NEAR(it_jpl->second.second, new_upper, 1e-6);
      EXPECT_TRUE(env->applyCommand(cmd_jpl));
      EXPECT_EQ(revision + 1, env->getRevision());
      EXPECT_EQ(revision + 1, env->getCommandHistory().size());
      EXPECT_EQ(env->getCommandHistory().back(), cmd_jpl);

      auto cmd_jvl = std::make_shared<ChangeJointVelocityLimitsCommand>("joint_a1", new_velocity);
      EXPECT_EQ(cmd_jvl->getType(), CommandType::CHANGE_JOINT_VELOCITY_LIMITS);
      EXPECT_EQ(cmd_jvl->getLimits().size(), 1);
      auto it_jvl = cmd_jvl->getLimits().find("joint_a1");
      EXPECT_TRUE(it_jvl != cmd_jvl->getLimits().end());
      EXPECT_EQ(it_jvl->first, "joint_a1");
      EXPECT_NEAR(it_jvl->second, new_velocity, 1e-6);
      EXPECT_TRUE(env->applyCommand(cmd_jvl));
      EXPECT_EQ(revision + 2, env->getRevision());
      EXPECT_EQ(revision + 2, env->getCommandHistory().size());
      EXPECT_EQ(env->getCommandHistory().back(), cmd_jvl);

      auto cmd_jal = std::make_shared<ChangeJointAccelerationLimitsCommand>("joint_a1", new_acceleration);
      EXPECT_EQ(cmd_jal->getType(), CommandType::CHANGE_JOINT_ACCELERATION_LIMITS);
      EXPECT_EQ(cmd_jal->getLimits().size(), 1);
      auto it_jal = cmd_jal->getLimits().find("joint_a1");
      EXPECT_TRUE(it_jal != cmd_jal->getLimits().end());
      EXPECT_EQ(it_jal->first, "joint_a1");
      EXPECT_NEAR(it_jal->second, new_acceleration, 1e-6);
      EXPECT_TRUE(env->applyCommand(cmd_jal));
      EXPECT_EQ(revision + 3, env->getRevision());
      EXPECT_EQ(revision + 3, env->getCommandHistory().size());
      EXPECT_EQ(env->getCommandHistory().back(), cmd_jal);
    }
    else
    {
      env->changeJointPositionLimits("joint_a1", new_lower, new_upper);
      EXPECT_EQ(revision + 1, env->getRevision());
      EXPECT_EQ(revision + 1, env->getCommandHistory().size());
      env->changeJointVelocityLimits("joint_a1", new_velocity);
      EXPECT_EQ(revision + 2, env->getRevision());
      EXPECT_EQ(revision + 2, env->getCommandHistory().size());
      env->changeJointAccelerationLimits("joint_a1", new_acceleration);
      EXPECT_EQ(revision + 3, env->getRevision());
      EXPECT_EQ(revision + 3, env->getCommandHistory().size());
    }

    // Check that the environment returns the correct limits
    JointLimits new_limits = *(env->getJointLimits("joint_a1"));
    EXPECT_NEAR(new_limits.lower, new_lower, 1e-5);
    EXPECT_NEAR(new_limits.upper, new_upper, 1e-5);
    EXPECT_NEAR(new_limits.velocity, new_velocity, 1e-5);
    EXPECT_NEAR(new_limits.acceleration, new_acceleration, 1e-5);

    // Check that the manipulator correctly set the limits
    auto kin = env->getManipulatorManager()->getFwdKinematicSolver("manipulator");
    EXPECT_NEAR(kin->getLimits().joint_limits(0, 0), new_lower, 1e-5);
    EXPECT_NEAR(kin->getLimits().joint_limits(0, 1), new_upper, 1e-5);
    EXPECT_NEAR(kin->getLimits().velocity_limits(0), new_velocity, 1e-5);
    EXPECT_NEAR(kin->getLimits().acceleration_limits(0), new_acceleration, 1e-5);
  }
  {
    Eigen::MatrixX2d original;
    Eigen::MatrixX2d new_limits;
    {
      auto kin = env->getManipulatorManager()->getFwdKinematicSolver("manipulator");
      original = kin->getLimits().joint_limits;
    }
    env->changeJointPositionLimits("joint_a1", 0, 1);
    {
      auto kin = env->getManipulatorManager()->getFwdKinematicSolver("manipulator");
      new_limits = kin->getLimits().joint_limits;
    }
    EXPECT_EQ(original.rows(), new_limits.rows());
    EXPECT_EQ(original.cols(), new_limits.cols());
  }
}

template <typename S>
void runChangeJointOriginCommandTest(bool use_command = false)
{
  // Get the environment
  auto env = getEnvironment<S>();
  EXPECT_EQ(env->getRevision(), 2);
  EXPECT_EQ(env->getCommandHistory().size(), 2);

  const std::string link_name1 = "link_n1";
  const std::string joint_name1 = "joint_n1";
  Link link_1(link_name1);

  Joint joint_1(joint_name1);
  joint_1.parent_link_name = env->getRootLinkName();
  joint_1.child_link_name = link_name1;
  joint_1.type = JointType::FIXED;

  env->addLink(link_1, joint_1);
  EXPECT_EQ(env->getRevision(), 3);
  EXPECT_EQ(env->getCommandHistory().size(), 3);
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name1) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name1) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name1) == env->getCurrentState()->joints.end());

  env->getSceneGraph()->saveDOT(tesseract_common::getTempPath() + "before_change_joint_origin_unit.dot");

  Eigen::Isometry3d new_origin = Eigen::Isometry3d::Identity();
  new_origin.translation()(0) += 1.234;

  if (use_command)
  {
    auto cmd = std::make_shared<ChangeJointOriginCommand>(joint_name1, new_origin);
    EXPECT_EQ(cmd->getType(), CommandType::CHANGE_JOINT_ORIGIN);
    EXPECT_EQ(cmd->getJointName(), joint_name1);
    EXPECT_TRUE(new_origin.isApprox(cmd->getOrigin()));
    EXPECT_TRUE(env->applyCommand(cmd));
    EXPECT_EQ(env->getCommandHistory().back(), cmd);
  }
  else
  {
    EXPECT_TRUE(env->changeJointOrigin(joint_name1, new_origin));
  }
  EXPECT_EQ(env->getRevision(), 4);
  EXPECT_EQ(env->getCommandHistory().size(), 4);

  // Check that the origin got updated
  EXPECT_TRUE(env->getJoint(joint_name1)->parent_to_joint_origin_transform.isApprox(new_origin));
  EXPECT_TRUE(env->getCurrentState()->link_transforms.at(link_name1).isApprox(new_origin));
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.at(joint_name1).isApprox(new_origin));

  env->getSceneGraph()->saveDOT(tesseract_common::getTempPath() + "after_change_joint_origin_unit.dot");
}

template <typename S>
void runChangeLinkCollisionEnabledCommandTest(bool use_command = false)
{
  // Get the environment
  /** @todo update contact manager to have function to check collision object enabled state */
  auto env = getEnvironment<S>();
  EXPECT_EQ(env->getRevision(), 2);
  EXPECT_EQ(env->getCommandHistory().size(), 2);

  std::string link_name = "link_1";
  EXPECT_TRUE(env->getSceneGraph()->getLinkCollisionEnabled(link_name));

  if (use_command)
  {
    auto cmd = std::make_shared<ChangeLinkCollisionEnabledCommand>(link_name, false);
    EXPECT_TRUE(cmd != nullptr);
    EXPECT_EQ(cmd->getType(), CommandType::CHANGE_LINK_COLLISION_ENABLED);
    EXPECT_EQ(cmd->getEnabled(), false);
    EXPECT_EQ(cmd->getLinkName(), link_name);
    EXPECT_TRUE(env->applyCommand(cmd));
    EXPECT_EQ(env->getCommandHistory().back(), cmd);
  }
  else
  {
    EXPECT_TRUE(env->setLinkCollisionEnabled(link_name, false));
  }

  EXPECT_EQ(env->getRevision(), 3);
  EXPECT_EQ(env->getCommandHistory().size(), 3);
  EXPECT_FALSE(env->getSceneGraph()->getLinkCollisionEnabled(link_name));
}

template <typename S>
void runChangeLinkVisibilityCommandTest(bool use_command = false)
{
  // Get the environment
  /** @todo update contact manager to have function to check collision object enabled state */
  auto env = getEnvironment<S>();
  EXPECT_EQ(env->getRevision(), 2);
  EXPECT_EQ(env->getCommandHistory().size(), 2);

  std::string link_name = "link_1";
  EXPECT_TRUE(env->getSceneGraph()->getLinkVisibility(link_name));

  if (use_command)
  {
    auto cmd = std::make_shared<ChangeLinkVisibilityCommand>(link_name, false);
    EXPECT_TRUE(cmd != nullptr);
    EXPECT_EQ(cmd->getType(), CommandType::CHANGE_LINK_VISIBILITY);
    EXPECT_EQ(cmd->getEnabled(), false);
    EXPECT_EQ(cmd->getLinkName(), link_name);
    EXPECT_TRUE(env->applyCommand(cmd));
    EXPECT_EQ(env->getCommandHistory().back(), cmd);
  }
  else
  {
    EXPECT_TRUE(env->setLinkVisibility(link_name, false));
  }

  EXPECT_EQ(env->getRevision(), 3);
  EXPECT_EQ(env->getCommandHistory().size(), 3);
  EXPECT_FALSE(env->getSceneGraph()->getLinkVisibility(link_name));
}

template <typename S>
void runChangePairContactMarginCommandTest()
{
  std::string link_name1 = "link_1";
  std::string link_name2 = "link_2";
  double margin = 0.1;

  // Get the environment
  auto env = getEnvironment<S>();
  EXPECT_EQ(env->getRevision(), 2);
  EXPECT_EQ(env->getCommandHistory().size(), 2);
  EXPECT_NEAR(
      env->getDiscreteContactManager()->getCollisionMarginData().getPairCollisionMarginData(link_name1, link_name2),
      0.0,
      1e-6);
  EXPECT_NEAR(
      env->getContinuousContactManager()->getCollisionMarginData().getPairCollisionMarginData(link_name1, link_name2),
      0.0,
      1e-6);

  std::unordered_map<tesseract_common::LinkNamesPair, double, tesseract_common::PairHash> link_pair_margin;
  auto ordered_pair = tesseract_common::makeOrderedLinkPair(link_name1, link_name2);
  link_pair_margin[ordered_pair] = margin;
  auto cmd = std::make_shared<ChangePairContactMarginCommand>(link_pair_margin);
  EXPECT_TRUE(cmd != nullptr);
  EXPECT_EQ(cmd->getType(), CommandType::CHANGE_PAIR_CONTACT_MARGIN);
  EXPECT_EQ(cmd->getPairCollisionMarginData().size(), 1);
  EXPECT_NEAR(cmd->getPairCollisionMarginData().at(ordered_pair), margin, 1e-6);
  EXPECT_TRUE(env->applyCommand(cmd));

  EXPECT_EQ(env->getRevision(), 3);
  EXPECT_EQ(env->getCommandHistory().size(), 3);
  EXPECT_EQ(env->getCommandHistory().back(), cmd);
  EXPECT_NEAR(
      env->getDiscreteContactManager()->getCollisionMarginData().getPairCollisionMarginData(link_name1, link_name2),
      margin,
      1e-6);
  EXPECT_NEAR(
      env->getContinuousContactManager()->getCollisionMarginData().getPairCollisionMarginData(link_name1, link_name2),
      margin,
      1e-6);
}

template <typename S>
void runMoveJointCommandTest(bool use_command = false)
{
  // Get the environment
  auto env = getEnvironment<S>();
  EXPECT_EQ(env->getRevision(), 2);
  EXPECT_EQ(env->getCommandHistory().size(), 2);

  const std::string link_name1 = "link_n1";
  const std::string link_name2 = "link_n2";
  const std::string joint_name1 = "joint_n1";
  const std::string joint_name2 = "joint_n2";
  Link link_1(link_name1);
  Link link_2(link_name2);

  Joint joint_1(joint_name1);
  joint_1.parent_link_name = env->getRootLinkName();
  joint_1.child_link_name = link_name1;
  joint_1.type = JointType::FIXED;

  Joint joint_2(joint_name2);
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = link_name1;
  joint_2.child_link_name = link_name2;
  joint_2.type = JointType::FIXED;

  env->addLink(link_1, joint_1);
  EXPECT_EQ(env->getRevision(), 3);
  EXPECT_EQ(env->getCommandHistory().size(), 3);

  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name1) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name1) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name1) == env->getCurrentState()->joints.end());

  env->addLink(link_2, joint_2);
  EXPECT_EQ(env->getRevision(), 4);
  EXPECT_EQ(env->getCommandHistory().size(), 4);

  std::vector<std::string> link_names = env->getLinkNames();
  std::vector<std::string> joint_names = env->getJointNames();
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name1) != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name2) != link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) != joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) != joint_names.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name1) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name1) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name1) == env->getCurrentState()->joints.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name2) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name2) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name2) == env->getCurrentState()->joints.end());

  env->getSceneGraph()->saveDOT(tesseract_common::getTempPath() + "before_move_joint_unit.dot");

  if (use_command)
  {
    auto cmd = std::make_shared<MoveJointCommand>(joint_name1, "tool0");
    EXPECT_TRUE(cmd != nullptr);
    EXPECT_EQ(cmd->getType(), CommandType::MOVE_JOINT);
    EXPECT_EQ(cmd->getJointName(), joint_name1);
    EXPECT_EQ(cmd->getParentLink(), "tool0");
    EXPECT_TRUE(env->applyCommand(cmd));
    EXPECT_EQ(env->getCommandHistory().back(), cmd);
  }
  else
  {
    EXPECT_TRUE(env->moveJoint(joint_name1, "tool0"));
  }
  link_names = env->getLinkNames();
  joint_names = env->getJointNames();
  EXPECT_TRUE(env->getJoint(joint_name1)->parent_link_name == "tool0");
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name1) != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name2) != link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) != joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) != joint_names.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name1) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name1) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name1) == env->getCurrentState()->joints.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name2) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name2) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name2) == env->getCurrentState()->joints.end());

  env->getSceneGraph()->saveDOT(tesseract_common::getTempPath() + "after_move_joint_unit.dot");
}

template <typename S>
void runMoveLinkCommandTest(bool use_command = false)
{
  // Get the environment
  auto env = getEnvironment<S>();
  EXPECT_EQ(env->getRevision(), 2);
  EXPECT_EQ(env->getCommandHistory().size(), 2);

  const std::string link_name1 = "link_n1";
  const std::string link_name2 = "link_n2";
  const std::string joint_name1 = "joint_n1";
  const std::string joint_name2 = "joint_n2";
  Link link_1(link_name1);
  Link link_2(link_name2);

  Joint joint_1(joint_name1);
  joint_1.parent_link_name = env->getRootLinkName();
  joint_1.child_link_name = link_name1;
  joint_1.type = JointType::FIXED;

  Joint joint_2(joint_name2);
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = link_name1;
  joint_2.child_link_name = link_name2;
  joint_2.type = JointType::FIXED;

  env->addLink(link_1, joint_1);
  EXPECT_EQ(env->getRevision(), 3);
  EXPECT_EQ(env->getCommandHistory().size(), 3);

  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name1) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name1) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name1) == env->getCurrentState()->joints.end());

  env->addLink(link_2, joint_2);
  EXPECT_EQ(env->getRevision(), 4);
  EXPECT_EQ(env->getCommandHistory().size(), 4);

  std::vector<std::string> link_names = env->getLinkNames();
  std::vector<std::string> joint_names = env->getJointNames();
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name1) != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name2) != link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) != joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) != joint_names.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name1) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name1) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name1) == env->getCurrentState()->joints.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name2) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name2) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name2) == env->getCurrentState()->joints.end());

  env->getSceneGraph()->saveDOT(tesseract_common::getTempPath() + "before_move_link_unit.dot");

  std::string moved_joint_name = joint_name1 + "_moved";
  Joint move_link_joint = joint_1.clone(moved_joint_name);
  move_link_joint.parent_link_name = "tool0";
  if (use_command)
  {
    auto cmd = std::make_shared<MoveLinkCommand>(move_link_joint);
    EXPECT_TRUE(cmd != nullptr);
    EXPECT_EQ(cmd->getType(), CommandType::MOVE_LINK);
    EXPECT_TRUE(cmd->getJoint() != nullptr);
    EXPECT_TRUE(env->applyCommand(cmd));
    EXPECT_EQ(env->getCommandHistory().back(), cmd);
  }
  else
  {
    EXPECT_TRUE(env->moveLink(move_link_joint));
  }
  link_names = env->getLinkNames();
  joint_names = env->getJointNames();
  EXPECT_TRUE(env->getJoint(moved_joint_name)->parent_link_name == "tool0");
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name1) != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name2) != link_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), moved_joint_name) != joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) != joint_names.end());

  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name1) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name1) ==
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(moved_joint_name) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name1) == env->getCurrentState()->joints.end());
  EXPECT_TRUE(env->getCurrentState()->link_transforms.find(link_name2) !=
              env->getCurrentState()->link_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joint_transforms.find(joint_name2) !=
              env->getCurrentState()->joint_transforms.end());
  EXPECT_TRUE(env->getCurrentState()->joints.find(joint_name2) == env->getCurrentState()->joints.end());

  env->getSceneGraph()->saveDOT(tesseract_common::getTempPath() + "after_move_link_unit.dot");
}

template <typename S>
void runCurrentStatePreservedWhenEnvChangesTest()
{
  // Get the environment
  auto env = getEnvironment<S>();

  // Set the initial state of the robot
  std::unordered_map<std::string, double> joint_states;
  joint_states["joint_a1"] = 0.0;
  joint_states["joint_a2"] = 0.0;
  joint_states["joint_a3"] = 0.0;
  joint_states["joint_a4"] = -1.57;
  joint_states["joint_a5"] = 0.0;
  joint_states["joint_a6"] = 0.0;
  joint_states["joint_a7"] = 0.0;
  env->setState(joint_states);

  EnvState::ConstPtr current_state = env->getCurrentState();
  for (auto& joint_state : joint_states)
  {
    EXPECT_NEAR(current_state->joints.at(joint_state.first), joint_state.second, 1e-5);
  }

  Link link("link_n1");

  Joint joint("joint_n1");
  joint.parent_link_name = env->getRootLinkName();
  joint.child_link_name = "link_n1";
  joint.type = JointType::FIXED;

  env->addLink(link, joint);

  current_state = env->getCurrentState();
  for (auto& joint_state : joint_states)
  {
    EXPECT_NEAR(current_state->joints.at(joint_state.first), joint_state.second, 1e-5);
  }
}

template <typename S>
void runApplyCommandsTest()
{
  // Get the environment
  auto env = getEnvironment<S>();

  const std::string link_name1 = "link_n1";
  const std::string link_name2 = "link_n2";
  const std::string joint_name1 = "joint_n1";
  auto link_1 = std::make_shared<Link>(link_name1);
  auto link_2 = std::make_shared<Link>(link_name2);

  auto joint_1 = std::make_shared<Joint>(joint_name1);
  joint_1->parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1->parent_link_name = "base_link";
  joint_1->child_link_name = link_name1;
  joint_1->type = JointType::FIXED;

  // Empty or invalid
  {
    Commands commands;
    EXPECT_TRUE(env->applyCommands(commands));
    commands.push_back(nullptr);
    EXPECT_FALSE(env->applyCommands(commands));
  }

  // Add
  {
    {
      Commands commands{ std::make_shared<AddCommand>(*link_1, *joint_1) };
      EXPECT_TRUE(env->applyCommands(commands));
      EXPECT_FALSE(env->applyCommands(commands));

      std::vector<std::string> link_names = env->getLinkNames();
      std::vector<std::string> joint_names = env->getJointNames();
      EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name1) != link_names.end());
      EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) != joint_names.end());
    }
    {
      Commands commands{ std::make_shared<AddCommand>(*link_2) };
      EXPECT_TRUE(env->applyCommands(commands));
      std::vector<std::string> link_names = env->getLinkNames();
      std::vector<std::string> joint_names = env->getJointNames();
      EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), link_name2) != link_names.end());
      EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), "joint_" + link_name2) != joint_names.end());
    }
  }
  /// @todo Add tests for applying commands to the environment
}

template <typename S>
void runEnvCloneTest()
{
  // Get the environment
  auto env = getEnvironment<S>();

  // Clone the environment
  auto clone = env->clone();

  // Check the basics
  EXPECT_EQ(clone->getName(), env->getName());
  EXPECT_EQ(clone->getRevision(), env->getRevision());

  // Check that all links got cloned
  std::vector<std::string> link_names = env->getLinkNames();
  std::vector<std::string> clone_link_names = clone->getLinkNames();
  for (const auto& name : link_names)
    EXPECT_TRUE(std::find(clone_link_names.begin(), clone_link_names.end(), name) != clone_link_names.end());

  // Check that all joints got cloned
  std::vector<std::string> joint_names = env->getJointNames();
  std::vector<std::string> clone_joint_names = clone->getJointNames();
  for (const auto& name : joint_names)
    EXPECT_TRUE(std::find(clone_joint_names.begin(), clone_joint_names.end(), name) != clone_joint_names.end());

  // Check that the command history is preserved
  auto history = env->getCommandHistory();
  auto clone_history = clone->getCommandHistory();
  ASSERT_EQ(history.size(), clone_history.size());
  for (std::size_t i = 0; i < history.size(); i++)
  {
    EXPECT_EQ(history[i]->getType(), clone_history[i]->getType());
  }

  // Check active links
  std::vector<std::string> active_link_names = env->getActiveLinkNames();
  std::vector<std::string> clone_active_link_names = clone->getActiveLinkNames();
  for (const auto& name : active_link_names)
    EXPECT_TRUE(std::find(clone_active_link_names.begin(), clone_active_link_names.end(), name) !=
                clone_active_link_names.end());

  // Check active joints
  std::vector<std::string> active_joint_names = env->getActiveJointNames();
  std::vector<std::string> clone_active_joint_names = clone->getActiveJointNames();
  for (const auto& name : active_joint_names)
    EXPECT_TRUE(std::find(clone_active_joint_names.begin(), clone_active_joint_names.end(), name) !=
                clone_active_joint_names.end());

  // Check that the state is preserved
  Eigen::VectorXd joint_vals = env->getCurrentState()->getJointValues(active_joint_names);
  Eigen::VectorXd clone_joint_vals = clone->getCurrentState()->getJointValues(active_joint_names);
  EXPECT_TRUE(joint_vals.isApprox(clone_joint_vals));
}

template <typename S>
void runEnvSetStateTest()
{
  // Get the environment
  auto env = getEnvironment<S>();

  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
  Eigen::Isometry3d pose;
  Eigen::VectorXd jvals;
  jvals.resize(7);
  jvals.setZero();
  std::vector<std::string> joint_names = { "base_link-base", "joint_a1", "joint_a2", "joint_a3",      "joint_a4",
                                           "joint_a5",       "joint_a6", "joint_a7", "joint_a7-tool0" };
  std::vector<std::string> link_names = { "base",   "base_link", "link_1", "link_2", "link_3",
                                          "link_4", "link_5",    "link_6", "link_7", "tool0" };
  std::vector<std::string> active_joint_names = { "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                                  "joint_a5", "joint_a6", "joint_a7" };

  std::vector<EnvState::Ptr> states;

  env->setState(active_joint_names, jvals);
  states.push_back(std::make_shared<EnvState>(*(env->getCurrentState())));
  states.push_back(env->getState(active_joint_names, jvals));

  for (auto& current_state : states)
  {
    // Check joints and links size
    EXPECT_EQ(current_state->joint_transforms.size(), 9);
    EXPECT_EQ(current_state->link_transforms.size(), 10);
    EXPECT_EQ(current_state->joints.size(), 7);

    // Check joints and links names
    for (const auto& joint_name : joint_names)
    {
      EXPECT_TRUE(current_state->joint_transforms.find(joint_name) != current_state->joint_transforms.end());
    }

    for (const auto& link_name : link_names)
    {
      EXPECT_TRUE(current_state->link_transforms.find(link_name) != current_state->link_transforms.end());
    }

    for (const auto& joint_name : active_joint_names)
    {
      EXPECT_TRUE(current_state->joints.find(joint_name) != current_state->joints.end());
    }

    EXPECT_TRUE(current_state->link_transforms["base_link"].isApprox(Eigen::Isometry3d::Identity()));

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
      EXPECT_TRUE(current_state->link_transforms["link_1"].isApprox(result));
    }

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity() * Eigen::Translation3d(-0.00043624, 0, 0.36);
      EXPECT_TRUE(current_state->link_transforms["link_2"].isApprox(result));
    }

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity() * Eigen::Translation3d(-0.00043624, 0, 0.36);
      EXPECT_TRUE(current_state->link_transforms["link_3"].isApprox(result));
    }

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.36 + 0.42);
      EXPECT_TRUE(current_state->link_transforms["link_4"].isApprox(result));
    }

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.36 + 0.42);
      EXPECT_TRUE(current_state->link_transforms["link_5"].isApprox(result));
    }

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.36 + 0.42 + 0.4);
      EXPECT_TRUE(current_state->link_transforms["link_6"].isApprox(result));
    }

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.36 + 0.42 + 0.4);
      EXPECT_TRUE(current_state->link_transforms["link_7"].isApprox(result));
    }

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 1.306);
      EXPECT_TRUE(current_state->link_transforms["tool0"].isApprox(result));
    }
  }
}

template <typename S>
void runEnvSetStateTest2()
{
  // Get the environment
  auto env = getEnvironment<S>();

  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
  Eigen::Isometry3d pose;
  Eigen::VectorXd jvals;
  jvals.resize(7);
  jvals.setZero();
  std::vector<std::string> joint_names = { "base_link-base", "joint_a1", "joint_a2", "joint_a3",      "joint_a4",
                                           "joint_a5",       "joint_a6", "joint_a7", "joint_a7-tool0" };
  std::vector<std::string> link_names = { "base",   "base_link", "link_1", "link_2", "link_3",
                                          "link_4", "link_5",    "link_6", "link_7", "tool0" };
  std::vector<std::string> active_joint_names = { "joint_a1", "joint_a2", "joint_a3", "joint_a4",
                                                  "joint_a5", "joint_a6", "joint_a7" };

  Eigen::Vector3d axis(0, 1, 0);
  jvals(1) = M_PI_2;
  std::vector<EnvState::Ptr> states;

  env->setState(active_joint_names, jvals);
  states.push_back(std::make_shared<EnvState>(*(env->getCurrentState())));
  states.push_back(env->getState(active_joint_names, jvals));

  for (auto& current_state : states)
  {
    // Check joints and links size
    EXPECT_EQ(current_state->joint_transforms.size(), 9);
    EXPECT_EQ(current_state->link_transforms.size(), 10);
    EXPECT_EQ(current_state->joints.size(), 7);

    // Check joints and links names
    for (const auto& joint_name : joint_names)
    {
      EXPECT_TRUE(current_state->joint_transforms.find(joint_name) != current_state->joint_transforms.end());
    }

    for (const auto& link_name : link_names)
    {
      EXPECT_TRUE(current_state->link_transforms.find(link_name) != current_state->link_transforms.end());
    }

    int cnt = 0;
    for (const auto& joint_name : active_joint_names)
    {
      EXPECT_TRUE(current_state->joints.find(joint_name) != current_state->joints.end());
      EXPECT_NEAR(current_state->joints[joint_name], jvals(cnt++), 1e-5);
    }

    EXPECT_TRUE(current_state->link_transforms["base_link"].isApprox(Eigen::Isometry3d::Identity()));
    EXPECT_TRUE(current_state->link_transforms["base"].isApprox(Eigen::Isometry3d::Identity()));

    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
      EXPECT_TRUE(current_state->link_transforms["link_1"].isApprox(result));
    }

    {
      Eigen::Isometry3d result = Eigen::Translation3d(-0.00043624, 0, 0.36) * Eigen::AngleAxisd(M_PI_2, axis);
      EXPECT_TRUE(current_state->link_transforms["link_2"].isApprox(result, 1e-4));
    }

    {
      Eigen::Isometry3d result = Eigen::Translation3d(-0.00043624, 0, 0.36) * Eigen::AngleAxisd(M_PI_2, axis);
      EXPECT_TRUE(current_state->link_transforms["link_3"].isApprox(result, 1e-4));
    }

    {
      Eigen::Isometry3d result =
          Eigen::Translation3d(0.42 - 0.00043624, 0, 0.36 - 0.00043624) * Eigen::AngleAxisd(M_PI_2, axis);
      EXPECT_TRUE(current_state->link_transforms["link_4"].isApprox(result, 1e-4));
    }

    {
      Eigen::Isometry3d result =
          Eigen::Translation3d(0.42 - 0.00043624, 0, 0.36 - 0.00043624) * Eigen::AngleAxisd(M_PI_2, axis);
      EXPECT_TRUE(current_state->link_transforms["link_5"].isApprox(result, 1e-4));
    }

    {
      Eigen::Isometry3d result =
          Eigen::Translation3d(0.42 + 0.4 - 0.00043624, 0, 0.36 - 0.00043624) * Eigen::AngleAxisd(M_PI_2, axis);
      EXPECT_TRUE(current_state->link_transforms["link_6"].isApprox(result, 1e-4));
    }

    {
      Eigen::Isometry3d result =
          Eigen::Translation3d(0.42 + 0.4 - 0.00043624, 0, 0.36 - 0.00043624) * Eigen::AngleAxisd(M_PI_2, axis);
      EXPECT_TRUE(current_state->link_transforms["link_7"].isApprox(result, 1e-4));
    }

    {
      Eigen::Isometry3d result =
          Eigen::Translation3d(1.306 - 0.36 - 0.00043624, 0, 0.36 - 0.00043624) * Eigen::AngleAxisd(M_PI_2, axis);
      EXPECT_TRUE(current_state->link_transforms["tool0"].isApprox(result, 1e-4));
    }
  }
}

TEST(TesseractEnvironmentUnit, EnvCloneContactManagerUnit)  // NOLINT
{
  runContactManagerCloneTest<KDLStateSolver>();
  runContactManagerCloneTest<OFKTStateSolver>();
}

TEST(TesseractEnvironmentUnit, EnvAddAndRemoveAllowedCollisionCommandUnit)  // NOLINT
{
  runAddandRemoveAllowedCollisionCommandTest<KDLStateSolver>();
  runAddandRemoveAllowedCollisionCommandTest<OFKTStateSolver>();
}

TEST(TesseractEnvironmentUnit, EnvAddandRemoveLink)  // NOLINT
{
  runAddandRemoveLinkTest<KDLStateSolver>();
  runAddandRemoveLinkTest<OFKTStateSolver>();

  runAddandRemoveLinkTest<KDLStateSolver>(true);
  runAddandRemoveLinkTest<OFKTStateSolver>(true);
}

TEST(TesseractEnvironmentUnit, EnvAddKinematicsInformationCommandUnit)  // NOLINT
{
  runAddKinematicsInformationCommandTest<KDLStateSolver>();
  runAddKinematicsInformationCommandTest<OFKTStateSolver>();

  runAddKinematicsInformationCommandTest<KDLStateSolver>(true);
  runAddKinematicsInformationCommandTest<OFKTStateSolver>(true);
}

TEST(TesseractEnvironmentUnit, EnvAddSceneGraphCommandUnit)
{
  runAddSceneGraphCommandTest<KDLStateSolver>();
  runAddSceneGraphCommandTest<OFKTStateSolver>();

  runAddSceneGraphCommandTest<KDLStateSolver>(true);
  runAddSceneGraphCommandTest<OFKTStateSolver>(true);
}

TEST(TesseractEnvironmentUnit, EnvChangeDefaultContactMarginCommandUnit)
{
  runChangeDefaultContactMarginCommandTest<KDLStateSolver>();
  runChangeDefaultContactMarginCommandTest<OFKTStateSolver>();
}

TEST(TesseractEnvironmentUnit, EnvChangeJointLimitsCommandUnit)  // NOLINT
{
  runChangeJointOriginCommandTest<KDLStateSolver>();
  runChangeJointLimitsCommandTest<OFKTStateSolver>();

  runChangeJointLimitsCommandTest<KDLStateSolver>(true);
  runChangeJointLimitsCommandTest<OFKTStateSolver>(true);
}

TEST(TesseractEnvironmentUnit, EnvChangeJointOriginCommandUnit)  // NOLINT
{
  runChangeJointOriginCommandTest<KDLStateSolver>();
  runChangeJointOriginCommandTest<OFKTStateSolver>();

  runChangeJointOriginCommandTest<KDLStateSolver>(true);
  runChangeJointOriginCommandTest<OFKTStateSolver>(true);
}

TEST(TesseractEnvironmentUnit, EnvChangeLinkCollisionEnabledCommandUnit)  // NOLINT
{
  runChangeLinkCollisionEnabledCommandTest<KDLStateSolver>();
  runChangeLinkCollisionEnabledCommandTest<OFKTStateSolver>();

  runChangeLinkCollisionEnabledCommandTest<KDLStateSolver>(true);
  runChangeLinkCollisionEnabledCommandTest<OFKTStateSolver>(true);
}

TEST(TesseractEnvironmentUnit, EnvChangeLinkVisibilityCommandUnit)  // NOLINT
{
  runChangeLinkVisibilityCommandTest<KDLStateSolver>();
  runChangeLinkVisibilityCommandTest<OFKTStateSolver>();

  runChangeLinkVisibilityCommandTest<KDLStateSolver>(true);
  runChangeLinkVisibilityCommandTest<OFKTStateSolver>(true);
}

TEST(TesseractEnvironmentUnit, EnvChangePairContactMarginCommandUnit)  // NOLINT
{
  runChangePairContactMarginCommandTest<KDLStateSolver>();
  runChangePairContactMarginCommandTest<OFKTStateSolver>();
}

TEST(TesseractEnvironmentUnit, EnvMoveJointCommandUnit)  // NOLINT
{
  runMoveJointCommandTest<KDLStateSolver>();
  runMoveJointCommandTest<OFKTStateSolver>();

  runMoveJointCommandTest<KDLStateSolver>(true);
  runMoveJointCommandTest<OFKTStateSolver>(true);
}

TEST(TesseractEnvironmentUnit, EnvMoveLinkCommandUnit)  // NOLINT
{
  runMoveLinkCommandTest<KDLStateSolver>();
  runMoveLinkCommandTest<OFKTStateSolver>();

  runMoveLinkCommandTest<KDLStateSolver>(true);
  runMoveLinkCommandTest<OFKTStateSolver>(true);
}

TEST(TesseractEnvironmentUnit, EnvCurrentStatePreservedWhenEnvChanges)  // NOLINT
{
  runCurrentStatePreservedWhenEnvChangesTest<KDLStateSolver>();
  runCurrentStatePreservedWhenEnvChangesTest<OFKTStateSolver>();
}

TEST(TesseractEnvironmentUnit, EnvApplyCommands)  // NOLINT
{
  runApplyCommandsTest<KDLStateSolver>();
  runApplyCommandsTest<OFKTStateSolver>();
}

TEST(TesseractEnvironmentUnit, EnvClone)  // NOLINT
{
  runEnvCloneTest<KDLStateSolver>();
  runEnvCloneTest<OFKTStateSolver>();
}

TEST(TesseractEnvironmentUnit, EnvSetState)  // NOLINT
{
  runEnvSetStateTest<KDLStateSolver>();
  runEnvSetStateTest<OFKTStateSolver>();

  runEnvSetStateTest2<KDLStateSolver>();
  runEnvSetStateTest2<OFKTStateSolver>();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
