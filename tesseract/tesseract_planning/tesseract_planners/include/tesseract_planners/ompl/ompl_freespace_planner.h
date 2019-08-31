#ifndef TESSERACT_MOTION_PLANNERS_OMPL_FREESPACE_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_OMPL_FREESPACE_PLANNER_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/SimpleSetup.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_planners/core/planner.h>
#include <tesseract_planners/core/waypoint.h>

namespace tesseract_planners
{

template <typename PlannerSettingsType>
struct OMPLFreespacePlannerConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OMPLFreespacePlannerConfig() {}
  /** @brief Determines the constraint placed at the start of the trajectory */
  WaypointPtr start_waypoint;
  /** @brief Determines the constraint placed at the end of the trajectory */
  WaypointPtr end_waypoint;

  /** @brief If true, collision checking will be enabled. Default: true*/
  bool collision_check = true;
  /** @brief If true, use continuous collision checking */
  bool collision_continuous = true;
  /** @brief Max distance over which collisions are checked */
  double collision_safety_margin = 0.025;
  /** @brief Max planning time allowed */
  double planning_time = 5.0;
  /** @brief Simplify trajectory */
  bool simplify = true;
  /** @brief This scales the variables search space. Must be same size as number of joints.
   *         If empty it defaults to all ones */
  Eigen::VectorXd weights;

  /** @brief Set the resolution at which state validity needs to be
   * verified in order for a motion between two states to be considered valid. */
  double longest_valid_segment_fraction = 0.01;  // 1%

  /** @brief Planner settings */
  PlannerSettingsType settings;

  /** @brief Tesseract object. ***REQUIRED*** */
  tesseract::Tesseract::ConstPtr tesseract;

  /** @brief Manipulator used for pathplanning ***REQUIRED*** */
  std::string manipulator;

  /** @brief The ompl state validity checker. If nullptr it uses OMPLFreespacePlanner::isStateValid. */
  ompl::base::StateValidityCheckerFn svc;

  /** @brief The ompl motion validator. If nullptr and continuous collision checking enabled it used
   * ContinuousMotionValidator */
  ompl::base::MotionValidatorPtr mv;
};

/**
 * @brief This planner is intended to provide an easy to use interface to TrajOpt for freespace planning. It is made to
 * take a start and end point and automate the generation of the TrajOpt problem.
 */
template <typename PlannerType, typename PlannerSettingsType>
class OMPLFreespacePlanner : public BasicPlanner
{
public:
  /** @brief Construct a basic planner */
  OMPLFreespacePlanner(std::string name = "OMPL_FREESPACE");

  /**
   * @brief Set the configuration for the planner
   *
   * This must be called prior to calling solve.
   *
   * @param config The planners configuration
   * @return True if successful otherwise false
   */
    bool setConfiguration(const OMPLFreespacePlannerConfig<PlannerSettingsType>& config);

  /**
   * @brief Sets up the TrajOpt problem then solves using TrajOptMotionPlanner. It is intended to simplify setting up
   * and solving freespace motion problems.
   *
   * This planner (and the associated config passed to the setConfiguration) does not expose all of the available
   * configuration data in TrajOpt. This is done to simplify the interface. However, many problems may require more
   * specific setups. In that case, the source code for this planner may be used as an example.
   *
   * Note: This does not use the request information because everything is provided by config parameter
   *
   * @param response The results of the optimization. Primary output is the optimized joint trajectory
   * @return true if optimization complete
   */
  bool solve(PlannerResponse& response) override;

  bool terminate() override;

  void clear() override;


private:
  ompl::base::ValidStateSamplerPtr allocDiscreteValidStateSampler(const ompl::base::SpaceInformation *si) const;
  ompl::base::StateSamplerPtr allocWeightedRealVectorStateSampler(const ompl::base::StateSpace *space) const;

protected:
  /** @brief The ompl planner planner */
  std::shared_ptr<OMPLFreespacePlannerConfig<PlannerSettingsType>> config_;

  /** @brief The ompl planner motion validator */
  ompl::base::MotionValidatorPtr motion_validator_;

  /** @brief The ompl planner simple setup */
  ompl::geometric::SimpleSetupPtr simple_setup_;

  /** @brief The tesseract kinematics object */
  tesseract_kinematics::ForwardKinematicsConstPtr kin_;

  /** @brief The mapping of environment links to kinematics links */
  tesseract_environment::AdjacencyMapConstPtr adj_map_;

  tesseract_collision::DiscreteContactManagerPtr discrete_contact_manager_;
  tesseract_collision::ContinuousContactManagerPtr continuous_contact_manager_;
};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_FREESPACE_PLANNER_H
