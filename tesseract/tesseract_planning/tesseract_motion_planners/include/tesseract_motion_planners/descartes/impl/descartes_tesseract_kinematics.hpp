#include <tesseract_motion_planners/descartes/descartes_tesseract_kinematics.h>
#include <Eigen/Eigen>
#include <console_bridge/console.h>

namespace tesseract_motion_planners
{
template <typename FloatType>
bool DescartesTesseractKinematics<FloatType>::ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
                                                 std::vector<FloatType>& solution_set) const
{
  return ik(p, is_valid_fn_, redundant_sol_fn_, solution_set);
}

template <typename FloatType>
bool DescartesTesseractKinematics<FloatType>::ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
                                                 const descartes_light::IsValidFn<FloatType>& is_valid_fn,
                                                 const descartes_light::GetRedundantSolutionsFn<FloatType>& redundant_sol_fn,
                                                 std::vector<FloatType>& solution_set) const
{
//  // Convert to appropriate Eigen types
//  Eigen::Isometry3d p_double;
//  p_double = p.template cast<double>();
//  Eigen::VectorXd solution_eigen;

//  // Solve IK
//  bool success = tesseract_ik_->calcInvKin(solution_eigen, p_double, ik_seed_);

//  // Convert back to vector
//  std::vector<FloatType> solution_vec(solution_eigen.data(),
//                                      solution_eigen.data() + solution_eigen.rows() * solution_eigen.cols());
//  solution_set = solution_vec;

//  // Get redundant solutions
//  FloatType* sol[tesseract_ik_->numJoints()];
//  std::copy(solution_set.begin(), solution_set.end(), sol);
//  std::vector<FloatType> redundant_sol = redundant_sol_fn(*sol);

//  // Add redundant solutions if they are valid
//  if (is_valid_fn(redundant_sol.data()))
//    solution_set = redundant_sol;
//  solution_set.insert(std::end(solution_set), std::begin(redundant_sol), std::end(redundant_sol));

//  return success;
  return true;
}

template <typename FloatType>
bool DescartesTesseractKinematics<FloatType>::fk(const FloatType* pose,
                                                 Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const
{
  assert(pose);

  // Convert the Array to an Eigen VectorXd
  Eigen::VectorXd joints(tesseract_fk_->numJoints());
  for (int i = 0; static_cast<unsigned int>(i) < tesseract_fk_->numJoints(); i++)
    joints(i, 0) = pose[i];

  // Get the solution from the Tesseract Kinematics
  Eigen::Isometry3d solution_double;
  bool success = tesseract_fk_->calcFwdKin(solution_double, joints);

  // Cast from double to FloatType
  solution = solution_double.cast<FloatType>();
  return success;
}

template <typename FloatType>
int DescartesTesseractKinematics<FloatType>::dof() const
{
  assert(tesseract_fk_->numJoints() < std::numeric_limits<int>::max());
  return static_cast<int>(tesseract_fk_->numJoints());
}

template <typename FloatType>
void DescartesTesseractKinematics<FloatType>::analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const
{
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "AnalyzeIK: ", ";");

  std::stringstream ss;
  ss << p.matrix().format(CommaInitFmt);
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  std::string valid_fn_defined = "\tIs Valid Function: " + std::string(is_valid_fn_ ? "True" : "False");
  CONSOLE_BRIDGE_logInform(valid_fn_defined.c_str());
  std::string redundant_fn_defined = "\tIs Valid Function: " + std::string(is_valid_fn_ ? "True" : "False");
  CONSOLE_BRIDGE_logInform(redundant_fn_defined.c_str());

  std::vector<FloatType> solution_set;
  ik(p, nullptr, nullptr, solution_set);
  ss.clear();
  ss << "\tSampling without functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  solution_set.clear();
  ik(p, is_valid_fn_, nullptr, solution_set);
  ss.clear();
  ss << "\tSampling with only IsValid functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  solution_set.clear();
  ik(p, nullptr, redundant_sol_fn_, solution_set);
  ss.clear();
  ss << "\tSampling with only Redundant Solutions functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  solution_set.clear();
  ik(p, is_valid_fn_, redundant_sol_fn_, solution_set);
  ss.clear();
  ss << "\tSampling with both functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());
}

template <typename FloatType>
void DescartesTesseractKinematics<FloatType>::setIKSeed(
    const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1> >& seed)
{
  assert(seed.size() == dof());
  ik_seed_ = seed.template cast<double>();
}

template <typename FloatType>
void DescartesTesseractKinematics<FloatType>::setIKSeed(const std::vector<FloatType>& seed)
{
  assert(seed.size() == dof());
  std::vector<double> seed_copy;
  for (auto& i : seed)
    seed_copy.push_back(static_cast<double>(i));
  ik_seed_ = Eigen::Map<Eigen::VectorXd>(seed_copy.data(), seed_copy.size());
}
}  // namespace tesseract_motion_planners
