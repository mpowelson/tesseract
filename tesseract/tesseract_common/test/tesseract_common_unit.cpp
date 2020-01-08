#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_common/interpolators.h>

TEST(TesseractCommonUnit, isNumeric)  // NOLINT
{
  std::vector<std::string> true_test = { "1",     "1.5",  "-1",     "-1.5",  "1e-5",    "1e5",
                                         "-1e-5", "-1e5", "1.0e-5", "1.0e5", "-1.0e-5", "-1.0e5" };

  EXPECT_TRUE(tesseract_common::isNumeric(true_test));
  for (const auto& s : true_test)
  {
    EXPECT_TRUE(tesseract_common::isNumeric(s));
  }

  std::vector<std::string> false_test = { "a", "test sdfs", "1 2", "1.0 2.0", "+", "-", "=" };
  EXPECT_FALSE(tesseract_common::isNumeric(false_test));
  for (const auto& s : false_test)
  {
    EXPECT_FALSE(tesseract_common::isNumeric(s));
  }
}

TEST(TesseractCommonUnit, toNumeric)  // NOLINT
{
  std::vector<std::string> true_test = { "1",     "1.5",  "-1",     "-1.5",  "1e-5",    "1e5",
                                         "-1e-5", "-1e5", "1.0e-5", "1.0e5", "-1.0e-5", "-1.0e5" };

  std::vector<double> true_test_value = { 1, 1.5, -1, -1.5, 1e-5, 1e5, -1e-5, -1e5, 1.0e-5, 1.0e5, -1.0e-5, -1.0e5 };

  EXPECT_TRUE(tesseract_common::isNumeric(true_test));
  for (size_t i = 0; i < true_test.size(); ++i)
  {
    double value = 0;
    EXPECT_TRUE(tesseract_common::toNumeric<double>(true_test[i], value));
    EXPECT_NEAR(value, true_test_value[i], 1e-8);
  }

  std::vector<std::string> false_test = { "a", "test sdfs", "1 2", "1.0 2.0", "+", "-", "=" };
  EXPECT_FALSE(tesseract_common::isNumeric(false_test));
  for (const auto& s : false_test)
  {
    double value = 0;
    EXPECT_FALSE(tesseract_common::toNumeric(s, value));
    EXPECT_NEAR(value, 0, 1e-8);
  }
}

TEST(TesseractCommonUnit, splineFunctionNoDerivatives)  // NOLINT
{
  Eigen::VectorXd xvals(10);
  Eigen::VectorXd yvals(xvals.rows());

  xvals << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9;
  yvals = xvals.array().square();

  // Fit the spline
  tesseract_common::SplineFunction spline(xvals, yvals);

  // Check that input knots are hit exactly
  for (long ind = 0; ind < xvals.size(); ind++)
    EXPECT_NEAR(spline(xvals[ind]), yvals[ind], 1e-8);

  Eigen::VectorXd xvals_interp(8);
  xvals_interp << 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5;
  // Check that the intermediate points are within 1% for a quadratic (they should be really close)
  for (long ind = 0; ind < xvals_interp.size(); ind++)
  {
    double gt = xvals_interp[ind] * xvals_interp[ind];
    EXPECT_NEAR(spline(xvals_interp[ind]), gt, gt * 0.01 + 1e-8);
  }
}

TEST(TesseractCommonUnit, splineFunctionWithDerivatives)  // NOLINT
{
  Eigen::VectorXd xvals(10);
  Eigen::VectorXd yvals(xvals.rows());

  xvals << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9;
  yvals = xvals.array().cube();

  Eigen::VectorXd derivatives(2);
  derivatives << 0., 0.;
  Eigen::VectorXi indices(2);
  indices << 0, static_cast<int>(xvals.size() - 1);

  // Fit the spline
  tesseract_common::SplineFunction spline(xvals, yvals, derivatives, indices);

  // Check that input knots are hit exactly
  for (long ind = 0; ind < xvals.size(); ind++)
    EXPECT_NEAR(spline(xvals[ind]), yvals[ind], 1e-8);

  // Note: Interpolating using derivatives is not tested here because it requires patches to Eigen. See note in class
  // documentation
}

TEST(TesseractCommonUnit, interpolateCubicSpline)  // NOLINT
{
  tesseract_common::TrajArray input_traj(10, 5);
  Eigen::VectorXd t_in = Eigen::VectorXd::LinSpaced(input_traj.rows(), 0.0, static_cast<double>(input_traj.rows() - 1));
  input_traj.col(0) = t_in.array().square();
  input_traj.col(1) = t_in.array().cube();
  input_traj.col(2) = t_in.array().sin();
  input_traj.col(3) = t_in.array().cos();
  input_traj.col(4) = t_in.array().exp();
  std::cout << "Input Trajectory: \n" << input_traj << std::endl;

  const int result_length = 46;
  tesseract_common::TrajArray results = tesseract_common::interpolateCubicSpline(input_traj, result_length);
  std::cout << "Spline Results: \n" << results << std::endl;

  EXPECT_EQ(results.rows(), result_length);
  EXPECT_EQ(results.cols(), input_traj.cols());

  // Check that all of the knots are hit exactly
  for (long ind = 0; ind < input_traj.rows(); ind++)
  {
    EXPECT_NEAR(input_traj.col(0)[ind], results.col(0)[ind * 5], 1e-8);
    EXPECT_NEAR(input_traj.col(1)[ind], results.col(1)[ind * 5], 1e-8);
    EXPECT_NEAR(input_traj.col(2)[ind], results.col(2)[ind * 5], 1e-8);
    EXPECT_NEAR(input_traj.col(3)[ind], results.col(3)[ind * 5], 1e-8);
    EXPECT_NEAR(input_traj.col(4)[ind], results.col(4)[ind * 5], 1e-8);
  }

  // Check that the intermediate points are within a small percentage. The polynomials should be really close
  Eigen::VectorXd t_interp =
      Eigen::VectorXd::LinSpaced(results.rows(), 0.0, static_cast<double>(input_traj.rows() - 1));
  for (long ind = 0; ind < results.rows(); ind++)
  {
    EXPECT_NEAR(t_interp.array().square()[ind], results.col(0)[ind], t_interp.array().square()[ind] * 0.01 + 1e-8);
    EXPECT_NEAR(t_interp.array().cube()[ind], results.col(1)[ind], t_interp.array().cube()[ind] * 0.01 + 1e-8);
    EXPECT_NEAR(t_interp.array().sin()[ind], results.col(2)[ind], 1.0 * 0.05 + 1e-8);
    EXPECT_NEAR(t_interp.array().cos()[ind], results.col(3)[ind], 1.0 * 0.05 + 1e-8);
    EXPECT_NEAR(t_interp.array().exp()[ind], results.col(4)[ind], t_interp.array().exp()[ind] * 0.10 + 1e-8);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
