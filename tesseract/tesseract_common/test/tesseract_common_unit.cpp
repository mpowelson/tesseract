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
    EXPECT_NEAR(spline(xvals_interp[ind]), gt, gt * 0.01);
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
