#include <Eigen/Core>
#include <unsupported/Eigen/Splines>

#include <iostream>

/** @brief Uses Eigen/Splines to fit a 3rd order B-Spline to the input data and provides an operator for retrieving intermediate points
 *
 * Note: B-Spline may be lower order than requested for short input vectors */
class SplineFunction
{
public:
  SplineFunction(const Eigen::Ref<Eigen::VectorXd>& x_vec, const Eigen::Ref<Eigen::VectorXd>& y_vec)
    : x_min(x_vec.minCoeff())
    , x_max(x_vec.maxCoeff())
    ,
    // Scale x values to [0:1] and curve fit with a 3rd order B-Spline.
    spline_(Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(y_vec.transpose(),
                                                                        std::min<int>(x_vec.rows() - 1, 3),
                                                                        scaled_values(x_vec)))
  {
  }

  /** @brief Returns the y value at point x in the spline */
  double operator()(double x) const
  { 
    // x values need to be scaled to [0:1] in extraction as well.
    return spline_(scaled_value(x))(0);
  }

  /** @brief Returns a vector of interpolated y values for each x in the input vector */
  Eigen::VectorXd operator()(const Eigen::Ref<Eigen::VectorXd>& x_vec) const
  {
    return x_vec.unaryExpr([this](double x) { return spline_(scaled_value(x))(0); });
  }

private:
  /** @brief Scales value to [0:1] based on x_min and x_max */
  double scaled_value(double x) const { return (x - x_min) / (x_max - x_min); }

  /** @brief Scales vector such that each value is [0:1] based on x_min and x_max
   *
   * It is a requirement for Interpolate that x be on the interval [0:1] */
  Eigen::RowVectorXd scaled_values(const Eigen::Ref<Eigen::VectorXd>& x_vec) const
  {
    return x_vec.unaryExpr([this](double x) { return scaled_value(x); }).transpose();
  }

  /** @brief Minimum value in x_vec. Used to scale inputs to [0:1] */
  double x_min;
  /** @brief Maximum value in x_vec. Used to scale inputs to [0:1] */
  double x_max;

  /** @brief One dimensional Spline that is used to interpolate the data
   *
   * Note: operator(double) returns an array of points. In this case the 0th element is the y value*/
  Eigen::Spline<double, 1> spline_;
};

int main(int argc, char const* argv[])
{
  Eigen::VectorXd xvals(10);
  Eigen::VectorXd yvals(xvals.rows());

  xvals << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9;
  yvals << 0, 2, 4, 8, 16, 32, 64, 128, 256, 512;

  SplineFunction s(xvals, yvals);

  std::cout << "3.5 -> " << s(3.5) << std::endl;

  Eigen::VectorXd xvals2(20);
  xvals2 << 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9, 9.5;
  std::cout << s(xvals2) << std::endl;
}
