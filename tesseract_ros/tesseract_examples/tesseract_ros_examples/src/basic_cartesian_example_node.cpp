#include <Eigen/Core>
#include <unsupported/Eigen/Splines>
#include <tesseract_ros_examples/madplotlib.h>
#include <tesseract_common/interpolators.h>
#include <QApplication>

#include <iostream>



int main(int argc, char* argv[])
{
  Eigen::VectorXd xvals(10);
  Eigen::VectorXd yvals(xvals.rows());

  xvals << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9;
  yvals << 0, 1, 10, 9, 34, 25, 74, 49, 130, 81;

  Eigen::VectorXd derivatives(2);
  derivatives << 0., 0.;

  Eigen::VectorXi indices(2);
  indices << 0, xvals.size() - 1;

  tesseract_common::SplineFunction s(xvals, yvals, derivatives, indices);
//  SplineFunction s(xvals, yvals);

  std::cout << "3.5 -> " << s(3.5) << std::endl;

  Eigen::VectorXd xvals2(20);
  xvals2 << 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9, 9.5;
  Eigen::VectorXd results = s(xvals2);
  std::cout << results << std::endl;

  Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(1000, 1, 9);
  Eigen::VectorXd gt = t.array().square();

  Eigen::VectorXd spline = s(t);

  QApplication app(argc, argv);
  Madplotlib plt(false);
  plt.plot(t.cast<float>().array(), spline.cast<float>().array(), QString("."));
  plt.show();
//  plt.clear();
//  plt.plot(t.cast<float>().array(), gt.cast<float>().array(), color = QColor(0x008FD5), QString("label=ground_true"));
//  plt.show();
  plt.savefig("spline_results_deriv.png");
}
