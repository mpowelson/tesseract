#include <Eigen/Eigen>
#include <iostream>
#include <math.h>

//Just for plotting
#include <tesseract_ros_examples/basic_cartesian_example.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_rosutils/plotting.h>
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM =
    "robot_description_semantic"; /**< Default ROS parameter for robot description */

#include <pcl_ros/point_cloud.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <trajopt/utils.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


int main(int argc, char** argv)
{
  // All of this is just to use the rviz plot axis tool
  ros::init(argc, argv, "rotation_example_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  tesseract_scene_graph::ResourceLocatorFn locator = tesseract_rosutils::locateResource;
  auto tesseract = std::make_shared<tesseract::Tesseract>();
  if (!tesseract->init(urdf_xml_string, srdf_xml_string, locator))
    return -1;
  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);

  // Create plotting tool
  tesseract_rosutils::ROSPlottingPtr plotter = std::make_shared<tesseract_rosutils::ROSPlotting>(tesseract->getEnvironment());
  plotter->waitForInput();

//  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);




  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "world";
  msg->width = 1;
//  msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));


  // Here is the actual example
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  plotter->plotAxis(target, 1.0);

  // Loop over some rotations and check that the error is negative if they are less than the tolerances
  double rx_res = 9.99;
  double ry_res = 9.99;
  double rz_res = 9.99;
  for (double rx = -180; rx < 180; rx += rx_res)
  {
    for (double ry = -180; ry < 180; ry += ry_res)
    {
      for (double rz = -180; rz < 180; rz += rz_res)
      {
//        double rz = 0.00001;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.rotate(Eigen::AngleAxisd(rz * M_PI / 180., Eigen::Vector3d(0, 0, 1)));
        pose.rotate(Eigen::AngleAxisd(ry * M_PI / 180., Eigen::Vector3d(0, 1, 0)));
        pose.rotate(Eigen::AngleAxisd(rx * M_PI / 180., Eigen::Vector3d(1, 0, 0)));

        Eigen::Isometry3d pose_delta = pose;
        Eigen::Vector3d pnt = trajopt::calcRotationalError(pose_delta.linear().matrix());
        msg->points.push_back (pcl::PointXYZ(pnt(0),pnt(1),pnt(2)));




        // eulerAngles(2,1,0) returns the rotation in ZYX Euler angles. Since these can be converted to fixed axis
        // rotations by reversing the order, this corresponds to XYZ rotations about the fixed target axis
        Eigen::Vector3d euler_delta = pose_delta.linear().eulerAngles(2, 1, 0);
        Eigen::Vector3d fixed_delta(euler_delta(2), euler_delta(1), euler_delta(0));
        Eigen::Vector3d fixed_delta_degrees = fixed_delta * 180. / M_PI;

        Eigen::Matrix3d rot = pose_delta.rotation().matrix();
        double rx_out = atan2(rot(2, 1), rot(2, 2)) * 180. / M_PI;
        double ry_out = atan2(-rot(2, 0), sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2))) * 180. / M_PI;
        double rz_out = atan2(rot(1, 0), rot(0, 0)) * 180. / M_PI;
        Eigen::Vector3d fixed_delta_degrees2(rx_out, ry_out, rz_out);

        Eigen::Vector3d out = fixed_delta_degrees;
        if(out(0) > -10 && out(0) <10 && out(1) > -45 && out(1) < 45 && out(2) > 0 && out(2) < 5)
          plotter->plotAxis(pose, 0.5);


//        if ( std::abs(out(0)-rx) > 1e-5 || std::abs(out(1)-ry) > 1e-5 || std::abs(out(2)-rz) > 1e-5)
//          std::cout << "rx_in/rx_out: " << rx << "/" << out(0) << "  ry_in/ry_out:" << ry << "/" << out(1) << "  rz_in/rz_out:" << rz << "/" << out(2) << std::endl;
      }
    }
  }
  msg->height = msg->points.size();
  pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
  pub.publish (msg);
  ROS_WARN("Done");
  ros::spin();
}
