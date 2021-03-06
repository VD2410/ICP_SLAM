//
// Created by rakesh on 17/08/18.
//

#include <icp_slam/utils.h>

namespace icp_slam
{
namespace utils
{

cv::Mat laserScanToPointMat(const sensor_msgs::LaserScanConstPtr &scan)
{
  cv::Mat scan_mat(scan->ranges.size(), 2, CV_32F);
  float current_scan_angle;

  for(int i = 0; i < scan->ranges.size(); i++){
    current_scan_angle = (scan->angle_min + i * scan->angle_increment);
    
    scan_mat.at<float>(i,0) = scan->ranges[i] * std::cos(current_scan_angle);
    scan_mat.at<float>(i,1) = scan->ranges[i] * std::sin(current_scan_angle);
  }
  return scan_mat;
}

cv::Mat transformPointMat(tf::Transform transform, cv::Mat &point_mat)
{
  assert(point_mat.data);
  assert(!point_mat.empty());

  cv::Mat point_mat_homogeneous(3, point_mat.rows, CV_32F, cv::Scalar(1.0f));

  cv::Mat(point_mat.t()).copyTo(point_mat_homogeneous.rowRange(0, 2));

  auto T = transformToMatrix(transform);
  cv::Mat transformed_point_mat = T * point_mat_homogeneous;
  return cv::Mat(transformed_point_mat.t()).colRange(0, 2);
}

} // namespace utils
} // namespace icp_slam