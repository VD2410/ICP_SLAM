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
  cv::Mat matrix(scan->ranges,2, CV_32S);
  auto min_angle_ = utils.radianToDegree(scan->angle_min);
  auto max_angle_ = utils.radianToDegree(scan->angle_max);
  // while(scan->ranges){
  // 	matrix = sca
  // }

  angle_range = max_angle_ - min_angle_;
  inc = angle_range/scan.ranges.size();
  auto s=laser_scan.ranges;
  for(int i=0;s.size()>i;i++){
  	angle = i*inc + angle_min;
  	utils.polarToCartesian(s[i],angle,matrix[i][0],matrix[i][1]);
  }

  return matrix;

  printf(matrix);
	  


}

cv::Mat transformPointMat(tf::Transform transform, cv::Mat &point_mat)
{
  assert(point_mat.data);
  assert(!point_mat.empty());

  cv::Mat point_mat_homogeneous(3, point_mat.rows, CV_32F, cv::Scalar(1.0f));

  cv::Mat(point_mat.t()).copyTo(point_mat_homogeneous.rowRange(0, 2));

  auto T = transformToMatrix(transform);
  cv::Mat transformed_point_mat =  T * point_mat_homogeneous;
  return cv::Mat(transformed_point_mat.t()).colRange(0, 2);
}

} // namespace utils
} // namespace icp_slam