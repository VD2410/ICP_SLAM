//
// Created by rakesh on 13/08/18.
//
#include <cmath>
#include <map>
#include <numeric>
#include <chrono>

#include <boost/atomic/atomic.hpp>
#include <opencv2/flann/miniflann.hpp>

#include <icp_slam/icp_slam.h>
#include <icp_slam/utils.h>
#include <icp_slam/config.h>
#include <typeinfo>
#include <iostream>

#define TIME_DIFF(tic, toc) ((std::chrono::duration<double, std::milli>((toc) - (tic))).count())

namespace icp_slam
{
  bool first_scan = true;
  bool max_itr = false;
  int i =0;
  int itr;

ICPSlam::ICPSlam(tfScalar max_keyframes_distance, tfScalar max_keyframes_angle, double max_keyframes_time)
  : max_keyframes_distance_(max_keyframes_distance),
    max_keyframes_angle_(max_keyframes_angle),
    max_keyframes_time_(max_keyframes_time),
    last_kf_laser_scan_(new sensor_msgs::LaserScan()),
    is_tracker_running_(false)
{
  last_kf_tf_odom_laser_.stamp_ = ros::Time(0);
}

bool ICPSlam::track(const sensor_msgs::LaserScanConstPtr &laser_scan,
                    const tf::StampedTransform &current_frame_tf_odom_laser,
                    tf::StampedTransform &tf_map_laser)
{
  if (is_tracker_running_){
    ROS_WARN_THROTTLE(1.0, "Couldn't track frame, tracker already running");
    return false;
  }

  if(first_scan){
    first_scan = false;
    *last_kf_laser_scan_ = *laser_scan;
    last_kf_tf_odom_laser_ = current_frame_tf_odom_laser;
  }

  // TODO: find the pose of laser in map frame
  // if a new keyframe is created, run ICP
  if(isCreateKeyframe(current_frame_tf_odom_laser, last_kf_tf_odom_laser_)){

    tf::Transform T_2_1 = last_kf_tf_odom_laser_.inverse() * current_frame_tf_odom_laser;
    tf::Transform new_T_2_1 = icpRegistration(last_kf_laser_scan_, laser_scan, T_2_1);
    tf_map_laser.setData(last_kf_tf_odom_laser_ * new_T_2_1);
    last_kf_tf_odom_laser_ = current_frame_tf_odom_laser;
    *last_kf_laser_scan_ = *laser_scan;
    return true;
  }else{
    tf::Transform T_2_1 = last_kf_tf_odom_laser_.inverse() * current_frame_tf_odom_laser;
    tf_map_laser.setData(last_kf_tf_odom_laser_ * T_2_1);
    return false;
  }
}

bool ICPSlam::isCreateKeyframe(const tf::StampedTransform &current_frame_tf, const tf::StampedTransform &last_kf_tf) const
{
  assert(current_frame_tf.frame_id_ == last_kf_tf.frame_id_);
  assert(current_frame_tf.child_frame_id_ == last_kf_tf.child_frame_id_);

  auto current_frame_tf_x = current_frame_tf.getOrigin().x();
  auto current_frame_tf_y = current_frame_tf.getOrigin().y();
  auto current_frame_position = (current_frame_tf_x, current_frame_tf_y);

  auto last_frame_tf_x = last_kf_tf.getOrigin().x();
  auto last_frame_tf_y = last_kf_tf.getOrigin().y();
  auto last_frame_position = (last_frame_tf_x, last_frame_tf_y);

  // TODO: check whether you want to create keyframe (based on max_keyframes_distance_, max_keyframes_angle_, max_keyframes_time_)
  auto keyframes_distance_ = cv::norm(current_frame_position - last_frame_position);
  auto keyframes_angle_ = tf::getYaw(current_frame_tf.getRotation()) - tf::getYaw(last_kf_tf.getRotation());
  auto keyframes_time_ =  current_frame_tf.stamp_.toSec() - last_kf_tf.stamp_.toSec();

  ROS_WARN_THROTTLE(1.0, "%f %f %f", keyframes_distance_, keyframes_angle_, keyframes_time_);

  if(keyframes_distance_ > max_keyframes_distance_ || keyframes_angle_ > max_keyframes_angle_ || keyframes_time_ > max_keyframes_time_){
    return true;
  }
  return false;
}

void ICPSlam::closestPoints(cv::Mat &point_mat1,
                            cv::Mat &point_mat2,
                            std::vector<int> &closest_indices,
                            std::vector<float> &closest_distances_2)
{
  // uses FLANN for K-NN e.g. http://www.morethantechnical.com/2010/06/06/iterative-closest-point-icp-with-opencv-w-code/
  closest_indices = std::vector<int>(point_mat1.rows, -1);
  closest_distances_2 = std::vector<float>(point_mat1.rows, -1);


  cv::Mat multi_channeled_mat1;
  cv::Mat multi_channeled_mat2;

  point_mat1.convertTo(multi_channeled_mat1, CV_32FC2);
  point_mat2.convertTo(multi_channeled_mat2, CV_32FC2);

  cv::flann::Index flann_index(multi_channeled_mat2, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees

  cv::Mat mat_indices(point_mat1.rows, 1, CV_32S);
  cv::Mat mat_dists(point_mat1.rows, 1, CV_32F);
  flann_index.knnSearch(multi_channeled_mat1, mat_indices, mat_dists, 1, cv::flann::SearchParams(64) );

  int* indices_ptr = mat_indices.ptr<int>(0);
  //float* dists_ptr = mat_dists.ptr<float>(0);
  for (int i=0;i<mat_indices.rows;++i) {
    closest_indices[i] = indices_ptr[i];
  }

  mat_dists.copyTo(cv::Mat(closest_distances_2));

  // ---------------------------- naive version ---------------------------- //
  // max allowed distance between corresponding points
//  const float max_distance = 0.5;
//
//  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
//  {
//    int closest_point_idx = -1;
//    float closest_distance_2 = std::pow(max_distance, 2.0f);
//
//    for (size_t j = 0, len_j = (size_t)point_mat2.rows; j < len_j; j++)
//    {
//      auto distance2 =
//        std::pow(point_mat2.at<float>(j, 0) - point_mat1.at<float>(i, 0), 2.0f)
//        + std::pow(point_mat2.at<float>(j, 1) - point_mat1.at<float>(i, 1), 2.0f);
//
//      if (distance2 < closest_distance_2)
//      {
//        closest_distance_2 = distance2;
//        closest_point_idx = (int)j;
//      }
//    }
//
//    if (closest_point_idx >= 0)
//    {
//      closest_indices[i] = closest_point_idx;
//      closest_distances_2[i] = closest_distance_2;
//    }
//  }
}

static int count = 0;

void ICPSlam::vizClosestPoints(cv::Mat &point_mat1,
                               cv::Mat &point_mat2,
                               const tf::Transform &T_2_1)
{
  i++;	
  assert(point_mat1.size == point_mat2.size);

  const float resolution = 0.005;

  float *float_array = (float*)(point_mat1.data);
  float size_m = std::accumulate(
    float_array, float_array + point_mat1.total(), std::numeric_limits<float>::min(),
    [](float max, float current)
    {
      return current > max ? current : max;
    }
  );
  // add some slack
  size_m += 0.5;

  int size_pix = (int)(size_m / resolution);

  cv::Mat img(
    size_pix,
    size_pix,
    CV_8UC3,
    cv::Scalar(0, 0, 0)
  );

  auto meters_to_pix = [&size_pix, resolution](float meters) {
    int pix = (int)(meters / resolution + size_pix / 2.0f);
    pix = std::max(0, pix);
    pix = std::min(size_pix - 1, pix);
    return pix;
  };

  cv::Mat transformed_point_mat2 = utils::transformPointMat(T_2_1.inverse(), point_mat2);

  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
  {
    float x1 = point_mat1.at<float>(i, 0);
    float y1 = point_mat1.at<float>(i, 1);
    float x2 = transformed_point_mat2.at<float>(i, 0);
    float y2 = transformed_point_mat2.at<float>(i, 1);

    auto pix_x1 = meters_to_pix(x1);
    auto pix_y1 = meters_to_pix(y1);
    auto pix_x2 = meters_to_pix(x2);
    auto pix_y2 = meters_to_pix(y2);

    cv::Point point1(pix_x1, pix_y1);
    cv::Point point2(pix_x2, pix_y2);

    cv::circle(img, point1, 5, cv::Scalar(0, 0, 255), -1);
    cv::circle(img, point2, 5, cv::Scalar(255, 0, 0), -1);

    cv::line(img, point1, point2, cv::Scalar(0, 255, 0), 2);
  }
  std::cout<<"...keyframe"<<itr;
  cv::Mat tmp;
  cv::flip(img, tmp, 0);
  auto file_path="./img"+std::to_string(i)+".png";
  cv::imwrite(file_path, img);
}

tf::Transform ICPSlam::icpRegistration(const sensor_msgs::LaserScanConstPtr &laser_scan1, const sensor_msgs::LaserScanConstPtr &laser_scan2, const tf::Transform &T_2_1){
  
  // Converting the laser scan to point clouds  
  cv::Mat laser_scan1_mat = utils::laserScanToPointMat(laser_scan1);
  cv::Mat laser_scan2_mat = utils::laserScanToPointMat(laser_scan2);

  cv::Mat relative_last_scan2_mat = utils::transformPointMat(T_2_1, laser_scan2_mat);
  tf::Transform new_T_2_1;
  itr = 0;
  max_itr = false;
  do {

    new_T_2_1 = icpIteration(laser_scan1_mat, relative_last_scan2_mat);
    relative_last_scan2_mat = utils::transformPointMat(new_T_2_1, laser_scan2_mat);

    // Increment the iteration
    itr++;
  } while (!max_itr);
  return new_T_2_1;
}

tf::Transform ICPSlam::icpIteration(cv::Mat &point_mat1, cv::Mat &point_mat2){
  
  std::vector<int> closest_indices(point_mat2.size().height);
  std::vector<float> closest_distances_2(point_mat2.size().height);

  // Finding the closest point indices and their distances
  closestPoints(point_mat1, point_mat2, closest_indices, closest_distances_2);

  // Reodering the point_mat2
  cv::Mat closest_point_mat2 = point_mat2.clone();
  for(int i = 0; i < point_mat2.size().height; i++){
    closest_point_mat2.at<float>(i, 0) = point_mat2.at<float>(closest_indices[i], 0);
    closest_point_mat2.at<float>(i, 1) = point_mat2.at<float>(closest_indices[i], 1);
  }

  // Removing the outlier points
  float mean;
  float std_dev;
  utils::meanAndStdDev(closest_distances_2, mean, std_dev); 
  cv::Mat modified_point_mat1(0, 2, CV_32F);
  cv::Mat modified_point_mat2(0, 2, CV_32F);

  for(int i = 0; i < closest_distances_2.size(); i++){
    if(closest_distances_2[i] <= mean + 2 * std_dev){
      modified_point_mat1.push_back(point_mat1.row(i));
      modified_point_mat2.push_back(closest_point_mat2.row(i));
    }
  }
  
  // Calculating the center of mass
  cv::Mat com_point_mat1 = cv::Mat::zeros(1, 2, CV_32F);
  cv::Mat com_point_mat2 = cv::Mat::zeros(1, 2, CV_32F);
  
  for(int i = 0; i < modified_point_mat1.size().height; i++){
    com_point_mat1.row(0) = com_point_mat1.row(0) + modified_point_mat1.row(i);
    com_point_mat2.row(0) = com_point_mat2.row(0) + modified_point_mat2.row(i);
  }

  com_point_mat1 /= modified_point_mat1.size().height;
  com_point_mat2 /= modified_point_mat2.size().height;

  // Subtract center of mass from point clouds
  cv::Mat modified_point_mat1_prime(modified_point_mat1.size(), CV_32F);
  cv::Mat modified_point_mat2_prime(modified_point_mat2.size(), CV_32F);

  for(int i = 0; i < modified_point_mat1.size().height; i++){
    modified_point_mat1_prime.row(i) = modified_point_mat1.row(i) - com_point_mat1.row(0);
    modified_point_mat2_prime.row(i) = modified_point_mat2.row(i) - com_point_mat2.row(0);
  }

  // Calculate W
  //cv::Mat W = modified_point_mat1_prime.t() * modified_point_mat2_prime;
  cv::Mat W = cv::Mat::zeros(2, 2, CV_32F);
  
  for (int i = 0; i < modified_point_mat1_prime.size().height; i++){
    W.at<float>(0,0) += modified_point_mat1_prime.at<float>(i,0) * modified_point_mat2_prime.at<float>(i,0);
    W.at<float>(0,1) += modified_point_mat1_prime.at<float>(i,0) * modified_point_mat2_prime.at<float>(i,1);
    W.at<float>(1,0) += modified_point_mat1_prime.at<float>(i,1) * modified_point_mat2_prime.at<float>(i,0);
    W.at<float>(1,1) += modified_point_mat1_prime.at<float>(i,1) * modified_point_mat2_prime.at<float>(i,1);
  }

  // SVD
  cv::SVD svd(W);

  cv::Mat u = svd.u;
  cv::Mat vt = svd.vt;

  //cv::Mat R = u * vt;
  cv::Mat R = cv::Mat::zeros(2, 2, CV_32F);
  
  for (int i = 0; i < u.size().height; i++){
    R.at<float>(0,0) = u.at<float>(0,0) * vt.at<float>(0,0) + u.at<float>(0,1) * vt.at<float>(1,0);
    R.at<float>(0,1) = u.at<float>(0,0) * vt.at<float>(1,0) + u.at<float>(1,0) * vt.at<float>(1,1);
    R.at<float>(1,0) = u.at<float>(1,0) * vt.at<float>(0,0) + u.at<float>(1,1) * vt.at<float>(1,0);
    R.at<float>(1,1) = u.at<float>(1,0) * vt.at<float>(0,1) + u.at<float>(1,1) * vt.at<float>(1,1);
  }

  cv::Mat t = com_point_mat1 - com_point_mat2 * R; 

  // Calculating the error
  cv::Mat error = cv::Mat::zeros(1, 2, CV_32F);
  for(int i = 0; i < modified_point_mat1.size().height; i++){
    auto temp = (modified_point_mat1.row(i) - modified_point_mat2.row(i) * R - t);
    
    cv::Mat temp_pow(temp.size(), CV_32F);
    cv::pow(temp, 2.0, temp_pow);
    error += temp_pow;
  }
  error /= modified_point_mat1.size().height;
  if((error.at<float>(0,0) <= 0.1 && error.at<float>(0,1) <= 0.1) || itr > 100){
    max_itr = true;
  }

  tf::Vector3 t_v3(t.at<float>(0,0), t.at<float>(0,1), 0);
  tf::Transform transform(tf::createQuaternionFromRPY(0,0,std::atan(R.at<float>(1,0) / R.at<float>(0,0))).normalize(), t_v3);
  vizClosestPoints(point_mat1, point_mat2, transform);
  return transform;
}

} // namespace icp_slam

