// Rasha

/* This file is part of RGBDSLAM.
 *
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RANSAC_H
#define RANSAC_H

#include "ros/ros.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <tf/transform_datatypes.h>
//#include "pcl_ros/point_cloud.h"
//#include "pcl/point_types.h"
#include <Eigen/StdVector>
#include "g2o/types/slam3d/se3quat.h"
#include "transformation_from_correspondences.h"
#include "Frame.h"
#include"ORBmatcher.h"

namespace ORB_SLAM2
{

//typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > std_vector_of_eigen_vector4f;

class Settings
{
public:
	Settings(): ransac_iterations(5), refine_iterations(10), min_inliers_threshold(100), max_inlier_error(0.03),
	dist_ratio(0.7), only_map(false)
{};

	int ransac_iterations;
	int refine_iterations;
	int min_inliers_threshold;
	double max_inlier_error;
	double dist_ratio;
	bool only_map;
};

template<class TFrame>
class Ransac {
public:

	Ransac(){}

public:

  Settings settings;
  std::vector<cv::DMatch> initial_matches;
  bool second_time_;
	std::vector<std::vector<cv::DMatch> > BF_matches;

  bool getTransformation(TFrame& last_frame, TFrame& current_frame, cv::Mat& transformation,
//  bool getTransformation(Frame& last_frame, Frame& current_frame, cv::Mat& transformation,
		  vector<MapPoint*>& vpMapPointMatches, int& matches_size, bool second_time, std::map<int, cv::DMatch>& query_vec, float& match_perc);
//		  vector<MapPoint*>& vpMapPointMatches, int& matches_size, bool second_time = false);

private:

  unsigned int featureMatching(TFrame& last_frame, TFrame& current_frame, std::vector<cv::DMatch>& matches, bool second_time);
//  unsigned int featureMatching(Frame& last_frame, Frame& current_frame, std::vector<cv::DMatch>& matches, bool second_time);

  bool runRansac(const std::vector<Eigen::Vector4f>& last_3d_points,
  		const std::vector<Eigen::Vector4f>& current_3d_points,
                                         std::vector<cv::DMatch>& initial_matches,
                                         Eigen::Matrix4f& resulting_transformation,
                                         float& rmse,
                                         std::vector<cv::DMatch>& matches);
  void getRefinedTransformation(const std::vector<Eigen::Vector4f>& last_3d_points,
  		const std::vector<Eigen::Vector4f>& current_3d_points,
  		std::vector<cv::DMatch>& initial_matches,
  		std::vector<cv::DMatch>& inlier,
          int n_iterations,
          Eigen::Matrix4f& refined_transformation,
          double& refined_error,
          std::vector<cv::DMatch>& refined_matches);


  void computeInliersAndError(const std::vector<cv::DMatch> & all_matches,
                                    const Eigen::Matrix4f& transformation4f,
                                    const std::vector<Eigen::Vector4f>& origins,
                                    const std::vector<Eigen::Vector4f>& earlier,
                                    std::vector<cv::DMatch>& inliers, //pure output var
                                    double& return_mean_error,//pure output var: rms-mahalanobis-distance
                                    //std::vector<double>& errors,
                                    double squaredMaxInlierDistInM);

  std::vector<cv::DMatch> sample_matches_prefer_by_distance(unsigned int sample_size,
		  std::vector<cv::DMatch>& matches_with_depth,const std::vector<Eigen::Vector4f>& current_3d_points,
			const std::vector<Eigen::Vector4f>& last_3d_points);

  Eigen::Matrix4f getTransformFromMatches(const std::vector<Eigen::Vector4f>& current_3d_points,
  		const std::vector<Eigen::Vector4f>& last_3d_points,
           const std::vector<cv::DMatch>& matches);

  double errorFunction2Fast(const Eigen::Vector4f& x1,
                        const Eigen::Vector4f& x2,
                        const Eigen::Matrix4d& transformation, double& with_depth_error) ;

};

} //namespace ORB_SLAM


#include "../src/Ransac.cc"

#endif // RANSAC_H

