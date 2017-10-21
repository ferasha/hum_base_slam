
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

#include "Ransac.h"

namespace ORB_SLAM2
{

template<class TFrame>
unsigned int Ransac<TFrame>::featureMatching(TFrame& last_frame, TFrame& current_frame, std::vector<cv::DMatch>& matches, bool try_again)
//unsigned int Ransac::featureMatching(Frame& last_frame, Frame& current_frame, std::vector<cv::DMatch>& matches, bool try_again)
{

	matches.clear();

//	std::cout<<"current_frame.mDescriptors.rows "<<current_frame.mDescriptors.rows<<std::endl;
//	std::cout<<"dist_ratio "<<settings.dist_ratio<<std::endl;

	cv::Ptr<cv::DescriptorMatcher> matcher = new cv::BFMatcher(cv::NORM_HAMMING);

	const vector<MapPoint*>& last_frame_mvpMapPoints = last_frame.GetMapPointMatches();

	BF_matches.clear();

	matcher->knnMatch(current_frame.mDescriptors, last_frame.mDescriptors, BF_matches, 2);
			for (unsigned int i=0; i<BF_matches.size(); i++)
			{
				if (BF_matches[i].size() > 1) {
					if (BF_matches[i][0].distance < settings.dist_ratio * BF_matches[i][1].distance) {
	//				double dist_perc = (v_matches[i][1].distance - v_matches[i][0].distance)*1.0/v_matches[i][0].distance;
	//				if (dist_perc >= 0.2) {
			//         	if(last_frame_mvpMapPoints[v_matches[i][0].trainIdx]->Observations()>0)
						if (settings.only_map) {
						     MapPoint* pMP = last_frame_mvpMapPoints[BF_matches[i][0].trainIdx];
						     if (!pMP)
								continue;
						     else if(pMP->Observations()<1 || pMP->isBad())
						    	 continue;
						}

			         		matches.push_back(BF_matches[i][0]);
	//				}


		//				if (v_matches.size() == 92) {
/*
									cv::DMatch m1 = v_matches[i][0];
									cv::DMatch m2 = v_matches[i][1];

									std::cout<<i<<": "<<
											m1.trainIdx<<" "<<
											last_frame.mvKeys[m1.trainIdx].pt.x<<","<<
											last_frame.mvKeys[m1.trainIdx].pt.y<<" "<<
											last_frame.mvKeys[m1.trainIdx].response<<" "<<
											m1.queryIdx<<" "<<
											current_frame.mvKeys[m1.queryIdx].pt.x<<","<<
											current_frame.mvKeys[m1.queryIdx].pt.y<<" "<<
											current_frame.mvKeys[m1.queryIdx].response<<" "<<
											m1.distance<<" "<<
											m1.distance/m2.distance<<" "<<
											abs(current_frame.mvKeys[m1.queryIdx].pt.y - last_frame.mvKeys[m1.trainIdx].pt.y)<<
											std::endl;
*/
/*
									std::cout<<"   "<<
											m2.trainIdx<<" "<<
											last_frame.mvKeys[m2.trainIdx].pt.x<<","<<
											last_frame.mvKeys[m2.trainIdx].pt.y<<" "<<
											last_frame.mvKeys[m2.trainIdx].response<<" "<<
											m2.queryIdx<<" "<<
											current_frame.mvKeys[m2.queryIdx].pt.x<<","<<
											current_frame.mvKeys[m2.queryIdx].pt.y<<" "<<
											current_frame.mvKeys[m2.queryIdx].response<<" "<<
											m2.distance<<
											std::endl;
		//				}
*/

					}

				}
				else {
					std::cout<<"v_matches[i].size() is less than 1 !!!"<<std::endl;
				}
			}

//	std::cout<<"new matching v_matches.size() "<<v_matches.size()<<" matches "<<matches.size()<<std::endl;

	if (false && try_again && matches.size() < 10) {

		std::cout<<"adding additional matches "<<std::endl;
		for (unsigned int i=0; i<BF_matches.size(); i++)
		{
				if (BF_matches[i][0].distance < 0.8 * BF_matches[i][1].distance && BF_matches[i][0].distance >= 0.7 * BF_matches[i][1].distance) {
					matches.push_back(BF_matches[i][0]);

//					cv::DMatch m1 = v_matches[i][0];
//					cv::DMatch m2 = v_matches[i][1];

//					std::cout<<i<<": "<<
//							m1.trainIdx<<" "<<
//							last_frame.mvKeys[m1.trainIdx].pt.x<<","<<
//							last_frame.mvKeys[m1.trainIdx].pt.y<<" "<<
//							last_frame.mvKeys[m1.trainIdx].response<<" "<<
//							m1.queryIdx<<" "<<
//							current_frame.mvKeys[m1.queryIdx].pt.x<<","<<
//							current_frame.mvKeys[m1.queryIdx].pt.y<<" "<<
//							current_frame.mvKeys[m1.queryIdx].response<<" "<<
//							m1.distance<<" "<<
//							m1.distance/m2.distance<<" "<<
//							abs(current_frame.mvKeys[m1.queryIdx].pt.y - last_frame.mvKeys[m1.trainIdx].pt.y)<<
//							std::endl;

				}
		}
		std::cout<<"new matches size "<<matches.size()<<std::endl;

/*
		 cv::Mat initial_matches_img, matches_img;
		 drawMatches(current_frame.mImRGB, current_frame.mvKeys,last_frame.mImRGB, last_frame.mvKeys,
				 matches, initial_matches_img, cv::Scalar::all(-1), cv::Scalar::all(-1));
		 cv::imshow("intial_matches", initial_matches_img);

		 cv::waitKey(0);
*/
	}

	return matches.size();

//	matcher->match( this->feature_descriptors_, other->feature_descriptors_, *matches );
	if (current_frame.mDescriptors.rows > 600)
		matcher->match( current_frame.mDescriptors(cv::Range(0,600), cv::Range::all()), last_frame.mDescriptors, matches );
	else
		matcher->match( current_frame.mDescriptors, last_frame.mDescriptors, matches );

	 return matches.size();

	////////////////////////////////////////////////////////////



	if (!try_again) {

		cv::Ptr<cv::DescriptorMatcher> matcher = new cv::BFMatcher(cv::NORM_HAMMING);

	//	matcher->match( this->feature_descriptors_, other->feature_descriptors_, *matches );
		if (current_frame.mDescriptors.rows > 600)
			matcher->match( current_frame.mDescriptors(cv::Range(0,600), cv::Range::all()), last_frame.mDescriptors, matches );
		else
			matcher->match( current_frame.mDescriptors, last_frame.mDescriptors, matches );

		 return matches.size();
	}

	else
	{
/*
	cv::Ptr<cv::DescriptorMatcher> matcher = new cv::BFMatcher(cv::NORM_HAMMING, true);

	std::vector<std::vector<cv::DMatch> > v_matches;

//	if (last_frame.mDescriptors.rows > 600)
//		matcher->knnMatch( last_frame.mDescriptors(cv::Range(0,600), cv::Range::all()), current_frame.mDescriptors, v_matches, 1 );
//	else
//	matcher->knnMatch( last_frame.mDescriptors, current_frame.mDescriptors, v_matches, 1 );
	matcher->knnMatch( current_frame.mDescriptors, last_frame.mDescriptors, v_matches, 1 );
	std::cout<<"current_frame.mDescriptors.size() "<<current_frame.mDescriptors.size()<<
			"last_frame.mDescriptors.size() "<<last_frame.mDescriptors.size()<<
			"v_matches "<<v_matches.size()<<std::endl;

	for (int i=0; i<v_matches.size(); i++)
	{
		for (int j=0; j<v_matches[i].size(); j++) {
			matches.push_back(v_matches[i][j]);
		}
	}
*/
/*
	cv::Ptr<cv::DescriptorMatcher> matcher = new cv::BFMatcher(cv::NORM_HAMMING);
	std::vector<std::vector<cv::DMatch> > v_matches1, v_matches2;
	matcher->knnMatch( current_frame.mDescriptors, last_frame.mDescriptors, v_matches1, 20 );
	matcher->knnMatch( last_frame.mDescriptors, current_frame.mDescriptors, v_matches2, 20 );
	for (int i=0; i<v_matches2.size(); i++)
	{
		for (int j=0; j<v_matches2[i].size(); j++) {
			cv::DMatch& m = v_matches2[i][j];
			int older_index = m.trainIdx;
			if (m.distance > v_matches2[i][0].distance)
				continue;
			for (int j2=0; j2<v_matches1[older_index].size(); j2++) {
				cv::DMatch& reverse_m = v_matches1[older_index][j2];
				if (reverse_m.distance > v_matches1[older_index][0].distance)
					continue;
				if (reverse_m.trainIdx == m.queryIdx)
					matches.push_back(v_matches1[older_index][j2]);
				//	matches.push_back(v_matches2[i][j]);
			}
		}
	}
*/

/*
		cv::Ptr<cv::DescriptorMatcher> matcher = new cv::BFMatcher(cv::NORM_HAMMING);
		std::vector<std::vector<cv::DMatch> > v_matches1, v_matches2;
		matcher->knnMatch( current_frame.mDescriptors, last_frame.mDescriptors, v_matches1, 5 );
//		matcher->knnMatch( last_frame.mDescriptors, current_frame.mDescriptors, v_matches2, 100 );
		for (int i=0; i<v_matches1.size(); i++)
		{
			for (int j=0; j<v_matches1[i].size(); j++) {
				cv::DMatch& m = v_matches1[i][j];
				if (m.distance > 45)
					break;
				matches.push_back(v_matches1[i][j]);
			}
		}
*/
/*		for (int i=0; i<v_matches2.size(); i++)
		{
			for (int j=0; j<v_matches2[i].size(); j++) {
				cv::DMatch& m = v_matches2[i][j];
				if (m.distance >= 50)
					break;
				int older_index = m.trainIdx;
				for (int j2=0; j2<v_matches1[older_index].size(); j2++) {
					cv::DMatch& reverse_m = v_matches1[older_index][j2];
					if (reverse_m.trainIdx == m.queryIdx) {
						matches.push_back(v_matches1[older_index][j2]);
						//matches.push_back(v_matches1[i][j]);
						break;
					}
				}
			}
		}
*/

	std::vector<std::vector<cv::DMatch> > v_matches1, v_matches2;
	cv::Ptr<cv::DescriptorMatcher> matcher = new cv::BFMatcher(cv::NORM_HAMMING);
//	std::vector<cv::DMatch> temp_matches;
	cv::Mat mask(current_frame.N, last_frame.N, CV_8UC1, cv::Scalar(0));
	for (int i=0; i<current_frame.N; i++) {
//		std::vector<size_t> indices = last_frame.GetFeaturesInArea(current_frame.mvKeys[i].pt.x,
//				current_frame.mvKeys[i].pt.y, 400, 100);
		std::vector<size_t> indices = last_frame.GetFeaturesInArea(current_frame.mvKeys[i].pt.x,
				current_frame.mvKeys[i].pt.y, 400);
		for (size_t f=0; f<indices.size(); f++){
			mask.ptr<uchar>(i)[indices[f]] = 255;
		}
	}
	matcher->knnMatch( current_frame.mDescriptors, last_frame.mDescriptors, v_matches1, 1, mask);

	std::vector<cv::DMatch> temp_matches2;
	cv::Mat mask2(last_frame.N, current_frame.N, CV_8UC1, cv::Scalar(0));
	for (int i=0; i<last_frame.N; i++) {
//		std::vector<size_t> indices2 = current_frame.GetFeaturesInArea(last_frame.mvKeys[i].pt.x,
//				last_frame.mvKeys[i].pt.y, 400, 100);
		std::vector<size_t> indices2 = current_frame.GetFeaturesInArea(last_frame.mvKeys[i].pt.x,
				last_frame.mvKeys[i].pt.y, 400);
		for (size_t f=0; f<indices2.size(); f++){
			mask2.ptr<uchar>(i)[indices2[f]] = 255;
		}
	}
	matcher->knnMatch( last_frame.mDescriptors, current_frame.mDescriptors, v_matches2, 5, mask2);


/*
	std::sort(temp_matches.begin(), temp_matches.end());
	for (unsigned int i=0; i<temp_matches.size(); i++)
	{
		if (temp_matches[i].distance > 60)
			break;
		matches.push_back(temp_matches[i]);
	}
*/

	std::vector<cv::DMatch> temp_matches;
	std::vector<cv::Point2f> matchedp1, matchedp2;


			for (unsigned int i=0; i<v_matches2.size(); i++)
			{
				for (unsigned int j=0; j<v_matches2[i].size(); j++) {
					cv::DMatch& m = v_matches2[i][j];
//					if (m.distance > 60)
//						break;
					int older_index = m.trainIdx;
					for (unsigned int j2=0; j2<v_matches1[older_index].size(); j2++) {
						cv::DMatch& reverse_m = v_matches1[older_index][j2];
						if (reverse_m.trainIdx == m.queryIdx) {
					//		matches.push_back(v_matches1[older_index][j2]);
							temp_matches.push_back(v_matches1[older_index][j2]);
							matchedp1.push_back(current_frame.mvKeys[m.trainIdx].pt);
							matchedp2.push_back(last_frame.mvKeys[m.queryIdx].pt);
							//matches.push_back(v_matches1[i][j]);
							break;
						}
					}
				}
			}


/*
	for (unsigned int i=0; i<v_matches1.size(); i++)
	{
		for (unsigned int j=0; j<v_matches1[i].size(); j++) {
			cv::DMatch& m = v_matches1[i][j];
			temp_matches.push_back(v_matches1[i][j]);
			matchedp1.push_back(current_frame.mvKeys[m.queryIdx].pt);
			matchedp2.push_back(last_frame.mvKeys[m.trainIdx].pt);
		}
	}
*/
	std::cout<<"temp_matches.size() "<<temp_matches.size()<<std::endl;

	cv::Mat inlier_mask, homography;
	homography = findHomography(matchedp1, matchedp2, cv::RANSAC, 3, inlier_mask);

	std::cout<<"inlier_mask.size() "<<inlier_mask.size()<<std::endl;

	for (int i=0; i<inlier_mask.rows; i++) {
		if (inlier_mask.at<uchar>(i)) {
			matches.push_back(temp_matches[i]);
		}
	}

	std::cout<<"matches.size() "<<matches.size()<<std::endl;
	std::sort(matches.begin(), matches.end());
	 return matches.size();



	}

}

template<class TFrame>
bool Ransac<TFrame>::getTransformation(TFrame& last_frame, TFrame& current_frame,
//bool Ransac::getTransformation(Frame& last_frame, Frame& current_frame,
		cv::Mat& transformation, vector<MapPoint*>& vpMapPointMatches, int& matches_size, bool try_again,
		std::map<int, cv::DMatch>& query_vec, float& match_perc)
//				cv::Mat& transformation, vector<MapPoint*>& vpMapPointMatches, int& matches_size, bool second_time)

{
	std::vector < Eigen::Vector4f > last_3d_points;
	std::vector < Eigen::Vector4f > current_3d_points;
	Eigen::Matrix4f transf;
	float rmse;
	std::vector < cv::DMatch > matches;
	std::vector < cv::DMatch > matches_old;

	vpMapPointMatches = vector<MapPoint*>(current_frame.N,
			static_cast<MapPoint*>(NULL));

//	second_time_ = second_time;
	featureMatching(last_frame, current_frame, initial_matches, try_again);

	const vector<MapPoint*>& last_frame_mvpMapPoints = last_frame.GetMapPointMatches();

	bool found_transformation = false;

	second_time_  =false;
	if (!second_time_) {
		last_3d_points.resize(last_frame_mvpMapPoints.size());
		current_3d_points.resize(current_frame.mvKeys.size());
		for (unsigned int i = 0; i < initial_matches.size(); i++) {
			cv::DMatch& m = initial_matches[i];
/*
			if (settings.only_map) {
			     MapPoint* pMP = last_frame_mvpMapPoints[m.trainIdx];
			     if (!pMP)
					continue;
			     else if(pMP->Observations()<1)
			    	 continue;
			}
*/
			cv::Mat l_point = last_frame.UnprojectStereo(m.trainIdx, false);
			last_3d_points[m.trainIdx] = Eigen::Vector4f(l_point.at<float>(0),
					l_point.at<float>(1), l_point.at<float>(2), 1.0);

			cv::Mat c_point = current_frame.UnprojectStereo(m.queryIdx, false);
			current_3d_points[m.queryIdx] = Eigen::Vector4f(
					c_point.at<float>(0), c_point.at<float>(1),
					c_point.at<float>(2), 1.0);
		}

		found_transformation = false;

		if (try_again){
			found_transformation = runRansac(last_3d_points, current_3d_points,
				initial_matches, transf, rmse, matches);
		}

		std::cout<<"found_transformation "<<found_transformation<<std::endl;
//		std::cout<<transf<<std::endl;

        std::cout<<"old initial_matches "<<initial_matches.size()<<std::endl;
        std::cout<<"old matches "<<matches.size()<<std::endl;
/*
		settings.dist_ratio = 2.0;
		featureMatching(last_frame, current_frame, initial_matches, try_again);
*/
        initial_matches.clear();
		for (unsigned int i=0; i<BF_matches.size(); i++)
		{
			if (BF_matches[i].size() > 1) {
				if (BF_matches[i][0].distance < 2.0 * BF_matches[i][1].distance) {
					initial_matches.push_back(BF_matches[i][0]);
				}
			}
		}

        std::cout<<"new initial_matches "<<initial_matches.size()<<std::endl;

		for (unsigned int i = 0; i < initial_matches.size(); i++) {
			cv::DMatch& m = initial_matches[i];
/*
			if (settings.only_map) {
			     MapPoint* pMP = last_frame_mvpMapPoints[m.trainIdx];
			     if (!pMP)
					continue;
			     else if(pMP->Observations()<1)
			    	 continue;
			}
*/
			cv::Mat l_point = last_frame.UnprojectStereo(m.trainIdx, false);
			last_3d_points[m.trainIdx] = Eigen::Vector4f(l_point.at<float>(0),
					l_point.at<float>(1), l_point.at<float>(2), 1.0);

			cv::Mat c_point = current_frame.UnprojectStereo(m.queryIdx, false);
			current_3d_points[m.queryIdx] = Eigen::Vector4f(
					c_point.at<float>(0), c_point.at<float>(1),
					c_point.at<float>(2), 1.0);
		}
        double inlier_error;
			const float max_dist_m = settings.max_inlier_error; //2.0;
        computeInliersAndError(initial_matches, transf,
        		current_3d_points, last_3d_points,
        		matches, inlier_error, max_dist_m*max_dist_m);
        std::cout<<"new matches "<<matches.size()<<std::endl;

/*
		if (found_transformation && (abs(transf(0,3)) >= 0.2 || abs(transf(1,3)) >= 0.2
		    		|| abs(transf(2,3)) >= 0.2))
			found_transformation = false;
*/	
//		found_transformation = false;
		if (found_transformation) {
	//		std::cout << transf << std::endl;
			cv::Mat _src(4, 4, CV_32F, (void*) transf.data(),
					transf.stride() * sizeof(float));
			cv::transpose(_src, transformation);

//			std::map<int, cv::DMatch> query_vec;
			std::map<int, cv::DMatch> train_vec;
			for (unsigned int i = 0; i < matches.size(); i++) {
				cv::DMatch& m = matches[i];
		//		vpMapPointMatches[m.queryIdx] = last_frame_mvpMapPoints[m.trainIdx];
				if (query_vec.count(m.queryIdx) > 0) {
					if (query_vec[m.queryIdx].distance <= m.distance)
						continue;
				}
	/*			if (train_vec.count(m.trainIdx) > 0) {
					if (train_vec[m.trainIdx].distance <= m.distance)
						continue;
				}
	*/			vpMapPointMatches[m.queryIdx] = last_frame_mvpMapPoints[m.trainIdx];
				query_vec[m.queryIdx] = m;
		//		train_vec[m.trainIdx] = m;

				// 	if (!last_frame_mvpMapPoints[m.trainIdx])
				// 		std::cout<<"empty mappoint!!"<<std::endl;
			}

		} else {
//			std::map<int, cv::DMatch> query_vec;
			std::map<int, cv::DMatch> train_vec;
			for (unsigned int i = 0; i < matches.size(); i++) {
				cv::DMatch& m = matches[i];
				if (query_vec.count(m.queryIdx) > 0) {
					if (query_vec[m.queryIdx].distance <= m.distance)
						continue;
				}
/*				if (train_vec.count(m.trainIdx) > 0) {
					if (train_vec[m.trainIdx].distance <= m.distance)
						continue;
				}*/
				vpMapPointMatches[m.queryIdx] = last_frame_mvpMapPoints[m.trainIdx];
				query_vec[m.queryIdx] = m;
	//			train_vec[m.trainIdx] = m;
			}
		}

	} else {
//		std::map<int, cv::DMatch> query_vec;
		std::map<int, cv::DMatch> train_vec;
		for (unsigned int i = 0; i < initial_matches.size(); i++) {
			cv::DMatch& m = initial_matches[i];
			if (query_vec.count(m.queryIdx) > 0) {
				if (query_vec[m.queryIdx].distance <= m.distance)
					continue;
			}
			if (train_vec.count(m.trainIdx) > 0) {
				if (train_vec[m.trainIdx].distance <= m.distance)
					continue;
			}
			vpMapPointMatches[m.queryIdx] = last_frame_mvpMapPoints[m.trainIdx];
			query_vec[m.queryIdx] = m;
			train_vec[m.trainIdx] = m;
			//	vpMapPointMatches[m.queryIdx] = last_frame.UnprojectStereo(m.trainIdx, true);
		}
	}

	if (!try_again)
		matches_size = initial_matches.size();
	else
		matches_size = matches.size();

	std::cout << "---------------id " << current_frame.mnId << " matched with "
			<< last_frame.mnId << " initial matches "
			<< (initial_matches).size() << " final inliers " << matches.size()
			<< " " << matches.size() * 1.0 / (initial_matches).size()
			<< " try_again:" << try_again << std::endl;

	match_perc = matches.size() * 1.0 / (initial_matches).size();
//	 if (second_time_) {

if (false && (!found_transformation || (initial_matches).size() <= 10)){

	cv::Ptr<cv::DescriptorMatcher> matcher = new cv::BFMatcher(cv::NORM_HAMMING);

	std::vector<std::vector<cv::DMatch> > v_matches;
	matcher->knnMatch(current_frame.mDescriptors, last_frame.mDescriptors, v_matches, 2);
			for (unsigned int i=0; i<v_matches.size(); i++)
			{
				if (v_matches[i].size() > 1) {
					if (v_matches[i][0].distance < 0.7 * v_matches[i][1].distance) {
									cv::DMatch m1 = v_matches[i][0];
									cv::DMatch m2 = v_matches[i][1];

									std::cout<<i<<": "<<
											m1.trainIdx<<" "<<
											last_frame.mvKeys[m1.trainIdx].pt.x<<","<<
											last_frame.mvKeys[m1.trainIdx].pt.y<<" "<<
											last_frame.mvKeys[m1.trainIdx].response<<" "<<
											m1.queryIdx<<" "<<
											current_frame.mvKeys[m1.queryIdx].pt.x<<","<<
											current_frame.mvKeys[m1.queryIdx].pt.y<<" "<<
											current_frame.mvKeys[m1.queryIdx].response<<" "<<
											m1.distance<<" "<<
											m1.distance/m2.distance<<" "<<
											abs(current_frame.mvKeys[m1.queryIdx].pt.y - last_frame.mvKeys[m1.trainIdx].pt.y)<<
											std::endl;

					}

				}
			}

	 cv::Mat initial_matches_img, matches_img;
	 drawMatches(current_frame.mImRGB, current_frame.mvKeys,last_frame.mImRGB, last_frame.mvKeys,
	 initial_matches, initial_matches_img, cv::Scalar::all(-1), cv::Scalar::all(-1));
	 cv::imshow("intial_matches", initial_matches_img);

	 drawMatches(current_frame.mImRGB, current_frame.mvKeys,last_frame.mImRGB, last_frame.mvKeys,
	 matches, matches_img, cv::Scalar::all(-1), cv::Scalar::all(-1));
	 cv::imshow("matches", matches_img);

	 cv::waitKey(0);
}
//	 }


	if (second_time_)
		return (initial_matches.size() >= 10);
	else
		return found_transformation;
}

template<class TFrame>
bool Ransac<TFrame>::runRansac(const std::vector<Eigen::Vector4f>& last_3d_points,
//bool Ransac::runRansac(const std::vector<Eigen::Vector4f>& last_3d_points,
		const std::vector<Eigen::Vector4f>& current_3d_points, std::vector<cv::DMatch>& initial_matches,
        Eigen::Matrix4f& resulting_transformation, float& rmse, std::vector<cv::DMatch>& matches)
//		const std::vector<Eigen::Vector4f>& current_3d_points, std::vector<cv::DMatch>& initial_matches,
//        Eigen::Matrix4f& resulting_transformation, float& rmse, std::vector<cv::DMatch>& matches)

		{
//	ScopedTimer s(__FUNCTION__);
  const unsigned int sample_size = 4;// chose this many randomly from the correspondences:
 // unsigned int min_inlier_threshold = settings.min_inliers_threshold;
  rmse = 1e6;

  double refined_error;
  std::vector<cv::DMatch> refined_matches;
  Eigen::Matrix4f refined_transformation;

  unsigned int best_sample_inliers_size = 0;
  std::vector<cv::DMatch> best_inliers;

//  std::sort(initial_matches->begin(), initial_matches->end()); //sort by distance, which is the nn_ratio

  srand(1);

//  std::cout<<"trying the identity transformation"<<std::endl;
  best_inliers.clear();
  getRefinedTransformation(last_3d_points, current_3d_points, initial_matches, best_inliers, settings.refine_iterations, refined_transformation, refined_error, refined_matches);
  if (refined_matches.size() > best_sample_inliers_size  )
  {
  	best_sample_inliers_size = refined_matches.size();
  	best_inliers.assign(refined_matches.begin(), refined_matches.end());

      resulting_transformation = refined_transformation;
      rmse = refined_error;
      matches.assign(refined_matches.begin(), refined_matches.end());
  //    ROS_WARN("identity transformation was better!");
  }
/*
  if (best_sample_inliers_size < 200) {
	  std::cout<<"trying the first 4 matches"<<std::endl;
	  std::vector<cv::DMatch> inlier;
	  inlier.assign(initial_matches.begin(), initial_matches.begin()+4);
	  getRefinedTransformation(last_3d_points, current_3d_points, initial_matches, inlier, settings.refine_iterations, refined_transformation, refined_error, refined_matches);
	  if (refined_matches.size() > best_sample_inliers_size  )
	  {
		best_sample_inliers_size = refined_matches.size();
		best_inliers.assign(refined_matches.begin(), refined_matches.end());

		  resulting_transformation = refined_transformation;
		  rmse = refined_error;
		  matches.assign(refined_matches.begin(), refined_matches.end());
	  //    ROS_WARN("identity transformation was better!");
	  }
  }
*/
  if (second_time_ || best_sample_inliers_size < 0.7*initial_matches.size()) {
	  for(int n = 0; (n < 5 && initial_matches.size() >= sample_size); n++)
	  {
		std::vector<cv::DMatch> inlier = sample_matches_prefer_by_distance(sample_size, initial_matches,
				current_3d_points, last_3d_points);
		getRefinedTransformation(last_3d_points, current_3d_points, initial_matches, inlier, settings.refine_iterations, refined_transformation, refined_error, refined_matches);

		if (refined_matches.size() >= best_sample_inliers_size  )
		{
			best_sample_inliers_size = refined_matches.size();
			best_inliers.assign(refined_matches.begin(), refined_matches.end());

			resulting_transformation = refined_transformation;
			rmse = refined_error;
			matches.assign(refined_matches.begin(), refined_matches.end());

			if (!second_time_ && best_sample_inliers_size >= 0.7*initial_matches.size())
				break;
		}
	  }
  }


 // still not good, try again
 // if (matches.size() < 0.3*initial_matches.size()) {
  if (second_time_ || matches.size() < 0.5*initial_matches.size()) {
      ROS_WARN("still not good, trying again...");

	  for(int n = 0; (n < settings.ransac_iterations && initial_matches.size() >= sample_size); n++)
	  {
		std::vector<cv::DMatch> inlier = sample_matches_prefer_by_distance(sample_size, initial_matches,
				current_3d_points, last_3d_points);
		getRefinedTransformation(last_3d_points, current_3d_points, initial_matches, inlier, settings.refine_iterations, refined_transformation, refined_error, refined_matches);

		if (refined_matches.size() >= best_sample_inliers_size  )
		{
			best_sample_inliers_size = refined_matches.size();
			best_inliers.assign(refined_matches.begin(), refined_matches.end());

			resulting_transformation = refined_transformation;
			rmse = refined_error;
			matches.assign(refined_matches.begin(), refined_matches.end());

			if (!second_time_ && best_sample_inliers_size >= 0.7*initial_matches.size())
				break;
		}
	  }
  }

//  return (matches.	size() >= 20);
//  return (matches.size() >= 100);
//  return (matches.size() >= 50);
//  if (!second_time_)
//	  return (matches.size() >= (unsigned int)settings.min_inliers_threshold);
 // else {
	  //  return (matches.size() >= 20);
	  return (initial_matches.size() > 0 && matches.size() >= 4);//0.2*initial_matches.size());
 // }
}

template<class TFrame>
void Ransac<TFrame>::getRefinedTransformation(const std::vector<Eigen::Vector4f>& last_3d_points,
//void Ransac::getRefinedTransformation(const std::vector<Eigen::Vector4f>& last_3d_points,
		const std::vector<Eigen::Vector4f>& current_3d_points,
		std::vector<cv::DMatch>& initial_matches, std::vector<cv::DMatch>& inlier,
        int n_iterations, Eigen::Matrix4f& refined_transformation,
        double& refined_error, std::vector<cv::DMatch>& refined_matches)
{
	refined_error = 1e6;
	double last_inlier_error = 1e6;
	double last_inlier_size = 0;
	double inlier_error;
	const float max_dist_m = settings.max_inlier_error; //2.0;
	refined_matches.clear();

    for(int refinements = 0; refinements < n_iterations; refinements++)
    {
        Eigen::Matrix4f transformation;
        if (inlier.size() >= 4) {
        	transformation = getTransformFromMatches(current_3d_points, last_3d_points, inlier);
 //       	for (int in=0; in < inlier.size(); in++) {
 //       		std::cout<<inlier[in].queryIdx<<" with "<< inlier[in].trainIdx<< ", ";
 //       	}
 //       	std::cout<<std::endl;
        }
        else if (refinements == 0)
            transformation = Eigen::Matrix4f::Identity();
        else
        	break;

        if (transformation != transformation)
        	ROS_ERROR_STREAM("nan values in transformation matrix");

        computeInliersAndError(initial_matches, transformation,
        		current_3d_points, last_3d_points,
        		inlier, inlier_error, max_dist_m*max_dist_m);

 //       std::cout<<refinements<<" refined_error "<<refined_error<<" inlier_error "<<inlier_error
 //       		<<" refined_matches.size() "<<refined_matches.size()<<" inlier.size() "<<inlier.size()<<std::endl;

   //     if (inlier.size() >= refined_matches.size() ){ //&& inlier_error <= refined_error) {
           if ( inlier_error <= refined_error) {
        	refined_transformation = transformation;
          refined_error = inlier_error;
          refined_matches.assign(inlier.begin(), inlier.end());
        }

//         if ( (inlier.size() == last_inlier_size && inlier_error == last_inlier_error))
 //       	 break;

         if (inlier_error == last_inlier_error) {
        	 if (inlier_error == 1e9)
        		 break;
        	 else if (inlier.size() == last_inlier_size)
        		 break;
         }

        last_inlier_size = inlier.size();
        last_inlier_error = inlier_error;
   }
}

template<class TFrame>
std::vector<cv::DMatch> Ransac<TFrame>::sample_matches_prefer_by_distance(unsigned int sample_size,
//std::vector<cv::DMatch> Ransac::sample_matches_prefer_by_distance(unsigned int sample_size,
		std::vector<cv::DMatch>& matches_with_depth, const std::vector<Eigen::Vector4f>& current_3d_points,
		const std::vector<Eigen::Vector4f>& last_3d_points)
{
//	ScopedTimer s(__FUNCTION__);
    //Sample ids to pick matches lateron (because they are unique and the
    //DMatch operator< overload breaks uniqueness of the Matches if they have the
    //exact same distance, e.g., 0.0)
//    std::set<std::vector<cv::DMatch>::size_type> sampled_ids;
    std::vector<int> sampled_ids;
    int safety_net = 0;
    while(sampled_ids.size() < sample_size && matches_with_depth.size() >= sample_size){
      //generate a set of samples. Using a set solves the problem of drawing a sample more than once
      int id1 = rand() % matches_with_depth.size();
      int id2 = rand() % matches_with_depth.size();

      if(id1 > id2) id1 = id2; //use smaller one => increases chance for lower id
      cv::DMatch& m1 = matches_with_depth[id1];
//      cv::DMatch& m2 = matches_with_depth[id2];

      bool found = false;
      for (unsigned int s=0; s<sampled_ids.size(); s++) {
    	     cv::DMatch& m3 = matches_with_depth[sampled_ids[s]];
    	     if (m3.trainIdx == m1.trainIdx && m3.queryIdx == m1.queryIdx) {
    	    	 found = true;
    	     }
      }
      if (!found)
    	     sampled_ids.push_back(id1);

    	  //     if (last_3d_points[m2.trainIdx][2] < last_3d_points[m1.trainIdx][2])
 //   	  id1 = id2;
 //     if(id1 > id2) id1 = id2; //use smaller one => increases chance for lower id
//      sampled_ids.insert(id1);
      if(++safety_net > 10000){ ROS_ERROR("Infinite Sampling"); break; }
    }

    //Given the ids, construct the resulting vector
    std::vector<cv::DMatch> sampled_matches;
    sampled_matches.reserve(sampled_ids.size());
    for (unsigned int i=0; i<sampled_ids.size(); i++) {
    	int id = sampled_ids[i];
    	sampled_matches.push_back(matches_with_depth[id]);
    }
/*    BOOST_FOREACH(std::vector<cv::DMatch>::size_type id, sampled_ids){
      sampled_matches.push_back(matches_with_depth[id]);
    }
 */
    return sampled_matches;
}

template<class TFrame>
Eigen::Matrix4f Ransac<TFrame>::getTransformFromMatches(const std::vector<Eigen::Vector4f>& current_3d_points,
//Eigen::Matrix4f Ransac::getTransformFromMatches(const std::vector<Eigen::Vector4f>& current_3d_points,
		const std::vector<Eigen::Vector4f>& last_3d_points,
         const std::vector<cv::DMatch>& matches)
{
//	ScopedTimer s(__FUNCTION__);

  pcl::TransformationFromCorrespondences tfc;

  std::vector<Eigen::Vector4f> t, f;
  float weight = 1.0;

//  BOOST_FOREACH(const cv::DMatch& m, matches)
  for (unsigned int i=0; i<matches.size(); i++)
  {
	  const cv::DMatch& m = matches[i];
//	  std::cout<<"transform "<<m.queryIdx<<" "<<m.trainIdx<<std::endl;
//    Eigen::Vector3f from = newer_node->feature_locations_3d_[m.queryIdx].head<3>();
//    Eigen::Vector3f to = earlier_node->feature_locations_3d_[m.trainIdx].head<3>();
	  Eigen::Vector3f from = current_3d_points[m.queryIdx].head<3>();
	  Eigen::Vector3f to = last_3d_points[m.trainIdx].head<3>();

    if(isnan(from(2)) || isnan(to(2))) {
    	ROS_ERROR_STREAM("encountered nan depth when computing the transform");
    	continue;
    }

    weight = 1.0/(from(2) * to(2)); //1.0;

    tfc.add(from, to, weight);// 1.0/(to(2)*to(2)));//the further, the less weight b/c of quadratic accuracy decay
  }

  // get relative movement from samples
  return tfc.getTransformation().matrix();
}

template<class TFrame>
void Ransac<TFrame>::computeInliersAndError(const std::vector<cv::DMatch> & all_matches,
//void Ransac::computeInliersAndError(const std::vector<cv::DMatch> & all_matches,
                                  const Eigen::Matrix4f& transformation4f,
                                  const std::vector<Eigen::Vector4f>& origins,
                                  const std::vector<Eigen::Vector4f>& earlier,
                                  std::vector<cv::DMatch>& inliers, //pure output var
                                  double& return_mean_error,//pure output var: rms-mahalanobis-distance
                                  //std::vector<double>& errors,
                                  double squaredMaxInlierDistInM)
{
//	ScopedTimer s(__FUNCTION__);

  inliers.clear();
  assert(all_matches.size() > 0);
//  inliers.reserve(all_matches.size());
  //errors.clear();
  const size_t all_matches_size = all_matches.size();
  double mean_error = 0.0;
  Eigen::Matrix4d transformation4d = transformation4f.cast<double>();

//parallelization is detrimental here
//#pragma omp parallel for reduction (+: mean_error)
  for(unsigned int i=0; i < all_matches_size; ++i)
  //BOOST_FOREACH(const cv::DMatch& m, all_matches)
  {
    const cv::DMatch& m = all_matches[i];
    const Eigen::Vector4f& origin = origins[m.queryIdx];
    const Eigen::Vector4f& target = earlier[m.trainIdx];
//    std::cout<<m.queryIdx<<" "<<m.trainIdx<<std::endl;
    if(origin(2) == 0.0 || target(2) == 0.0){ //does NOT trigger on NaN
       continue;
    }
//    std::cout<<origin<<std::endl;
//    std::cout<<target<<std::endl;

//   	std::cout<<i<<" "<<m.queryIdx<<" -> "<<m.trainIdx;

    double with_depth_error;
    double mahal_dist = errorFunction2Fast(origin, target, transformation4d, with_depth_error); //errorFunction2(origin, target, transformation4d);

//    std::cout<<" outlier? "<<(mahal_dist > squaredMaxInlierDistInM)<<std::endl;

//    if (i==248)
//    	std::cout<<i<<" "<<m.queryIdx<<" "<<m.trainIdx<<" "<<mahal_dist<<" "<<(mahal_dist > squaredMaxInlierDistInM)<<std::endl;
    //    double mahal_dist = errorFunction2(origin, target, transformation4d);

//    std::cout<<"mahal_dist "<<mahal_dist<<" mahal_dist_fast "<<mahal_dist_fast;

//      std::cout<<"mahal_dist "<<mahal_dist<<" with_depth "<<with_depth_error<<
//    		  " outlier "<<(int)(mahal_dist > squaredMaxInlierDistInM)<<std::endl;

    if(mahal_dist > squaredMaxInlierDistInM){
//       	std::cout<<i<<" "<<m.queryIdx<<" -> "<<m.trainIdx<<" dis: "<<mahal_dist<<" points: "<<origin(0)<<","<<origin(1)<<","<<origin(2)
 //      			<<"->"<<target(0)<<","<<target(1)<<","<<target(2)<<std::endl;
 //   	std::cout<<" outlier"<<std::endl;
    	continue; //ignore outliers
    }
    else
    {
 //   	std::cout<<" inlier"<<std::endl;
    }
    if(!(mahal_dist >= 0.0)){
      ROS_WARN_STREAM("Mahalanobis_ML_Error: "<<mahal_dist);
      ROS_WARN_STREAM("Transformation for error !>= 0:\n" << transformation4d << "Matches: " << all_matches.size());
      continue;
    }
    mean_error += mahal_dist;
//#pragma omp critical
    inliers.push_back(m); //include inlier

  }


  if (inliers.size()<3){ //at least the samples should be inliers
    ROS_DEBUG("No inliers at all in %d matches!", (int)all_matches.size()); // only warn if this checks for all initial matches
    return_mean_error = 1e9;
  } else {
    mean_error /= inliers.size();
    return_mean_error = sqrt(mean_error);
  }

}

template<class TFrame>
double Ransac<TFrame>::errorFunction2Fast(const Eigen::Vector4f& x1,
//double Ransac::errorFunction2Fast(const Eigen::Vector4f& x1,
                      const Eigen::Vector4f& x2,
                      const Eigen::Matrix4d& transformation, double& with_depth_error)
{
	  bool nan1 = isnan(x1(2));
	  bool nan2 = isnan(x2(2));
	  if(nan1||nan2){
	    //TODO: Handle Features with NaN, by reporting the reprojection error
	    return std::numeric_limits<double>::max();
	  }
	  Eigen::Vector4d x_1 = x1.cast<double>();
	  Eigen::Vector4d x_2 = x2.cast<double>();

	  Eigen::Matrix4d tf_12 = transformation;
	  Eigen::Vector3d mu_2 = x_2.head<3>();
	  Eigen::Vector3d mu_1_in_frame_2 = (tf_12 * x_1).head<3>(); // μ₁⁽²⁾  = T₁₂ μ₁⁽¹⁾
	  //New Shortcut to determine clear outliers
//	  double delta_sq_norm = (mu_1_in_frame_2 - mu_2).head<2>().squaredNorm();
	  with_depth_error = (mu_1_in_frame_2 - mu_2).head<3>().squaredNorm();

//	  std::cout<<" dist:"<<with_depth_error<<" points: "<<mu_1_in_frame_2(0)<<","<<mu_1_in_frame_2(1)<<","<<mu_1_in_frame_2(2)
//	     			<<"->"<<mu_2(0)<<","<<mu_2(1)<<","<<mu_2(2);

	  return with_depth_error;
}

}
