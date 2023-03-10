/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //
 //                           License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
 // Copyright (C) 2009-2010, Willow Garage Inc., all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#include "feature_adjuster.h"
//#include "opencv2/features2d/precomp.hpp"
#include "opencv2/features2d/features2d.hpp"
#ifdef CV_NONFREE
#include "opencv2/nonfree/features2d.hpp"
#endif
#include "opencv2/imgproc/imgproc.hpp"
#include "aorb.h"
#include <cassert>
#include <iostream>
#include <algorithm> //for min
//#include <ros/ros.h>
using namespace cv;


DetectorAdjuster::DetectorAdjuster(const char* detector_name, double initial_thresh, double min_thresh, double max_thresh, double increase_factor, double decrease_factor ) :
    thresh_(initial_thresh), 
    min_thresh_(min_thresh), max_thresh_(max_thresh),
    increase_factor_(increase_factor), decrease_factor_(decrease_factor),
    detector_name_(detector_name)
{
}

void DetectorAdjuster::detectImpl(const Mat& image, std::vector<KeyPoint>& keypoints, const Mat& mask) const
{
    Ptr<FeatureDetector> detector; 
    if(strcmp(detector_name_, "FAST") == 0){
      //detector->set("threshold", static_cast<int>(thresh_));
      detector = new FastFeatureDetector(thresh_);
    }
    else if(strcmp(detector_name_, "STAR") == 0){
      //Default params except last
  //    detector = new StarFeatureDetector(16,static_cast<int>(thresh_),10,8,5);
      detector = new StarFeatureDetector(16,static_cast<int>(thresh_),10,8,5);
 //     std::cout<<"STAR detector in feature_adjuster"<<std::endl;
    }
    else if(strcmp(detector_name_, "AORB") == 0){
      //Default params except last
      detector = new AorbFeatureDetector(10000, 1.2, 8, 15, 0, 2, 0, 31, static_cast<int>(thresh_));
      //detector->set("fastThreshold", static_cast<int>(thresh_));//Not threadsafe (parallelized grid)
    }
#ifdef CV_NONFREE
    else if(strcmp(detector_name_, "SURF") == 0){
      //detector->set("hessianThreshold", thresh_);//Not threadsafe (parallelized grid)
      detector = new SurfFeatureDetector(thresh_);
    }
    else if(strcmp(detector_name_, "SIFT") == 0){
      //detector->set("contrastThreshold", thresh_);
      detector = new SiftFeatureDetector(0 /*max_features*/, 3 /*default lvls/octave*/, thresh_);
    }
#else
    else if(strcmp(detector_name_, "SIFT") == 0 || strcmp(detector_name_, "SURF") == 0){
        std::cerr << "OpenCV non-free functionality (" << detector_name_ << ") not built in.";
        std::cerr << "To enable non-free functionality build with CV_NONFREE set.";
        std::cerr << "Using ORB.";
        detector = new AorbFeatureDetector(10000, 1.2, 8, 15, 0, 2, 0, 31, static_cast<int>(thresh_));
    }
#endif
    else {
      FeatureDetector::create(detector_name_);
      std::cerr << "Unknown detector, "<< std::string(detector_name_)<<", not setting threshold"<<std::endl;
    }
    //ROS_INFO("Calling Detect with threshold %f", thresh_);
    //std::cout << "Performing detection with " << detector_name_ << ". Threshold: " << thresh_ << std::endl;
    detector->detect(image, keypoints, mask);
}

void DetectorAdjuster::setDecreaseFactor(double new_factor){
  decrease_factor_ = new_factor;
}
void DetectorAdjuster::setIncreaseFactor(double new_factor){
  increase_factor_ = new_factor;
}

void DetectorAdjuster::tooFew(int, int)
{
    thresh_ *= decrease_factor_;
    if (thresh_ < min_thresh_)
            thresh_ = min_thresh_;
}

void DetectorAdjuster::tooMany(int, int)
{
    thresh_ *= increase_factor_;
    if (thresh_ > max_thresh_)
            thresh_ = max_thresh_;
}

//return whether or not the threshhold is beyond
//a useful point
bool DetectorAdjuster::good() const
{
    return (thresh_ > min_thresh_) && (thresh_ < max_thresh_);
}

Ptr<AdjusterAdapter> DetectorAdjuster::clone() const
{
    Ptr<AdjusterAdapter> cloned_obj(new DetectorAdjuster(detector_name_, thresh_, min_thresh_, max_thresh_, increase_factor_, decrease_factor_ ));
    return cloned_obj;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////


 VideoDynamicAdaptedFeatureDetector::VideoDynamicAdaptedFeatureDetector(Ptr<AdjusterAdapter> a,
                                         int min_features, int max_features, int max_iters ) :
        escape_iters_(max_iters), min_features_(min_features), max_features_(max_features), adjuster_(a)
{}

cv::Ptr<StatefulFeatureDetector> VideoDynamicAdaptedFeatureDetector::clone() const 
{
  StatefulFeatureDetector* fd = new VideoDynamicAdaptedFeatureDetector(adjuster_->clone(), //clone adjuster, so threshold is not shared!
                                                                       min_features_, 
                                                                       max_features_, 
                                                                       escape_iters_);
  cv::Ptr<StatefulFeatureDetector> cloned_obj(fd);
  return cloned_obj;
}

bool VideoDynamicAdaptedFeatureDetector::empty() const
{
    return !adjuster_ || adjuster_->empty();
}

bool hasNonZero(const cv::Mat& img){
  for (int y = 0; y < img.cols; y++) {
    for(int x = 0; x < img.rows; x++) {
      if(img.at<uchar>(x,y) != 0) return true;
    }
  }
  return false;
}
void VideoDynamicAdaptedFeatureDetector::detectImpl(const cv::Mat& _image, std::vector<KeyPoint>& keypoints, const cv::Mat& _mask) const
{
    //In contraast to the original, no oscillation testing is needed as
    //the loop is broken out of anyway, if too many features were found.

    //break if the desired number hasn't been reached.
    int iter_count = escape_iters_;
    bool checked_for_non_zero_mask = false;

    do { // detect at least once
        keypoints.clear();

        //the adjuster takes care of calling the detector with updated parameters
        adjuster_->detect(_image, keypoints,_mask);
        //ROS_INFO("Detected %zu keypoints", keypoints.size());
        int found_keypoints = static_cast<int>(keypoints.size());
        if(found_keypoints < min_features_ )
        {
            adjuster_->tooFew(min_features_, found_keypoints);
            //Specific to depth images
            if(found_keypoints == 0 && !checked_for_non_zero_mask){
              checked_for_non_zero_mask = true;
              if(!hasNonZero(_mask)){
        //        std::cout << ("Breaking detection iterations, because of missing depth");
                break; //does not help to iterate if no points have depth
              }
            }
        }
        else if( int(keypoints.size()) > max_features_ )
        {
            adjuster_->tooMany(max_features_, (int)keypoints.size());
            break;//FIXME: Too many is ok as they are clipped anyway?
        }
        else
            break;

        iter_count--;
    } while( iter_count > 0 && adjuster_->good() );

}

/*
 *  VideoGridAdaptedFeatureDetector
 */
VideoGridAdaptedFeatureDetector::VideoGridAdaptedFeatureDetector( const cv::Ptr<StatefulFeatureDetector>& _detector, int _maxTotalKeypoints, int _gridRows, int _gridCols, int _edgeThreshold)
    : maxTotalKeypoints(_maxTotalKeypoints), gridRows(_gridRows), gridCols(_gridCols), edgeThreshold(_edgeThreshold)
{
  detectors.push_back(_detector);//Use original one
  while(detectors.size() < gridRows*gridCols){
    detectors.push_back(_detector->clone());//clone, so the state is not shared
  }
}

bool VideoGridAdaptedFeatureDetector::empty() const
{
    for(std::vector<cv::Ptr<StatefulFeatureDetector> >::const_iterator it = detectors.begin(); it != detectors.end(); ++it){
      if((*it)->empty()) return true;
    }
    return false;
}

struct ResponseComparator
{
    bool operator() (const KeyPoint& a, const KeyPoint& b)
    {
        return std::abs(a.response) > std::abs(b.response);
    }
};

void keepStrongest( int N, vector<KeyPoint>& keypoints )
{
    if( (int)keypoints.size() > N )
    {
        std::vector<cv::KeyPoint>::iterator nth = keypoints.begin() + N;
        std::nth_element( keypoints.begin(), nth, keypoints.end(), ResponseComparator() );
        keypoints.erase( nth, keypoints.end() );
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
///Helper function for detectImpl below
static void aggregateKeypointsPerGridCell(std::vector<std::vector<cv::KeyPoint> >& sub_keypoint_vectors, //will be modified to global coordinates
                                          std::vector<cv::KeyPoint>& keypoints_out, //output
                                          cv::Size gridSize, 
                                          cv::Size imageSize, 
                                          int edgeThreshold) 
{
    for(int i = 0; i < gridSize.height; ++i)
    {
        int rowstart = std::max((i*imageSize.height)/gridSize.height - edgeThreshold, 0);
        for( int j = 0; j < gridSize.width; ++j )
        {
            int colstart = std::max((j*imageSize.width)/gridSize.width - edgeThreshold, 0);

            std::vector<cv::KeyPoint>& cell_keypoints = sub_keypoint_vectors[j+i*gridSize.width];
            std::vector<cv::KeyPoint>::iterator it = cell_keypoints.begin(), end = cell_keypoints.end();
            for( ; it != end; ++it )
            {
                it->pt.x += colstart;
                it->pt.y += rowstart;
            }
            keypoints_out.insert(keypoints_out.end(), cell_keypoints.begin(), cell_keypoints.end());
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
void VideoGridAdaptedFeatureDetector::detectImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask ) const
{
    std::vector<std::vector<cv::KeyPoint> > sub_keypoint_vectors(gridCols*gridRows);
    keypoints.reserve(maxTotalKeypoints);
    int maxPerCell = maxTotalKeypoints / (gridRows * gridCols);
#pragma omp parallel for
    for( int i = 0; i < gridRows; ++i )
    {
        int rowstart = std::max((i*image.rows)/gridRows - edgeThreshold, 0);
        int rowend   = std::min(image.rows, ((i+1)*image.rows)/gridRows + edgeThreshold);
        cv::Range row_range(rowstart, rowend);
#pragma omp parallel for
        for( int j = 0; j < gridCols; ++j )
        {
            int colstart = std::max((j*image.cols)/gridCols - edgeThreshold, 0);
            int colend   = std::min(image.cols, ((j+1)*image.cols)/gridCols + edgeThreshold);
            cv::Range col_range(colstart, colend);
            cv::Mat sub_image = image(row_range, col_range);
            cv::Mat sub_mask;
            if( !mask.empty()){
                sub_mask = mask(row_range, col_range);
            }

            std::vector<cv::KeyPoint>& sub_keypoints = sub_keypoint_vectors[j+i*gridCols];
            detectors[j+i*gridCols]->detect( sub_image, sub_keypoints, sub_mask );
            keepStrongest( maxPerCell, sub_keypoints );
        }
    }
    aggregateKeypointsPerGridCell(sub_keypoint_vectors, keypoints, cv::Size(gridCols, gridRows), image.size(), edgeThreshold);
}

cv::Ptr<StatefulFeatureDetector> VideoGridAdaptedFeatureDetector::clone() const 
{
  StatefulFeatureDetector* fd = new VideoGridAdaptedFeatureDetector(detectors[0]->clone(), //clone detector, so threshold is not shared!
                                                                    maxTotalKeypoints, 
                                                                    gridRows, gridCols, 
                                                                    edgeThreshold);
  cv::Ptr<StatefulFeatureDetector> cloned_obj(fd);
  return cloned_obj;
}

StatefulFeatureDetector* adjusterWrapper(cv::Ptr<DetectorAdjuster> detadj, int min, int max)
{
  int iterations = 5;
//  ROS_INFO("Using adjusted keypoint detector with %d maximum iterations, keeping the number of keypoints between %d and %d", iterations, min, max);
  return new  VideoDynamicAdaptedFeatureDetector(detadj, min, max, iterations);
}

StatefulFeatureDetector* adjustedGridWrapper(cv::Ptr<DetectorAdjuster> detadj)
{
  //Try to get between "max" keypoints and 1.5 times of that.
  //We actually want exactly "max_keypoints" keypoints. Therefore
  //it's better to overshoot and then cut away the excessive keypoints
  int min = 600; //Shall not get below max
  int max = min * 1.5; //

  int gridRes = 3;
  int gridcells = gridRes*gridRes;
  int gridmin = round(min/static_cast<float>(gridcells));
  int gridmax =  round(max/static_cast<float>(gridcells));
//  ROS_INFO("Using gridded keypoint detector with %dx%d cells, keeping %d keypoints in total.", gridRes, gridRes, max);
  
  StatefulFeatureDetector* detector = adjusterWrapper(detadj, gridmin, gridmax);

  return new VideoGridAdaptedFeatureDetector(detector, max, gridRes, gridRes);
}

//Use Grid or Dynamic or GridDynamic as prefix of FAST, SIFT, SURF or AORB
FeatureDetector* createDetector(const std::string& detectorName){

	std::cout<<"detectorName_string "<<detectorName<<std::endl;
	std::cout<<"detectorName_char_to_string "<<std::string(detectorName.c_str())<<std::endl;
  //For anything but SIFTGPU
//  const char* name = "AORB";
//  DetectorAdjuster* detAdj = new DetectorAdjuster(name, 20);
  int thresh_ = 20;
  if (detectorName == "STAR")
	  thresh_ = 10;
  const char* name = "STAR";
  DetectorAdjuster* detAdj = new DetectorAdjuster(name, thresh_);
  return adjustedGridWrapper(detAdj);
}

DescriptorExtractor* createDescriptorExtractor(const std::string& descriptorType) 
{
    DescriptorExtractor* extractor = 0;
    extractor = new OrbDescriptorExtractor();
    return extractor;
}
