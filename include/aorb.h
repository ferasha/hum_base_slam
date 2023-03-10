#ifndef RGBDSLAM_AORB
#define RGBDSLAM_AORB
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

#include "opencv2/core/core.hpp"
/*!
 Adjustable ORB implementation. Modified from original ORB by Felix Endres
*/

using namespace std;

namespace cv {
class AORB : public Feature2D
{
public:
    // the size of the signature in bytes
    enum { kBytes = 32, HARRIS_SCORE=0, FAST_SCORE=1 };

    explicit AORB(int nfeatures = 500, float scaleFactor = 1.2f, int nlevels = 8, int edgeThreshold = 31,
                 int firstLevel = 0, int WTA_K=2, int scoreType=0, int patchSize=31, int fastThreshold = 20 );

    // returns the descriptor size in bytes
    int descriptorSize() const;
    // returns the descriptor type
    int descriptorType() const;

    // Compute the AORB features and descriptors on an image
    void operator()(InputArray image, InputArray mask, vector<KeyPoint>& keypoints) const;

    // Compute the AORB features and descriptors on an image
    void operator()( InputArray image, InputArray mask, vector<KeyPoint>& keypoints,
                     OutputArray descriptors, bool useProvidedKeypoints=false ) const;
  
    AlgorithmInfo* info() const;
    
protected:

    void computeImpl( const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors ) const;
    void detectImpl( const Mat& image, vector<KeyPoint>& keypoints, const Mat& mask=Mat() ) const;
    
    CV_PROP_RW int nfeatures;
    CV_PROP_RW double scaleFactor;
    CV_PROP_RW int nlevels;
    CV_PROP_RW int edgeThreshold;
    CV_PROP_RW int fastThreshold;
    CV_PROP_RW int firstLevel;
    CV_PROP_RW int WTA_K;
    CV_PROP_RW int scoreType;
    CV_PROP_RW int patchSize;
};
    
typedef AORB AorbFeatureDetector;
typedef AORB AorbDescriptorExtractor;


}
#endif
