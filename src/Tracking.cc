/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>


using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;


    int nUseORB = fSettings["useORB"];
    mbORB = nUseORB;
    cout << endl << "Descriptor: ";
    if (mbORB)
    	cout << "ORB" << endl;
    else
    	cout << "New descriptor" << endl;


    max_inlier_error = fSettings["max_inlier_error"];
    std::cout<<"max_inlier_error "<<max_inlier_error<<std::endl;

    min_inliers_threshold = fSettings["min_inliers_threshold"];
    std::cout<<"min_inliers_threshold "<<min_inliers_threshold<<std::endl;

    optimization_max_error = fSettings["optimization_max_error"];
    std::cout<<"optimization_max_error "<<optimization_max_error<<std::endl;

    ransac_iterations = fSettings["ransac_iterations"];
    std::cout<<"ransac_iterations "<<ransac_iterations<<std::endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    detector_ = createDetector("ORB");

    if (mbORB)
    	extractor_ = createDescriptorExtractor("ORB");
    else
//    	extractor_ = new cv::SURF();
    	extractor_ = new NewDesc();

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

    mbadsequentialMain = 0;
    mbadsequential = 0;
    mbadlocalmap = 0;


}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
	cv::Mat color_img;
	imRGB.copyTo(color_img);

    mImGray = imRGB;
//    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

//    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
//        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    if (mZero.empty())
    	mZero = cv::Mat::zeros(imD.size(), imD.type());

    cv::Mat imDepth;// = imD;
	cv::Mat mask;
	if(imD.type() == CV_32F) {
		imDepth = cv::Mat::zeros(imD.size(), imD.type());
		mask = (imD == imD);
		imD.copyTo(imDepth, mask);
		cv::cvtColor(color_img, color_img, CV_BGR2RGB);
	}
	else {
//		std::cout<<"type "<<imD.type()<<std::endl;
		imD.convertTo(imDepth,CV_32F,mDepthMapFactor);
		mask = (imD != mZero);
	}

    if (!mbORB) {
    	cameraFrame frame(imDepth, color_img);
		static_cast<cv::Ptr<NewDesc> >(extractor_)->currentFrame = frame;
    }

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,
    		extractor_, mbORB, mask, detector_);

    Track();
/*
    mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

    if (mCurrentFrame.mnId == 0)
    	StereoInitialization();
    else
    	CreateNewKeyFrame();

    mpLocalMapper->MainProcessing();
*/
    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    unique_lock<mutex> lockT(mpMap->mMutexLCTracking);

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;
        bool try_again = false;
        int nmatchesM = 100;
        bool bOKMap = true;
        bool bOKRansac = true;


        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame(mLastFrame);

                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
//                    bOK = TrackReferenceKeyFrame();
                	std::cout<<"mnLastRelocFrameId "<<mnLastRelocFrameId<<std::endl;
//                	UpdateLastFrame();
                   	bOK = TrackLastFrameRansac(mLastFrame, nmatchesM, try_again);
                    bOKRansac = bOK;
                }
                else
                {
//                    bOK = TrackWithMotionModel();
 //                   UpdateLastFrame();
                    UpdateFrame(mLastFrame);
                	bOK = TrackLastFrameRansac(mLastFrame, nmatchesM, try_again);
                    bOKRansac = bOK;
                }
            }
            else
            {
            	bOK = false;
            	std::cout<<"Relocalization"<<std::endl;
                //bOK = Relocalization();
            	/*
            	for (std::list<Frame>::reverse_iterator it=potentialRansacKFs.rbegin();
            			it!=potentialRansacKFs.rend(); ++it) {
            			 CheckReplacedInLastFrame(*it);
                         UpdateFrame(*it);
                         std::cout<<"Relocalization tracking with potentialRansacKFs ID "<<it->mnId
                          		 <<" out of "<<potentialRansacKFs.size()<<std::endl;
                         bOK = TrackLastFrameRansac(*it, nmatchesM, try_again);
                         bOKRansac = bOK;
                         if (bOK)
                        	 break;
                	}
            	*/

            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated
        	bOK = TrackNoMapping();
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK) {
                bOK = TrackLocalMap(mLastFrame);
                bOKMap = bOK;
            }
            if(!bOK){
            	for (std::list<Frame>::reverse_iterator it=potentialRansacKFs.rbegin();
            			it!=potentialRansacKFs.rend(); ++it) {
            			 CheckReplacedInLastFrame(*it);
                         UpdateFrame(*it);
                         if (mCurrentFrame.mnId - it->mnId > 1)
                        	 std::cout<<"tracking with potentialRansacKFs ID "<<it->mnId
                        		 <<" out of "<<potentialRansacKFs.size()<<std::endl;
                         bOK = TrackLastFrameRansac(*it, nmatchesM, try_again);
                         bOKRansac = bOK;
                         if (bOK) {
                        	 bOKMap = TrackLocalMap(*it);
                             bOK = bOKMap;
                      /*       if (!bOK){
                            	 bOK = TrackLastFrameRansac(*it, nmatchesM, try_again);
                            	 bOKRansac = bOK;
                             }
                      */
                         }
                         if (bOK)
                        	 break;
                	}
            }

            if (!bOKRansac) {
            	mbadsequentialMain++;
                std::cout<<"TrackSequential-Main not ok!"<<std::endl;
            }

            if (!bOKMap)
            {
         	   mbadlocalmap++;
         	   std::cout<<"TrackLocalMap not OK"<<std::endl;
            }
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap(mLastFrame);
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }
/*
            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();
*/
            // Check if we need to insert a new keyframe
 //           if(NeedNewKeyFrame())
 //           if (mCurrentFrame.mnId>=mnLastKeyFrameId+10)
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }

            mCurrentFrame.mTRelative = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
            potentialRansacKFs.push_back(mCurrentFrame);
        }

        while (potentialRansacKFs.size() > 5) {
        	potentialRansacKFs.pop_front();
       	}

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

    std::cout<<mCurrentFrame.mnId<<"("<<mCurrentFrame.mpReferenceKF->mnFrameId<<","
    		<<mCurrentFrame.mpReferenceKF->mnId<<") "<<std::endl;

    if (mState == OK)
    	mpLocalMapper->MainProcessing();

}

bool Tracking::TrackLastFrameRansac(Frame& olderFrame, int& nmatchesMap, bool& try_again, bool all_matches,
		double dist_ratio, bool only_mappoints)
{
//	std::cout<<"TrackLastFrameRansac "<<std::endl;

	try_again = true;
/*
	if (dist_ratio > 1.5)
		dist_ratio = 0.7;
*/
//	dist_ratio = 0.7;

	only_mappoints = false;

	std::cout<<"only_mappoints "<<only_mappoints<<" dist_ratio "<<dist_ratio<<std::endl;

	Ransac<Frame> ransac;
//	Ransac ransac;
	cv::Mat transf;
    vector<MapPoint*> vpMapPointMatches;
    int matches_size;
    float match_perc;

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    mCurrentFrame.SetPose(olderFrame.mTcw);

    ransac.settings.max_inlier_error = max_inlier_error;
    ransac.settings.min_inliers_threshold = min_inliers_threshold;
    ransac.settings.dist_ratio = dist_ratio;
    ransac.settings.only_map = only_mappoints;

    mCurrentFrame.mnInliers = 0;

    int olderframe_mappoints = olderFrame.N;
    for (int i=0; i<olderFrame.N; i++){
        MapPoint* pMP = olderFrame.mvpMapPoints[i];
        if(!pMP)
        	olderframe_mappoints--;
        else if(pMP->Observations()<1 || pMP->isBad())
        {
        	olderframe_mappoints--;
        }
    }
    std::cout<<"olderframe_mappoints "<<olderframe_mappoints<<std::endl;

    std::map<int, cv::DMatch> query_vec;
    bool good_tranf = ransac.getTransformation(olderFrame, mCurrentFrame, transf,
    		vpMapPointMatches, matches_size, try_again, query_vec, match_perc);

    nmatchesMap = 0;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;

    float diff_norm = 0;

    if(!good_tranf) {
  //  	std::cout<<"not good ransac transf returning"<<std::endl;
 //   	return false;
    }
    else if (!transf.empty() && (abs(transf.at<float>(0,3)) >= 0.2 || abs(transf.at<float>(1,3)) >= 0.2
     		|| abs(transf.at<float>(2,3)) >= 0.2)) {
     	std::cout<<"very large ransac translation"<<std::endl;
 //    	if (!all_matches)
 //    		return false;
     }

    mCurrentFrame.mnInliers = matches_size;

    bool track_res = true;
/*
	std::vector<cv::DMatch> matches;
	for (std::map<int, cv::DMatch>::iterator it=query_vec.begin(); it!=query_vec.end(); it++){
		matches.push_back(it->second);
	}
*/
/*
	 cv::Mat initial_matches_img;
	 drawMatches(mCurrentFrame.mImRGB, mCurrentFrame.mvKeys,olderFrame.mImRGB, olderFrame.mvKeys,
			 matches, initial_matches_img, cv::Scalar::all(-1), cv::Scalar::all(-1));
	 cv::imshow("initial_matches_4", initial_matches_img);

	 cv::waitKey(0);
*/
    std::cout<<"matches_size "<<matches_size<<std::endl;
    if (matches_size < 100)
    {
    	std::cout<<"matches fewer than 100"<<std::endl;
    	/*
    	std::vector<cv::DMatch> matches;
    	for (std::map<int, cv::DMatch>::iterator it=query_vec.begin(); it!=query_vec.end(); it++){
    		matches.push_back(it->second);
    	}

		 cv::Mat initial_matches_img;
		 drawMatches(mCurrentFrame.mImRGB, mCurrentFrame.mvKeys,olderFrame.mImRGB, olderFrame.mvKeys,
				 matches, initial_matches_img, cv::Scalar::all(-1), cv::Scalar::all(-1));
		 cv::imshow("initial_matches_4", initial_matches_img);

		 cv::waitKey(0);
*/
		 track_res = false;
    }

    else {
//    float avg_chi2;
//   bool po_result = true;

//    mCurrentFrame.SetPose(transf*olderFrame.mTcw);
//    bool po_result = Optimizer::PoseOptimization(&mCurrentFrame, 2.0, avg_chi2, true);
    int po_result = Optimizer::PoseOptimization(&mCurrentFrame);

//    std::cout<<"po inliers "<<po_result<<std::endl;;

    int nmatchespo = 0;
    // Discard outliers
     nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
            }
            else {
            	nmatchespo++;
            	if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
            }
        }
    }

    if (nmatchesMap < 10) {
    	std::cout<<"nmatchesMap less than 10: "<<nmatchesMap<<std::endl;
    }
    std::cout<<"nmatchespo "<<nmatchespo<<" nmatchesMap "<<nmatchesMap<<std::endl;


    cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
    olderFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
    olderFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));

    float prev_L1_norm = 0;
    if (!mVelocity.empty())
    	prev_L1_norm = abs(mVelocity.at<float>(0,3)) + abs(mVelocity.at<float>(1,3)) + abs(mVelocity.at<float>(2,3));

    mVelocity = mCurrentFrame.mTcw*LastTwc;

    float L1_norm = abs(mVelocity.at<float>(0,3)) + abs(mVelocity.at<float>(1,3)) + abs(mVelocity.at<float>(2,3));

    std::cout<<mVelocity.at<float>(0,3)<<" "<<mVelocity.at<float>(1,3)<<" "<<mVelocity.at<float>(2,3)<<
				 " -- prev_L1 norm "<<prev_L1_norm<<" L1 norm "<<L1_norm<<" norm larger than 0.2 "<<(L1_norm>0.2) <<std::endl;

    cv::Mat pose1 = mCurrentFrame.mTcw;
    cv::Mat pose2 = olderFrame.mTcw;

    float da = abs(pose1.at<float>(0,3)-pose2.at<float>(0,3));
    float db = abs(pose1.at<float>(1,3)-pose2.at<float>(1,3));
    float dc = abs(pose1.at<float>(2,3)-pose2.at<float>(2,3));

    diff_norm = da + db + dc;
    std::cout<<"current_frame po "<<pose1.at<float>(0,3)<<" "<<pose1.at<float>(1,3)<<" "<<pose1.at<float>(2,3)<<
    		" older_frame po "<<pose2.at<float>(0,3)<<" "<<pose2.at<float>(1,3)<<" "<<pose2.at<float>(2,3)<<
    		" da,db,dc "<<da<<","<<db<<","<<dc<<
				 " diff norm "<<diff_norm<<" diff norm larger than 0.2 "<<(diff_norm>0.2 || diff_norm==0) <<std::endl;


    if (!po_result) {
    	std::cout<<"not good pose optimization"<<std::endl;
    	track_res = false;
    }
    else {
    	/*
    	if (diff_norm == 0)
        	track_res = false;
    	else if (diff_norm > 0.2 && mCurrentFrame.mnId - olderFrame.mnId == 1)
        	track_res = false;
        else if (diff_norm > 0.3)
        	track_res = false;
        */
//    	float max_dist = 0.06;
    	float max_dist = 0.2;//1.0; //0.15; //0.1;
    	float max_dist_norm = 0.3;//; //0.2; //0.15;
    	if ((mCurrentFrame.mnId - olderFrame.mnId) > 1) {
//    		max_dist = 0.15 + 0.05*(mCurrentFrame.mnId - olderFrame.mnId);
//    		max_dist_norm = 0.25 + 0.05*(mCurrentFrame.mnId - olderFrame.mnId);
    		max_dist = max_dist + 0.05*(mCurrentFrame.mnId - olderFrame.mnId);
    		max_dist_norm = max_dist_norm + 0.05*(mCurrentFrame.mnId - olderFrame.mnId);
    	}

  //  		max_dist = 0.04*(mCurrentFrame.mnId - olderFrame.mnId);
  //  	float max_dist = 0.2*(mCurrentFrame.mnId - olderFrame.mnId);
    	if (diff_norm == 0)
        	track_res = false;
    	else if (da>max_dist || db>max_dist || dc>max_dist || diff_norm >max_dist_norm) {
    		std::cout<<"large indiv trans"<<std::endl;
 //   		if (match_perc < 0.7 || po_result < 10)
//        	if (po_result < 20 || dist_ratio >= 0.8)
 //   			track_res = false;
    	}

//    	if (diff_norm > 0.2 && po_result < 20 && dist_ratio >= 0.8)
//        if (diff_norm > 0.2 && po_result < 10 && dist_ratio >= 0.8)
//    		track_res = false;

//        if (diff_norm > 0.2 && dist_ratio >= 0.8)
//    		track_res = false;

//good 369
//        if ((nmatchesMap < 5 && po_result < 20) || nmatchesMap == 0)
//        	track_res = false;

    	/*
        if (nmatchesMap < 10)
        	if (only_mappoints)
        		track_res = false;
        	else if (po_result < 20)
        		track_res = false;
*/

    	 if (nmatchesMap < 10)
    		 std::cout<<"nmatchesMap less than 10"<<std::endl;

  //      if (diff_norm > 0.2 && nmatchesMap < 15 && dist_ratio >= 0.8)
  //      	track_res = false;

  //  	 	if (po_result < 20 && dist_ratio >= 0.8)
  //  	 		track_res = false;

/*
        if (L1_norm > 0.2 && mCurrentFrame.mnId - olderFrame.mnId == 1) {
        	return false;
        }
        else if (L1_norm > 0.25)
        	return false;
*/
    }
    }

    std::cout<<"track_res "<<track_res<<std::endl;
/*
    if ((!track_res || diff_norm > 0.2) && matches_size >=4){
//        if (matches_size >=4){

    	std::vector<cv::DMatch> matches, inliers_corr;
    	for (std::map<int, cv::DMatch>::iterator it=query_vec.begin(); it!=query_vec.end(); it++){
    		matches.push_back(it->second);
    		if (!mCurrentFrame.mvbOutlier[it->first])
    			inliers_corr.push_back(it->second);
    	}


		 cv::Mat initial_matches_img;
		 drawMatches(mCurrentFrame.mImRGB, mCurrentFrame.mvKeys,olderFrame.mImRGB, olderFrame.mvKeys,
				 matches, initial_matches_img, cv::Scalar::all(-1), cv::Scalar::all(-1));
		 cv::imshow("intial_matches", initial_matches_img);

		 cv::Mat inliers_corr_img;
		 drawMatches(mCurrentFrame.mImRGB, mCurrentFrame.mvKeys,olderFrame.mImRGB, olderFrame.mvKeys,
				 inliers_corr, inliers_corr_img, cv::Scalar::all(-1), cv::Scalar::all(-1));
		 cv::imshow("inlier correspondences", inliers_corr_img);

		 cv::waitKey(0);
    }
*/
/*
    if (!track_res && !try_again) {
    	try_again = true;
    	track_res = TrackLastFrameRansac(olderFrame, nmatchesMap, try_again);
    }
*/
/*
    if (!track_res && dist_ratio >= 0.8){
    	if (only_mappoints) {
    		only_mappoints = false;
    	}

    	if (dist_ratio == 2.0)
    		dist_ratio = 1.0;
    	else
    		dist_ratio = dist_ratio - 0.1;

    	try_again = true;
    	track_res = TrackLastFrameRansac(olderFrame, nmatchesMap, try_again, false, dist_ratio, only_mappoints);
    }
*/

    if (!track_res && dist_ratio < 2.0){
    	if (only_mappoints) {
    		only_mappoints = false;
    	}

    	if (dist_ratio >= 1.0)
    		dist_ratio = 2.0;
    	else
    		dist_ratio = dist_ratio + 0.1;

    	try_again = true;
    	track_res = TrackLastFrameRansac(olderFrame, nmatchesMap, try_again, false, dist_ratio, only_mappoints);
    }

/*
    if (!track_res && dist_ratio >= 0.8){
    	if (dist_ratio == 2.0)
    		dist_ratio = 1.0;
    	else
    		dist_ratio = dist_ratio - 0.1;

    	try_again = true;
    	track_res = TrackLastFrameRansac(olderFrame, nmatchesMap, try_again, false, dist_ratio);
    }
*/
 /*
    if (!track_res && dist_ratio < 1.1){
		dist_ratio = dist_ratio + 0.1;

    	try_again = true;
    	track_res = TrackLastFrameRansac(olderFrame, nmatchesMap, try_again, false, dist_ratio);
    }
*/
    try_again = false;
    return track_res;

}

void Tracking::UpdateFrame(Frame& frame)
{
    cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
    frame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
    frame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));

    cv::Mat old_pose = frame.mTcw;

    // Update pose according to reference keyframe
    KeyFrame* pRef = frame.mpReferenceKF;

    if (pRef->isBad())
    	std::cout<<"UpdateFrame pRef->isBad() frame "<<frame.mnId<<" pRef->mnId "<<pRef->mnId<<
    	" pRef->mnFrameId "<<pRef->mnFrameId<<std::endl;

    if (pRef->mnFrameId == frame.mnId)
    	frame.SetPose(pRef->GetPose());
    else {
       	std::cout<<"frame and keyframe are different: frame "<<frame.mnId<<" pRef->mnId "<<pRef->mnId<<
        	    	" pRef->mnFrameId "<<pRef->mnFrameId<<std::endl;
    	frame.SetPose(frame.mTRelative*pRef->GetPose());
    }

    cv::Mat change = frame.mTcw*LastTwc;
    if (abs(change.at<float>(0,3)) > 0.1 || abs(change.at<float>(1,3)) > 0.1 || abs(change.at<float>(2,3)) > 0.1) {
    	std::cout<<"change larger than 0.1, frame.id: "<<frame.mnId<<std::endl;
    	std::cout<<change.at<float>(0,3)<<" "<<change.at<float>(1,3)<<" "<<change.at<float>(2,3)<<std::endl;

     	std::cout<<"id "<<frame.mnId<<" before: "<<old_pose.at<float>(0,3)<<" "
     			<<old_pose.at<float>(1,3)<<" "<<old_pose.at<float>(2,3)<<std::endl;

     	std::cout<<"id "<<frame.mnId<<" after: "<<frame.mTcw.at<float>(0,3)<<" "
     			<<frame.mTcw.at<float>(1,3)<<" "<<frame.mTcw.at<float>(2,3)<<std::endl;
    }

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(frame.N);
    for(int i=0; i<frame.N;i++)
    {
        float z = frame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = frame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1 || pMP->isBad())
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = frame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&frame,i);

            frame.mvpMapPoints[i]=pNewMP;

 //           mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>1000)
            break;
    }

    std::cout<<"nPoints "<<nPoints<<std::endl;

}

bool Tracking::TrackNoMapping(){

   	bool bOK = false;

	if(mState==LOST)
    {
        bOK = Relocalization();
    }
    else
    {
        if(!mbVO)
        {
            // In last frame we tracked enough MapPoints in the map

            if(!mVelocity.empty())
            {
                bOK = TrackWithMotionModel();
            }
            else
            {
                bOK = TrackReferenceKeyFrame();
            }
        }
        else
        {
            // In last frame we tracked mainly "visual odometry" points.

            // We compute two camera poses, one from motion model and one doing relocalization.
            // If relocalization is sucessfull we choose that solution, otherwise we retain
            // the "visual odometry" solution.

            bool bOKMM = false;
            bool bOKReloc = false;
            vector<MapPoint*> vpMPsMM;
            vector<bool> vbOutMM;
            cv::Mat TcwMM;
            if(!mVelocity.empty())
            {
                bOKMM = TrackWithMotionModel();
                vpMPsMM = mCurrentFrame.mvpMapPoints;
                vbOutMM = mCurrentFrame.mvbOutlier;
                TcwMM = mCurrentFrame.mTcw.clone();
            }
            bOKReloc = Relocalization();

            if(bOKMM && !bOKReloc)
            {
                mCurrentFrame.SetPose(TcwMM);
                mCurrentFrame.mvpMapPoints = vpMPsMM;
                mCurrentFrame.mvbOutlier = vbOutMM;

                if(mbVO)
                {
                    for(int i =0; i<mCurrentFrame.N; i++)
                    {
                        if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                        {
                            mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                        }
                    }
                }
            }
            else if(bOKReloc)
            {
                mbVO = false;
            }

            bOK = bOKReloc || bOKMM;
        }
    }

	return bOK;
}

void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame(Frame& frame)
{
    for(int i =0; i<frame.N; i++)
    {
        MapPoint* pMP = frame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                frame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
	std::cout<<"TrackReferenceKeyFrame "<<std::endl;

    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

//    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
//        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());


//    std::cout<<"vDepthIdx.size() "<<vDepthIdx.size()<<std::endl;

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

  //          mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }
/*
        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
*/
    }
//    std::cout<<"nPoints "<<nPoints<<std::endl;

}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap(Frame& olderFrame)
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

 //   return true;

    SearchLocalPoints();

    cv::Mat pose1 = mCurrentFrame.mTcw;
    cv::Mat pose2 = olderFrame.mTcw;

    float da = abs(pose1.at<float>(0,3)-pose2.at<float>(0,3));
    float db = abs(pose1.at<float>(1,3)-pose2.at<float>(1,3));
    float dc = abs(pose1.at<float>(2,3)-pose2.at<float>(2,3));

    float diff_norm_po = da + db + dc;

    cv::Mat pose_po = mCurrentFrame.mTcw.clone();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;
    int allinliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                	allinliers++;
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

	std::cout<<"TrackLocalMap mnMapMatchesInliers "<<mnMatchesInliers<<" allinliers "<<allinliers<<std::endl;

//    return true;

    pose1 = mCurrentFrame.mTcw;
    pose2 = olderFrame.mTcw;

    da = abs(pose1.at<float>(0,3)-pose2.at<float>(0,3));
    db = abs(pose1.at<float>(1,3)-pose2.at<float>(1,3));
    dc = abs(pose1.at<float>(2,3)-pose2.at<float>(2,3));

    float diff_norm = da + db + dc;
    std::cout<<"current_frame po "<<pose1.at<float>(0,3)<<" "<<pose1.at<float>(1,3)<<" "<<pose1.at<float>(2,3)<<
    		" older_frame po "<<pose2.at<float>(0,3)<<" "<<pose2.at<float>(1,3)<<" "<<pose2.at<float>(2,3)<<
    		" da,db,dc "<<da<<","<<db<<","<<dc<<
				 " diff norm "<<diff_norm<<" diff norm larger than 0.2 "<<(diff_norm>0.2 || diff_norm==0) <<std::endl;

    float max_dist = 0.2; //0.15; //0.1;
    float max_dist_norm = 0.3; //0.2; //0.15;

    if (da>max_dist || db>max_dist || dc>max_dist || diff_norm >max_dist_norm) {
    	std::cout<<"large tracklocalmap indiv trans"<<std::endl;
 //   	return false;
    }

    if (diff_norm > diff_norm_po && diff_norm - diff_norm_po > 0.01){
    	std::cout<<"larger than diff_norm_po "<<std::endl;
//    	return false;
    }

    da = abs(pose1.at<float>(0,3)-pose_po.at<float>(0,3));
    db = abs(pose1.at<float>(1,3)-pose_po.at<float>(1,3));
    dc = abs(pose1.at<float>(2,3)-pose_po.at<float>(2,3));

    diff_norm = da + db + dc;
    std::cout<<"dap,dbp,dcp "<<da<<","<<db<<","<<dc<<
				 " diff norm p "<<diff_norm<<" diff norm p larger than 0.2 "<<(diff_norm>0.2 || diff_norm==0) <<std::endl;

    max_dist = 0.2; //0.15; //0.1;
    max_dist_norm = 0.2; //0.2; //0.15;

    if (da>max_dist || db>max_dist || dc>max_dist || diff_norm >max_dist_norm) {
    	std::cout<<"large tracklocalmap diff_p"<<std::endl;
  //  	return false;
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
//    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50) {
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<20) {
 //   	std::cout<<"TrackLocalMap mnMatchesInliers "<<mnMatchesInliers<<std::endl;
 //      	std::cout<<"mnLastRelocFrameId "<<mnLastRelocFrameId<<" mMaxFrames "<<mMaxFrames<<std::endl;
 //   	return false;
    }

//    if(mnMatchesInliers<30) {
    if(mnMatchesInliers<10) {
//    	std::cout<<"TrackLocalMap mnMatchesInliers "<<mnMatchesInliers<<std::endl;
  //  	return false;
    }
    else
        return true;

    return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    std::cout<<"here "<<bLocalMappingIdle<<std::endl;

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
	/*
    if(!mpLocalMapper->SetNotStop(true)) {
    	std::cout<<"did not create a new keyframe"<<std::endl;
    	return;
    }
    */
    while(!mpLocalMapper->SetNotStop(true)) {
    	usleep(3000);
    }

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

//    mpLocalMapper->MainProcessing();

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.7);
        int th = 1;
        if(mSensor==System::RGBD)
            th=5;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }

}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
	std::cout<<"Relocalization "<<std::endl;

    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{
    mpViewer->RequestStop();

    cout << "System Reseting" << endl;
    while(!mpViewer->isStopped())
        usleep(3000);

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM
