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

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include<mutex>
#include<thread>


namespace ORB_SLAM2
{

LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale,
		const string &strSettingPath):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0)
{
    mnCovisibilityConsistencyTh = 3;
    fabmapLC.initFabmap(strSettingPath);
    detected = 0;
    computesim3 = 0;

}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void LoopClosing::MainLCLoop(){

    if(CheckNewKeyFrames())
    {
		if (DetectLoopFabmap())
//		if (DetectLoop())
    	{
			detected++;
//			return;
		   // Compute similarity transformation [sR|t]
		   // In the stereo/RGBD case s=1
		   if(ComputeSim3())
//			if (ComputeSim3_old())
		   {
			   computesim3++;
			   // Perform loop fusion and pose graph optimization
			   CorrectLoop();
		   }
		}
    }
}

void LoopClosing::Run()
{
    mbFinished =false;

    while(1)
    {
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Detect loop candidates and check covisibility consistency
         //   if(DetectLoop())
        	if (DetectLoopFabmap())
            {
            	detected++;
               // Compute similarity transformation [sR|t]
               // In the stereo/RGBD case s=1
               if(ComputeSim3())
               {
            	   computesim3++;
                   // Perform loop fusion and pose graph optimization
                   CorrectLoop();
               }
            }
        }       

        ResetIfRequested();

        if(CheckFinish() && !CheckNewKeyFrames())
            break;

        usleep(1000);
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    std::cout<<"KFs "<<vpKFs.size()<<" detected "<<detected<<" computesim3 "<<computesim3<<std::endl;

    SetFinish();
}

void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
//    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::DetectLoopFabmap()
{
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

//    mfabmapallKFs++;

    vector<KeyFrame*> vpCandidateKFs = fabmapLC.checkForLoopClosure(mpCurrentKF);
//    if (!vpCandidateKFs.empty())
//    	mfabmapCandidate++;

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();
        mpCurrentKF->SetErase();
        return false;
    }
    
    else {
  /*
    	if ((mpCurrentKF->mnFrameId - vpCandidateKFs[0]->mnFrameId) < 1000 || mpCurrentKF->mnFrameId < 2000){
//  	if ((mpCurrentKF->mnFrameId - vpCandidateKFs[0]->mnFrameId) < 100){  
          mpKeyFrameDB->add(mpCurrentKF);
            mvConsistentGroups.clear();
            mpCurrentKF->SetErase();
            return false;
    	}
   */
    }
    
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->mnId == vpCandidateKFs[0]->mnId) {
        	std::cout<<"loop KF is one of the covisible KFs"<<std::endl;
        	return false;
        }
    }

    mvpEnoughConsistentCandidates = vpCandidateKFs;
    return true;
}


bool LoopClosing::DetectLoop()
{
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;
    }

    std::cout<<"min score "<<minScore<<std::endl;

    // Query the database imposing the minimum score
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();
        mpCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    mvpEnoughConsistentCandidates.clear();

    std::cout<<fixed<<"DBcurrentKF "<<mpCurrentKF->mnFrameId<<" "<<mpCurrentKF->mTimeStamp;

    bool first_time = true;

    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];
        std::cout<<fixed<<" candidate "<<pCandidateKF->mnFrameId<<" "<<pCandidateKF->mTimeStamp<<" ";

        std::cout<<std::endl;
        cv::Mat transf;
        vector<MapPoint*> vpMapPointMatches;
        cv::Mat imRGBCurrent;
        cv::Mat imRGBOld;
        double dist_ratio = 0.7;
        int nmatches = 0;

 //       if (mpCurrentKF->mnFrameId > 430) {
			while(nmatches<20 && dist_ratio < 1.2)
			{
				nmatches = RunRansac(imRGBCurrent, imRGBOld, vpMapPointMatches, dist_ratio, pCandidateKF, transf);
				if (nmatches > 20 && first_time){
					std::cout<<"lc nmatches larger than 0"<<std::endl;
					first_time = false;
				}
				dist_ratio = dist_ratio + 0.1;
			}
  //      }

        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }

	if (first_time) {
		std::cout<<"lc no candidate is good"<<std::endl;
	}

    std::cout<<std::endl;

    // Update Covisibility Consistent Groups
    mvConsistentGroups = vCurrentConsistentGroups;


    // Add Current Keyframe to database
    mpKeyFrameDB->add(mpCurrentKF);

    if(mvpEnoughConsistentCandidates.empty())
    {
        mpCurrentKF->SetErase();
        return false;
    }
    else
    {
        return true;
    }

    mpCurrentKF->SetErase();
    return false;
}

int LoopClosing::RunRansac(cv::Mat& imRGBCurrent, cv::Mat& imRGBOld, vector<MapPoint*>& vpMapPointMatches,
		double dist_ratio, KeyFrame *pKF, cv::Mat& transf){

	int nmatches;
	float match_perc;
    std::map<int, cv::DMatch> query_vec;

    Ransac<KeyFrame> ransac;
    ransac.settings.dist_ratio = dist_ratio;
    bool found_tranf = ransac.getTransformation(*pKF, *mpCurrentKF, transf,
    		vpMapPointMatches, nmatches, true, query_vec, match_perc);
    std::cout<<"found transf "<<found_tranf<<" loop closure matches "<<nmatches<<std::endl;

/*
    if (nmatches > 0) {

//    	std::cout<<"lc nmatches larger than 0"<<std::endl;

        if (nmatches < 20) {

    	std::vector<cv::DMatch> matches;
		for (std::map<int, cv::DMatch>::iterator it=query_vec.begin(); it!=query_vec.end(); it++){
			matches.push_back(it->second);
		}

		std::stringstream path;
		path<<fixed<<"/media/rasha/Seagate Backup Plus Drive/nao_motion_capture_2/experiment8/rgb/"<<
					setprecision(6)<<mpCurrentKF->mTimeStamp<<".png";
		std::stringstream pathOld;
		pathOld<<fixed<<"/media/rasha/Seagate Backup Plus Drive/nao_motion_capture_2/experiment8/rgb/"<<
					setprecision(6)<<pKF->mTimeStamp<<".png";

		imRGBCurrent = cv::imread(path.str(),CV_LOAD_IMAGE_COLOR);
		imRGBOld = cv::imread(pathOld.str(),CV_LOAD_IMAGE_COLOR);


		cv::Mat initial_matches_img;
		drawMatches(imRGBCurrent, mpCurrentKF->mvKeys, imRGBOld, pKF->mvKeys,
				 matches, initial_matches_img, cv::Scalar::all(-1), cv::Scalar::all(-1));
		cv::imshow("initial_matches", initial_matches_img);

			 cv::waitKey(0);
        }
    }
*/

    return nmatches;

}

bool LoopClosing::ComputeSim3_old()
{
    // For each consistent loop candidate we try to compute a Sim3

    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    for(int i=0; i<nInitialCandidates; i++)
    {
        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->SetNotErase();

        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }

        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);

        std::cout<<"bownmatches "<<nmatches<<std::endl;
        if(nmatches<10)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
        	if (mpCurrentKF->mnId > 0) {
/* 
	       	std::stringstream path;
        	path<<fixed<<"/media/rasha/Seagate Backup Plus Drive/TUM_dataset/rgbd_dataset_freiburg2_pioneer_360/rgb/"<<
        	    		setprecision(6)<<mpCurrentKF->mTimeStamp<<".png";
        	std::stringstream pathOld;
        	pathOld<<fixed<<"/media/rasha/Seagate Backup Plus Drive/TUM_dataset/rgbd_dataset_freiburg2_pioneer_360/rgb/"<<
        	    		setprecision(6)<<pKF->mTimeStamp<<".png";
*/
/*
	       	std::stringstream path;
        	path<<fixed<<"/media/rasha/Seagate Backup Plus Drive/nao_motion_capture_2/experiment8/rgb/"<<
        	    		setprecision(6)<<mpCurrentKF->mTimeStamp<<".png";
        	std::stringstream pathOld;
        	pathOld<<fixed<<"/media/rasha/Seagate Backup Plus Drive/nao_motion_capture_2/experiment8/rgb/"<<
        	    		setprecision(6)<<pKF->mTimeStamp<<".png";

        	cv::Mat imRGBCurrent = cv::imread(path.str(),CV_LOAD_IMAGE_COLOR);
            cv::Mat imRGBOld = cv::imread(pathOld.str(),CV_LOAD_IMAGE_COLOR);

        	std::stringstream name1;
        	name1<<fixed<<mpCurrentKF->mnFrameId<<" "<<setprecision(6)<<mpCurrentKF->mTimeStamp;

        	std::stringstream name2;
        	name2<<fixed<<pKF->mnFrameId<<" "<<setprecision(6)<<pKF->mTimeStamp;

        	cv::namedWindow(name2.str());

        	cv::namedWindow(name1.str());
        	cv::moveWindow(name1.str(), 1000,0);

        	cv::imshow(name1.str(), imRGBCurrent);
        	cv::imshow(name2.str(), imRGBOld);

    	 	cv::waitKey(0);

    	    cv::destroyAllWindows();
*/

        	}

                vbDiscarded[i] = true;
                continue;
/*
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);
            pSolver->SetRansacParameters(0.99,20,300);
            vpSim3Solvers[i] = pSolver;
 */
        }

        nCandidates++;
    }

    bool bMatch = false;

    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();
                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);

                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);

                // If optimization is succesful stop ransacs and continue
                if(nInliers>=20)
                {
                    bMatch = true;
                    mpMatchedKF = pKF;
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);

                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    if(nTotalMatches>=40)
    {
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    else
    {
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

}


bool LoopClosing::ComputeSim3()
{
    // For each consistent loop candidate we try to compute a Sim3
    bool bMatch = false;
    cv::Mat transf;

    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    cv::Mat old_pose = mpCurrentKF->GetPose().clone();

//    unique_lock<mutex> lockT(mpMap->mMutexLCTracking);
/*
    std::cout<<"pause mapping"<<std::endl;
    mpLocalMapper->RequestStop();

    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }
*/
    // Get Map Mutex
//    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    for(int i=0; i<nInitialCandidates; i++)
    {
        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->SetNotErase();

        if ((mpCurrentKF->mnFrameId - pKF->mnFrameId) < 1000 || mpCurrentKF->mnFrameId < 2000) {
        	std::cout<<"lc pkf "<<pKF->mnId<<"("<<pKF->mnFrameId<<") mpCurrentKF "<<mpCurrentKF->mnId<<"("<<mpCurrentKF->mnFrameId<<")"<<std::endl;
    //    	continue;
        } else {
        	std::cout<<"lc larger than 1000"<<std::endl;
        }

        if(pKF->isBad())
        {
        	std::cout<<"lc pkf "<<pKF->mnId<<" is bad"<<std::endl;
            vbDiscarded[i] = true;
            continue;
        }

//        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);
/*
        int empty_points = 0;
        int bad_points = 0;
        for(size_t i=0; i<pKF->N;i++)
        {
             MapPoint* pMP = pKF->GetMapPoint(i);
            if(!pMP) {
            	empty_points++;
                cv::Mat x3D = pKF->UnprojectStereo(i);
                pMP = new MapPoint(x3D,pKF,mpMap);
                pMP->AddObservation(pKF,i);
                pKF->AddMapPoint(pMP,i);

                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pMP);
             //   tempIds.push_back(i);
                //     pMP->SetBadFlag();
          //      mlpRecentAddedMapPoints.push_back(pMP);
            }
         //   else if(pMP->isBad() || pMP->Observations()<1)
                else if(pMP->isBad())
            {
                bad_points++;
         //       cv::Mat x3D = pKF->UnprojectStereo(i);
         //       pMP->SetWorldPos(x3D);

                cv::Mat x3D = pKF->UnprojectStereo(i);
                MapPoint* newpoint = new MapPoint(x3D,pKF,mpMap);
                newpoint->AddObservation(pKF,i);
                pKF->AddMapPoint(newpoint,i);

                newpoint->ComputeDistinctiveDescriptors();
                newpoint->UpdateNormalAndDepth();
                mpMap->AddMapPoint(newpoint);

          //      pMP->Replace(newpoint);

            }
        }

        std::cout<<"empty_points "<<empty_points<<" bad points "<<bad_points<<std::endl;
*/
/*
    	std::stringstream path;
    	path<<fixed<<"/media/rasha/Seagate Backup Plus Drive/nao_motion_capture_2/experiment8/rgb/"<<
    	    		setprecision(6)<<mpCurrentKF->mTimeStamp<<".png";
    	std::stringstream pathOld;
    	pathOld<<fixed<<"/media/rasha/Seagate Backup Plus Drive/nao_motion_capture_2/experiment8/rgb/"<<
    	    		setprecision(6)<<pKF->mTimeStamp<<".png";

    	cv::Mat imRGBCurrent = cv::imread(path.str(),CV_LOAD_IMAGE_COLOR);
        cv::Mat imRGBOld = cv::imread(pathOld.str(),CV_LOAD_IMAGE_COLOR);
*/
        cv::Mat imRGBCurrent;
        cv::Mat imRGBOld;

        double dist_ratio = 1.0;
        int nmatches = 0;
        {
//			unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

			while(nmatches<20 && dist_ratio >= 0.7)
			{
//			    vector<MapPoint*> tempPointMatches;
//				nmatches = RunRansac(imRGBCurrent, imRGBOld, tempPointMatches, dist_ratio, pKF, transf);
				nmatches = RunRansac(imRGBCurrent, imRGBOld, vvpMapPointMatches[i], dist_ratio, pKF, transf);
				dist_ratio = dist_ratio - 0.1;
			}
			if (nmatches < 20) {
				std::cout<<"lc few ransac matches "<<nmatches<<std::endl;
				vbDiscarded[i] = true;
				continue;
			}
			else {
		//		bMatch = true;
/*
			    std::vector<MapPoint*> currenkKFPoints = mpCurrentKF->GetMapPointMatches();

				for (int p=0; p<vvpMapPointMatches[i].size(); p++) {
					if (vvpMapPointMatches[i][p]){
						MapPoint* pMP = currenkKFPoints[p];
						if (!pMP || pMP->isBad()) {

			                cv::Mat x3D = mpCurrentKF->UnprojectStereo(p);
			                MapPoint* newpoint = new MapPoint(x3D,mpCurrentKF,mpMap);
			                newpoint->AddObservation(mpCurrentKF,p);
			                mpCurrentKF->AddMapPoint(newpoint,p);

			                newpoint->ComputeDistinctiveDescriptors();
			                newpoint->UpdateNormalAndDepth();
			                mpMap->AddMapPoint(newpoint);
						}
					}

				}
				*/
				Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);
				pSolver->SetRansacParameters(0.99,20,300);
				vpSim3Solvers[i] = pSolver;
			}
        }

        nCandidates++;
    }

    /*
    if (bMatch) {
        KeyFrame* pKF = mvpEnoughConsistentCandidates[0];

        cv::Mat R(3,3,CV_32F);
        cv::Mat t(1,3,CV_32F);

        transf.rowRange(0,3).colRange(0,3).copyTo(R);
        transf.rowRange(0,3).col(3).copyTo(t);

        std::cout<<transf<<std::endl;
        std::cout<<R<<std::endl;
        std::cout<<t<<std::endl;

        const float s = 1.0f; //pSolver->GetEstimatedScale();
  //      matcher.SearchBySim3(mpCurrentKF,pKF,vvpMapPointMatches[0],s,R,t,7.5);

        g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
        const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vvpMapPointMatches[0], gScm, 10, mbFixScale);

        // If optimization is succesful stop ransacs and continue
        if(nInliers>=20)
        {
        	std::cout<<"lc pkf "<<pKF->mnId<<" nInliers "<<nInliers<<std::endl;
            bMatch = true;
            mpMatchedKF = pKF;
            g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
            mg2oScw = gScm*gSmw;
            mScw = Converter::toCvMat(mg2oScw);

            mvpCurrentMatchedPoints = vvpMapPointMatches[0];

        }
        else {
           	std::cout<<"lc pkf "<<pKF->mnId<<" nInliers "<<nInliers<<" not good "<<std::endl;
           	bMatch = false;
        }
    }

*/

/*
    if (bMatch) {

        KeyFrame* pKF = mvpEnoughConsistentCandidates[0];
    	bMatch = false;
		mpCurrentKF->SetPose(pKF->GetPose());
		int n = Optimizer::PoseOptimization(mpCurrentKF, vvpMapPointMatches[0]);

		std::cout<<"lc po inliers "<<n<<std::endl;

		if (n > 5){
			cv::Mat poseNew = mpCurrentKF->GetPose();
			cv::Mat poseKF = pKF->GetPose();

			float daN = abs(poseNew.at<float>(0,3)-poseKF.at<float>(0,3));
			float dbN = abs(poseNew.at<float>(1,3)-poseKF.at<float>(1,3));
			float dcN = abs(poseNew.at<float>(2,3)-poseKF.at<float>(2,3));

			float daO = abs(old_pose.at<float>(0,3)-poseKF.at<float>(0,3));
			float dbO = abs(old_pose.at<float>(1,3)-poseKF.at<float>(1,3));
			float dcO = abs(old_pose.at<float>(2,3)-poseKF.at<float>(2,3));

			if ((daN + dbN + dcN) < (daO + dbO + dcO)) {
				bMatch = true;
				mpMatchedKF = pKF;
			}
			else
				std::cout<<"distance larger than before"<<std::endl;

			std::cout<<"poseKF "<<poseKF.at<float>(0,3)<<" "<<poseKF.at<float>(1,3)<<" "<<poseKF.at<float>(2,3)<<std::endl;
			std::cout<<"mpCurrentKF new "<<poseNew.at<float>(0,3)<<" "<<poseNew.at<float>(1,3)<<" "<<poseNew.at<float>(2,3)<<std::endl;
			std::cout<<"mpCurrentKF old "<<old_pose.at<float>(0,3)<<" "<<old_pose.at<float>(1,3)<<" "<<old_pose.at<float>(2,3)<<std::endl;
			std::cout<<"new "<<daN<<" "<<dbN<<" "<<dcN<<std::endl;
			std::cout<<"old "<<daO<<" "<<dbO<<" "<<dcO<<std::endl;
		}
		else
			std::cout<<"lc few correspondences"<<std::endl;

    }
    */


    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
 //   while(nCandidates>0 && !bMatch)
    if(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
       //     if(bNoMore)
       //     {
                vbDiscarded[i]=true;
                nCandidates--;
      //      }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();
                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);

                g2o::Sim3 gScmold(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
    			cv::Mat old_pose = mpCurrentKF->GetPose();
    			cv::Mat poseKF = pKF->GetPose();
                g2o::Sim3 gSmwold(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                g2o::Sim3 mg2oScwold = gScmold*gSmwold;
                cv::Mat poseNew = Converter::toCvMat(mg2oScwold);

    			std::cout<<"poseKF "<<poseKF.at<float>(0,3)<<" "<<poseKF.at<float>(1,3)<<" "<<poseKF.at<float>(2,3)<<std::endl;
    			std::cout<<"mpCurrentKF old "<<old_pose.at<float>(0,3)<<" "<<old_pose.at<float>(1,3)<<" "<<old_pose.at<float>(2,3)<<std::endl;
    			std::cout<<"mpCurrentKF new 1 "<<poseNew.at<float>(0,3)<<" "<<poseNew.at<float>(1,3)<<" "<<poseNew.at<float>(2,3)<<std::endl;


                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);

                // If optimization is succesful stop ransacs and continue
                if(nInliers>=20)
                {
                	std::cout<<"lc pkf "<<pKF->mnId<<" nInliers "<<nInliers<<std::endl;
                    bMatch = true;
                    mpMatchedKF = pKF;
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);

        			std::cout<<"mpCurrentKF new 2 "<<mScw.at<float>(0,3)<<" "<<mScw.at<float>(1,3)<<" "<<mScw.at<float>(2,3)<<std::endl;

                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
                else
                	std::cout<<"few inliers in OptimizeSim3 "<<nInliers<<std::endl;
            }
        }
    }

 //   mpLocalMapper->Release();

    if(!bMatch)
    {
    	std::cout<<"no bmatch returning"<<std::endl;
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
//        mpCurrentKF->SetPose(old_pose);
        return false;
    }
/*
    mScw = mpCurrentKF->GetPose();
    cv::Mat Rcw = mScw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = mScw.rowRange(0,3).col(3);

    mg2oScw = g2o::Sim3(Converter::toMatrix3d(Rcw),Converter::toVector3d(tcw),1.0);
    mvpCurrentMatchedPoints = vvpMapPointMatches[0];
    mpCurrentKF->SetPose(old_pose);
*/

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }
   	std::cout<<"nTotalMatches "<<nTotalMatches<<std::endl;

    if(nTotalMatches>=40)
    {
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    else
    {
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

}

void LoopClosing::CorrectLoop()
{
    cout << "Loop detected!" << endl;

    // Send a stop signal to Local Mapping

    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
    }

    // Get Map Mutex
//    unique_lock<mutex> lockT(mpMap->mMutexLCTracking);
/*
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();

    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }
*/
    // Get Map Mutex
//    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();


    {
//        // Get Map Mutex
//        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;

            cv::Mat Tiw = pKFi->GetPose();

            if(pKFi!=mpCurrentKF)
            {
                cv::Mat Tic = Tiw*Twc;
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
                //Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi]=g2oCorrectedSiw;
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3[pKFi]=g2oSiw;
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            cv::Mat old_pose = pKFi->GetPose();

            float da = abs(correctedTiw.at<float>(0,3)-old_pose.at<float>(0,3));
            float db = abs(correctedTiw.at<float>(1,3)-old_pose.at<float>(1,3));
            float dc = abs(correctedTiw.at<float>(2,3)-old_pose.at<float>(2,3));

            if (da > 0.1 || db > 0.1 || dc > 0.1) {
            	std::cout<<"correctloop larger than 0.1"<<std::endl;
            	std::cout<<da<<" "<<db<<" "<<dc<<std::endl;

            	std::cout<<"id "<<pKFi->mnId<<" before correctloop "<<
                    old_pose.at<float>(0,3)<<" "<<old_pose.at<float>(1,3)<<
                    		" "<<old_pose.at<float>(2,3)<<std::endl;

            	        std::cout<<"id "<<pKFi->mnId<<" after correctloop "<<
            	        		correctedTiw.at<float>(0,3)<<" "<<correctedTiw.at<float>(1,3)<<
            	        		" "<<correctedTiw.at<float>(2,3)<<std::endl;
            }

            pKFi->SetPose(correctedTiw);

            // Make sure connections are updated
            pKFi->UpdateConnections();
        }

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        {
            if(mvpCurrentMatchedPoints[i])
            {
                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
                if(pCurMP)
                    pCurMP->Replace(pLoopMP);
                else
                {
                    mpCurrentKF->AddMapPoint(pLoopMP,i);
                    pLoopMP->AddObservation(mpCurrentKF,i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }

    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3);


    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;

    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        pKFi->UpdateConnections();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

    // Add loop edge
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);

/*
    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);
*/

 //   RunGlobalBundleAdjustment(mpCurrentKF->mnId);
    // Loop closed. Release Local Mapping.
 //   mpLocalMapper->Release();

    cout << "Loop Closed!" << endl;

    mLastLoopKFid = mpCurrentKF->mnId;   
}

void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);

    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));
        matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);

//        // Get Map Mutex
//        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        const int nLP = mvpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if(pRep)
            {
                pRep->Replace(mvpLoopMapPoints[i]);
            }
        }
    }
}


void LoopClosing::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        usleep(5000);
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
    }
}

void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
{
    cout << "Starting Global Bundle Adjustment" << endl;

    int idx =  mnFullBAIdx;
    Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        if(idx!=mnFullBAIdx)
            return;

        if(!mbStopGBA)
        {
            cout << "Global Bundle Adjustment finished" << endl;
            cout << "Updating map ..." << endl;

            // Get Map Mutex
 //           unique_lock<mutex> lockT(mpMap->mMutexLCTracking);
/*
            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped

            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                usleep(1000);
            }
*/
            // Get Map Mutex
 //           unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

            // Correct keyframes starting at map first keyframe
            list<KeyFrame*> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());

            while(!lpKFtoCheck.empty())
            {
                KeyFrame* pKF = lpKFtoCheck.front();
                const set<KeyFrame*> sChilds = pKF->GetChilds();
                cv::Mat Twc = pKF->GetPoseInverse();
                for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFrame* pChild = *sit;
                    if(pChild->mnBAGlobalForKF!=nLoopKF)
                    {
                        cv::Mat Tchildc = pChild->GetPose()*Twc;
                        pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                        pChild->mnBAGlobalForKF=nLoopKF;

                    }
                    lpKFtoCheck.push_back(pChild);
                }

                pKF->mTcwBefGBA = pKF->GetPose();
                pKF->SetPose(pKF->mTcwGBA);
                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

            for(size_t i=0; i<vpMPs.size(); i++)
            {
                MapPoint* pMP = vpMPs[i];

                if(pMP->isBad())
                    continue;

                if(pMP->mnBAGlobalForKF==nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else
                {
                    // Update according to the correction of its reference keyframe
                    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                    // Backproject using corrected camera
                    cv::Mat Twc = pRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                    cv::Mat twc = Twc.rowRange(0,3).col(3);

                    pMP->SetWorldPos(Rwc*Xc+twc);
                }
            }

  //          mpLocalMapper->Release();

            cout << "Map updated!" << endl;
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}


} //namespace ORB_SLAM
