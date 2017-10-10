/*
 * FabmapLC.cc
 *
 *  Created on: Oct 2, 2016
 *      Author: rasha
 */

#include "FabmapLC.h"

namespace ORB_SLAM2 {

FabmapLC::FabmapLC() {
	// TODO Auto-generated constructor stub

}

FabmapLC::~FabmapLC() {
	// TODO Auto-generated destructor stub
}


void FabmapLC::initFabmap(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

	std::string vocabPath_, clTreePath_, trainbowsPath_;

	vocabPath_ = (std::string)fSettings["vocab"];
	clTreePath_ = (std::string)fSettings["clTree"];
	trainbowsPath_ = (std::string)fSettings["trainbows"];

	matcher = new cv::BFMatcher(cv::NORM_HAMMING);

	good_matches = 0;
	all_appearance_matches = 0;

	cv::Mat vocab;
	cv::Mat clTree;
	cv::Mat trainbows;

	ROS_INFO("Loading codebook...");

	cv::FileStorage fs;

	fs.open(vocabPath_,
					cv::FileStorage::READ);
	fs["Vocabulary"] >> vocab;
	fs.release();
	ROS_INFO("Vocabulary with %d words, %d dims loaded",vocab.rows,vocab.cols);

	fs.open(clTreePath_,
					cv::FileStorage::READ);
	fs["Tree"] >> clTree;
	fs.release();
	ROS_INFO("Chow Liu Tree loaded");

	fs.open(trainbowsPath_,
					cv::FileStorage::READ);
	fs["Trainbows"] >> trainbows;
	fs.release();
	ROS_INFO("Trainbows loaded");

	ROS_INFO("Setting the Vocabulary...");
    matcher->add( std::vector<cv::Mat>(1, vocab) );
    bowClusterSize = vocab.rows;

	ROS_INFO("Initialising FabMap2 with Chow Liu tree...");


	//create options flags
	std::string new_place_method, bayes_method;
	int simple_motion;

	new_place_method = (std::string)fSettings["NewPlaceMethod"];
	bayes_method = (std::string)fSettings["BayesMethod"];
	simple_motion = (int)fSettings["SimpleMotion"];

/*
	local_nh_.param<std::string>("NewPlaceMethod", new_place_method, "Meanfield");
	local_nh_.param<std::string>("BayesMethod", bayes_method, "ChowLiu");
	local_nh_.param<int>("SimpleMotion", simple_motion, 0);
*/

	int options = 0;
	if(new_place_method == "Sampled") {
		options |= of2::FabMap::SAMPLED;
	} else {
		options |= of2::FabMap::MEAN_FIELD;
	}
	if(bayes_method == "ChowLiu") {
		options |= of2::FabMap::CHOW_LIU;
	} else {
		options |= of2::FabMap::NAIVE_BAYES;
	}
	if(simple_motion) {
		options |= of2::FabMap::MOTION_MODEL;
	}

	//create an instance of the desired type of FabMap
	std::string fabMapVersion;
	double pzge, pzgne;

	fabMapVersion = (std::string)fSettings["FabMapVersion"];
	pzge = (double)fSettings["PzGe"];
	pzgne = (double)fSettings["PzGne"];

/*
	local_nh_.param<std::string>("FabMapVersion", fabMapVersion, "FABMAPFBO");
	local_nh_.param<double>("PzGe", pzge, 0.39);
	local_nh_.param<double>("PzGne", pzgne, 0);
	local_nh_.param<int>("NumSamples", num_samples, 3000);
*/

	if(fabMapVersion == "FABMAP2") {
		fabMap = new of2::FabMap2(clTree,
															pzge,
															pzgne,
															options);
	} else {
		ROS_ERROR("Could not identify openFABMAPVersion from node params");
	//	return false;
	}

	ROS_INFO("Adding the trained bag of words...");
	fabMap->addTraining(trainbows);

	last_matched_frame_id = 0;
}

void FabmapLC::computeBowImageDescriptor(KeyFrame* KF, cv::Mat& imgDescriptor)
{
    std::vector<cv::DMatch> matches;
    matcher->match( KF->mDescriptors, matches );

    imgDescriptor = cv::Mat( 1, bowClusterSize, CV_32FC1, cv::Scalar::all(0.0) );
    float *dptr = (float*)imgDescriptor.data;
    for( size_t i = 0; i < matches.size(); i++ )
    {
        int queryIdx = matches[i].queryIdx;
        int trainIdx = matches[i].trainIdx; // cluster index
        CV_Assert( queryIdx == (int)i );

        dptr[trainIdx] = dptr[trainIdx] + 1.f;
    }

    // Normalize image descriptor.
    imgDescriptor /= KF->mDescriptors.rows;

}


std::vector<KeyFrame*> FabmapLC::checkForLoopClosure(KeyFrame* KF)
{
	bool only_new_places_ = true;
	bool firstFrame_ = (KF->mnId > 0) ? false:true;
	int match_node_id = -1;

	KFmap[KF->mnId] = KF;

	cv::Mat bow, buffer_bow;
	computeBowImageDescriptor(KF, bow);
	bool loop_closure = false;

	if (!bow.empty()) {
		if (!firstFrame_) {

			if (!buffer.empty()) {
				int n_added = location_node_map.size();
				KeyFrame* buffer_KF = buffer.front();
				while ((KF->mnFrameId - buffer_KF->mnFrameId) >= 10)   //this ensures that the keyframe in question is not matched to recent frames (the keyframes are moved from the buffer to fabmap descriptor pool after a while)
				{
					if (!(buffer_KF->isBad())) {
						if ((buffer_KF->mnFrameId -last_matched_frame_id) >= 10) {
						//this ensures that the fabmap descriptor pool doesn't include consecutive frames (that are most probably similar in appearance) so that a keyframe isn't matched to two or more keyframes
							last_matched_frame_id = buffer_KF->mnFrameId;
							computeBowImageDescriptor(buffer_KF, buffer_bow);
							fabMap->add(buffer_bow);
							location_node_map[n_added] = buffer_KF->mnId;
							ROS_WARN_STREAM("Adding bow of new place... ID "<<n_added<<" "<<
									KF->mnFrameId<<"->"<<buffer_KF->mnFrameId);
							n_added++;
						}
					}
					else
						std::cout<<"buffer has a bad KF, discarding it "<<KF->mnId<<"->"<<buffer_KF->mnId<<std::endl;
					buffer.pop_front();
					if (buffer.empty())
						break;
					buffer_KF = buffer.front();
				}
			}

			std::vector<of2::IMatch> matches;
			fabMap->compare(bow, matches, !only_new_places_);
			std::sort(matches.begin(), matches.end());

//			if (!only_new_places_)
//				ROS_ERROR("only_new_places is false");
//			else
//			{
				of2::IMatch bestMatch = matches.back();

				if (bestMatch.imgIdx == -1) {
/*
					ROS_WARN_STREAM("bestMatch.imgIdx = -1. Adding bow of new place... ID "<<matches.size()-1);
					fabMap->add(bow);
					location_node_map[matches.size()-1] = KF->mnId;
*/				}
				else
				{
					if (bestMatch.match >= 0.98) {
						good_matches += 1.0;
					//	if (bestMatch.imgIdx < matches.size()-2) {
							loop_closure = true;
							match_node_id = location_node_map[bestMatch.imgIdx];
					//	}
					}
					else {
	/*					ROS_WARN_STREAM("bestMatch.match < 0.98. Adding bow of new place... ID "<<matches.size()-1);
						fabMap->add(bow);
						location_node_map[matches.size()-1] = KF->mnId;
*/					}
					all_appearance_matches += 1.0;
				}

//				if ((KF->mnFrameId -last_matched_frame_id) >= 10) {
					buffer.push_back(KF);
//					last_matched_frame_id = KF->mnFrameId;
//				}

				/*
				if ((KF->mnFrameId -last_matched_frame_id) >= 10) {
					ROS_WARN_STREAM("Adding bow of new place... ID "<<matches.size()-1<<" "<<
							KF->mnFrameId<<"->"<<last_matched_frame_id);
					fabMap->add(bow);
					location_node_map[matches.size()-1] = KF->mnId;
					last_matched_frame_id = KF->mnFrameId;
				}
				else
					ROS_WARN_STREAM("did not add new place. Diff less than 10: "<<
												KF->mnFrameId<<"->"<<last_matched_frame_id);
*/

				ROS_INFO_STREAM("image_number "<< KF->mnFrameId<<" KF_Id "<<KF->mnId<<
					       " toLocation " << bestMatch.imgIdx << " (KF id "<<match_node_id<<")" <<
						  " Match "<< bestMatch.match <<
						  " good matches "<< good_matches <<"/" << all_appearance_matches <<
						  " ("<< good_matches / ((all_appearance_matches==0)? 1:all_appearance_matches) << ")" <<
						  " loop_closure? "<<loop_closure
				);

/*
				for (int i=matches.size()-1; i>0, i > matches.size()-11; i--){
					bool l = false;
					int m_id = -1;
					int m_frameid = -1;
					of2::IMatch m = matches[i];
					if (m.match >= 0.98) {
						l = true;
						m_id = location_node_map[m.imgIdx];
						KeyFrame* matched_KF = KFmap[match_node_id];
						m_frameid = matched_KF->mnFrameId;
					}
				ROS_INFO_STREAM("image_number "<< KF->mnFrameId<<" KF_Id "<<KF->mnId<<
					       " toLocation " << m.imgIdx << " (KF id "<<m_id<<", frame_id "<<m_frameid<<")" <<
						  " Match "<< m.match <<
						  " loop_closure? "<<l
				);
				}
*/
		//	}

		}
		else {
			ROS_WARN_STREAM("Adding bow of new place...");
			fabMap->add(bow);
			location_node_map[0] = KF->mnId;
			firstFrame_ = false;
		}
	} else {
		ROS_WARN("--Image not descriptive enough, ignoring.");
	}

	vector<KeyFrame*> vpCandidateKFs;
	if (match_node_id > -1) {
		KeyFrame* matched_KF = KFmap[match_node_id];
		if (!matched_KF->isBad()) {
			if ((KF->mnId - matched_KF->mnId) > 1)
				vpCandidateKFs.push_back(KFmap[match_node_id]);
			else
				std::cout<<"consecutive KFs, will discard it "<<KF->mnId<<"->"<<matched_KF->mnId<<std::endl;
		}
		else {
			std::cout<<"matched with bad KF, will discard it "<<KF->mnId<<"->"<<matched_KF->mnId;
			int new_id = match_node_id;
			for (int i=1; i<=4; i++){
				new_id = match_node_id - i;
				if (new_id >= 0 && !KFmap[new_id]->isBad()) {
					vpCandidateKFs.push_back(KFmap[new_id]);
					std::cout<<" adding prev KF "<<new_id;
					break;
				}
				else {
					new_id = match_node_id + i;
					if (!KFmap[new_id]->isBad()) {
						vpCandidateKFs.push_back(KFmap[new_id]);
						std::cout<<" adding next KF "<<new_id;
						break;
					}
				}
			}

			if (vpCandidateKFs.empty()) {
				KeyFrame* pparentKF = matched_KF->GetParent();
				while(pparentKF->isBad())
				{
					pparentKF = pparentKF->GetParent();
				}
				std::cout<<" adding parent KF "<<pparentKF->mnId;
				vpCandidateKFs.push_back(pparentKF);
			}

			std::cout<<std::endl;
		}
	}
	return vpCandidateKFs;
}

/*
void FabmapLC::verifyLoopClosure(Node* new_node, int to_node_id){
	Node* to_node = graph_[to_node_id];
	std::cout<<"match loop closure candidate"<<std::endl;
	MatchingResult mr = new_node->matchNodePair(to_node);
//	mr.edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity()*20000000;
	if (mr.valid_tf)
	{
		if (addEdgeToG2O(mr.edge, to_node, new_node, true, false))
		  {
			new_node->valid_tf_estimate_ = true;
			graph_[new_node->id_] = new_node;
			std::cout<<"id "<<new_node->id_<<": loopclosure matches "<<mr.inlier_matches.size()<<" with "<<mr.edge.id1<<
					" curr_best_result "<<curr_best_result_.inlier_matches.size()<<" with "<<curr_best_result_.edge.id1<<std::endl;
			if (mr.inlier_matches.size() > curr_best_result_.inlier_matches.size()) {
				curr_best_result_ = mr;
				std::cout<<"change best location estimate"<<std::endl;
				boost::mutex::scoped_lock lock(m_mutex);
				g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[curr_best_result_.edge.id1]->vertex_id_));
				g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[curr_best_result_.edge.id2]->vertex_id_));
				v2->setEstimate(v1->estimate() * curr_best_result_.edge.transform);
			}
			optimizeGraph();
		  }
		else
			ROS_ERROR_STREAM("did not add edge between "<<mr.edge.id2<<" and "<<mr.edge.id1);
	}
}

*/

} /* namespace ORB_SLAM2 */
