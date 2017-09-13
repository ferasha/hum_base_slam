/*
 * FabmapLC.h
 *
 *  Created on: Oct 2, 2016
 *      Author: rasha
 */

#ifndef FABMAPLC_H_
#define FABMAPLC_H_

#include "ros/ros.h"
#include "KeyFrame.h"
#include "openfabmap2/openfabmap.hpp"

namespace ORB_SLAM2 {

class FabmapLC {
public:
	FabmapLC();
	virtual ~FabmapLC();

	std::vector<KeyFrame*> checkForLoopClosure(KeyFrame* KF);
    void verifyLoopClosure(KeyFrame* newKF, int to_node_id);
    void initFabmap(const string &strSettingPath);
    void computeBowImageDescriptor(KeyFrame* KF, cv::Mat& imgDescriptor);

    of2::FabMap *fabMap;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    int bowClusterSize;
    float good_matches;
    float all_appearance_matches;
    std::map<int, int> location_node_map;
    std::map<int, KeyFrame*> KFmap;
    int last_matched_frame_id;
	std::list<KeyFrame*> buffer;


};

} /* namespace ORB_SLAM2 */

#endif /* FABMAPLC_H_ */
