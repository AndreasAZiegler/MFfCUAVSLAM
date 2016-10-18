#ifndef MACSLAM_VIEWER_H_
#define MACSLAM_VIEWER_H_

//C++
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

//MAC-SLAM
#include <macslam/Tracking.h>
#include <macslam/Map.h>
#include <macslam/CentralControl.h>

using namespace std;
using namespace estd;

namespace macslam{

//forward decs
class Tracking;
//----------

class FrameViewer
{
public:
    typedef boost::shared_ptr<FrameViewer> fviewptr;
    typedef boost::shared_ptr<Tracking> trackptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
public:
    FrameViewer(mapptr pMap, const string &strSettingPath, ccptr pCC);

    void UpdateAndDraw(trackptr pTracker);

private:
    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);
    // Update info from the last processed frame.
    void Update(trackptr pTracker);

    // Draw last processed frame.
    cv::Mat DrawFrame();

    ros::NodeHandle mNh;
    ros::Publisher mPubIm;
    tf::TransformBroadcaster mPubTf;

    ccptr mpCC;
    mapptr mpMap;
    double T_; // 1/fps in ms

    // Info of the frame to be drawn
    cv::Mat mIm;
    int N;
    vector<cv::KeyPoint> mvCurrentKeys;
    vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;

    mutex mMutex;
};


} //end namespace

#endif
