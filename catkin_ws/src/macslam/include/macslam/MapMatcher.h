#ifndef MACSLAM_MAPMATCHER_H_
#define MACSLAM_MAPMATCHER_H_

//C++
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <sstream>

//ROS
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//MAC-SLAM
#include <helper/estd.h>
#include <macslam/Datatypes.h>
#include <macslam/Database.h>
#include <macslam/Map.h>
#include <macslam/KeyFrame.h>
#include <macslam/ORBVocabulary.h>
#include <macslam/ORBmatcher.h>
#include <macslam/Sim3Solver.h>
#include <macslam/Converter.h>
#include <macslam/Optimizer.h>
#include <macslam/MapMerger.h>

using namespace std;
using namespace estd;

namespace macslam{

//forward decs
class KeyFrameDatabase;
class MapMerger;
class Map;
//-------------

class MapMatcher : public boost::enable_shared_from_this<MapMatcher>
{
public:
    typedef boost::shared_ptr<MapMatcher> matchptr;
    typedef boost::shared_ptr<MapMerger> mergeptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<Map> mapptr;

    typedef pair<set<kfptr>,int> ConsistentGroup;
public:
    MapMatcher(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, dbptr pDB, vocptr pVoc, mapptr pMap0, mapptr pMap1, mapptr pMap2, mapptr pMap3, MapMatchingParams MMParams);

    void Run();
    void InsertKF(kfptr pKF);
    void EraseKFs(vector<kfptr> vpKFs);
    void RequestReset();
    void PublishMergedMap(mapptr pMap, set<size_t> suAssClientsC, set<size_t> suAssClientsM);
    void UseMyMarkerPublisher(visualization_msgs::Marker markerMsg, string PubIdentifier);
    void UseMyPclPublisher(sensor_msgs::PointCloud2 pclMsg, string PubIdentifier);
private:
    bool CheckKfQueue();
    bool DetectMatches();
    bool ComputeSim3();
    void CorrectLoop();
    void PublishLoopEdges();
    void ClearLoopEdges();

    //Reset
    void ResetIfRequested();
    bool mbResetRequested;
    mutex mMutexReset;

    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;

    ros::Publisher mPubMarker;

    visualization_msgs::Marker mMapMatchEdgeMsg;
    tf::TransformListener mTfListen;
    double mScaleFactor;

    dbptr mpKFDB;
    vocptr mpVoc;
    map<size_t,mapptr> mmpMaps;
    set<mapptr> mspMaps;

    mapptr mpMap0;
    mapptr mpMap1;
    mapptr mpMap2;
    mapptr mpMap3;

    mergeptr mpMapMerger;

    int mMapMatchRate;
//    int mLockSleep;

    const MapMatchingParams mMMParams;

    std::list<kfptr> mlKfInQueue;
    std::mutex mMutexKfInQueue;

    // Loop detector parameters
    kfptr mpCurrentKF;
    kfptr mpMatchedKF;
    mapptr mpCurrMap;

    float mnCovisibilityConsistencyTh;
    std::map<mapptr,std::vector<ConsistentGroup>> mmvConsistentGroups;
    std::vector<kfptr> mvpEnoughConsistentCandidates;
    std::vector<kfptr> mvpCurrentConnectedKFs;
    std::vector<mpptr> mvpCurrentMatchedPoints;
    std::vector<mpptr> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    long unsigned int mLastLoopKFid;

    cv::Mat mMatchMatrix;
    std::map<mapptr,std::map<mapptr,vector<MapMatchHit>>> mFoundMatches;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;
};

} //end namespace

#endif
