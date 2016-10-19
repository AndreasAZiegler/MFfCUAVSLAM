#ifndef MACSLAM_MAPMERGER_H_
#define MACSLAM_MAPMERGER_H_

//C++
#include <boost/shared_ptr.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>

//ROS
#include <ros/ros.h>
#include <ros/publisher.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//MCP SLAM
#include <helper/estd.h>
#include <macslam/Converter.h>
#include <macslam/Datatypes.h>
#include <macslam/CentralControl.h>
#include <macslam/Map.h>
#include <macslam/KeyFrame.h>
#include <macslam/Optimizer.h>
#include <macslam/MapMatcher.h>
#include <macslam/ClientHandler.h>

#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;
using namespace estd;

namespace macslam{

//forward decs
class MapMatcher;
class ClientHandler;
class Map;
class CentralControl;
//-----------------

class MapMerger
{
public:
    typedef boost::shared_ptr<MapMerger> mergeptr;
    typedef boost::shared_ptr<MapMatcher> matchptr;
    typedef boost::shared_ptr<ClientHandler> chptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
public:
    typedef pair<set<kfptr>,int> ConsistentGroup;
    typedef map<kfptr,g2o::Sim3,std::less<kfptr>,
        Eigen::aligned_allocator<std::pair<const kfptr, g2o::Sim3> > > KeyFrameAndPose;
public:
    MapMerger(MapMergerParams Params, matchptr pMatcher, ros::NodeHandle Nh, ros::NodeHandle NhPrivate);
    mapptr MergeMaps(mapptr pMapCurr, mapptr pMapMatch, MapMatchHit vMatchHit, std::shared_ptr<g2o::Sim3> g2oScw_end);
    void optimizeEssentialGraph(mapptr pFusedMap, mapptr pMapCurr, mapptr pMapMatch, vector<MapMatchHit> vMatchHits);
    void globalBundleAdjustment(mapptr pFusedMap, mapptr pMapCurr, mapptr pMapMatch, vector<MapMatchHit> vMatchHits, std::shared_ptr<g2o::Sim3> g2oScw);

    bool isBusy();
private:
    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, std::vector<mpptr> vpLoopMapPoints);
    void SetBusy();
    void SetIdle();

    MapMergerParams mMyParams;
    matchptr mpMatcher;

    bool bIsBusy;
    mutex mMutexBusy;

    std::vector<kfptr> mvpCurrentConnectedKFs;
//    std::vector<mpptr> mvpCurrentMatchedPoints;
//    std::vector<mpptr> mvpLoopMapPoints;
//    g2o::Sim3 mg2oScw;

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    //visualization
    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;
//    ros::Publisher mPubPointMatchesC;
//    ros::Publisher mPubPointMatchesM;
//    ros::Publisher mPubFusedPoints;
//    ros::Publisher mPubLoopConnections;
    ros::Publisher mPubMarker;
    ros::Publisher mPubMarkerArray;
};

} //end namespace

#endif
