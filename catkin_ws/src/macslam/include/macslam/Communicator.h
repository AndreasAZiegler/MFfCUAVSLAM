#ifndef MACSLAM_COMMUNICATOR_H_
#define MACSLAM_COMMUNICATOR_H_

//C++
//#include <opencv2/opencv.hpp>
//#include <cv_bridge/cv_bridge.h>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/bind.hpp>
#include <mutex>
#include <sstream>
#include <algorithm>

#include <time.h>

//ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>

//MAC-SLAM
#include <helper/estd.h>
#include <macslam/Datatypes.h>
#include <macslam/CentralControl.h>
#include <macslam/ORBVocabulary.h>
#include <macslam/KeyFrame.h>
#include <macslam/MapPoint.h>
#include <macslam/Map.h>
#include <macslam/Database.h>
#include <macslam/MapMatcher.h>
#include <macslam/Mapping.h>

//Msgs
#include <macslam_msgs/macKeyFrame.h>
#include <macslam_msgs/macMapPoint.h>
#include <macslam_msgs/macMap.h>

using namespace std;
using namespace estd;

namespace macslam
{

//forward decs

class MapMatcher;
class Map;
class LocalMapping;
class MapMatcher;
class CentralControl;
class KeyFrameDatabase;
//------------------

class Communicator : public boost::enable_shared_from_this<Communicator>
{
public:
    typedef boost::shared_ptr<Communicator> commptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
    typedef boost::shared_ptr<MapMatcher> matchptr;
    typedef boost::shared_ptr<LocalMapping> mappingptr;
public:
    //---constructor---
    Communicator(ccptr pCC, vocptr pVoc, mapptr pMap, dbptr pKFDB);

    //---main---
    void RunClient();
    void RunServer();

    //---getter/setter---
    void SetMapping(mappingptr pMapping) {mpMapping = pMapping;}
    void ChangeMap(mapptr pMap){mpMap = pMap;}
    void SetMapMatcher(matchptr pMatch) {mpMapMatcher = pMatch;}
    size_t GetClientId(){return mClientId;}

    //---callbacks---
//    void KeyFrameCb(macslam_msgs::macKeyFrameConstPtr pMsg);
//    void MapPointCb(macslam_msgs::macMapPointConstPtr pMsg);
    void MapCb(macslam_msgs::macMapConstPtr pMsg);
//    void FusionCb(mcpslam_msgs::MCPFusionConstPtr pMsg);
    void ResetCb(std_msgs::BoolConstPtr pMsg);
    void RequestReset();

    //--- data transfer---
    void PassKftoComm(kfptr pKf);
    void PassMptoComm(mpptr pMp);
    void DeleteMpFromBuffer(mpptr pMP);

    //--- loop closure handling ---
//    void LoopClosed(){mbLoopClosed = true;}

    //---mutexes
//    mutex mMutexForMapping; //Assure no publishing whilst changing data || Performance: maybe better solution (e.g. more fine-grained locking)
//    mutex mMutexForTracking; //Assure no publishing whilst changing data
    bool mbStrictLock;

protected:
    //---infrastructure---
    ccptr mpCC;
    mapptr mpMap;
    dbptr mpDatabase;
    vocptr mpVoc;
    mappingptr mpMapping;
    matchptr mpMapMatcher;
    size_t mClientId;

    bool mbFirstKF;

    int mKfItBound;
    int mMpItBound;

    int mCommRate;

    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;

//    ros::Publisher mPubKF;
//    ros::Publisher mPubMP;
    ros::Publisher mPubMap;
//    ros::Publisher mPubFusion;
    ros::Publisher mPubReset;

//    ros::Subscriber mSubKF;
//    ros::Subscriber mSubMP;
    ros::Subscriber mSubMap;
    ros::Subscriber mSubReset;

    string mSubKfTopicName;
    string mSubMpTopicName;
//    int mPubKfBufferSize,mPubMpBufferSize,mSubKfBufferSize,mSubMpBufferSize;
    int mPubMapBufferSize, mSubMapBufferSize;

    //---visualization---
    int mVisRate;
    bool mbShowCovGraph;
    bool mbShowMapPoints;
    bool mbShowKFs;
    bool mbShowTraj;

    //---buffer checks---
    bool CheckBufferKfIn();
    bool CheckBufferKfOut();
    bool CheckBufferMpIn();
    bool CheckBufferMpOut();

    //---publish/receive---
//    void PublishKfOut();
//    void PublishMpOut();
    #ifndef MAPFUSION
    void PublishMapServer();
    #endif
    void PublishMapClient();
    void ProcessKfInServer();
    #ifndef MAPFUSION
    void ProcessKfInClient();
    #endif
    void ProcessMpInServer();
    #ifndef MAPFUSION
    void ProcessMpInClient();
    #endif
    size_t mMsgCountLastMapMsg;

    //---IO buffers---
    list<kfptr> mlBufferKfOut; //Performance: Replace lists with sortedlists
    list<mpptr> mlBufferMpOut;
    list<macslam_msgs::macKeyFrame> mlBufferKfIn;
    list<macslam_msgs::macMapPoint> mlBufferMpIn;


    //---Reset---
    void ResetIfRequested();
    void ResetCommunicator();
    void ResetDatabase();
    void ResetMap();
    void ResetMapping();
    void ResetMapMatcher();
    bool mbResetRequested;
    mutex mMutexReset;

    //--- loop closure handling ---
//    bool mbLoopClosed;
    #ifdef HACKZ
    void CorrectAfterLC(kfptr pKF, g2o::Sim3 g2oS_loop);
    void CorrectAfterLC(mpptr pMP, g2o::Sim3 g2oS_loop);
    int mLoopCorrectionThreshold;
    cv::Mat mloopCorrectionPoseTcw;
    idpair mMaxId;
    #endif

    //--mutexes
    mutex mMutexBufferKfOut;
    mutex mMutexBufferMpOut;
    mutex mMutexBufferKfIn;
    mutex mMutexBufferMpIn;

    //monitoring
    int mVerboseMode;
    set<size_t> msSentMPs;
    set<size_t> msRecMPs;

    size_t mInMpCount;
    size_t mInKfCount;
    size_t mInMapCount;
    size_t mOutMpCount;
    size_t mOutKfCount;
    size_t mOutMapCount;
    size_t mOutMpBufferedCount;
    size_t mOutKfBufferedCount;
//    size_t mReceivedBytes;
//    double mdElCout;
//    double mdElTotal;
//    int mItTotal;
    int mCoutRate;
    int mCoutCount;
    int mPubCount;

    //time measurement
//    double mdKfInTime;
//    double mdMpInTime;
//    double mdVisTime;
//    double mdCoutTime;
//    double mdCommTime;
//    double mdTotalTime;
//    size_t mnKfInRuns;
//    size_t mnMpInRuns;
//    size_t mnVisRuns;
//    size_t mnCoutRuns;
//    size_t mnCommRuns;

#ifdef STATS
    //statistics
    bool mbTotalCommTime;

    double mdTimePerKfIn_mean;
    double mdTimePerKfIn_ssd;
    double mdTimePerKfIn_var;
    size_t mnKFsProcessedIn;
    double mdTimePerMpIn_mean;
    double mdTimePerMpIn_ssd;
    double mdTimePerMpIn_var;
    size_t mnMPsProcessedIn;
    double mdTimePerKfOut_mean;
    double mdTimePerKfOut_ssd;
    double mdTimePerKfOut_var;
    size_t mnKFsProcessedOut;
    double mdTimePerMpOut_mean;
    double mdTimePerMpOut_ssd;
    double mdTimePerMpOut_var;
    size_t mnMPsProcessedOut;

    double mdCommTime_total;
    double mdCommTime_mean;
    double mdCommTime_ssd;
    double mdCommTime_var;
    size_t mnCommRuns;

    struct timeval mtStartTotal;
    struct timeval mtLastStamp;
    bool mbFirstMessage;
    int mVectIndex;
    double mdTimeThresClient;
    double mdTimeThresServer;
    vector<double> mvtTimeStamp;
    vector<double> mvdLoadIn;
    vector<double> mvdLoadOut;
    size_t mdSizeOfMsgIn; //in Byte
    size_t mdSizeOfMsgOut;
#endif

};

} //end ns

#endif
