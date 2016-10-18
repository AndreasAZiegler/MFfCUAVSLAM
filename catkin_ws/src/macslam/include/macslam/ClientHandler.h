#ifndef MACSLAM_CLIENTHANDLER_H_
#define MACSLAM_CLIENTHANDLER_H_

//C++
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <mutex>
#include <sstream>

//MAC-SLAM
#include <helper/estd.h>
#include <macslam/Datatypes.h>
#include <macslam/CentralControl.h>
#include <macslam/ORBVocabulary.h>
#include <macslam/Map.h>
#include <macslam/Database.h>
#include <macslam/Mapping.h>
#include <macslam/Communicator.h>
#include <macslam/MapMatcher.h>
#include <macslam/Mapping.h>
#include <macslam/Communicator.h>
#include <macslam/LoopFinder.h>
#include <macslam/Viewer.h>
#include <macslam/Tracking.h>

#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;
using namespace estd;

namespace macslam{

//forward decs
class Tracking;
class FrameViewer;
//class ServerSystem;
//class LocalMapping;
//class ServerCommunicator;
//class LoopFinder;
//----------------


class ClientHandler : public boost::enable_shared_from_this<ClientHandler>
{
public:
    typedef boost::shared_ptr<Tracking> trackptr;
    typedef boost::shared_ptr<FrameViewer> fviewptr;
    typedef boost::shared_ptr<LocalMapping> mappingptr;
    typedef boost::shared_ptr<Communicator> commptr;
    typedef boost::shared_ptr<LoopFinder> lfptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<MapMatcher> matchptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
public:
    ClientHandler(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, vocptr pVoc, dbptr pDB, mapptr pMap, size_t ClientId, uidptr pUID, eSystemState SysState, const string &strSettingsFile);
    void InitializeThreads();

    //---getter/setter---
    void SetMapMatcher(matchptr pMatch);
    void ChangeMap(mapptr pMap, g2o::Sim3 g2oS_wnewmap_wcurmap);
    commptr GetCommPtr(){return mpComm;}

    //---agent side---
    void CamImgCb(sensor_msgs::ImageConstPtr pMsg);
    void Reset();
private:

    //infrastructure
    ccptr mpCC;
    mapptr mpMap;
    dbptr mpKFDB;
    vocptr mpVoc;
    commptr mpComm;
    mappingptr mpMapping;
    //agent only
    trackptr mpTracking;
    fviewptr mpFrameViewer;
    bool mbVisualization;
    //server only
    #ifndef MAPFUSION
    lfptr mpLoopFinder;
    #endif
    matchptr mpMapMatcher;
    uidptr mpUID;
    eSystemState mSysState;

    const string mstrSettingsFile;

    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;
    ros::Subscriber mSubCam;

    //threads
    threadptr mptMapping;
    threadptr mptComm;
    threadptr mptLoopClosure;

    //data
    size_t mClientId;
    g2o::Sim3 mg2oS_wcurmap_wclientmap; //transforamtion from map into client

    //reset
    bool mbReset;

    //mutexes
    mutex mMutexThreads;
    mutex mMutexReset;

    //monitoring
    int mVerboseMode;
    struct timeval mtStartTrack, mtNowTrack;
    double mdElTotalTrack;
};

} //end ns

#endif
