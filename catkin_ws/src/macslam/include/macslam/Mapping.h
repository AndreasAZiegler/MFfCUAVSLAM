#ifndef MACSLAM_MAPPING_H_
#define MACSLAM_MAPPING_H_

//C++
#include <boost/shared_ptr.hpp>
#include <mutex>

//MAC-SLAM
#include <helper/estd.h>
#include <macslam/Datatypes.h>
#include <macslam/CentralControl.h>
#include <macslam/KeyFrame.h>
#include <macslam/Map.h>
#include <macslam/MapMatcher.h>
#include <macslam/LoopFinder.h>
#include <macslam/Database.h>
#include <macslam/ORBmatcher.h>
#include <macslam/Optimizer.h>
#include <macslam/Communicator.h>

using namespace std;

namespace macslam{

//forward decs
class KeyFrame;
class MapPoint;
class Map;
class MapMatcher;
struct CentralControl;
class LoopFinder;
class KeyFrameDatabase;
//--------------------

class LocalMapping
{
public:
    typedef boost::shared_ptr<LocalMapping> mappingptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
    typedef boost::shared_ptr<MapMatcher> matchptr;
    typedef boost::shared_ptr<LoopFinder> lfptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<Communicator> commptr;
public:
    //constructor
    LocalMapping(ccptr pCC, mapptr pMap, dbptr pDB, int MappingRate, double KfCullingRedundancyThres);

    //getter/setter
//    void SetTracker(Tracking* pTracker);
    void SetCommunicator(commptr pComm) {mpComm = pComm;}
    #ifndef MAPFUSION
    void SetLoopFinder(lfptr pLF) {mpLoopFinder = pLF;}
    #endif
    void SetMapMatcher(matchptr pMatch) {mpMapMatcher = pMatch;}

    void ChangeMap(mapptr pMap){mpMap = pMap;}

    // Main function
    void RunClient();
    void RunServer();

    void InsertKeyFrame(kfptr pKF);
    void InsertKfForLBA(kfptr pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

//    void LockbOpt(){mbLockForOpt = true;}
//    bool GetbOpt(){return mbLockForOpt;}
//    void UnlockbOpt(){mbLockForOpt = false;}

protected:

    bool CheckNewKeyFrames();
    bool CheckKfsForLBA();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat ComputeF12(kfptr &pKF1, kfptr &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    //infrastructure
    ccptr mpCC;
    mapptr mpMap;
    dbptr mpKFDB;
    commptr mpComm;
    #ifndef MAPFUSION
    lfptr mpLoopFinder;
    #endif
    matchptr mpMapMatcher;
    int mMappingRate;
    double mRedundancyThres;

    //data
    std::list<kfptr> mlNewKeyFrames;
    std::list<kfptr> mlKfsForLBA;
    kfptr mpCurrentKeyFrame;
    std::list<mpptr> mlpRecentAddedMapPoints;

    //reset/interrupt
    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;

    bool mbAcceptKeyFrames;

    //mutexes
    std::mutex mMutexAccept;
    std::mutex mMutexStop;
    std::mutex mMutexNewKFs;
    std::mutex mMutexKFsForLBA;

    size_t mClientId;

//    bool mbLockForOpt;

    //debugging
    int mVerboseMode;
    size_t mAddedKfs, mCulledKfs;
    vector<double> mvCulledKfRatios;
    double mCulledKfsRatioMean, mCulledKfRatioStdDev;
    vector<size_t> mCulledKfIds;
};

} //end namespace

#endif
