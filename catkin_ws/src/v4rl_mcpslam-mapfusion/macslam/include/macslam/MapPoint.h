#ifndef MACSLAM_MAPPOINT_H_
#define MACSLAM_MAPPOINT_H_

//C++
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>

//MAC-SLAM
#include <helper/estd.h>
#include <macslam/Datatypes.h>
#include <macslam/ORBVocabulary.h>
#include <macslam/Converter.h>
#include <macslam/KeyFrame.h>
#include <macslam/Communicator.h>
#include <macslam/Map.h>
#include <macslam/Frame.h>

#include <thirdparty/DBoW2/DBoW2/BowVector.h>
#include <thirdparty/DBoW2/DBoW2/FeatureVector.h>
#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include <macslam_msgs/macMapPoint.h>

using namespace std;
using namespace estd;

namespace macslam{

//forward decs
class KeyFrame;
class Map;
class Communicator;
//-------------

class MapPoint : public boost::enable_shared_from_this<MapPoint>
{
public:
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<Communicator> commptr;
    typedef boost::shared_ptr<Frame> frameptr;
public:
    //---constructor---
    MapPoint(const cv::Mat &Pos, kfptr pRefKF, mapptr pMap, size_t ClientId, commptr pComm, eSystemState SysState, size_t UniqueId);
    MapPoint(const cv::Mat &Pos, mapptr pMap, frameptr pFrame, const int &idxF, commptr pComm, eSystemState SysState, size_t UniqueId);
    MapPoint(bool NonUsedValue, mapptr pMap, eSystemState SysState); //constructs an "empty" MapPoint
    MapPoint& operator=(MapPoint&& rhs);
    MapPoint(macslam_msgs::macMapPoint *pMsg, g2o::Sim3 g2oS_wm_wc, g2o::Sim3 mg2oS_loop, mapptr pMap, commptr pComm, eSystemState SysState, size_t UniqueId = defid);

    //---communication---
    void ConvertToMessageClient(macslam_msgs::macMapPoint &Msg);
    void ConvertForeignPointToMessageClient(macslam_msgs::macMapPoint &Msg);
    void ConvertToMessageServer(macslam_msgs::macMapPoint &Msg, g2o::Sim3 mg2oS_wcurmap_wclientmap);
    bool ConvertToPosMessageServer(macslam_msgs::macMapPoint &Msg, g2o::Sim3 mg2oS_wcurmap_wclientmap);
    //performance: idea: keep message as member, update when MP changes, just return message instead of calculating every time
    void UpdateFromMessageServer(macslam_msgs::macMapPoint* pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap, g2o::Sim3 mg2oS_loop);
    void UpdateFromMessageClient(macslam_msgs::macMapPoint* pMsg);
    void SendMe();
    void SendMeMultiUse();
    void MarkInOutBuffer() {unique_lock<mutex> lock(mMutexOut); mbInOutBuffer = true;}
    void UnMarkInOutBuffer() {unique_lock<mutex> lock(mMutexOut); mbInOutBuffer = false;}
    void EraseInOutBuffer();
    bool IsInOutBuffer() {unique_lock<mutex> lock(mMutexOut); return mbInOutBuffer;}
    void AddInformedClient(size_t ClientId){unique_lock<mutex> lock(mMutexOut); msuSentToClient.insert(ClientId);}
    bool SentToClient(size_t ClientId){unique_lock<mutex> lock(mMutexOut); return msuSentToClient.count(ClientId);}

    //---set/get pointers---
    void ReplaceAndLock(mpptr pMP);
    void ReplaceMap(mapptr pNewMap);
    void AddCommPtr(commptr pComm){mspComm.insert(pComm);}
    set<commptr> GetCommPtrs(){return mspComm;}

    //---visualization---
    void ShowMyValues();
    void ShowMyObservations();
    bool ChangedByServer(){return mbChangedByServer;}
    void SetChangedByServer(){mbChangedByServer = true;}
    uint32_t GetColor();
    uint32_t GetColorRGB();
    uint32_t mRGB;
    uint32_t mGray;
//    void SetMultiUse(){mbMultiUse = true;}
    void SetMultiUse(){mbMultiUse = true;SendMe();}
    bool mbMultiUse;

    void SetWorldPos(const cv::Mat &Pos, bool bLock);
    cv::Mat GetWorldPos();
    bool IsPosLocked(){unique_lock<mutex> lock(mMutexPos); return mbPoseLock;}

    cv::Mat GetNormal();
    kfptr GetReferenceKeyFrame();

    std::map<kfptr,size_t> GetObservations();
    int Observations();

    void AddObservation(kfptr pKF,size_t idx, bool bLock = false);
    void EraseObservation(kfptr pKF, bool bLock = false);

    int GetIndexInKeyFrame(kfptr pKF);
    bool IsInKeyFrame(kfptr pKF);

    void SetBadFlag();
    bool isBad() {unique_lock<mutex> lock(mMutexFeatures); unique_lock<mutex> lock2(mMutexPos); return mbBad;}
    bool IsEmpty() {unique_lock<mutex> lock(mMutexFeatures); unique_lock<mutex> lock2(mMutexPos); return mbIsEmpty;}
    bool mbOmitSending;

    void Replace(mpptr pMP);
    mpptr GetReplaced();
    bool mbDoNotReplace;

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, const float &logScaleFactor);

public:
    static long unsigned int nNextId;
    idpair mId;
    size_t mUniqueId;
    idpair mFirstKfId;
    idpair mFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    idpair mTrackReferenceForFrame;
    idpair mLastFrameSeen;

    // Variables used by local mapping
    idpair mBALocalForKF;
    idpair mFuseCandidateForKF;

    // Variables used by loop closing
    idpair mLoopPointForKF_LC;
    idpair mCorrectedByKF_LC;
    size_t mCorrectedReference_LC;
//    idpair mBAGlobalForKF_LC;
    bool mbLoopCorrected;

    // Variables used by map matching
    idpair mLoopPointForKF_MM;
    idpair mCorrectedByKF_MM;
    size_t mCorrectedReference_MM;

    idpair mBAGlobalForKF;

    cv::Mat mPosGBA;

    //---mutexes---
    static mutex mGlobalMutex;

protected:
    //---communication---
    bool mbInOutBuffer;
    bool mbSentOnce;
    std::vector<kfptr> mvpNewObs;
    std::vector<size_t> mvpNewObsIds;
    std::vector<bool> mvbNewObsErase;
    int mnVisibleAdded;
    int mnFoundAdded;
    set<size_t> msuSentToClient;

//    bool mbDescriptorSet;
//    bool mbNormalAndDepthSet;

    //---infrastructure---
    mapptr mpMap;
//    commptr mpComm;
    set<commptr> mspComm;
    eSystemState mSysState;

    //---visualization---
    bool mbChangedByServer;

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     bool mbPoseLock;
     bool mbPoseChanged;

     // Keyframes observing the point and associated index in keyframe
     std::map<kfptr,size_t> mObservations;
     std::map<kfptr,bool> mObservationsLock;

     // Mean viewing direction
     cv::Mat mNormalVector;
     bool mbNormalAndDepthChanged;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;
     bool mbDescriptorChanged;

     // Reference KeyFrame
     kfptr mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     mpptr mpReplaced;
     bool mbIsEmpty;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     //---mutexes---
     std::mutex mMutexOut; //Performance: Maybe this Mutex is no necessary -> aquiring the other mutexes
     std::mutex mMapMutex;std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //end namespace

#endif
