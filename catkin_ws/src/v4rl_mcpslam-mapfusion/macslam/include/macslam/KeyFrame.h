#ifndef MACSLAM_KEYFRAME_H_
#define MACSLAM_KEYFRAME_H_

//C++
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>

//MAC-SLAM
#include <macslam/Datatypes.h>
#include <macslam/ORBVocabulary.h>
#include <macslam/Converter.h>
#include <macslam/MapPoint.h>
#include <macslam/Database.h>
#include <macslam/Map.h>
#include <macslam/Communicator.h>

//Thirdparty
#include <thirdparty/DBoW2/DBoW2/BowVector.h>
#include <thirdparty/DBoW2/DBoW2/FeatureVector.h>
#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

//MSGS
#include <macslam_msgs/macKeyFrame.h>

using namespace std;
using namespace estd;

namespace macslam{

//forward decs
class Communicator;
class Frame;
//------------

class KeyFrame : public boost::enable_shared_from_this<KeyFrame>
{
public:
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<Communicator> commptr;
public:
    //---constructor---
    KeyFrame(Frame &F, mapptr pMap, dbptr pKFDB, commptr pComm, eSystemState SysState, size_t UniqueId);
    KeyFrame(bool NonUsedValue, mapptr pMap, eSystemState SysState); //constructs an "empty" KeyFrame
    KeyFrame& operator=(KeyFrame&& rhs);
    KeyFrame(macslam_msgs::macKeyFrame* pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap, g2o::Sim3 mg2oS_loop, vocptr pVoc, mapptr pMap, dbptr pKFDB, commptr pComm,
             std::vector<cv::KeyPoint> vKeys, std::vector<cv::KeyPoint>  vKeysUn, cv::Mat Descriptors, cv::Mat K,
             std::vector<float> vScaleFactors, std::vector<float> vLevelSigma2, std::vector<float> vInvLevelSigma,
             eSystemState SysState, size_t UniqueId);

    //---communication---
    void ConvertToMessageClient(macslam_msgs::macKeyFrame &Msg);
    void ConvertToMessageServer(macslam_msgs::macKeyFrame &Msg, g2o::Sim3 mg2oS_wcurmap_wclientmap);
    bool ConvertToPoseMessageServer(macslam_msgs::macKeyFrame &Msg, g2o::Sim3 mg2oS_wcurmap_wclientmap);
    //performance: idea: keep message as member, update when MP changes, just return message instead of calculating every time
    void UpdateFromMessageServer(macslam_msgs::macKeyFrame *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap, g2o::Sim3 mg2oS_loop);
    void UpdateFromMessageClient(macslam_msgs::macKeyFrame *pMsg);
    void SendMe();
    void SendMeDebug();
    void MarkInOutBuffer() {unique_lock<mutex> lock(mMutexOut); mbInOutBuffer = true;}
    void UnMarkInOutBuffer() {unique_lock<mutex> lock(mMutexOut); mbInOutBuffer = false;}
    bool IsInOutBuffer() {unique_lock<mutex> lock(mMutexOut); return mbInOutBuffer;}
    void AddInformedClient(size_t ClientId){unique_lock<mutex> lock(mMutexOut); msuSentToClient.insert(ClientId);}
    bool SentToClient(size_t ClientId){unique_lock<mutex> lock(mMutexOut); return msuSentToClient.count(ClientId);}

    //---set/get pointers---
    void ReplaceMap(mapptr pNewMap){unique_lock<mutex> lockMap(mMapMutex); mpMap = pNewMap;}
    mapptr GetMapptr() {return mpMap;}
    void AddCommPtr(commptr pComm){mspComm.insert(pComm);}
    set<commptr> GetCommPtrs(){return mspComm;}

    //---visualization---
    void ShowMyValues();
    bool ChangedByServer(){return mbChangedByServer;}
    void SetChangedByServer(){mbChangedByServer = true;}
    cv::Mat mImg;
    uint32_t GetMPColors(mpptr pMP);
    uint32_t GetMPColorsRGB(mpptr pMP);

    // Pose functions
    void SetPose(const cv::Mat &Tcw, bool bLock);
    void SetPoseDebug(const cv::Mat &Tcw, bool bLock);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();
    bool IsPoseLocked(){unique_lock<mutex> lock(mMutexPose); return mbPoseLock;}

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(kfptr pKF, const int &weight);
    void EraseConnection(kfptr pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<kfptr> GetConnectedKeyFrames();
    std::vector<kfptr > GetVectorCovisibleKeyFrames();
    std::vector<kfptr> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<kfptr> GetCovisiblesByWeight(const int &w);
    int GetWeight(kfptr pKF);

    // Spanning tree functions
    void AddChild(kfptr pKF);
    void EraseChild(kfptr pKF);
    void ChangeParent(kfptr pKF);
    std::set<kfptr> GetChilds();
    kfptr GetParent();
    bool hasChild(kfptr pKF);

    // Loop Edges
    void AddLoopEdge(kfptr pKF);
    std::set<kfptr> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(mpptr pMP, const size_t &idx, bool bLock = false);
    void EraseMapPointMatch(const size_t &idx, bool bLock = false);
    void EraseMapPointMatch(mpptr pMP, bool bLock = false);
    void ReplaceMapPointMatch(const size_t &idx, mpptr pMP, bool bLock = false);
    std::set<mpptr> GetMapPoints();
    std::vector<mpptr> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    mpptr GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad / empty flag
    void SetBadFlag();
    bool isBad() {unique_lock<mutex> lock(mMutexConnections); return mbBad;}
    bool IsEmpty() {unique_lock<mutex> lock(mMutexConnections); return mbIsEmpty;}
    bool mbOmitSending;

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:
    //---environment---
    double mdServerTimestamp;
    /*const*/ double mTimeStamp;

    //---IDs---
    static long unsigned int nNextId;
    const idpair mFrameId;
    idpair mId;
    size_t mUniqueId;
    size_t mVisId;

    size_t mOrigClientId;
    static size_t mNumKeyFramesMap1;
    static size_t mNumKeyFramesMap2;

    // Grid (to speed up feature matching)
    /*const*/ int mnGridCols;
    /*const*/ int mnGridRows;
    /*const*/ float mfGridElementWidthInv;
    /*const*/ float mfGridElementHeightInv;

    // Variables used by the tracking
    idpair mTrackReferenceForFrame;
    idpair mFuseTargetForKF;

    // Variables used by the local mapping
    idpair mBALocalForKF;
    idpair mBAFixedForKF;

    // Variables used by the keyframe database
    idpair mLoopQuery;
    idpair mMatchQuery;
    int mnLoopWords;
    float mLoopScore;
    idpair mRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    idpair mBAGlobalForKF;
    bool mbLoopCorrected;

    // Calibration parameters
    /*const*/ float fx, fy, cx, cy, invfx, invfy;

    // Number of KeyPoints
    /*const*/ int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    /*const*/ std::vector<cv::KeyPoint> mvKeys;
    /*const*/ std::vector<cv::KeyPoint> mvKeysUn;
    /*const*/ cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    /*const*/ int mnScaleLevels;
    /*const*/ float mfScaleFactor;
    /*const*/ float mfLogScaleFactor;
    /*const*/ std::vector<float> mvScaleFactors;
    /*const*/ std::vector<float> mvLevelSigma2;
    /*const*/ std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    /*const*/ int mnMinX;
    /*const*/ int mnMinY;
    /*const*/ int mnMaxX;
    /*const*/ int mnMaxY;
    /*const*/ cv::Mat mK;


    // The following variables need to be accessed trough a mutex to be thread safe.
protected:
    //---communication---
    bool mbInOutBuffer;
    bool mbSentOnce;
    std::vector<mpptr> mvpNewMapPoints;
    std::vector<size_t> mvpNewMapPointsVectIds;
    std::vector<bool> mvbNewMapPointsDelete;
    set<size_t> msuSentToClient;

    //---infrastructure---
    mapptr mpMap;
//    commptr mpComm;
    set<commptr> mspComm;
    eSystemState mSysState;
    dbptr mpKeyFrameDB;
    vocptr mpORBvocabulary;

    //---visualization---
    bool mbChangedByServer;

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;
    bool mbPoseLock;
    bool mbPoseChanged;
    cv::Mat Cw; // Stereo middel point. Only for visualization //Performance:: delete

    // MapPoints associated to keypoints
    std::vector<mpptr> mvpMapPoints;
    std::vector<bool> mvbMapPointsLock;    

    // Grid over the image to speed up feature matching
    void AssignFeaturesToGrid();
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<kfptr,int> mConnectedKeyFrameWeights;
    std::vector<kfptr> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    kfptr mpParent;
    std::set<kfptr> mspChildrens;
    std::set<kfptr> mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;
    bool mbIsEmpty;

    float mHalfBaseline; // Only for visualization //Performance:: delete

    //---mutexes---
    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
    std::mutex mMutexOut;  //Performance: Maybe this Mutex is no necessary -> acquiring the other mutexes
    std::mutex mMapMutex;
};

} //end namespace

#endif
