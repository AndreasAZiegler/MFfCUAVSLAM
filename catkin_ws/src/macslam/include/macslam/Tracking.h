#ifndef MACSLAM_TRACKING_H_
#define MACSLAM_TRACKING_H_

//C++
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <mutex>
#include <iostream>
#include <opencv2/opencv.hpp>

//MCP SLAM
#include <helper/estd.h>
#include <macslam/Converter.h>
#include <macslam/ORBextractor.h>
#include <macslam/ORBVocabulary.h>
#include <macslam/ORBmatcher.h>
#include <macslam/Frame.h>
#include <macslam/MapPoint.h>
#include <macslam/Map.h>
#include <macslam/Database.h>
#include <macslam/KeyFrame.h>
#include <macslam/Mapping.h>
#include <macslam/PnPSolver.h>
#include <macslam/Optimizer.h>
#include <macslam/Viewer.h>
#include <macslam/Initializer.h>
//#include <macslam/ClientHandler.h>
#include <macslam/CentralControl.h>
#include <macslam/Viewer.h>

using namespace std;
using namespace estd;

namespace macslam{

//forward decs
class FrameViewer;
class Initializer;
class ClientHandler;
class Frame;
//------------

class Tracking : public boost::enable_shared_from_this<Tracking>
{
public:
    typedef boost::shared_ptr<Tracking> trackptr;
    typedef boost::shared_ptr<FrameViewer> fviewptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<Communicator> commptr;
    typedef boost::shared_ptr<LocalMapping> mappingptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<Initializer> initptr;
    typedef boost::shared_ptr<Frame> frameptr;
public:
    Tracking(ccptr pCC, vocptr pVoc, fviewptr pFrameViewer, mapptr pMap,
             dbptr pKFDB, const string &strSettingPath, size_t ClientId, bool bVisMode);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(mappingptr pLocalMapper) {mpLocalMapper = pLocalMapper;}
    void SetCommunicator(commptr pComm) {mpComm = pComm;}
//    void SetLoopClosing(LoopClosing* pLoopClosing);
//    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
//    void InformOnlyTracking(const bool &flag);


public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Current Frame
    frameptr mCurrentFrame;
    cv::Mat mImGray;
    cv::Mat mImRGB;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    frameptr mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<kfptr> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    void Reset();

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    //Other Thread Pointers
    mappingptr mpLocalMapper;
//    LoopClosing* mpLoopClosing;

    //ORB
    extractorptr mpORBextractor;
    extractorptr mpIniORBextractor;

    //BoW
    vocptr mpORBVocabulary;
    dbptr mpKeyFrameDB;

    // Initalization (only for monocular)
    initptr mpInitializer;

    //Local Map
    kfptr mpReferenceKF;
    std::vector<kfptr> mvpLocalKeyFrames;
    std::vector<mpptr> mvpLocalMapPoints;

    // System
    ccptr mpCC;

    //Drawers
    fviewptr mpFrameViewer;
    bool mbVisMode;

    //Communicator
    commptr mpComm;

    //Map
    mapptr mpMap;

    //ID
    size_t mClientId;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
//    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;
    int mnMatchesInliersThres;
    float mthRefRatio;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    kfptr mpLastKeyFrame;
    frameptr mLastFrame;
    idpair mLastKeyFrameId;
    idpair mLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

//    list<mpptr> mlpTemporalPoints; //not needed in mono case

//    unique_lock<mutex> mLockComm;

    //monitoring
    int mVerboseMode;
};

} //end namespace

#endif
