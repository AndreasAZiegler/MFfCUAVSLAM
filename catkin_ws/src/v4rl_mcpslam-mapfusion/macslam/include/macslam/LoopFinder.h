#ifndef MACSLAM_LOOPFINDER_H_
#define MACSLAM_LOOPFINDER_H_

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

//MCP SLAM
#include <helper/estd.h>
#include <macslam/Datatypes.h>
#include <macslam/CentralControl.h>
#include <macslam/Database.h>
#include <macslam/Map.h>
#include <macslam/KeyFrame.h>
#include <macslam/ORBVocabulary.h>
#include <macslam/ORBmatcher.h>
#include <macslam/Sim3Solver.h>
#include <macslam/Converter.h>
#include <macslam/Optimizer.h>

using namespace std;
using namespace estd;

namespace macslam{

//forward decs
class KeyFrameDatabase;
class CentralControl;
class Map;
//---------------

class LoopFinder
{
public:
    typedef boost::shared_ptr<LoopFinder> lfptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
    typedef boost::shared_ptr<Map> mapptr;

    typedef pair<set<kfptr>,int> ConsistentGroup;
    typedef map<kfptr,g2o::Sim3,std::less<kfptr>,
            Eigen::aligned_allocator<std::pair<const kfptr, g2o::Sim3> > > KeyFrameAndPose;
public:
    LoopFinder(ccptr pCC, dbptr pDB, vocptr pVoc, mapptr pMap, LoopFinderParams LoopParams);

    void Run();
    void InsertKF(kfptr pKF);
    void EraseKFs(vector<kfptr> vpKFs);
    void RequestReset();

    void ChangeMap(mapptr pMap){mpMap = pMap;}
private:
    bool CheckKfQueue();
    bool DetectLoop();
    bool ComputeSim3();
    void CorrectLoop();
    void PublishLoopEdges();
    void ClearLoopEdges();

    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, std::vector<mpptr> vpLoopMapPoints);

    //Reset
    void ResetIfRequested();
    bool mbResetRequested;
    mutex mMutexReset;

    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;
    int mKFNewLoopThres;

    ros::Publisher mPubMarker;

    ccptr mpCC;
    dbptr mpKFDB;
    vocptr mpVoc;
    mapptr mpMap;
    const LoopFinderParams mParams;

    const int mLoopRate;
    const int mLockSleep;

    std::list<kfptr> mlKfInQueue;
    std::mutex mMutexKfInQueue;

    // Loop detector parameters
    kfptr mpCurrentKF;
    kfptr mpMatchedKF;

    float mnCovisibilityConsistencyTh;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<kfptr> mvpEnoughConsistentCandidates;
    std::vector<kfptr> mvpCurrentConnectedKFs;
    std::vector<mpptr> mvpCurrentMatchedPoints;
    std::vector<mpptr> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

//    long unsigned int mLastLoopKFid;
//    long unsigned int mLastLoopKFCLientid;
    size_t mKFcount;

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;

    //debugging
    int mVerboseMode;

    #ifdef HACKZ
    int mStartCount;
    #endif
};

} //end namespace

#endif
