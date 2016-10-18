#ifndef MACSLAM_MAP_H_
#define MACSLAM_MAP_H_

//C++
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <set>
#include <mutex>
#include <sstream>
//#include <unordered_map>

//ROS
#include <ros/ros.h>
#include <ros/publisher.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//MAC-SLAM
#include <helper/estd.h>
#include <macslam/Datatypes.h>
#include <macslam/CentralControl.h>
#include <macslam/MapPoint.h>
#include <macslam/KeyFrame.h>
#include <macslam/Communicator.h>


using namespace std;
using namespace estd;

namespace macslam{

//forward decs
class CentralControl;
class Communicator;
//------------

class Map : public boost::enable_shared_from_this<Map>
{
public:
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
    typedef boost::shared_ptr<Communicator> commptr;
public:
    //---Constructor---
    Map(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, size_t MapId, eSystemState SysState);
    Map(const mapptr &pMapA, const mapptr &pMapB); //merge constructor
    void UpdateAssociatedData(); //need to be called after merge constructor. Cannot be called by merge constructor because of usage of shared_from_this()
    Map& operator=(Map& rhs);

    //---infrastructure---
    set<size_t> msuAssClients;
    void AddCCPtr(ccptr pCC);
    set<ccptr> GetCCPtrs();
    bool mbOutdated;
    string mOdomFrame;
    size_t mMapId;
    eSystemState mSysState;

    //---visualization---
    sensor_msgs::PointCloud2 GetMapPointsAsPclMsg(double ScaleFactor = 1.0, string FrameId = "odom");
    visualization_msgs::Marker GetCovGraphAsMarkerMsg(double MarkerSize, double ScaleFactor = 1.0, string FrameId = "odom", string ns = "empty", int Id = 0, double r=0.0, double g=1.0, double b=0.0);

    void PubMapPointsAsPclMsg(double ScaleFactor = 1.0, string FrameId = "empty");
    void PubMapPointsAsPclMsgV2(double ScaleFactor = 1.0, string FrameId = "empty");
    void PubCovGraphAsMarkerMsg(double MarkerSize = 0.1, double ScaleFactor = 1.0, string FrameId = "empty", string ns = "empty", double r=0.0, double g=1.0, double b=0.0);
    void PubKeyFrames(string FrameId = "empty", string ns = "empty", double r=0.0, double g=1.0, double b=0.0);
    void PubTrajectoryClient(string FrameId);
    void PubTrajectoryServer(string FrameId);

//    void ClearKeyFrames(string FrameId, string ns);
    void ClearKeyFrames();
    void ClearMapPoints();
    void ClearTrajectories();

    //---Add/Erase data---
    void AddKeyFrame(kfptr pKF);
    void AddMapPoint(mpptr pMP);
    void EraseMapPoint(mpptr pMP);
    void EraseKeyFrame(kfptr pKF);
    void SetReferenceMapPoints(const std::vector<mpptr> &vpMPs);

    //---Setter---
//    void SetCommunicator(commptr pComm) {mpComm = pComm;}
    void SetCommunicator(commptr pComm) {mspComm.insert(pComm);}

    //---Getter---
    kfptr GetKfPtr(size_t KfId, size_t ClientId);
    mpptr GetMpPtr(size_t MpId, size_t ClientId);
    bool IsKfDeleted(size_t KfId, size_t ClientId);
    bool IsMpDeleted(size_t MpId, size_t ClientId);

    vector<kfptr> GetAllKeyFrames();
    vector<mpptr> GetAllMapPoints();
//    std::vector<mpptr> GetAllMapPoints();
    std::vector<mpptr> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();
    long unsigned int GetMaxMPid();
    long unsigned int GetMaxKFidUnique();
    long unsigned int GetMaxMPidUnique();

    kfptr GetFromKfBuffer(size_t KfId, size_t ClientId);
    bool IsInKfBuffer(size_t KfId, size_t ClientId);
    kfptr SearchAndReturnFromKfBuffer(size_t KfId, size_t ClientId);
    void DeleteFromKfBuffer(kfptr pKF);
    mpptr GetFromMpBuffer(size_t MpId, size_t ClientId);
    bool IsInMpBuffer(size_t MpId, size_t ClientId);
    mpptr SearchAndReturnFromMpBuffer(size_t MpId, size_t ClientId);
    void DeleteFromMpBuffer(mpptr pMP);

    std::vector<mpptr> GetMvpReferenceMapPoints() {return mvpReferenceMapPoints;}
    std::map<idpair,mpptr> GetMmpMapPoints() {return mmpMapPoints;}
    std::map<idpair,kfptr> GetMmpKeyFrames() {return mmpKeyFrames;}
    std::map<idpair,mpptr> GetMmpErasedMapPoints() {return mmpErasedMapPoints;}
    std::map<idpair,kfptr> GetMmpErasedKeyFrames() {return mmpErasedKeyFrames;}
    map<idpair,kfptr> GetMmpKfBuffer() {return mmpKfBuffer;}
    map<idpair,mpptr> GetMmpMpBuffer() {return mmpMpBuffer;}
    std::set<mpptr> GetMspMapPoints() {return mspMapPoints;}
    std::set<kfptr> GetMspKeyFrames() {return mspKeyFrames;}

//    size_t GetNumOfKFs(){return mspKeyFrames.size();}
//    size_t GetNumOfMPs(){return mspMapPoints.size();}

    //---data---
    vector<kfptr> mvpKeyFrameOrigins;

    //---Reset---
    void clear();

    //---mutexes & sync---
    bool LockMapUpdate(){unique_lock<mutex> lock(mMutexMapUpdate); if(!mbLockMapUpdate){mbLockMapUpdate = true; return true;} else return false;}
    bool LockPointCreation(){unique_lock<mutex> lock(mMutexPointCreation); if(!mbLockPointCreation){mbLockPointCreation = true; return true;} else return false;}
    void UnLockMapUpdate(){unique_lock<mutex> lock(mMutexMapUpdate);if(mbLockMapUpdate){mbLockMapUpdate = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"Map\": Attempt to UnLock MapUpdate -- was not locked" << endl; throw estd::infrastructure_ex();}}
    void UnLockPointCreation(){unique_lock<mutex> lock(mMutexPointCreation);if(mbLockPointCreation){mbLockPointCreation = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"Map\": Attempt to UnLock PointCreation -- was not locked" << endl; throw estd::infrastructure_ex();}}
    int GetLockSleep();

protected:
    //---infrastructure---
    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;

    std::set<ccptr> mspCC;
    set<commptr> mspComm; //comm ptrs of associated clients -- necessary to pass to MPs/KFs
//    commptr mpComm;

    ros::Publisher mPubMapPoints0;
    ros::Publisher mPubMapPoints1;
    ros::Publisher mPubMapPoints2;
    ros::Publisher mPubMapPoints3;
    ros::Publisher mPubMapPointsx;
    ros::Publisher mPubMapPoints;
    ros::Publisher mPubMapPointsMultiUse;
    ros::Publisher mPubMapPointsCol;
    ros::Publisher mPubMarker;
    ros::Publisher mPubMarkerArray;
//    ros::Publisher mPubMarker;

    //---visualization---
    visualization_msgs::Marker mTraj0;
    visualization_msgs::Marker mTraj1;
    visualization_msgs::Marker mTraj2;
    visualization_msgs::Marker mTraj3;

    //---data---
    std::set<mpptr> mspMapPoints;
    std::set<kfptr> mspKeyFrames;

    std::vector<mpptr> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid; //Danger: check usage of this value
    long unsigned int mnMaxMPid;
    long unsigned int mnMaxKFidUnique;
    long unsigned int mnMaxMPidUnique;

    std::map<idpair,mpptr> mmpMapPoints; //Performance: second container is not nice, but fast...
    std::map<idpair,kfptr> mmpKeyFrames; //Performance: second container is not nice, but fast...
    std::map<idpair,mpptr> mmpErasedMapPoints;
    std::map<idpair,kfptr> mmpErasedKeyFrames;

    map<idpair,kfptr> mmpKfBuffer;
    map<idpair,mpptr> mmpMpBuffer;

    //---mutexes---
    std::mutex mMutexCC;
    std::mutex mMutexErased;
    std::mutex mMutexMap;
    mutex mMutexKfBuffer; //should not be necessary, buffers only accessed by comm thread
    mutex mMutexMpBuffer; //should not be necessary, buffers only accessed by comm thread
    bool mbLockMapUpdate;
    bool mbLockPointCreation;
    std::mutex mMutexMapUpdate;
    std::mutex mMutexPointCreation; // This avoid that two points are created simultaneously in separate threads (id conflict)

    //---debugging---
    int mVerboseMode;
};

} //end namespace

#endif
