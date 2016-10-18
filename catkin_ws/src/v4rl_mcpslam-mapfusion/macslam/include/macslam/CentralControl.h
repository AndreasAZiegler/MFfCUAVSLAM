#ifndef MACSLAM_CENTRALCONTROL_H_
#define MACSLAM_CENTRALCONTROL_H_

//C++
#include <boost/shared_ptr.hpp>
#include <mutex>

//ROS
#include <ros/ros.h>

//MAC-SLAM
#include <helper/estd.h>
#include <macslam/Datatypes.h>
#include <macslam/ClientHandler.h>

#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;
using namespace estd;

namespace macslam{

//forward decs
class ClientHandler;
//--------------

struct ColorRGB
{
    ColorRGB(double r=1,double g=1, double b=1) : mr(r),mg(g),mb(b)
        {}
    double mr;
    double mg;
    double mb;
};

struct ClientColors
{
    ClientColors(ColorRGB c0 = ColorRGB(1,0,0),ColorRGB c1 = ColorRGB(0,1,0),ColorRGB c2 = ColorRGB(1,1,0),ColorRGB c3 = ColorRGB(0,1,1),ColorRGB cx = ColorRGB(1,0,1))
        : mc0(c0),mc1(c1),mc2(c2),mc3(c3),mcx(cx)
        {}
    ColorRGB mc0;
    ColorRGB mc1;
    ColorRGB mc2;
    ColorRGB mc3;
    ColorRGB mcx;
};

struct CentralControl
{
public:
    typedef boost::shared_ptr<ClientHandler> chptr;
    typedef boost::shared_ptr<UniqueIdDispenser> uidptr;
public:
    CentralControl(ros::NodeHandle Nh, ros::NodeHandle NhPrivate,
                   size_t ClientId,
                   eSystemState SysState,
                   chptr pCH = nullptr,
                   uidptr pUID = nullptr,
                   double ScaleFactor = 1.0, double CovGraphMarkerSize = 0.001, double LoopEdgesMarkerSize = 0.001, double MarkerSphereDia = 1.0,
                   string OdomFrame = "empty",
                   double CamSize = 0.04,
                   double CamLineSize = 0.005,
                   double ColorR = 1.0,
                   double ColorG = 1.0,
                   double ColorB = 1.0,
                   int LockSleep = 1000,
                   bool UseImColor = false,
                   g2o::Sim3 g2oS_wc_wm = g2o::Sim3(),
                   g2o::Sim3 g2oS_loop = g2o::Sim3()
            )
        : mNh(Nh), mNhPrivate(NhPrivate),
          mClientId(ClientId),
          mpCH(pCH),
          mpUID(pUID),
          mScaleFactor(ScaleFactor), mCovGraphMarkerSize(CovGraphMarkerSize), mLoopEdgesMarkerSize(LoopEdgesMarkerSize), mMarkerSphereDiameter(MarkerSphereDia),
          mNativeOdomFrame(OdomFrame),
          mbOptActive(false),
          mSysState(SysState),
          mCamSize(CamSize),
          mCamLineSize(CamLineSize),
          mColorR(ColorR),mColorG(ColorG),mColorB(ColorB),
          mLockSleep(LockSleep),
          mbCommLock(false),mbMappingLock(false),mbPlaceRecLock(false),mbTrackingLock(false),
          mCols(ClientColors()),
          mbUseImgCol(UseImColor),
          mg2oS_wcurmap_wclientmap(g2oS_wc_wm),
          mg2oS_loop(g2oS_loop),
          mbCorrectAfterLoop(false),
          mbGotMerged(false)
        {}
    //ROS
    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;
    //Infrastucture
    uidptr mpUID;
    size_t mClientId;
    string mNativeOdomFrame;
    chptr mpCH;
    g2o::Sim3 mg2oS_wcurmap_wclientmap; //Sim3 world client to world map
    g2o::Sim3 mg2oS_loop;
    bool mbCorrectAfterLoop; //transforamtion correction should be applied after LC
    bool mbGotMerged; //prevent merged maps from publishing
    eSystemState mSysState;
    bool mbUseImgCol;
    //Timing
    int mLockSleep;
    //Params
    double mScaleFactor;
    double mCovGraphMarkerSize;
    double mLoopEdgesMarkerSize;
    double mMarkerSphereDiameter;
    double mCamSize;
    double mCamLineSize;
    double mColorR;
    double mColorG;
    double mColorB;
    ClientColors mCols;
    //Mutexes
//    mutex mMutexMapping;
//    mutex mMutexComm;
//    mutex mMutexPlaceRec;
    mutex mMutexCout;
    //Debugging
    bool mbOptActive;
    //Thread Sync
//    bool IsCommLocked(){unique_lock<mutex> mMutexComm; return mbCommLock;}
    bool LockComm(){unique_lock<mutex> lock(mMutexComm); if(!mbCommLock){mbCommLock = true; return true;} else return false;}
    bool LockMapping(){unique_lock<mutex> lock(mMutexMapping); if(!mbMappingLock){mbMappingLock = true; return true;} else return false;}
    bool LockPlaceRec(){unique_lock<mutex> lock(mMutexPlaceRec); if(!mbPlaceRecLock){mbPlaceRecLock = true; return true;} else return false;}
    bool LockTracking(){unique_lock<mutex> lock(mMutexTracking); if(!mbTrackingLock){mbTrackingLock = true; return true;} else return false;}
    void UnLockComm(){unique_lock<mutex> lock(mMutexComm);if(mbCommLock){mbCommLock = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"CentralControl\": Attempt to UnLock Comm -- was not locked" << endl; throw estd::infrastructure_ex();}}
    void UnLockMapping(){unique_lock<mutex> lock(mMutexMapping);if(mbMappingLock){mbMappingLock = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"CentralControl\": Attempt to UnLock Mapping -- was not locked" << endl; throw estd::infrastructure_ex();}}
    void UnLockPlaceRec(){unique_lock<mutex> lock(mMutexPlaceRec);if(mbPlaceRecLock){mbPlaceRecLock = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"CentralControl\": Attempt to UnLock PlaceRec -- was not locked" << endl; throw estd::infrastructure_ex();}}
    void UnLockTracking(){unique_lock<mutex> lock(mMutexTracking);if(mbTrackingLock){mbTrackingLock = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"CentralControl\": Attempt to UnLock Tracking -- was not locked" << endl; throw estd::infrastructure_ex();}}
private:
    //Thread Sync
    bool mbCommLock;
    bool mbMappingLock;
    bool mbPlaceRecLock;
    bool mbTrackingLock;
    //Mutexes
    mutex mMutexComm;
    mutex mMutexMapping;
    mutex mMutexPlaceRec;
    mutex mMutexTracking;
};

} //end namespace mcps

#endif
