#ifndef MACSLAM_SERVERSYSTEM_H_
#define MACSLAM_SERVERSYSTEM_H_

//C++
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <mutex>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

//macslam
#include <helper/estd.h>
#include <macslam/Datatypes.h>
#include <macslam/ORBVocabulary.h>
#include <macslam/Map.h>
#include <macslam/Database.h>
#include <macslam/ClientHandler.h>
#include <macslam/MapMatcher.h>

using namespace std;
using namespace estd;

namespace macslam{

class ServerSystem
{
public:
    typedef boost::shared_ptr<ClientHandler> chptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<MapMatcher> matchptr;
public:
    ServerSystem(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, const string &strVocFile);
    void InitializeClients();
    void InitializeMapMatcher();
private:
    void LoadVocabulary(const string &strVocFile);
    void InitializeMaps();
    void InitializeKFDB();
    void InitializeMapping();
    //void LoadSettingsFile(const string &strSettingsFile); // Param loading not necessary - all info comes with the messages

    //ROS infrastructure
    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;

    vocptr mpVoc;
    string mstrSettingsFile;
    dbptr mpKFDB;
    //mapptr mpMap;
//    commptr mpComm;
    matchptr mpMapMatcher;

    chptr mpClient0;
    mapptr mpMap0;
    chptr mpClient1;
    mapptr mpMap1;
    chptr mpClient2;
    mapptr mpMap2;
    chptr mpClient3;
    mapptr mpMap3;

    const uidptr mpUID;

    //threads
    threadptr mptMapMatching;

//    threadptr mptClient0;
//    threadptr mptClient1;
//    threadptr mptClient2;
//    threadptr mptClient3;

    int mNumOfClients;
    int mMaxClients;

    //monitoring
    int mVerboseMode;
    bool mbRecordMode; //interrupts after vocabulary is loaded to start bagfile recording
};

} //end namespace

#endif
