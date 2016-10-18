#ifndef MACSLAM_CLIENTSYSTEM_H_
#define MACSLAM_CLIENTSYSTEM_H_

//C++
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <mutex>

#include <time.h>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

//MCPSLAM
#include <helper/estd.h>
#include <macslam/Datatypes.h>
#include <macslam/Database.h>
#include <macslam/Map.h>
#include <macslam/ClientHandler.h>

using namespace std;

namespace macslam{

class ClientSystem
{
public:
    typedef boost::shared_ptr<ClientHandler> chptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<Map> mapptr;
public:
    ClientSystem(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, const string &strVocFile, const string &strSettingsFile);
private:
    void LoadVocabulary(const string &strVocFile);

    //ROS infrastructure
    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;
    size_t mClientId;

    //MCP-SLAM Infrastructure
    vocptr mpVoc;
    string mstrSettingsFile;
    dbptr mpKFDB;
//    mappingptr mpMapping;
//    trackptr mpTracking;
//    fviewptr mpFrameViewer;
    mapptr mpMap;
    chptr mpAgent;
//    commptr mpComm;

    const uidptr mpUID;

    //monitoring
    int mVerboseMode;
    bool mbRecordMode; //interrupts after vocabulary is loaded to start bagfile recording
    struct timeval mtStartTrack, mtNowTrack;
    double mdElTotalTrack;
};

} //end namespace

#endif // MCPSLAM_CLIENTSYSTEM_H_
