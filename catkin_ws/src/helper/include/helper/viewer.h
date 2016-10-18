#ifndef MACVIEWER_H
#define MACVIEWER_H

#include <ros/ros.h>
#include <ros/console.h>

#include<opencv2/opencv.hpp>

#include <ros/publisher.h>

#include <tf/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

using namespace std;

namespace mac_viewer {

class viewer
{
public:
    viewer(ros::NodeHandle Nh, ros::NodeHandle NhPrivate);

    ros::Subscriber mSubMatchMarker;
    ros::Publisher mPubMatchMarker;
    void MatchCB(visualization_msgs::Marker Msg);

    //+++ Client 0+++
    void MarkerCBC0(visualization_msgs::Marker Msg);

    //+++ Client 1+++
    void MarkerCBC1(visualization_msgs::Marker Msg);

    //+++ Client 2+++
    //+++ Client 3+++

    //+++ Server 0+++
    void MarkerCBS0(visualization_msgs::Marker Msg);
    void CloudCB0S0(sensor_msgs::PointCloud2 Msg);
    void CloudCB1S0(sensor_msgs::PointCloud2 Msg);
    void CloudCBMS0(sensor_msgs::PointCloud2 Msg);
    void LoopCBS0(visualization_msgs::Marker Msg);

    //+++ Server 1+++
    void MarkerCBS1(visualization_msgs::Marker Msg);
    void CloudCB0S1(sensor_msgs::PointCloud2 Msg);
    void CloudCB1S1(sensor_msgs::PointCloud2 Msg);
    void CloudCBMS1(sensor_msgs::PointCloud2 Msg);

    //+++ Server 2+++
    void MarkerCBS2(visualization_msgs::Marker Msg);

    //+++ Server 3+++
    void MarkerCBS3(visualization_msgs::Marker Msg);

private:
    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;

    ros::Publisher mPubMarkersAug;
    visualization_msgs::Marker mRect;

    //+++ Client 0+++
    ros::Subscriber mSubMarkerC0;
    ros::Publisher mPubMarkerC0;
    tf::TransformBroadcaster mPubTFC0;

    int mKfCutOffC0;
    double mScaleC0,mTrajSizeC0,mCamLineSizeC0;

    //+++ Client 1+++
    ros::Subscriber mSubMarkerC1;
    ros::Publisher mPubMarkerC1;
    tf::TransformBroadcaster mPubTFC1;

    int mKfCutOffC1;
    double mScaleC1,mTrajSizeC1,mCamLineSizeC1,mKFScaleC1;

    //+++ Client 2+++
    //+++ Client 3+++

    //+++ Server 0+++
    ros::Subscriber mSubMarkerS0;
    ros::Publisher mPubMarkerS0;
    ros::Publisher mPubMarkerS0v2;

    ros::Subscriber mSubCloud0S0;
    ros::Publisher mPubCloud0S0;

    ros::Subscriber mSubCloud1S0;
    ros::Publisher mPubCloud1S0;

    ros::Subscriber mSubCloudMS0;
    ros::Publisher mPubCloudMS0;

    ros::Subscriber mSubLoopMarkerS0;
    ros::Publisher mPubLoopMarkerS0;

    int mKfCutOffS0;
    double mScaleS0,mTrajSizeS0,mTrajSizeS0v2,mCamLineSizeS0,mKFScaleS0;
    double mBoxS0P1x,mBoxS0P1y,mBoxS0P2x,mBoxS0P2y,mBoxS0P3x,mBoxS0P3y,mBoxS0P4x,mBoxS0P4y;
    double mBoxS0xl,mBoxS0xh,mBoxS0zl,mBoxS0zh;
    double mS0zMin,mS0zMax;

    //+++ Server 1+++
    ros::Subscriber mSubMarkerS1;
    ros::Publisher mPubMarkerS1;
    ros::Publisher mPubMarkerS1v2;

    ros::Subscriber mSubCloud0S1;
    ros::Publisher mPubCloud0S1;

    ros::Subscriber mSubCloud1S1;
    ros::Publisher mPubCloud1S1;

    ros::Subscriber mSubCloudMS1;
    ros::Publisher mPubCloudMS1;

    int mKfCutOffS1;
    double mScaleS1,mTrajSizeS1,mCamLineSizeS1;
    double mBoxS1P1x,mBoxS1P1y,mBoxS1P2x,mBoxS1P2y,mBoxS1P3x,mBoxS1P3y,mBoxS1P4x,mBoxS1P4y;
    double mS1zMin,mS1zMax;

    //+++ Server 2+++
    ros::Subscriber mSubMarkerS2;
    ros::Publisher mPubMarkerS2;

    int mKfCutOffS2;
    double mScaleS2,mTrajSizeS2,mCamLineSizeS2;
    //+++ Server 3+++
    ros::Subscriber mSubMarkerS3;
    ros::Publisher mPubMarkerS3;

    int mKfCutOffS3;
    double mScaleS3,mTrajSizeS3,mCamLineSizeS3;

};

}

#endif
