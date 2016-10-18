#ifndef MCPSLAM_HELPER_H_
#define MCPSLAM_HELPER_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <helper/estd.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

namespace helper {

using namespace std;

class boundaries
{
public:
    boundaries(ros::NodeHandle Nh,ros::NodeHandle NhPrivate, int sleeptime);
    void run();

private:
    void SetMarkers();

    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;
    ros::Publisher mPubBounds;

    visualization_msgs::Marker mB0,mB1,mB2,mB3,mBL,mBS;
    vector<visualization_msgs::Marker> mvMsgs;

    int mSleepTime;
};

class republisher{
public:
    republisher(ros::NodeHandle& Nh, ros::NodeHandle& NhPrivate, std::string op_mode="none");

    void LeicaCb0(geometry_msgs::PointStampedConstPtr msg);
    void LeicaCb1(geometry_msgs::PointStampedConstPtr msg);
    void LeicaCb2(geometry_msgs::PointStampedConstPtr msg);
    void LeicaCb3(geometry_msgs::PointStampedConstPtr msg);

    void PubMesh(){mPubMesh.publish(mMesh);}

private:
    // ROS infrastructure
    ros::NodeHandle Nh_;
    ros::NodeHandle NhPrivate_;

    //Mode == GT

    int mLineLength;

    ros::Subscriber mSubLeica0;
    ros::Subscriber mSubLeica1;
    ros::Subscriber mSubLeica2;
    ros::Subscriber mSubLeica3;
    ros::Publisher mPubTraj0;
    ros::Publisher mPubTraj1;
    ros::Publisher mPubTraj2;
    ros::Publisher mPubTraj3;
    visualization_msgs::Marker mTraj0;
    geometry_msgs::Point mLastPoint0;
    visualization_msgs::Marker mTraj1;
    geometry_msgs::Point mLastPoint1;
    visualization_msgs::Marker mTraj2;
    geometry_msgs::Point mLastPoint2;
    visualization_msgs::Marker mTraj3;
    geometry_msgs::Point mLastPoint3;
    geometry_msgs::Point mIniPoint;
    bool mbIniPointSet;
    double mScale;

    //mesh marker
    ros::Publisher mPubMesh;
    visualization_msgs::Marker mMesh;
};

} //end NS helper

#endif // MCPSLAM_HELPER_H_
