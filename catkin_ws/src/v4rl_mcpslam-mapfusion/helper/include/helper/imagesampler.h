#ifndef IMAGESAMPLER_H
#define IMAGESAMPLER_H

#include <ros/ros.h>
#include <ros/console.h>

#include<opencv2/opencv.hpp>

#include <ros/publisher.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

using namespace std;

namespace img2cloud
{

class imagesampler
{
public:
    imagesampler(ros::NodeHandle Nh, ros::NodeHandle NhPrivate);

    void SampleFloorCloud();
    void SampleLongEdge();
    void SampleShortEdge();
    void PubFloor();

private:
    void LiftPoint(pcl::PointXYZRGB p,double stepz);

    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;

    ros::Publisher mPubFloor;

    string mImgPathFloor;
    string mImgPathLongEdge;
    string mImgPathShortEdge;

    int mCloudw;
    int mCloudh;
    int mCloudz;
    int mDensity;
    double mdScale;
    double mdZ1;

    double mStepW,mStepH;

    pcl::PointCloud<pcl::PointXYZRGB> mCloudFloor;
};

}

#endif
