#include <helper/viewer.h>

namespace mac_viewer {

viewer::viewer(ros::NodeHandle Nh, ros::NodeHandle NhPrivate)
    : mNh(Nh), mNhPrivate(NhPrivate)
{
    mSubMatchMarker = mNh.subscribe<visualization_msgs::Marker>("MapMatcherMarkers",10,&viewer::MatchCB,this);
    mPubMatchMarker = mNh.advertise<visualization_msgs::Marker>("Match",10);

    //++++++++++ Client 0++++++++++
    mSubMarkerC0 = mNh.subscribe<visualization_msgs::Marker>("MarkerMapClient0",1000,&viewer::MarkerCBC0,this);
    mPubMarkerC0 = mNh.advertise<visualization_msgs::Marker>("MarkerC0",1000);

    mNhPrivate.param("KfCutOffC0",mKfCutOffC0,10);
    mNhPrivate.param("ScaleC0",mScaleC0,1.0);
    mNhPrivate.param("TrajSizeC0",mTrajSizeC0,1.0);
    mNhPrivate.param("CamLineSizeC0",mCamLineSizeC0,1.0);

    //++++++++++ Client 1++++++++++
    mSubMarkerC1 = mNh.subscribe<visualization_msgs::Marker>("MarkerMapClient1",1000,&viewer::MarkerCBC1,this);
    mPubMarkerC1 = mNh.advertise<visualization_msgs::Marker>("MarkerC1",1000);

    mNhPrivate.param("KfCutOffC1",mKfCutOffC1,10);
    mNhPrivate.param("ScaleC1",mScaleC1,1.0);
    mNhPrivate.param("KFScaleC1",mKFScaleC1,1.0);
    mNhPrivate.param("TrajSizeC1",mTrajSizeC1,1.0);
    mNhPrivate.param("CamLineSizeC1",mCamLineSizeC1,1.0);

    //++++++++++ Client 2++++++++++
    //++++++++++ Client 3++++++++++

    //++++++++++ Server 0++++++++++
    mSubMarkerS0 = mNh.subscribe<visualization_msgs::Marker>("MarkerMapServer0",1000,&viewer::MarkerCBS0,this);
    mPubMarkerS0 = mNh.advertise<visualization_msgs::Marker>("MarkerS0",1000);
    mPubMarkerS0v2= mNh.advertise<visualization_msgs::Marker>("MarkerS0v2",1000);

    mSubCloud0S0 = mNh.subscribe<sensor_msgs::PointCloud2>("/MapPoints0MapServer0",1000,&viewer::CloudCB0S0,this);
    mPubCloud0S0 = mNh.advertise<sensor_msgs::PointCloud2>("Cloud0S0",1000);

    mSubCloud1S0 = mNh.subscribe<sensor_msgs::PointCloud2>("/MapPoints1MapServer0",1000,&viewer::CloudCB1S0,this);
    mPubCloud1S0 = mNh.advertise<sensor_msgs::PointCloud2>("Cloud1S0",1000);

    mSubCloudMS0 = mNh.subscribe<sensor_msgs::PointCloud2>("/MapPointsMultiUseMapServer0",1000,&viewer::CloudCBMS0,this);
    mPubCloudMS0 = mNh.advertise<sensor_msgs::PointCloud2>("CloudMS0",1000);

    mSubLoopMarkerS0 = mNh.subscribe<visualization_msgs::Marker>("LoopMarker1",10,&viewer::LoopCBS0,this);
    mPubLoopMarkerS0 = mNh.advertise<visualization_msgs::Marker>("LoopS0",10);

    mNhPrivate.param("KfCutOffS0",mKfCutOffS0,10);
    mNhPrivate.param("ScaleS0",mScaleS0,1.0);
    mNhPrivate.param("KFScaleS0",mKFScaleS0,1.0);
    mNhPrivate.param("TrajSizeS0",mTrajSizeS0,1.0);
    mNhPrivate.param("TrajSizeS0v2",mTrajSizeS0v2,1.0);
    mNhPrivate.param("CamLineSizeS0",mCamLineSizeS0,1.0);
    mNhPrivate.param("BoxS0P1x",mBoxS0P1x,0.0);
    mNhPrivate.param("BoxS0P1y",mBoxS0P1y,0.0);
    mNhPrivate.param("BoxS0P2x",mBoxS0P2x,0.0);
    mNhPrivate.param("BoxS0P2y",mBoxS0P2y,0.0);
    mNhPrivate.param("BoxS0P3x",mBoxS0P3x,0.0);
    mNhPrivate.param("BoxS0P3y",mBoxS0P3y,0.0);
    mNhPrivate.param("BoxS0P4x",mBoxS0P4x,0.0);
    mNhPrivate.param("BoxS0P4y",mBoxS0P4y,0.0);

    mNhPrivate.param("BoxS0xl",mBoxS0xl,0.0);
    mNhPrivate.param("BoxS0xh",mBoxS0xh,0.0);
    mNhPrivate.param("BoxS0zl",mBoxS0zl,0.0);
    mNhPrivate.param("BoxS0zh",mBoxS0zh,0.0);

    mNhPrivate.param("S0zMin",mS0zMin,-10000.0);
    mNhPrivate.param("S0zMax",mS0zMax,10000.0);

    //++++++++++ Server 1++++++++++
    mSubMarkerS1 = mNh.subscribe<visualization_msgs::Marker>("MarkerMapServer1",1000,&viewer::MarkerCBS1,this);
    mPubMarkerS1 = mNh.advertise<visualization_msgs::Marker>("MarkerS1",1000);
    mPubMarkerS1v2 = mNh.advertise<visualization_msgs::Marker>("MarkerS1v2",1000);

    mSubCloud0S1 = mNh.subscribe<sensor_msgs::PointCloud2>("/MapPointS1MapServer1",1000,&viewer::CloudCB0S1,this);
    mPubCloud0S1 = mNh.advertise<sensor_msgs::PointCloud2>("Cloud0S1",1000);

    mSubCloud1S1 = mNh.subscribe<sensor_msgs::PointCloud2>("/MapPoints1MapServer1",1000,&viewer::CloudCB1S1,this);
    mPubCloud1S1 = mNh.advertise<sensor_msgs::PointCloud2>("Cloud1S1",1000);

    mSubCloudMS1 = mNh.subscribe<sensor_msgs::PointCloud2>("/MapPointsMultiUseMapServer1",1000,&viewer::CloudCBMS1,this);
    mPubCloudMS1 = mNh.advertise<sensor_msgs::PointCloud2>("CloudMS1",1000);

    mNhPrivate.param("KfCutOffS1",mKfCutOffS1,10);
    mNhPrivate.param("ScaleS1",mScaleS1,1.0);
    mNhPrivate.param("TrajSizeS1",mTrajSizeS1,1.0);
    mNhPrivate.param("CamLineSizeS1",mCamLineSizeS1,1.0);
    mNhPrivate.param("BoxS1P1x",mBoxS1P1x,0.0);
    mNhPrivate.param("BoxS1P1y",mBoxS1P1y,0.0);
    mNhPrivate.param("BoxS1P2x",mBoxS1P2x,0.0);
    mNhPrivate.param("BoxS1P2y",mBoxS1P2y,0.0);
    mNhPrivate.param("BoxS1P3x",mBoxS1P3x,0.0);
    mNhPrivate.param("BoxS1P3y",mBoxS1P3y,0.0);
    mNhPrivate.param("BoxS1P4x",mBoxS1P4x,0.0);
    mNhPrivate.param("BoxS1P4y",mBoxS1P4y,0.0);
    mNhPrivate.param("S1zMin",mS1zMin,-10000.0);
    mNhPrivate.param("S1zMax",mS1zMax,10000.0);

    //++++++++++ Server 2++++++++++
    mSubMarkerS2 = mNh.subscribe<visualization_msgs::Marker>("MarkerMapServer2",1000,&viewer::MarkerCBS2,this);
    mPubMarkerS2 = mNh.advertise<visualization_msgs::Marker>("MarkerS2",1000);

    mNhPrivate.param("KfCutOffS2",mKfCutOffS2,10);
    mNhPrivate.param("ScaleS2",mScaleS2,1.0);
    mNhPrivate.param("TrajSizeS2",mTrajSizeS2,1.0);
    mNhPrivate.param("CamLineSizeS2",mCamLineSizeS2,1.0);
    //++++++++++ Server 3++++++++++
    mSubMarkerS3 = mNh.subscribe<visualization_msgs::Marker>("MarkerMapServer3",1000,&viewer::MarkerCBS3,this);
    mPubMarkerS3 = mNh.advertise<visualization_msgs::Marker>("MarkerS3",1000);

    mNhPrivate.param("KfCutOffS3",mKfCutOffS3,10);
    mNhPrivate.param("ScaleS3",mScaleS3,1.0);
    mNhPrivate.param("TrajSizeS3",mTrajSizeS3,1.0);
    mNhPrivate.param("CamLineSizeS3",mCamLineSizeS3,1.0);
    //++++++++++ Other ++++++++++

    mPubMarkersAug = mNh.advertise<visualization_msgs::Marker>("MarkerAug",5);

    mRect.header.frame_id = "odomS0";
    mRect.header.stamp = ros::Time::now();
    mRect.id = 0;
    mRect.ns = "rect1";
    mRect.type = visualization_msgs::Marker::LINE_STRIP;
    mRect.scale.x = mTrajSizeS0;
    mRect.action = visualization_msgs::Marker::ADD;
    mRect.color.r = 0.0f;
    mRect.color.g = 0.0f;
    mRect.color.b = 0.1f;
    mRect.color.a = 1.0;

    geometry_msgs::Point p0,p1,p2,p3;
    p0.x = mBoxS0xl;
    p0.y = 0.0;
    p0.z = mBoxS0zl;
    p1.x = mBoxS0xl;
    p1.y = 0.0;
    p1.z = mBoxS0zh;
    p2.x = mBoxS0xh;
    p2.y = 0.0;
    p2.z = mBoxS0zh;
    p3.x = mBoxS0xh;
    p3.y = 0.0;
    p3.z = mBoxS0zl;

    mRect.points.push_back(p0);
    mRect.points.push_back(p1);
    mRect.points.push_back(p2);
    mRect.points.push_back(p3);
    mRect.points.push_back(p0);

}

void viewer::MarkerCBS0(visualization_msgs::Marker Msg)
{
//    cout << "Msg.ns: " << Msg.ns << endl;

    if(Msg.ns == "Traj0")
    {
//        cout << "Traj0: Msg.ns: " << Msg.ns << endl;
        //Trajectory


        Msg.scale.x = mTrajSizeS0;
//        cout << "Traj0: Msg.scale.x: " << Msg.scale.x << endl;

        vector<geometry_msgs::Point> vPoints;

//        Msg.color.r = 0.0f;
//        Msg.color.g = 0.0f;
//        Msg.color.b = 0.0f;

        int cKFs = 0;
//        for(vector<geometry_msgs::Point>::iterator vit = Msg.points.begin();vit!=Msg.points.end();++vit)
        for(int vit = 0;vit < Msg.points.size();++vit)
        {
            ++cKFs;
            if(cKFs >= mKfCutOffS0)
            {
//                cout << "erase KF" << endl;
//                Msg.points.erase(vit);
            }
            else
            {
                geometry_msgs::Point p = Msg.points[vit];
                p.x = mScaleS0*p.x;
                p.y = mScaleS0*p.y;
                p.z = mScaleS0*p.z;
//                vit->x = mScaleS0*vit->x;
//                vit->y = mScaleS0*vit->y;
//                vit->z = mScaleS0*vit->z;
                vPoints.push_back(p);
            }
        }
        Msg.points = vPoints;

        Msg.header.stamp = ros::Time::now();
//        Msg.header.frame_id = "odomHelp";

        mPubMarkerS0.publish(Msg);

        Msg.scale.x = mTrajSizeS0v2;

        mPubMarkerS0v2.publish(Msg);
    }
    else if(Msg.ns == "Traj1")
    {
        //Trajectory

        Msg.scale.x = mTrajSizeS1;
//        cout << "Traj1: Msg.scale.x: " << Msg.scale.x << endl;

        vector<geometry_msgs::Point> vPoints;

//        Msg.color.r = 0.0f;
//        Msg.color.g = 0.0f;
//        Msg.color.b = 1.0f;

        int cKFs = 0;
//        for(vector<geometry_msgs::Point>::iterator vit = Msg.points.begin();vit!=Msg.points.end();++vit)
        for(int vit = 0;vit < Msg.points.size();++vit)
        {
            ++cKFs;
            if(cKFs >= mKfCutOffS1)
            {
//                cout << "erase KF" << endl;
//                Msg.points.erase(vit);
            }
            else
            {
                geometry_msgs::Point p = Msg.points[vit];
                p.x = mScaleS0*p.x;
                p.y = mScaleS0*p.y;
                p.z = mScaleS0*p.z;
//                vit->x = mScaleS0*vit->x;
//                vit->y = mScaleS0*vit->y;
//                vit->z = mScaleS0*vit->z;
                vPoints.push_back(p);
            }
        }
        Msg.points = vPoints;

        Msg.header.stamp = ros::Time::now();
//        Msg.header.frame_id = "odomHelp";

        mPubMarkerS0.publish(Msg);

        Msg.scale.x = mTrajSizeS0v2;

        mPubMarkerS0v2.publish(Msg);
    }
    else if(Msg.ns == "Traj2")
    {
        //Trajectory

        Msg.scale.x = mTrajSizeS2;
//        cout << "Traj1: Msg.scale.x: " << Msg.scale.x << endl;

        vector<geometry_msgs::Point> vPoints;

        Msg.color.r = 0.0f;
        Msg.color.g = 0.7f;
        Msg.color.b = 0.0f;

        int cKFs = 0;
//        for(vector<geometry_msgs::Point>::iterator vit = Msg.points.begin();vit!=Msg.points.end();++vit)
        for(int vit = 0;vit < Msg.points.size();++vit)
        {
            ++cKFs;
            if(cKFs >= mKfCutOffS2)
            {
//                cout << "erase KF" << endl;
//                Msg.points.erase(vit);
            }
            else
            {
                geometry_msgs::Point p = Msg.points[vit];
                p.x = mScaleS0*p.x;
                p.y = mScaleS0*p.y;
                p.z = mScaleS0*p.z;
//                vit->x = mScaleS0*vit->x;
//                vit->y = mScaleS0*vit->y;
//                vit->z = mScaleS0*vit->z;
                vPoints.push_back(p);
            }
        }
        Msg.points = vPoints;

        mPubMarkerS0.publish(Msg);

        Msg.scale.x = mTrajSizeS0v2;

        mPubMarkerS0v2.publish(Msg);
    }
    else
    {
//        std::stringstream* ss0;
//        std::stringstream* ss1;
//        ss0 = new stringstream;
//        ss1 = new stringstream;

//        *ss0 << "comm0" << endl << "KFs0Map0";
//        *ss1 << "comm0" << endl << "KFs1Map0";

//        if(Msg.ns == ss0->str())
//        {
////            Msg.color.r = 0.0f;
////            Msg.color.g = 0.8f;
////            Msg.color.b = 0.0f;

//            Msg.scale.x = mCamLineSizeS0;

////            vector<geometry_msgs::Point> vPoints;

////            int cp = 0;
////            for(int vit = 0;vit < Msg.points.size();++vit)
////            {
////                geometry_msgs::Point p = Msg.points[vit];

////                if(cp == 0 || cp == 2 || cp == 4 || cp == 7)
////                {
////                    p.x = mScaleS0*p.x;
////                    p.y = mScaleS0*p.y;
////                    p.z = mScaleS0*p.z;
////                }
////                else
////                {
////                    p.x = mKFScaleS0*p.x;
////                    p.y = mKFScaleS0*p.y;
////                    p.z = mKFScaleS0*p.z;
////                }

////                vPoints.push_back(p);

////                ++cp;
////                if(cp == 12)
////                    cp = 0;
////            }

////            Msg.points = vPoints;

//            vector<geometry_msgs::Point> vPoints;

//            int cKFs = 0;
//    //        for(vector<geometry_msgs::Point>::iterator vit = Msg.points.begin();vit!=Msg.points.end();++vit)
//            for(int vit = 0;vit < Msg.points.size();++vit)
//            {
//                if(cKFs >= mKfCutOffS0*12)
//                {
//                    //...
//                }
//                else
//                {
//                    geometry_msgs::Point p = Msg.points[vit];
//                    p.x = mScaleS0*p.x;
//                    p.y = mScaleS0*p.y;
//                    p.z = mScaleS0*p.z;
//                    vPoints.push_back(p);
//                }
//                ++cKFs;
//            }
//            Msg.points = vPoints;

////            for(vector<geometry_msgs::Point>::iterator vit = Msg.points.begin();vit!=Msg.points.end();++vit)
////            {
////                vit->x = mScaleS0*vit->x;
////                vit->y = mScaleS0*vit->y;
////                vit->z = mScaleS0*vit->z;
////            }

//        }
//        else if(Msg.ns == ss1->str())
//        {
//            Msg.scale.x = mCamLineSizeS0;
////            for(vector<geometry_msgs::Point>::iterator vit = Msg.points.begin();vit!=Msg.points.end();++vit)
////            {
////                vit->x = mScaleS0*vit->x;
////                vit->y = mScaleS0*vit->y;
////                vit->z = mScaleS0*vit->z;
////            }
//            vector<geometry_msgs::Point> vPoints;

//            int cKFs = 0;
//    //        for(vector<geometry_msgs::Point>::iterator vit = Msg.points.begin();vit!=Msg.points.end();++vit)
//            for(int vit = 0;vit < Msg.points.size();++vit)
//            {
////                ++cKFs;
//                if(cKFs >= mKfCutOffS1)
//                {
//    //                cout << "erase KF" << endl;
//    //                Msg.points.erase(vit);
//                }
//                else
//                {
//                    geometry_msgs::Point p = Msg.points[vit];
//                    p.x = mScaleS0*p.x;
//                    p.y = mScaleS0*p.y;
//                    p.z = mScaleS0*p.z;
//    //                vit->x = mScaleS0*vit->x;
//    //                vit->y = mScaleS0*vit->y;
//    //                vit->z = mScaleS0*vit->z;
//                    vPoints.push_back(p);
//                }
//                ++cKFs;
//            }
//            Msg.points = vPoints;
//        }

//        delete ss0;
//        delete ss1;

//        cout << "Else: Msg.ns: " << Msg.ns << endl;
        //Keyframes

        Msg.scale.x = mCamLineSizeS0;

        Msg.header.stamp = ros::Time::now();

        mPubMarkerS0.publish(Msg);
        mPubMarkerS0v2.publish(Msg);
    }
}

void viewer::CloudCB0S0(sensor_msgs::PointCloud2 Msg)
{
    Msg.header.stamp = ros::Time::now();

    mPubCloud0S0.publish(Msg);
}

void viewer::CloudCB1S0(sensor_msgs::PointCloud2 Msg)
{
//    pcl::PointCloud<pcl::PointXYZ> cloud,cloudnew;
//    pcl::fromROSMsg(Msg,cloud);

//    for(vector<pcl::PointXYZ,Eigen::aligned_allocator<pcl::PointXYZ>>::iterator cit = cloud.points.begin();cit!=cloud.points.end();++cit)
//    {
//        pcl::PointXYZ p = *cit;

//        bool bx = mBoxS0xl <= p.x && p.x <= mBoxS0xh;
//        bool bz = mBoxS0zl <= p.z && p.z <= mBoxS0zh;

//        if(bx && bz)
//        {
////            cout << "skip point" << endl;
//            continue;
//        }
//        else
//            cloudnew.points.push_back(p);
//    }

//    sensor_msgs::PointCloud2 msgnew;
//    pcl::toROSMsg(cloudnew,msgnew);

//    msgnew.header = Msg.header;

//    mPubCloud1S0.publish(msgnew);
//    mPubMarkersAug.publish(mRect);

    Msg.header.stamp = ros::Time::now();

    mPubCloud1S0.publish(Msg);
}

void viewer::CloudCBMS0(sensor_msgs::PointCloud2 Msg)
{
    Msg.header.stamp = ros::Time::now();

    mPubCloudMS0.publish(Msg);
}

void viewer::MarkerCBS1(visualization_msgs::Marker Msg)
{
    if(Msg.ns == "Traj1")
    {
        Msg.scale.x = mTrajSizeS1;

        Msg.header.stamp = ros::Time::now();

        mPubMarkerS1.publish(Msg);

        Msg.scale.x = mTrajSizeS0v2;

        mPubMarkerS1v2.publish(Msg);
    }
}

void viewer::CloudCB0S1(sensor_msgs::PointCloud2 Msg)
{
    Msg.header.stamp = ros::Time::now();

    mPubCloud0S1.publish(Msg);
}

void viewer::CloudCB1S1(sensor_msgs::PointCloud2 Msg)
{
    Msg.header.stamp = ros::Time::now();

    mPubCloud1S1.publish(Msg);
}

void viewer::CloudCBMS1(sensor_msgs::PointCloud2 Msg)
{
    Msg.header.stamp = ros::Time::now();

    mPubCloudMS1.publish(Msg);
}

void viewer::MarkerCBS2(visualization_msgs::Marker Msg)
{
    if(Msg.ns == "Traj2")
    {
        Msg.scale.x = mTrajSizeS2;
        mPubMarkerS2.publish(Msg);
    }
}

void viewer::MarkerCBS3(visualization_msgs::Marker Msg)
{
    if(Msg.ns == "Traj0")
    {
        Msg.scale.x = mTrajSizeS3;
        mPubMarkerS3.publish(Msg);
    }
    else if(Msg.ns == "Traj1")
    {
        Msg.scale.x = mTrajSizeS3;
        mPubMarkerS3.publish(Msg);
    }
    else if(Msg.ns == "Traj2")
    {
        Msg.scale.x = mTrajSizeS3;
        mPubMarkerS3.publish(Msg);
    }
    else if(Msg.ns == "Traj3")
    {
        Msg.scale.x = mTrajSizeS3;
        mPubMarkerS3.publish(Msg);
    }
}

void viewer::MarkerCBC0(visualization_msgs::Marker Msg)
{
//    cout << "ns: " << Msg.ns << endl;

    if(Msg.ns == "NativeTraj")
    {
//        cout << "Pub" << endl;
        Msg.scale.x = mTrajSizeC0;

//        Msg.color.r = 0.0f;
//        Msg.color.g = 0.0f;
//        Msg.color.b = 0.0f;

        geometry_msgs::Point p;

//        for(vector<geometry_msgs::Point>::iterator vit = Msg.points.begin();vit!=Msg.points.end();++vit)
        for(int vit = 0;vit < Msg.points.size();++vit)
        {
            p = Msg.points[vit];
        }

        tf::Transform T;
        tf::Quaternion q;
        q.setX(0.0);
        q.setY(0.0);
        q.setZ(0.0);
        q.setW(1.0);
        T.setRotation(q);
        T.setOrigin(tf::Vector3(p.x,p.y,p.z));
        tf::StampedTransform Ts(T,ros::Time::now(),"odomC0","C0view");
        mPubTFC0.sendTransform(Ts);

        Msg.header.stamp = ros::Time::now();

        mPubMarkerC0.publish(Msg);
    }
    if(Msg.ns == "KFs1Map0")
    {
        Msg.header.stamp = ros::Time::now();

        mPubMarkerC0.publish(Msg);
    }
//ns: comm0KFs0Map0
//ns: comm0KFs1Map0
}

void viewer::MarkerCBC1(visualization_msgs::Marker Msg)
{
//    cout << "ns: " << Msg.ns << endl;

    if(Msg.ns == "NativeTraj")
    {
//        cout << "Pub" << endl;
        Msg.scale.x = mTrajSizeC1;

//        Msg.color.r = 0.0f;
//        Msg.color.g = 0.0f;
//        Msg.color.b = 0.0f;

        geometry_msgs::Point p;

//        for(vector<geometry_msgs::Point>::iterator vit = Msg.points.begin();vit!=Msg.points.end();++vit)
        for(int vit = 0;vit < Msg.points.size();++vit)
        {
            p = Msg.points[vit];
        }

        tf::Transform T;
        tf::Quaternion q;
        q.setX(0.0);
        q.setY(0.0);
        q.setZ(0.0);
        q.setW(1.0);
        T.setRotation(q);
        T.setOrigin(tf::Vector3(p.x,p.y,p.z));
        tf::StampedTransform Ts(T,ros::Time::now(),"odomC1","C1view");
        mPubTFC1.sendTransform(Ts);

        Msg.header.stamp = ros::Time::now();

        mPubMarkerC1.publish(Msg);
    }
    if(Msg.ns == "KFs0Map1")
    {
        Msg.scale.x = mCamLineSizeC1;

//        vector<geometry_msgs::Point> vPoints;

//        int cp = 0;
//        for(int vit = 0;vit < Msg.points.size();++vit)
//        {
//            geometry_msgs::Point p = Msg.points[vit];

//            if(cp == 0 || cp == 2 || cp == 4 || cp == 6)
//            {
//                p.x = mScaleC1*p.x;
//                p.y = mScaleC1*p.y;
//                p.z = mScaleC1*p.z;
//            }
//            else
//            {
////                p.x = mKFScaleC1*p.x;
////                p.y = mKFScaleC1*p.y;
////                p.z = mKFScaleC1*p.z;
//            }

//            vPoints.push_back(p);

//            ++cp;
//            if(cp == 16)
//                cp = 0;
//        }

//        Msg.points = vPoints;

        Msg.header.stamp = ros::Time::now();

        mPubMarkerC1.publish(Msg);
    }
//ns: comm0KFs0Map0
//ns: comm0KFs1Map0
}

void viewer::LoopCBS0(visualization_msgs::Marker Msg)
{
    Msg.header.stamp = ros::Time::now();

    mPubLoopMarkerS0.publish(Msg);
}

void viewer::MatchCB(visualization_msgs::Marker Msg)
{
    Msg.header.stamp = ros::Time::now();

    mPubMatchMarker.publish(Msg);
}

} //end ns
