#include <helper/republisher.h>

namespace helper {

boundaries::boundaries(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, int sleeptime)
    : mNh(Nh),mNhPrivate(NhPrivate),mSleepTime(sleeptime)
{
    this->SetMarkers();

    mPubBounds = mNh.advertise<visualization_msgs::Marker>("boundaries",10);
}

void boundaries::run()
{
    while(1)
    {
//        mPubBounds.publish(mB0);
////        cout << mB1.points.size() << endl;
//        mPubBounds.publish(mB1);
//        mPubBounds.publish(mB2);
//        mPubBounds.publish(mB3);
//        mPubBounds.publish(mBS);
//        mPubBounds.publish(mBL);

        for(vector<visualization_msgs::Marker>::const_iterator vit = mvMsgs.begin();vit!=mvMsgs.end();++vit)
        {
//            cout << vit->points.size() << endl;
            mPubBounds.publish(*vit);
        }

        usleep((mSleepTime));
    }
}

void boundaries::SetMarkers()
{
    double r0,r1,r2,r3,g0,g1,g2,g3,b0,b1,b2,b3;

    mNhPrivate.param("ColorR0",r0,1.0);
    mNhPrivate.param("ColorG0",g0,1.0);
    mNhPrivate.param("ColorB0",b0,1.0);

    mNhPrivate.param("ColorR1",r1,1.0);
    mNhPrivate.param("ColorG1",g1,1.0);
    mNhPrivate.param("ColorB1",b1,1.0);

    mNhPrivate.param("ColorR2",r2,1.0);
    mNhPrivate.param("ColorG2",g2,1.0);
    mNhPrivate.param("ColorB2",b2,1.0);

    mNhPrivate.param("ColorR3",r3,1.0);
    mNhPrivate.param("ColorG3",g3,1.0);
    mNhPrivate.param("ColorB3",b3,1.0);

    double d;
    mNhPrivate.param("BoundMarkerSize",d,1.0);

    geometry_msgs::Point p0,p1,p2,p3;

    mB0.header.frame_id = "world";
    mB0.header.stamp = ros::Time::now();
    mB0.id = 0;
    mB0.ns = "bounds0";
    mB0.type = visualization_msgs::Marker::LINE_STRIP;
    mB0.scale.x = d;
    mB0.action = visualization_msgs::Marker::ADD;
    mB0.color.r = r0;
    mB0.color.g = g0;
    mB0.color.b = b0;
    mB0.color.a = 1.0;

    mNhPrivate.param("x0bl",p0.x,0.0);
    mNhPrivate.param("y0bl",p0.y,0.0);
    p0.z = 0.0;
    mNhPrivate.param("x0br",p1.x,0.0);
    mNhPrivate.param("y0br",p1.y,0.0);
    p1.z = 0.0;
    mNhPrivate.param("x0tr",p2.x,0.0);
    mNhPrivate.param("y0tr",p2.y,0.0);
    p2.z = 0.0;
    mNhPrivate.param("x0tl",p3.x,0.0);
    mNhPrivate.param("y0tl",p3.y,0.0);
    p3.z = 0.0;
    mB0.points.push_back(p0);
    mB0.points.push_back(p1);
    mB0.points.push_back(p2);
    mB0.points.push_back(p3);
    mB0.points.push_back(p0);

    mB1.header.frame_id = "world";
    mB1.header.stamp = ros::Time::now();
    mB1.id = 0;
    mB1.ns = "bounds1";
    mB1.type = visualization_msgs::Marker::LINE_STRIP;
    mB1.scale.x = d;
    mB1.action = visualization_msgs::Marker::ADD;
    mB1.color.r = r1;
    mB1.color.g = g1;
    mB1.color.b = b1;
    mB1.color.a = 1.0;

    mNhPrivate.param("x1bl",p0.x,0.0);
    mNhPrivate.param("y1bl",p0.y,0.0);
    p0.z = 0.0;
    mNhPrivate.param("x1br",p1.x,0.0);
    mNhPrivate.param("y1br",p1.y,0.0);
    p1.z = 0.0;
    mNhPrivate.param("x1tr",p2.x,0.0);
    mNhPrivate.param("y1tr",p2.y,0.0);
    p2.z = 0.0;
    mNhPrivate.param("x1tl",p3.x,0.0);
    mNhPrivate.param("y1tl",p3.y,0.0);
    p3.z = 0.0;
    mB1.points.push_back(p0);
    mB1.points.push_back(p1);
    mB1.points.push_back(p2);
    mB1.points.push_back(p3);
    mB1.points.push_back(p0);

    mB2.header.frame_id = "world";
    mB2.header.stamp = ros::Time::now();
    mB2.id = 0;
    mB2.ns = "bounds2";
    mB2.type = visualization_msgs::Marker::LINE_STRIP;
    mB2.scale.x = d;
    mB2.action = visualization_msgs::Marker::ADD;
    mB2.color.r = r2;
    mB2.color.g = g2;
    mB2.color.b = b2;
    mB2.color.a = 1.0;

    mNhPrivate.param("x2bl",p0.x,0.0);
    mNhPrivate.param("y2bl",p0.y,0.0);
    p0.z = 0.0;
    mNhPrivate.param("x2br",p1.x,0.0);
    mNhPrivate.param("y2br",p1.y,0.0);
    p1.z = 0.0;
    mNhPrivate.param("x2tr",p2.x,0.0);
    mNhPrivate.param("y2tr",p2.y,0.0);
    p2.z = 0.0;
    mNhPrivate.param("x2tl",p3.x,0.0);
    mNhPrivate.param("y2tl",p3.y,0.0);
    p3.z = 0.0;
    mB2.points.push_back(p0);
    mB2.points.push_back(p1);
    mB2.points.push_back(p2);
    mB2.points.push_back(p3);
    mB2.points.push_back(p0);

    mB3.header.frame_id = "world";
    mB3.header.stamp = ros::Time::now();
    mB3.id = 0;
    mB3.ns = "bounds3";
    mB3.type = visualization_msgs::Marker::LINE_STRIP;
    mB3.scale.x = d;
    mB3.action = visualization_msgs::Marker::ADD;
    mB3.color.r = r3;
    mB3.color.g = g3;
    mB3.color.b = b3;
    mB3.color.a = 1.0;

    mNhPrivate.param("x3bl",p0.x,0.0);
    mNhPrivate.param("y3bl",p0.y,0.0);
    p0.z = 0.0;
    mNhPrivate.param("x3br",p1.x,0.0);
    mNhPrivate.param("y3br",p1.y,0.0);
    p1.z = 0.0;
    mNhPrivate.param("x3tr",p2.x,0.0);
    mNhPrivate.param("y3tr",p2.y,0.0);
    p2.z = 0.0;
    mNhPrivate.param("x3tl",p3.x,0.0);
    mNhPrivate.param("y3tl",p3.y,0.0);
    p3.z = 0.0;
    mB3.points.push_back(p0);
    mB3.points.push_back(p1);
    mB3.points.push_back(p2);
    mB3.points.push_back(p3);
    mB3.points.push_back(p0);

    mBS.header.frame_id = "world";
    mBS.header.stamp = ros::Time::now();
    mBS.id = 0;
    mBS.ns = "boundsS";
    mBS.type = visualization_msgs::Marker::LINE_STRIP;
    mBS.scale.x = d;
    mBS.action = visualization_msgs::Marker::ADD;
    mBS.color.r = 0.8;
    mBS.color.g = 0.0;
    mBS.color.b = 0.0;
    mBS.color.a = 1.0;

    mNhPrivate.param("xsbl",p0.x,0.0);
    mNhPrivate.param("ysbl",p0.y,0.0);
    p0.z = 0.0;
    mNhPrivate.param("xsbr",p1.x,0.0);
    mNhPrivate.param("ysbr",p1.y,0.0);
    p1.z = 0.0;
    mNhPrivate.param("xstr",p2.x,0.0);
    mNhPrivate.param("ystr",p2.y,0.0);
    p2.z = 0.0;
    mNhPrivate.param("xstl",p3.x,0.0);
    mNhPrivate.param("ystl",p3.y,0.0);
    p3.z = 0.0;
    mBS.points.push_back(p0);
    mBS.points.push_back(p1);
    mBS.points.push_back(p2);
    mBS.points.push_back(p3);
    mBS.points.push_back(p0);

    mBL.header.frame_id = "world";
    mBL.header.stamp = ros::Time::now();
    mBL.id = 0;
    mBL.ns = "boundsL";
    mBL.type = visualization_msgs::Marker::LINE_STRIP;
    mBL.scale.x = d;
    mBL.action = visualization_msgs::Marker::ADD;
    mBL.color.r = 8.0;
    mBL.color.g = 0.0;
    mBL.color.b = 0.0;
    mBL.color.a = 1.0;

    mNhPrivate.param("xlbl",p0.x,0.0);
    mNhPrivate.param("ylbl",p0.y,0.0);
    p0.z = 0.0;
    mNhPrivate.param("xlbr",p1.x,0.0);
    mNhPrivate.param("ylbr",p1.y,0.0);
    p1.z = 0.0;
    mNhPrivate.param("xltr",p2.x,0.0);
    mNhPrivate.param("yltr",p2.y,0.0);
    p2.z = 0.0;
    mNhPrivate.param("xltl",p3.x,0.0);
    mNhPrivate.param("yltl",p3.y,0.0);
    p3.z = 0.0;
    mBL.points.push_back(p0);
    mBL.points.push_back(p1);
    mBL.points.push_back(p2);
    mBL.points.push_back(p3);
    mBL.points.push_back(p0);

    mvMsgs.push_back(mB0);
    mvMsgs.push_back(mB1);
    mvMsgs.push_back(mB2);
    mvMsgs.push_back(mB3);
    mvMsgs.push_back(mBS);
    mvMsgs.push_back(mBL);
}

republisher::republisher(ros::NodeHandle& Nh, ros::NodeHandle& NhPrivate, std::string op_mode) :
    Nh_(Nh), NhPrivate_(NhPrivate)
{
    if(op_mode == "GT")
    {
        mSubLeica0 = Nh_.subscribe<geometry_msgs::PointStamped>("/leica/position",10000,boost::bind(&republisher::LeicaCb0,this,_1));
        mPubTraj0 = Nh_.advertise<visualization_msgs::Marker>("Leica0",10000);

        mSubLeica1 = Nh_.subscribe<geometry_msgs::PointStamped>("/leica/position1",10000,boost::bind(&republisher::LeicaCb1,this,_1));
        mPubTraj1 = Nh_.advertise<visualization_msgs::Marker>("Leica1",10000);

        mSubLeica2 = Nh_.subscribe<geometry_msgs::PointStamped>("/leica/position2",10000,boost::bind(&republisher::LeicaCb2,this,_1));
        mPubTraj2 = Nh_.advertise<visualization_msgs::Marker>("Leica2",10000);

        mSubLeica3 = Nh_.subscribe<geometry_msgs::PointStamped>("/leica/position3",10000,boost::bind(&republisher::LeicaCb3,this,_1));
        mPubTraj3 = Nh_.advertise<visualization_msgs::Marker>("Leica3",10000);

        double r0,r1,r2,r3,g0,g1,g2,g3,b0,b1,b2,b3;

        NhPrivate_.param("ColorR0",r0,1.0);
        NhPrivate_.param("ColorG0",g0,1.0);
        NhPrivate_.param("ColorB0",b0,1.0);

        NhPrivate_.param("ColorR1",r1,1.0);
        NhPrivate_.param("ColorG1",g1,1.0);
        NhPrivate_.param("ColorB1",b1,1.0);

        NhPrivate_.param("ColorR2",r2,1.0);
        NhPrivate_.param("ColorG2",g2,1.0);
        NhPrivate_.param("ColorB2",b2,1.0);

        NhPrivate_.param("ColorR3",r3,1.0);
        NhPrivate_.param("ColorG3",g3,1.0);
        NhPrivate_.param("ColorB3",b3,1.0);

        double x;

        NhPrivate_.param("x",x,0.1);
        NhPrivate_.param("scale",mScale,1.0);

        NhPrivate_.param("LineLength",mLineLength,10);

        mTraj0.header.frame_id = "odomLABag";
        mTraj0.header.stamp = ros::Time::now();
        mTraj0.id = 0;
        mTraj0.ns = "leicaGT0";
        mTraj0.type = visualization_msgs::Marker::LINE_STRIP;
//        mTraj0.type = visualization_msgs::Marker::LINE_LIST;
        mTraj0.scale.x = x;
        mTraj0.action = visualization_msgs::Marker::ADD;
        mTraj0.color.r = r0;
        mTraj0.color.g = g0;
        mTraj0.color.b = b0;
        mTraj0.color.a = 1.0;

        mTraj1.header.frame_id = "odomLABag";
        mTraj1.header.stamp = ros::Time::now();
        mTraj1.id = 0;
        mTraj1.ns = "leicaGT1";
        mTraj1.type = visualization_msgs::Marker::LINE_STRIP;
        mTraj1.scale.x = x;
        mTraj1.action = visualization_msgs::Marker::ADD;
        mTraj1.color.r = r1;
        mTraj1.color.g = g1;
        mTraj1.color.b = b1;
        mTraj1.color.a = 1.0;

        mTraj2.header.frame_id = "odomLABag";
        mTraj2.header.stamp = ros::Time::now();
        mTraj2.id = 0;
        mTraj2.ns = "leicaGT2";
        mTraj2.type = visualization_msgs::Marker::LINE_STRIP;
        mTraj2.scale.x = x;
        mTraj2.action = visualization_msgs::Marker::ADD;
        mTraj2.color.r = r2;
        mTraj2.color.g = g2;
        mTraj2.color.b = b2;
        mTraj2.color.a = 1.0;

        mTraj3.header.frame_id = "odomLABag";
        mTraj3.header.stamp = ros::Time::now();
        mTraj3.id = 0;
        mTraj3.ns = "leicaGT3";
        mTraj3.type = visualization_msgs::Marker::LINE_STRIP;
        mTraj3.scale.x = x;
        mTraj3.action = visualization_msgs::Marker::ADD;
        mTraj3.color.r = r3;
        mTraj3.color.g = g3;
        mTraj3.color.b = b3;
        mTraj3.color.a = 1.0;

        mbIniPointSet = false;

        /* data set leica21.bag
        mIniPoint.x = -6.5;
        mIniPoint.y = 8.8;
        mIniPoint.z = -5.0;
        */

        mIniPoint.x = -187.0;
        mIniPoint.y = 173.5;
        mIniPoint.z = -10.0;

        //mesh marker
        mPubMesh = Nh_.advertise<visualization_msgs::Marker>("mesh",1);

        mMesh.type = visualization_msgs::Marker::MESH_RESOURCE;
        mMesh.mesh_resource = "file:///home/pschmuck/house.dae";
        mMesh.header.frame_id = "odomM";
        mMesh.ns = "mesh";
        mMesh.id = 0;
        mMesh.header.stamp = ros::Time::now();
        mMesh.scale.x = 1.0;
        mMesh.scale.y = 2.0;
        mMesh.scale.z = 2.0;
        mMesh.action = visualization_msgs::Marker::ADD;

        mMesh.pose.position.x = 0.0;
        mMesh.pose.position.y = 0.0;
        mMesh.pose.position.z = 0.0;

        mMesh.color.a = 1.0;
        mMesh.color.r = 0.6;
        mMesh.color.g = 0.6;
        mMesh.color.b = 0.6;
    }
    else
    {
        ROS_ERROR_STREAM("In \" simple_repub::simple_repub(...)\": called republisher node without specification of operation mode");
        throw estd::infrastructure_ex();
    }
}

void republisher::LeicaCb0(geometry_msgs::PointStampedConstPtr msg)
{
//    static int linecount0 = 0;
//    static bool lineskip = false;

    geometry_msgs::Point p;
    p.x = mScale*(msg->point.x - mIniPoint.x);
    p.y = mScale*(msg->point.y - mIniPoint.y);
    p.z = mScale*(msg->point.z - mIniPoint.z);

//    if(linecount0 == 0)
//    {
//        //nothing
//    }
//    else
//    {
//        if(lineskip)
//        {
//            //nothing
//        }
//        else
//        {
//            mTraj0.points.push_back(mLastPoint0);
//            mTraj0.points.push_back(p);
//            mPubTraj0.publish(mTraj0);
//        }
//    }

//    mLastPoint0 = p;

//    ++linecount0;
//    if(linecount0 % mLineLength == 0)
//    {
//        lineskip = !lineskip;
//        linecount0 = 0;
//    }

//    return;

    mTraj0.points.push_back(p);

    mTraj0.header.stamp = ros::Time::now();
    mPubTraj0.publish(mTraj0);
}

void republisher::LeicaCb1(geometry_msgs::PointStampedConstPtr msg)
{
    geometry_msgs::Point p;
    p.x = mScale*(msg->point.x - mIniPoint.x);
    p.y = mScale*(msg->point.y - mIniPoint.y);
    p.z = mScale*(msg->point.z - mIniPoint.z);
    mTraj1.points.push_back(p);

    mTraj1.header.stamp = ros::Time::now();
    mPubTraj1.publish(mTraj1);
}

void republisher::LeicaCb2(geometry_msgs::PointStampedConstPtr msg)
{
    geometry_msgs::Point p;
    p.x = mScale*(msg->point.x - mIniPoint.x);
    p.y = mScale*(msg->point.y - mIniPoint.y);
    p.z = mScale*(msg->point.z - mIniPoint.z);
    mTraj2.points.push_back(p);
    mPubTraj2.publish(mTraj2);
}

void republisher::LeicaCb3(geometry_msgs::PointStampedConstPtr msg)
{
    geometry_msgs::Point p;
    p.x = mScale*(msg->point.x - mIniPoint.x);
    p.y = mScale*(msg->point.y - mIniPoint.y);
    p.z = mScale*(msg->point.z - mIniPoint.z);
    mTraj3.points.push_back(p);
    mPubTraj3.publish(mTraj3);
}

//void republisher::LeicaCb(geometry_msgs::PointStampedConstPtr msg)
//{
//    geometry_msgs::Point p;

//    if(!mbIniPointSet)
//    {
//        p.x = msg->point.x;
//        p.y = msg->point.y;
//        p.z = msg->point.z;
//        mIniPoint = p;

//        mbIniPointSet = true;
//    }

//    p.x = msg->point.x - mIniPoint.x;
//    p.y = msg->point.y - mIniPoint.y;
//    p.z = msg->point.z - mIniPoint.z;
//    cout << "mIniPoint.x: " << mIniPoint.x << endl;
//    cout << "mIniPoint.y: " << mIniPoint.y << endl;
//    cout << "mIniPoint.z: " << mIniPoint.z << endl;

//    mTraj.points.push_back(p);

////    cout << "#points: " << mTraj.points.size() << endl;

//    mPubTraj.publish(mTraj);
//}

} //end NS helper
