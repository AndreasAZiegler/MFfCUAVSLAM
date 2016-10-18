#include<helper/imagesampler.h>
namespace img2cloud {

imagesampler::imagesampler(ros::NodeHandle Nh, ros::NodeHandle NhPrivate)
    : mNh(Nh), mNhPrivate(NhPrivate)
{
    mNhPrivate.param("imgpathfloor",mImgPathFloor,string("nope"));
    mNhPrivate.param("imgpathlongedge",mImgPathLongEdge,string("nope"));
    mNhPrivate.param("imgpathshortedge",mImgPathShortEdge,string("nope"));

    mNhPrivate.param("width",mCloudw,10);
    mNhPrivate.param("height",mCloudh,10);
    mNhPrivate.param("zaxis",mCloudz,10);
    mNhPrivate.param("densitiy",mDensity,1);
    mNhPrivate.param("scale",mdScale,1.0);
    mNhPrivate.param("z1",mdZ1,1.0);

    mPubFloor = mNh.advertise<sensor_msgs::PointCloud2>("floor",1);
}

void imagesampler::SampleFloorCloud()
{
    cout << "Floor Image Path: " << mImgPathFloor << endl;

    //+++Filter 1 +++

    uint8_t thresRl1,thresGl1,thresBl1,thresRh1,thresGh1,thresBh1;
    int TRl1,TGl1,TBl1,TRh1,TGh1,TBh1;
    mNhPrivate.param("thresRl1",TRl1,0);
    mNhPrivate.param("thresGl1",TGl1,0);
    mNhPrivate.param("thresBl1",TBl1,0);

    mNhPrivate.param("thresRh1",TRh1,0);
    mNhPrivate.param("thresGh1",TGh1,0);
    mNhPrivate.param("thresBh1",TBh1,0);

    thresRl1 = static_cast<uint8_t>(TRl1);
    thresGl1 = static_cast<uint8_t>(TGl1);
    thresBl1 = static_cast<uint8_t>(TBl1);

    thresRh1 = static_cast<uint8_t>(TRh1);
    thresGh1 = static_cast<uint8_t>(TGh1);
    thresBh1 = static_cast<uint8_t>(TBh1);

    cout << "Filter 1: Thresholds low R|G|B: " << (int)thresRl1 << "|" << (int)thresGl1 << "|" << (int)thresBl1 << endl;
    cout << "Filter 1: Thresholds high R|G|B: " << (int)thresRh1 << "|" << (int)thresGh1 << "|" << (int)thresBh1 << endl;

    //+++Filter 2 +++

    uint8_t thresRl2,thresGl2,thresBl2,thresRh2,thresGh2,thresBh2;
    int TRl2,TGl2,TBl2,TRh2,TGh2,TBh2;
    mNhPrivate.param("thresRl2",TRl2,0);
    mNhPrivate.param("thresGl2",TGl2,0);
    mNhPrivate.param("thresBl2",TBl2,0);

    mNhPrivate.param("thresRh2",TRh2,0);
    mNhPrivate.param("thresGh2",TGh2,0);
    mNhPrivate.param("thresBh2",TBh2,0);

    thresRl2 = static_cast<uint8_t>(TRl2);
    thresGl2 = static_cast<uint8_t>(TGl2);
    thresBl2 = static_cast<uint8_t>(TBl2);

    thresRh2 = static_cast<uint8_t>(TRh2);
    thresGh2 = static_cast<uint8_t>(TGh2);
    thresBh2 = static_cast<uint8_t>(TBh2);

    cout << "Filter 2: Thresholds low R|G|B: " << (int)thresRl2 << "|" << (int)thresGl2 << "|" << (int)thresBl2 << endl;
    cout << "Filter 2: Thresholds high R|G|B: " << (int)thresRh2 << "|" << (int)thresGh2 << "|" << (int)thresBh2 << endl;


    //++++++++++

    cv::Mat img = cv::imread(mImgPathFloor,CV_LOAD_IMAGE_COLOR);

    if(!img.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return;
    }

    cout << "Floor Image Channels: " << img.channels() << endl;

    double steph = double(mCloudh)/(double(img.rows));
    mStepH = steph;
    double stepw = double(mCloudw)/(double(img.cols));
    mStepW = stepw;
    double stepz = min(steph,stepw);
    double starth = steph/2.0;
    double startw = stepw/2.0;
    double maxh = double(mCloudh)-starth; //coordinate frame is in bottom left of img
    double maxw = double(mCloudw)-startw; //coordinate frame is in bottom left of img

    for(int row=0;row<img.rows;++row)
    {
        for(int col=0;col<img.cols;++col)
        {
            pcl::PointXYZRGB p;

            double x = maxh-double(row)*steph;
            double y = maxw-double(col)*stepw;
            double z = 0.0;

            //OpenCV loads BGR images
            uint8_t r = img.at<cv::Vec3b>(row,col)[2];
            uint8_t g = img.at<cv::Vec3b>(row,col)[1];
            uint8_t b = img.at<cv::Vec3b>(row,col)[0];
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

            p.x = x;
            p.y = y;
            p.z = z;
            p.rgb = *reinterpret_cast<float*>(&rgb);

            //+++Filter 1 +++

            bool bR1 = r >= thresRl1 && r <= thresRh1;
            bool bG1 = g >= thresGl1 && g <= thresGh1;
            bool bB1 = b >= thresBl1 && b <= thresBh1;

            //+++Filter 2 +++

            bool bR2 = r >= thresRl2 && r <= thresRh2;
            bool bG2 = g >= thresGl2 && g <= thresGh2;
            bool bB2 = b >= thresBl2 && b <= thresBh2;

            //++++++++++

            if(bR1 && bG1 && bB1)
            {
                this->LiftPoint(p,stepz);

//                p.x = x;
//                p.y = y;
//                p.z = z;
//                p.rgb = *reinterpret_cast<float*>(&rgb);

//                mCloudFloor.points.push_back(p);
            }
            else if(bR2 && bG2 && bB2)
            {
                this->LiftPoint(p,stepz);

//                p.x = x;
//                p.y = y;
//                p.z = z;
//                p.rgb = *reinterpret_cast<float*>(&rgb);

//                mCloudFloor.points.push_back(p);
            }
            else
            {
               mCloudFloor.points.push_back(p);
            }
        }
    }

}

void imagesampler::SampleLongEdge()
{
    cv::Mat img = cv::imread(mImgPathLongEdge,CV_LOAD_IMAGE_COLOR);

    if(!img.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return;
    }

    double steph = double(mCloudh)/(double(img.cols));
    double stepz = double(mCloudz)/(double(img.rows));
    double starth = steph/2.0;
    double startz = stepz/2.0;
    double maxh = double(mCloudh)-starth; //coordinate frame is in bottom left of img
    double maxz = double(mCloudz)-startz; //coordinate frame is in bottom left of img
    double fixedY = mCloudw+0.5*mStepW;

    for(int row=0;row<img.rows;++row)
    {
        for(int col=0;col<img.cols;++col)
        {
            pcl::PointXYZRGB p;

            double x = starth+double(col)*steph;
            double y = fixedY;
            double z = maxz-double(row)*stepz;

            //OpenCV loads BGR images
            uint8_t r = img.at<cv::Vec3b>(row,col)[2];
            uint8_t g = img.at<cv::Vec3b>(row,col)[1];
            uint8_t b = img.at<cv::Vec3b>(row,col)[0];
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

            p.x = x;
            p.y = y;
            p.z = z;
            p.rgb = *reinterpret_cast<float*>(&rgb);

            //+++Filter 1 +++

            bool bR1 = r >= 250;
            bool bG1 = g >= 250;
            bool bB1 = b >= 250;

            if(bR1 && bG1 && bB1)
                continue;

            mCloudFloor.points.push_back(p);
        }
    }
}

void imagesampler::SampleShortEdge()
{
    cv::Mat img = cv::imread(mImgPathShortEdge,CV_LOAD_IMAGE_COLOR);

    if(!img.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return;
    }

    double stepw = double(mCloudw)/(double(img.cols));
    double stepz = double(mCloudz)/(double(img.rows));
    double startw = stepw/2.0;
    double startz = stepz/2.0;
    double maxw = double(mCloudw)-startw; //coordinate frame is in bottom left of img
    double maxz = double(mCloudz)-startz; //coordinate frame is in bottom left of img
    double fixedX = mCloudh+0.5*mStepH;


    for(int row=0;row<img.rows;++row)
    {
        for(int col=0;col<img.cols;++col)
        {
            pcl::PointXYZRGB p;

            double x = fixedX;
            double y = maxw-double(col)*stepw;
            double z = maxz-double(row)*stepz;

            //OpenCV loads BGR images
            uint8_t r = img.at<cv::Vec3b>(row,col)[2];
            uint8_t g = img.at<cv::Vec3b>(row,col)[1];
            uint8_t b = img.at<cv::Vec3b>(row,col)[0];
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

            p.x = x;
            p.y = y;
            p.z = z;
            p.rgb = *reinterpret_cast<float*>(&rgb);

            //+++Filter 1 +++

            bool bR1 = r >= 250;
            bool bG1 = g >= 250;
            bool bB1 = b >= 250;

            if(bR1 && bG1 && bB1)
                continue;

            mCloudFloor.points.push_back(p);
        }
    }

}

void imagesampler::LiftPoint(pcl::PointXYZRGB p, double stepz)
{
    for(double itz=0.0;itz<=mdZ1;itz+=stepz)
    {
        p.z = itz;
        mCloudFloor.points.push_back(p);
    }
}

void imagesampler::PubFloor()
{
    if(mCloudFloor.points.empty())
    {
        cout << "Floor cloud is empty -- nothing to publish" << endl;
        return;
    }
    else
    {
        sensor_msgs::PointCloud2 pclMsg;
        pcl::toROSMsg(mCloudFloor,pclMsg);
        pclMsg.header.frame_id = "odomFloorBag";
        pclMsg.header.stamp = ros::Time::now();
        mPubFloor.publish(pclMsg);
    }
}

}

