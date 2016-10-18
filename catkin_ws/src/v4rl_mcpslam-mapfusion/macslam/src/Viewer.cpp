#include <macslam/Viewer.h>

namespace macslam {

FrameViewer::FrameViewer(mapptr pMap, const string &strSettingPath, ccptr pCC)
    : mpMap(pMap),mpCC(pCC)//,mNh(mpCC->mNh)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    T_ = 1e3/fps;

//    cv::namedWindow("Current Frame");

    std::stringstream* ss;

    ss = new stringstream;
    *ss << "ImageClient" << mpCC->mClientId;

    mPubIm = mNh.advertise<sensor_msgs::Image>(ss->str(),10);

    delete ss;
}

void FrameViewer::UpdateAndDraw(trackptr pTracker)
{
    this->Update(pTracker);
    cv::Mat im = this->DrawFrame();

    std_msgs::Header h;
    h.stamp = ros::Time::now();
    cv_bridge::CvImagePtr pImg{new cv_bridge::CvImage(h,sensor_msgs::image_encodings::BGR8,im)};
//    sensor_msgs::ImageConstPtr pMsg = pImg->toImageMsg();
    mPubIm.publish(pImg->toImageMsg());

    try
    {
        if(pTracker->mCurrentFrame)
        {
            if(pTracker->mCurrentFrame->mTcw.cols == 4 && pTracker->mCurrentFrame->mTcw.rows == 4)
            {
                std::stringstream* ss;
                ss = new stringstream;
                *ss << "Cam" << mpCC->mClientId;

                cv::Mat Twc = pTracker->mCurrentFrame->mTcw.inv();
                cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                cv::Mat twc = Twc.rowRange(0,3).col(3);
                Eigen::Matrix<double,3,3> eRwc = Converter::toMatrix3d(Rwc);
                Eigen::Matrix<double,3,1> etwc = Converter::toVector3d(twc);
            //    Eigen::Quaterniond eq(eRwc);
            //    tf::Pose
                etwc = mpCC->mScaleFactor * etwc;
                tf::Matrix3x3 tRwc;
                tf::Vector3 ttwc;
                tf::matrixEigenToTF(eRwc,tRwc);
                tf::vectorEigenToTF(etwc,ttwc);

                ttwc[3]=0;
                tRwc[3][0]=0;
                tRwc[3][1]=0;
                tRwc[3][2]=0;
                tf::Transform T(tRwc,ttwc);
//                cout << "Pub Tf parent|child: " << mpCC->mNativeOdomFrame << "|" << ss->str() << endl;
                tf::StampedTransform ST(T,ros::Time::now(),mpCC->mNativeOdomFrame,ss->str());
                mPubTf.sendTransform(ST);

                delete ss;
            }
        }
    }
    catch (exception& e)
    {
        cout << e.what() << '\n';
    }

//    g2o::Sim3 g2oSwc(Converter::toMatrix3d(Rwc),Converter::toVector3d(twc),1.0);




//    cv::imshow("Current Frame",im);
//    cv::waitKey(T_);
}

cv::Mat FrameViewer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        for(int i=0;i<N;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}

void FrameViewer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameViewer::Update(trackptr pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame->mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = false; //pTracker->mbOnlyTracking;


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame->mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            mpptr pMP = pTracker->mCurrentFrame->mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame->mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

}
