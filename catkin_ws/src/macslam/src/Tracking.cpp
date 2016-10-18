#include <macslam/Tracking.h>

namespace macslam {

Tracking::Tracking(ccptr pCC, vocptr pVoc, fviewptr pFrameViewer, mapptr pMap, dbptr pKFDB, const string &strSettingPath, size_t ClientId, bool bVisMode)
    : mState(NO_IMAGES_YET),mpCC(pCC),mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB), mpInitializer(nullptr),
      mpFrameViewer(pFrameViewer), mpMap(pMap), mLastRelocFrameId(make_pair(0,0)), mClientId(ClientId), mbVisMode(bVisMode)
{
    mVerboseMode = extVerboseMode;

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

//    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
//    mMinFrames = 0;
//    mMaxFrames = fps;
    mpCC->mNhPrivate.param("MinFrames",mMinFrames,0);
    mpCC->mNhPrivate.param("MaxFrames",mMaxFrames,static_cast<int>(fps));
    mpCC->mNhPrivate.param("nMatchesInliersThres",mnMatchesInliersThres,15);
    mpCC->mNhPrivate.param("thRefRatio",mthRefRatio,0.9f);

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mVerboseMode > 0)
    {
        cout << endl << "Client " << mClientId << " Camera Parameters: " << endl;
        cout << "- fx: " << fx << endl;
        cout << "- fy: " << fy << endl;
        cout << "- cx: " << cx << endl;
        cout << "- cy: " << cy << endl;
        cout << "- k1: " << DistCoef.at<float>(0) << endl;
        cout << "- k2: " << DistCoef.at<float>(1) << endl;
        if(DistCoef.rows==5)
            cout << "- k3: " << DistCoef.at<float>(4) << endl;
        cout << "- p1: " << DistCoef.at<float>(2) << endl;
        cout << "- p2: " << DistCoef.at<float>(3) << endl;
        cout << "- fps: " << fps << endl;

        if(mbRGB)
            cout << "- color order: RGB (ignored if grayscale)" << endl;
        else
            cout << "- color order: BGR (ignored if grayscale)" << endl;

        cout << endl << "Client " << mClientId << " KeyFrame Creation Parameters: " << endl;

        cout << "mMinFrames: " << mMinFrames << endl;
        cout << "mMaxFrames: " << mMaxFrames << endl;
        cout << "mnMatchesInliersThres: " << mnMatchesInliersThres << endl;
        cout << "mthRefRatio: " << mthRefRatio << endl;
    }

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractor.reset(new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST));
    mpIniORBextractor.reset(new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST));

    if(mVerboseMode > 2)
    {
        cout << endl << "Client " << mClientId << " ORB Extractor Parameters: " << endl;
        cout << "- Number of Features: " << nFeatures << endl;
        cout << "- Scale Levels: " << nLevels << endl;
        cout << "- Scale Factor: " << fScaleFactor << endl;
        cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
        cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
    }

//    if(sensor==System::STEREO || sensor==System::RGBD)
//    {
//        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
//        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
//    }

//    if(sensor==System::RGBD)
//    {
//        mDepthMapFactor = fSettings["DepthMapFactor"];
//        if(mDepthMapFactor==0)
//            mDepthMapFactor=1;
//        else
//            mDepthMapFactor = 1.0f/mDepthMapFactor;
//    }


}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mpCC->mbUseImgCol)
    {
        im.copyTo(mImRGB);

//        if(mImRGB.channels()==4)
//        {
//            if(mbRGB)
//                cv::cvtColor(mImRGB,mImRGB,CV_RGBA2RGB);
//            else
//                cv::cvtColor(mImRGB,mImRGB,CV_BGRA2RGB);
//        }
    }

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame.reset(new Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mClientId));
    else
    {
        mCurrentFrame.reset(new Frame(mImGray,timestamp,mpORBextractor,mpORBVocabulary,mK,mDistCoef,mClientId));
    }

    Track();

    return mCurrentFrame->mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
//    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
    if(mVerboseMode == -9) cout << "xxx Tracking --> Lock MapUpdate xxx" << endl;
    if(mVerboseMode == -9) cout << "LockSleep: " << mpCC->mLockSleep << endl;
    while(!mpMap->LockMapUpdate()){usleep(mpCC->mLockSleep);}
    if(mVerboseMode == -9) cout << "xxx Tracking --> MapUpdate Locked xxx" << endl;

    //Comm Mutex cannot be acquired here. In case of wrong initialization, there is mutual dependency in the call of reset()

    if(!mbVisMode) cout << "+++ Tracking State: " << mState << endl;

    if(mState==NOT_INITIALIZED)
    {
//        mLockComm = unique_lock<mutex>(mpComm->mMutexForTracking,adopt_lock);

        //cout << "not init" << endl;
        MonocularInitialization();

        if(mbVisMode) mpFrameViewer->UpdateAndDraw(shared_from_this());

        if(mState!=OK)
        {
            mpMap->UnLockMapUpdate();
            return;
        }
    }
    else
    {
        // Get Communicator Mutex -> Comm cannot publish. Assure no publishing whilst changing data
//        if(mpComm->mbStrictLock) unique_lock<mutex> lockComm(mpComm->mMutexForTracking);
        if(mVerboseMode == -9) cout << "xxx Tracking --> Lock Tracking xxx" << endl;
        if(mVerboseMode == -9) cout << "LockSleep: " << mpCC->mLockSleep << endl;
        if(mpComm->mbStrictLock) while(!mpCC->LockTracking()){usleep(mpCC->mLockSleep);}
        if(mVerboseMode == -9) cout << "xxx Tracking --> Tracking Locked xxx" << endl;

        // System is initialized. Track Frame.
        bool bOK;
        //cout << "mstate: " << mState << endl;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(mState==OK)
        {
            // Local Mapping might have changed some MapPoints tracked in last frame
            CheckReplacedInLastFrame();

            if(mVelocity.empty() || mCurrentFrame->mId.first<mLastRelocFrameId.first+2)
            {
//                cout << "Track Ref Frame" << endl;
                bOK = TrackReferenceKeyFrame();
                //cout << "bOk: " << bOK << endl;
            }
            else
            {
//                cout << "Track With M Model" << endl;
                bOK = TrackWithMotionModel();
                if(!bOK){
//                    cout << "Track Ref Frame" << endl;
                    bOK = TrackReferenceKeyFrame();
                }
                //cout << "bOk: " << bOK << endl;
            }
        }
        else
        {
            //cout << "Reloc" << endl;
            bOK = Relocalization();
            //cout << "bOk: " << bOK << endl;
        }

        mCurrentFrame->mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        //cout << "TrackMap (if bOK)" << endl;
        if(bOK) bOK = TrackLocalMap();
        //cout << "bOk: " << bOK << endl;

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        if(mbVisMode) mpFrameViewer->UpdateAndDraw(shared_from_this());

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame->mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame->GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame->GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame->mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            // Clean temporal point matches
            for(int i=0; i<mCurrentFrame->N; i++)
            {
                mpptr pMP = mCurrentFrame->mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame->mvbOutlier[i] = false;
                        mCurrentFrame->mvpMapPoints[i]=nullptr;
                    }
            }

//            // Delete temporal MapPoints
            //not needed in mono case
//            for(list<mpptr>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
//            {
//                mpptr pMP = *lit;
//                delete pMP;
//            }
//            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame->N;i++)
            {
                if(mCurrentFrame->mvpMapPoints[i] && mCurrentFrame->mvbOutlier[i])
                    mCurrentFrame->mvpMapPoints[i]=nullptr;
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
//                mLockComm.unlock();
                mpCC->mpCH->Reset();
                if(mpComm->mbStrictLock) mpCC->UnLockTracking();
                mpMap->UnLockMapUpdate();
                return;
            }
        }

        if(!mCurrentFrame->mpReferenceKF)
            mCurrentFrame->mpReferenceKF = mpReferenceKF;

        mLastFrame.reset(new Frame(*mCurrentFrame));

        if(mpComm->mbStrictLock) mpCC->UnLockTracking();
////        mLockComm.unlock();
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame->mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame->mTcw*mCurrentFrame->mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame->mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

    mpMap->UnLockMapUpdate();
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        //if(mVerboseMode>4) cout << "mCurrentFrame->mvKeysUn.size(): " << mCurrentFrame->mvKeysUn.size() << endl;

        // Set Reference Frame
        if(mCurrentFrame->mvKeys.size()>100)
        {
            mInitialFrame.reset(new Frame(*mCurrentFrame));
            mLastFrame.reset(new Frame(*mCurrentFrame));
            mvbPrevMatched.resize(mCurrentFrame->mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame->mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame->mvKeysUn[i].pt;

            if(mpInitializer) mpInitializer = nullptr;
//                delete mpInitializer;

            mpInitializer.reset(new Initializer(*mCurrentFrame,1.0,200));

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        //if(mVerboseMode>4) cout << "mCurrentFrame->mvKeysUn.size(): " << mCurrentFrame->mvKeysUn.size() << endl;

        // Try to initialize
        if((int)mCurrentFrame->mvKeys.size()<=100)
        {
//            delete mpInitializer;
            mpInitializer = nullptr;
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(*mInitialFrame,*mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        //if(mVerboseMode>4) cout << "Ini Matches: " << nmatches << endl;

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            //delete mpInitializer;
            mpInitializer = nullptr;
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(*mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
//            cout << "Rcw: " << Rcw << endl;
//            cout << "tcw: " << tcw << endl;

            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
//                    cout << "mvIniP3D[" << i << "]: " << mvIniP3D[i] << endl;
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame->SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame->SetPose(Tcw);

//            if(mVerboseMode > 4) cout << "Matches after triangulation:" << nmatches << endl;

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Get Communicator Mutex -> Comm cannot publish. Assure no publishing whilst changing data
//    unique_lock<mutex> lockComm(mpComm->mMutexForTracking);
    if(mVerboseMode == -9) cout << "xxx Tracking --> Lock Tracking xxx" << endl;
    if(mVerboseMode == -9) cout << "LockSleep: " << mpCC->mLockSleep << endl;
    while(!mpCC->LockTracking()){usleep(mpCC->mLockSleep);}
    if(mVerboseMode == -9) cout << "xxx Tracking --> Tracking Locked xxx" << endl;

//    mLockComm.lock();

    // Create KeyFrames
    kfptr pKFini{new KeyFrame(*mInitialFrame,mpMap,mpKeyFrameDB,mpComm,eSystemState::CLIENT,-1)};
    if(mpCC->mbUseImgCol) pKFini->mImg = mImRGB;
    cout << "\033[1;33m!!! WARN !!!\033[0m \"Tracking::CreateInitialMapMonocular()(...)\" wrong image in Ini-KF" << endl;
    kfptr pKFcur{new KeyFrame(*mCurrentFrame,mpMap,mpKeyFrameDB,mpComm,eSystemState::CLIENT,-1)};
    if(mpCC->mbUseImgCol) pKFcur->mImg = mImRGB;


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        mpptr pMP{new MapPoint(worldPos,pKFcur,mpMap,mClientId,mpComm,eSystemState::CLIENT,-1)};

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
//        cout << "Tracking::CreateInitialMapMonocular() -> pMP->UpdateNormalAndDepth()" << endl;
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame->mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame->mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemntClient(mpMap,mClientId,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

//    cout << "invMedianDepth" << invMedianDepth << endl;

    if(mVerboseMode > 4) cout << "TrackedMapPoints: " << pKFcur->TrackedMapPoints(1) << endl;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
//        cout << "medianDepth" << medianDepth << endl;
//        cout << "TrackedMapPoints" << pKFcur->TrackedMapPoints(1) << endl;

        cout << "Wrong initialization, reseting..." << endl;
        mpCC->UnLockTracking();
//        lockComm.unlock();
////        mLockComm.unlock();
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w,false);

    // Scale points
    vector<mpptr> vpAllMapPoints = pKFini->GetMapPointMatches();

//    cout << "vpAllMapPoints.size()" << vpAllMapPoints.size() << endl;

    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            //cout << "resize" << endl;
            mpptr pMP = vpAllMapPoints[iMP];
            //cout << "Pos before: " << pMP->GetWorldPos() << endl;
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth,false);
//            pMP->UpdateNormalAndDepth(); //Performance: Check here update procedure
//            cout << "MP ID: " << pMP->mnId << endl;
//            cout << "Pos after scaling: " << pMP->GetWorldPos() << endl;
        }
    }

//    mcpslam_msgs::MCPKeyFrame Msg;
//    pKFini->ConvertToMessage(Msg);
//    pKFcur->ConvertToMessage(Msg);

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame->SetPose(pKFcur->GetPose());
    mLastKeyFrameId=mCurrentFrame->mId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame->mpReferenceKF = pKFcur;

    mLastFrame.reset(new Frame(*mCurrentFrame));

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;

    mpCC->UnLockTracking();
//    mLockComm.unlock();
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame->N; i++)
    {
        mpptr pMP = mLastFrame->mvpMapPoints[i];

        if(pMP)
        {
            mpptr pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame->mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame->ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<mpptr> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,*mCurrentFrame,vpMapPointMatches);

//    cout << "Matches: " << nmatches << endl;

    if(nmatches<15)
        return false;

    mCurrentFrame->mvpMapPoints = vpMapPointMatches;
    mCurrentFrame->SetPose(mLastFrame->mTcw);

    Optimizer::PoseOptimizationClient(*mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame->N; i++)
    {
        if(mCurrentFrame->mvpMapPoints[i])
        {
            if(mCurrentFrame->mvbOutlier[i])
            {
                mpptr pMP = mCurrentFrame->mvpMapPoints[i];

                mCurrentFrame->mvpMapPoints[i]=nullptr;
                mCurrentFrame->mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mLastFrameSeen = mCurrentFrame->mId;
                nmatches--;
            }
            else if(mCurrentFrame->mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

//    cout << "Matches after Outlier discarding: " << nmatchesMap << endl;

    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    kfptr pRef = mLastFrame->mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame->SetPose(Tlr*pRef->GetPose());
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points
    UpdateLastFrame();

    mCurrentFrame->SetPose(mVelocity*mLastFrame->mTcw);

    fill(mCurrentFrame->mvpMapPoints.begin(),mCurrentFrame->mvpMapPoints.end(),nullptr);

    // Project points seen in previous frame
    int th;
    th=7;
//    cout << "SearchByProjection(mCurrentFrame,mLastFrame,th)" << endl;
    int nmatches = matcher.SearchByProjection(*mCurrentFrame,*mLastFrame,th);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame->mvpMapPoints.begin(),mCurrentFrame->mvpMapPoints.end(),nullptr);
//        cout << "SearchByProjection(mCurrentFrame,mLastFrame,2*th)" << endl;
        nmatches = matcher.SearchByProjection(*mCurrentFrame,*mLastFrame,2*th);
    }

    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimizationClient(*mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame->N; i++)
    {
        if(mCurrentFrame->mvpMapPoints[i])
        {
            if(mCurrentFrame->mvbOutlier[i])
            {
                mpptr pMP = mCurrentFrame->mvpMapPoints[i];

                mCurrentFrame->mvpMapPoints[i]=nullptr;
                mCurrentFrame->mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mLastFrameSeen = mCurrentFrame->mId;
                nmatches--;
            }
            else if(mCurrentFrame->mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimizationClient(*mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame->N; i++)
    {
        if(mCurrentFrame->mvpMapPoints[i])
        {
            if(!mCurrentFrame->mvbOutlier[i])
            {
                mCurrentFrame->mvpMapPoints[i]->IncreaseFound();
                mnMatchesInliers++;
            }

        }
    }

//    cout << "TrackLocalMap() -> Inliers: " << mnMatchesInliers << endl;

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame->mId.first<mLastRelocFrameId.first+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame->mId.first<mLastRelocFrameId.first+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Stereo & RGB-D: Ratio of close "matches to map"/"total matches"
    // "total matches = matches to map + visual odometry matches"
    // Visual odometry matches will become MapPoints if we insert a keyframe.
    // This ratio measures how many MapPoints we could create if we insert a keyframe.
    int nMap = 0;
    int nTotal= 0;

    // There are no visual odometry matches in the monocular case
    nMap=1;
    nTotal=1;

    const float ratioMap = (float)nMap/fmax(1.0f,nTotal); // not relevant in this case, is 1 -- leftover from stereo case

    // Thresholds
    float thRefRatio = 0.9f;

    float thMapRatio = 0.35f;
    if(mnMatchesInliers>300)
        thMapRatio = 0.20f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame->mId.first>=mLastKeyFrameId.first+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame->mId.first>=mLastKeyFrameId.first+mMinFrames && bLocalMappingIdle);
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
//    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio || ratioMap<thMapRatio) && mnMatchesInliers>15);
    const bool c2 = ((mnMatchesInliers<nRefMatches*mthRefRatio || ratioMap<thMapRatio) && mnMatchesInliers>mnMatchesInliersThres);

    if((c1a||c1b)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    //kfptr pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
    kfptr pKF{new KeyFrame(*mCurrentFrame,mpMap,mpKeyFrameDB,mpComm,eSystemState::CLIENT,-1)};
    if(mpCC->mbUseImgCol) pKF->mImg = mImRGB;

    std::vector<mpptr> vpM = pKF->GetMapPointMatches();
    for(vector<mpptr>::const_iterator vit = vpM.begin();vit!=vpM.end();++vit)
    {
        mpptr pMPi = *vit;

        if(!pMPi)
            continue;

        if(pMPi->isBad())
            continue;

        if(pMPi->mId.second != mClientId)
        {
            cout << "\033[1;34m!!! NOTICE !!!\033[0m using MP from client " << pMPi->mId.second  << endl;
//            pMPi->mbMultiUse = true;
            pMPi->SetMultiUse();
        }
    }

    mpReferenceKF = pKF;
    mCurrentFrame->mpReferenceKF = pKF;

    mpLocalMapper->InsertKeyFrame(pKF);  //patriksc: kf is created and leaves tracker -> Interface Tracker -> Mapper

    mpLocalMapper->SetNotStop(false);

    mLastKeyFrameId = mCurrentFrame->mId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<mpptr>::iterator vit=mCurrentFrame->mvpMapPoints.begin(), vend=mCurrentFrame->mvpMapPoints.end(); vit!=vend; vit++)
    {
        mpptr pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = nullptr;
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mLastFrameSeen = mCurrentFrame->mId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    int seen = 0;
    int bad = 0;
    int notinfrustrum = 0;

    // Project points in frame and check its visibility
    for(vector<mpptr>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        mpptr pMP = *vit;
        if(pMP->mLastFrameSeen == mCurrentFrame->mId)
        {
            ++seen;
            continue;
        }
        if(pMP->isBad())
        {
            ++bad;
            continue;
        }
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame->isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
        else ++notinfrustrum;
    }

//    cout << "SearchLocalPoints() -> seen: " << seen << endl;
//    cout << "SearchLocalPoints() -> bad: " << bad << endl;
//    cout << "SearchLocalPoints() -> notinfrustrum: " << notinfrustrum << endl;
//    cout << "SearchLocalPoints() -> nToMatch: " << nToMatch << endl;

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame->mId.first<mLastRelocFrameId.first+2)
            th=5;

//        cout << "SearchLocalPoints() -> th: " << th << endl;
//        cout << "SearchLocalPoints() -> mvpLocalMapPoints: " << mvpLocalMapPoints.size() << endl;

//        cout << "SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th)" << endl;
        matcher.SearchByProjection(*mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);



    // Update
    UpdateLocalKeyFrames();

//    cout << "UpdateLocalMap() -> Localkfs: " << mvpLocalKeyFrames.size() << endl;

    UpdateLocalPoints();

//    cout << "UpdateLocalMap() -> Localmappoints: " << mvpLocalMapPoints.size() << endl;
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<kfptr>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        kfptr pKF = *itKF;
        const vector<mpptr> vpMPs = pKF->GetMapPointMatches();

//        cout << "UpdateLocalPoints() -> pKF->GetMapPointMatches(): " << pKF->GetMapPointMatches().size() << endl;
        int empty = 0;

        for(vector<mpptr>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            mpptr pMP = *itMP;
            if(!pMP)
            {
                ++empty;
                continue;
            }
            if(pMP->mTrackReferenceForFrame==mCurrentFrame->mId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mTrackReferenceForFrame=mCurrentFrame->mId;
            }
        }

//        cout << "UpdateLocalPoints() -> empty points: " << empty << endl;
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<kfptr,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame->N; i++)
    {
        if(mCurrentFrame->mvpMapPoints[i])
        {
            mpptr pMP = mCurrentFrame->mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<kfptr,size_t> observations = pMP->GetObservations();
                for(map<kfptr,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame->mvpMapPoints[i]=nullptr;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    kfptr pKFmax= nullptr;

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<kfptr,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        kfptr pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mTrackReferenceForFrame = mCurrentFrame->mId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<kfptr>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        kfptr pKF = *itKF;

        const vector<kfptr> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<kfptr>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            kfptr pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mTrackReferenceForFrame!=mCurrentFrame->mId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mTrackReferenceForFrame=mCurrentFrame->mId;
                    break;
                }
            }
        }

        const set<kfptr> spChilds = pKF->GetChilds();
        for(set<kfptr>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            kfptr pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mTrackReferenceForFrame!=mCurrentFrame->mId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mTrackReferenceForFrame=mCurrentFrame->mId;
                    break;
                }
            }
        }

        kfptr pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mTrackReferenceForFrame!=mCurrentFrame->mId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mTrackReferenceForFrame=mCurrentFrame->mId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame->mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame->ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<kfptr> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(*mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<mpptr> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        kfptr pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,*mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(*mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame->mTcw);

                set<mpptr> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame->mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame->mvpMapPoints[j]=nullptr;
                }

                int nGood = Optimizer::PoseOptimizationClient(*mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame->N; io++)
                    if(mCurrentFrame->mvbOutlier[io])
                        mCurrentFrame->mvpMapPoints[io]=nullptr;

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
//                    cout << "SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100)" << endl;
                    int nadditional =matcher2.SearchByProjection(*mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimizationClient(*mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame->N; ip++)
                                if(mCurrentFrame->mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame->mvpMapPoints[ip]);
//                            cout << "SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64)" << endl;
                            nadditional =matcher2.SearchByProjection(*mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimizationClient(*mCurrentFrame);

                                for(int io =0; io<mCurrentFrame->N; io++)
                                    if(mCurrentFrame->mvbOutlier[io])
                                        mCurrentFrame->mvpMapPoints[io]=nullptr;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mLastRelocFrameId = mCurrentFrame->mId;
        return true;
    }

}

void Tracking::Reset()
{
//    mpViewer->RequestStop();

    cout << "System Reseting" << endl;
//    while(!mpViewer->isStopped())
//        usleep(3000);

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

//    // Reset Loop Closing
//    cout << "Reseting Loop Closing...";
//    mpLoopClosing->RequestReset();
//    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
//        delete mpInitializer;
        mpInitializer = nullptr;
    }

    cout << "Reseting Communicator...";
    mpComm->RequestReset();

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    cout << "Reseting Done...";

//    mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

//    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

}
