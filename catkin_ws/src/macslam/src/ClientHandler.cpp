#include <macslam/ClientHandler.h>

namespace macslam {

ClientHandler::ClientHandler(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, vocptr pVoc, dbptr pDB, mapptr pMap, size_t ClientId, uidptr pUID, eSystemState SysState, const string &strSettingsFile)
    : mpVoc(pVoc),mpKFDB(pDB),mpMap(pMap),
      mNh(Nh),mNhPrivate(NhPrivate),
      mClientId(ClientId), mpUID(pUID), mSysState(SysState),
      mstrSettingsFile(strSettingsFile)
{
    mVerboseMode = extVerboseMode;

    if(mpVoc == nullptr || mpKFDB == nullptr || mpMap == nullptr || (mpUID == nullptr && mSysState == eSystemState::SERVER))
    {
        cout << ("In \" ClientHandler::ClientHandler(...)\": nullptr exception") << endl;
        throw estd::infrastructure_ex();
    }

    mpMap->msuAssClients.insert(mClientId);

//    cv::Mat tic = cv::Mat::zeros(3,1,5);
//    cv::Mat Ric = cv::Mat::eye(3,3,5);
//    mg2oSmc = g2o::Sim3(mcpb::Converter::toMatrix3d(Ric),mcpb::Converter::toVector3d(tic),1.0);
    mg2oS_wcurmap_wclientmap = g2o::Sim3(); //identity transforamtion

    if(mSysState == eSystemState::CLIENT)
    {
        std::string TopicNameCamSub;

        mNhPrivate.param("bVisualization",mbVisualization,true);
        mNhPrivate.param("TopicNameCamSub",TopicNameCamSub,string("nospec"));
        mSubCam = mNh.subscribe<sensor_msgs::Image>(TopicNameCamSub,10,boost::bind(&ClientHandler::CamImgCb,this,_1));
    }
}

void ClientHandler::InitializeThreads()
{
    cout << "Client " << mClientId << " --> Initialize Threads" << endl;

    double CovMarkerSize,LoopMarkerSize,ScaleFactor,MarkerSphereDia,CamSize,CamLineSize,r,g,b;
    string OdomFrame;
    int LockSleep;
    bool  bUseImColor;

    mNhPrivate.param("MarkerSize",CovMarkerSize,0.1);
    mNhPrivate.param("MarkerSizeLoop",LoopMarkerSize,0.1);
    mNhPrivate.param("ScaleFactor",ScaleFactor,1.0);
    mNhPrivate.param("MarkerSphereDiameter",MarkerSphereDia,1.0);
    mNhPrivate.param("CamSize",CamSize,0.04);
    mNhPrivate.param("CamLineSize",CamLineSize,0.005);
    mNhPrivate.param("ColorR",r,1.0);
    mNhPrivate.param("ColorG",g,1.0);
    mNhPrivate.param("ColorB",b,1.0);
    mNhPrivate.param("LockSleep",LockSleep,1000);
    mNhPrivate.param("bUseImgColor",bUseImColor,false);

    std::stringstream* ss;
    ss = new stringstream;
    if(mSysState == eSystemState::CLIENT)
    {
        *ss << "FrameId";
        mNhPrivate.param("ColorR",r,1.0);
        mNhPrivate.param("ColorG",g,1.0);
        mNhPrivate.param("ColorB",b,1.0);
    }
    else if(mSysState == eSystemState::SERVER)
    {
        *ss << "ColorR" << mClientId;
        mNhPrivate.param(ss->str(),r,1.0);
        ss = new stringstream;
        *ss << "ColorG" << mClientId;
        mNhPrivate.param(ss->str(),g,1.0);
        ss = new stringstream;
        *ss << "ColorB" << mClientId;
        mNhPrivate.param(ss->str(),b,1.0);

        ss = new stringstream;
        *ss << "FrameId" << mClientId;
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::InitializeThreads(): invalid systems state: " << mSysState << endl;
        throw infrastructure_ex();
    }

    mNhPrivate.param(ss->str(),OdomFrame,std::string("nospec"));

    delete ss;

    if(OdomFrame=="nospec")
    {
        ROS_ERROR_STREAM("In \" ServerCommunicator::ServerCommunicator(...)\": bad parameters");
        throw estd::infrastructure_ex();
    }

    //+++++ Local Mapping Params +++++
    int MappingRate;
    mNhPrivate.param("MappingRate",MappingRate,3000);
    double RedThres;
    mNhPrivate.param("KfCullingRedundancyThres",RedThres,0.9);    

    ColorRGB c0,c1,c2,c3;
    mNhPrivate.param("ColorR0",c0.mr,1.0);
    mNhPrivate.param("ColorG0",c0.mg,1.0);
    mNhPrivate.param("ColorB0",c0.mb,1.0);

    mNhPrivate.param("ColorR1",c1.mr,1.0);
    mNhPrivate.param("ColorG1",c1.mg,1.0);
    mNhPrivate.param("ColorB1",c1.mb,1.0);

    mNhPrivate.param("ColorR2",c2.mr,1.0);
    mNhPrivate.param("ColorG2",c2.mg,1.0);
    mNhPrivate.param("ColorB2",c2.mb,1.0);

    mNhPrivate.param("ColorR3",c3.mr,1.0);
    mNhPrivate.param("ColorG3",c3.mg,1.0);
    mNhPrivate.param("ColorB3",c3.mb,1.0);

    ClientColors ccols(c0,c1,c2,c3);

    if(mSysState == eSystemState::CLIENT)
    {
        mpCC.reset(new CentralControl(mNh,mNhPrivate,mClientId,mSysState,shared_from_this(),nullptr,ScaleFactor,CovMarkerSize,LoopMarkerSize,MarkerSphereDia,OdomFrame,CamSize,CamLineSize,r,g,b,LockSleep,bUseImColor));
        mpCC->mCols = ccols;

        //+++++ Create Drawers. These are used by the Viewer +++++
        mpFrameViewer.reset(new FrameViewer(mpMap, mstrSettingsFile,mpCC));

        //+++++ Initialize the Local Mapping thread and launch +++++
        mpMapping.reset(new LocalMapping(mpCC,mpMap,mpKFDB,MappingRate,RedThres));

        //+++++ Initialize the communication thread and launch +++++
        mpComm.reset(new Communicator(mpCC,mpVoc,mpMap,mpKFDB));
        mpComm->SetMapping(mpMapping);

        //set pointers between units
        mpMapping->SetCommunicator(mpComm);
//        mpMap->SetCommunicator(mpComm);

        mpMap->mOdomFrame = mpCC->mNativeOdomFrame;
        mpMap->AddCCPtr(mpCC);

        //launch threads
        mptMapping.reset(new thread(&LocalMapping::RunClient,mpMapping));
        mptComm.reset(new thread(&Communicator::RunClient,mpComm));

        //+++++ Initialize the tracking thread +++++
        //(it will live in the main thread of execution, the one that called this constructor)
        mpTracking.reset(new Tracking(mpCC, mpVoc, mpFrameViewer, mpMap, mpKFDB,
                                  mstrSettingsFile,mClientId,mbVisualization));

        mpTracking->SetCommunicator(mpComm);
        mpTracking->SetLocalMapper(mpMapping);
    }
    else if(mSysState == eSystemState::SERVER)
    {
        mpCC.reset(new CentralControl(mNh,mNhPrivate,mClientId,mSysState,shared_from_this(),mpUID,ScaleFactor,CovMarkerSize,LoopMarkerSize,MarkerSphereDia,OdomFrame,CamSize,CamLineSize,r,g,b,LockSleep,bUseImColor));
        mpCC->mCols = ccols;

        #ifndef MAPFUSION
        //+++++ Initialize the Loop Finder thread and launch +++++
        LoopFinderParams LCParams;
        mNhPrivate.param("LoopClosureRate",LCParams.mLoopRate,3000);
        mNhPrivate.param("SolverIterations",LCParams.mSolverIterations,5);
        mNhPrivate.param("MatchesThres",LCParams.mMatchesThres,20);
        mNhPrivate.param("InliersThres",LCParams.mInliersThres,20);
        mNhPrivate.param("TotalMatchesThres",LCParams.mTotalMatchesThres,40);
        mNhPrivate.param("Probability",LCParams.mProbability,0.99);
        mNhPrivate.param("MinInliers",LCParams.mMinInliers,6);
        mNhPrivate.param("MaxIterations",LCParams.mMaxIterations,300);
        mNhPrivate.param("MinHitsForMerge",LCParams.mMinHitsForMerge,3);
        mNhPrivate.param("GBAIterations",LCParams.mGBAIterations,20);
        mNhPrivate.param("LoopLockSleep",LCParams.mLoopLockSleep,1000);
        mpLoopFinder.reset(new LoopFinder(mpCC,mpKFDB,mpVoc,mpMap,LCParams));
        mptLoopClosure.reset(new thread(&LoopFinder::Run,mpLoopFinder));
        #endif

        //+++++ Initialize the Local Mapping thread +++++
        mpMapping.reset(new LocalMapping(mpCC,mpMap,mpKFDB,MappingRate,RedThres));
        #ifndef MAPFUSION
        mpMapping->SetLoopFinder(mpLoopFinder);
        #endif

        //+++++ Initialize the communication thread +++++
        mpComm.reset(new Communicator(mpCC,mpVoc,mpMap,mpKFDB));
        mpComm->SetMapping(mpMapping);

        //set pointers between units
        mpMapping->SetCommunicator(mpComm);
//        mpMap->SetCommunicator(mpComm);

        mpMap->mOdomFrame = mpCC->mNativeOdomFrame;
        mpMap->AddCCPtr(mpCC);

        //launch threads
        mptMapping.reset(new thread(&LocalMapping::RunServer,mpMapping));
        mptComm.reset(new thread(&Communicator::RunServer,mpComm));
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::InitializeThreads(): invalid systems state: " << mpCC->mSysState << endl;
        throw infrastructure_ex();
    }


    if(mpCC->mpCH == nullptr)
    {
        ROS_ERROR_STREAM("ClientHandler::InitializeThreads()\": mpCC->mpCH is nullptr");
        throw estd::infrastructure_ex();
    }

    cout << "Client " << mClientId << " --> Thread initialization finished" << endl;
}

void ClientHandler::ChangeMap(mapptr pMap, g2o::Sim3 g2oS_wnewmap_wcurmap)
{
    mpMap = pMap;

    mg2oS_wcurmap_wclientmap = g2oS_wnewmap_wcurmap*mg2oS_wcurmap_wclientmap;
    mpCC->mg2oS_wcurmap_wclientmap = mg2oS_wcurmap_wclientmap;

    bool bLockedComm = mpCC->LockComm(); //should be locked and therefore return false
    bool bLockedMapping = mpCC->LockMapping(); //should be locked and therefore return false

    if(bLockedComm || bLockedMapping)
    {
        if(bLockedComm) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::ChangeMap(): Comm not locked: " << endl;
        if(bLockedMapping) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::ChangeMap(): Mapping not locked: " << endl;
        throw infrastructure_ex();
    }

    //make sure Comm and Mapping aren't active
//    unique_lock<mutex> lockComm(mpCC->mMutexComm,try_to_lock);
//    if(lockComm) cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mClientId << ": In \"ClientHandler::ChangeMap(...)\": communicator was not locked" << endl;
//    unique_lock<mutex> lockMapping(mpCC->mMutexMapping,try_to_lock);
//    if(lockMapping) cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mClientId << ": In \"ClientHandler::ChangeMap(...)\": mappings was not locked" << endl;


    mpComm->ChangeMap(mpMap);
    mpMapping->ChangeMap(mpMap);
    #ifndef MAPFUSION
    mpLoopFinder->ChangeMap(mpMap);
    #endif
}


void ClientHandler::SetMapMatcher(matchptr pMatch)
{
    mpMapMatcher = pMatch;
    mpComm->SetMapMatcher(mpMapMatcher);
    mpMapping->SetMapMatcher(mpMapMatcher);
}

void ClientHandler::CamImgCb(sensor_msgs::ImageConstPtr pMsg)
{
//    cout << "Img size: " << pMsg->width << "," << pMsg->height << endl;

    static size_t iFramecount = 0;
    ++iFramecount;
    static struct timeval tStart;
    static bool bInitTime = false;
    static struct timeval tLast;

    if(!bInitTime)
    {
        gettimeofday(&tStart,NULL);
        gettimeofday(&tLast,NULL);
        bInitTime = true;
    }

    const int nskip = 10;
    if((iFramecount > 0) && (iFramecount % nskip == 0))
    {
        static struct timeval tNow;
        gettimeofday(&tNow,NULL);
//        double dElapsed = (tNow.tv_sec - tStart.tv_sec) + (tNow.tv_usec - tStart.tv_usec) / 1000000.0;
//        double dRate = static_cast<double>(iFramecount) / dElapsed;
//        cout << ">>>>> Input Framerate (avg since start): " << dRate << " fps" << endl;

        double dElapsed = (tNow.tv_sec - tLast.tv_sec) + (tNow.tv_usec - tLast.tv_usec) / 1000000.0;
        double dRate = nskip/dElapsed;
        cout << ">>>>> Current Framerate: " << dRate << " fps (avg over last " << nskip << " frames" << endl;
        tLast = tNow;
    }



    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(pMsg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Check reset
//    if(iFramecount == 30) mbReset =true;
    {
    unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracking->Reset();
            mbReset = false;
        }
    }

//    static int i = 0;
//    ++i;
//    cout << "Num of images: " << i << endl;
//    cout << "Timestamp: " << cv_ptr->header.stamp.toSec() << endl;

    if(mVerboseMode == -5) gettimeofday(&mtStartTrack,NULL);

    mpTracking->GrabImageMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    if(mVerboseMode == -5)
    {
        gettimeofday(&mtNowTrack,NULL);
        double dElTrack = (mtNowTrack.tv_sec - mtStartTrack.tv_sec) + (mtNowTrack.tv_usec - mtStartTrack.tv_usec) / 1000000.0;
        mdElTotalTrack += dElTrack;
        cout << "Tracking time last call: " << dElTrack << " sec" << endl;
        cout << "Mean tracking time: " << mdElTotalTrack/static_cast<double>(iFramecount) << " sec" << endl;
    }

}

void ClientHandler::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
    ROS_WARN_STREAM("ClientSystem::Reset() called - Danger: reset function use \"delete\" operator in original implementation");
}



}
