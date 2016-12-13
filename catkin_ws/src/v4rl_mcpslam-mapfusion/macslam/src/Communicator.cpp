#include <iostream>
#include <fstream>
#include <macslam/Communicator.h>

namespace macslam {

Communicator::Communicator(ccptr pCC, vocptr pVoc, mapptr pMap, dbptr pKFDB)
    : mpCC(pCC),
      mNh(pCC->mNh), mNhPrivate(pCC->mNhPrivate),
      mpVoc(pVoc), mpMap(pMap), mpDatabase(pKFDB),
      mClientId(pCC->mClientId), mbResetRequested(false), mbFirstKF(false), mFlagExported(false)
//      mbLoopClosed(false)
{
    mVerboseMode = extVerboseMode;

    mMsgCountLastMapMsg = 0;
//    mdKfInTime=0;mdMpInTime=0;mdVisTime=0;mdCoutTime=0;mdTotalTime=0;
//    mnKfInRuns=0;mnMpInRuns=0;mnVisRuns=0;mnCoutRuns=0;mnCommRuns=0;

    mInMpCount = 0;mInKfCount = 0;mInMapCount = 0;mOutMpCount = 0;mOutKfCount = 0;mOutMapCount = 0;mCoutCount = 0;mPubCount = 0;mOutKfBufferedCount=0;mOutMpBufferedCount=0;
//    mReceivedBytes = 0;
//    mdElCout = 0.0;mdElTotal = 0.0;mdCommTime = 0.0;mItTotal = 0.0;

    mNhPrivate.param("CoutRate",mCoutRate,100);
    mNhPrivate.param("bShowCovGraph",mbShowCovGraph,true);
    mNhPrivate.param("bShowMapPoints",mbShowMapPoints,true);
    mNhPrivate.param("bShowKFs",mbShowKFs,true);
    mNhPrivate.param("bShowTraj",mbShowTraj,true);

    mNhPrivate.param("bCommStrictLock",mbStrictLock,true);

    #ifdef HACKZ
    mNhPrivate.param("LoopCorrectionThreshold",mLoopCorrectionThreshold,999999);
    #endif

    //Topics
    std::stringstream* ss;
    string PubMapTopicName, PubResetTopicName; //,PubKfTopicName, PubMpTopicName;
    string MapInTopicName, ResetInTopicName; //,SubKfTopicName, SubMpTopicName;
    string SysType;

    if(mpCC->mSysState == eSystemState::CLIENT)
    {
        SysType = "Client";
    }
    else if(mpCC->mSysState == eSystemState::SERVER)
    {
        SysType = "Server";

				// Subscriber for the control of the map export
				mSubControl = mNh.subscribe<std_msgs::Bool>("publish_map", 1, boost::bind(&Communicator::PublishMapCb,this,_1));
		}
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ResetIfRequested(): invalid systems state: " << mpCC->mSysState << endl;
        throw infrastructure_ex();
    }

    ss = new stringstream;
    *ss << "MapOut" << SysType << mClientId;
    PubMapTopicName = ss->str();

    if(mpCC->mSysState == eSystemState::CLIENT)
    {
        mNhPrivate.param("MapInTopicName",MapInTopicName,std::string("nospec"));

        ss = new stringstream;
        *ss << "ResetOut" << SysType << mClientId;
        PubResetTopicName = ss->str();
    }
    else if(mpCC->mSysState == eSystemState::SERVER)
    {
        ss = new stringstream;
        *ss << "MapInTopicName" << mClientId;
        mNhPrivate.param(ss->str(),MapInTopicName,std::string("nospec"));

        ss = new stringstream;
        *ss << "ResetInTopicName" << mClientId;
        mNhPrivate.param(ss->str(),ResetInTopicName,std::string("nospec"));
    }

    delete ss;

    if(MapInTopicName=="nospec" || (ResetInTopicName=="nospec" && mpCC->mSysState == eSystemState::SERVER))
    {
        ROS_ERROR_STREAM("In \" Communicator::Communicator(...)\": bad IN topic name");
        cout << "Client " << mClientId << " \033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::Communicator(...): bad IN topic name" << endl;
        throw estd::infrastructure_ex();
    }

    //Load other params
    mNhPrivate.param("CommRate",mCommRate,10000);
    mNhPrivate.param("VisRate",mVisRate,100);
    mNhPrivate.param("PubMapBufferSize",mPubMapBufferSize,10);
    mNhPrivate.param("SubMapBufferSize",mSubMapBufferSize,10);
    mNhPrivate.param("KfItBound",mKfItBound,10);
    mNhPrivate.param("MpItBound",mMpItBound,10);

    //Publisher
    mPubMap = mNh.advertise<macslam_msgs::macMap>(PubMapTopicName,mPubMapBufferSize);
    if(mpCC->mSysState == eSystemState::CLIENT) mPubReset = mNh.advertise<std_msgs::Bool>(PubResetTopicName,1);

    //Subscriber
    mSubMap = mNh.subscribe<macslam_msgs::macMap>(MapInTopicName,mSubMapBufferSize,boost::bind(&Communicator::MapCb,this,_1));
    if(mpCC->mSysState == eSystemState::SERVER) mSubReset = mNh.subscribe<std_msgs::Bool>(ResetInTopicName,1,boost::bind(&Communicator::ResetCb,this,_1));

    //show params
    cout << SysType << " Communicator " << mClientId << " Params:" << endl;
    cout << "mCommRate: "  << mCommRate << endl;
    cout << "mCoutRate: "  << mCoutRate << endl;
    cout << "mVisRate: "  << mVisRate << endl;
    cout << "mKfItBound: "  << mKfItBound << endl;
    cout << "mMpItBound: "  << mMpItBound << endl;
    cout << "---INPUT---" << endl;
    cout << "MapInTopicName: "  << MapInTopicName << endl;
    cout << "SubMapBufferSize: "  << mSubMapBufferSize << endl;
    if(mpCC->mSysState == eSystemState::SERVER) cout << "ResetInTopicName" << ResetInTopicName << endl;
    cout << "---OUTPUT---" << endl;
    cout << "PubMapTopicName: "  << PubMapTopicName << endl;
    cout << "PubMapBufferSize: "  << mPubMapBufferSize << endl;
    #ifdef HACKZ
    cout << "mLoopCorrectionThreshold: " << mLoopCorrectionThreshold << endl;
    #endif
    if(mpCC->mSysState == eSystemState::CLIENT) cout << "PubResetTopicName" << PubResetTopicName << endl;

#ifdef STATS
    mbTotalCommTime = true;

    mdTimePerKfIn_mean=0.0;
    mdTimePerKfIn_ssd=0.0;
    mdTimePerKfIn_var=0.0;
    mnKFsProcessedIn=0;
    mdTimePerMpIn_mean=0.0;
    mdTimePerMpIn_ssd=0.0;
    mdTimePerMpIn_var=0.0;
    mnMPsProcessedIn=0;
    mdTimePerKfOut_mean=0.0;
    mdTimePerKfOut_ssd=0.0;
    mdTimePerKfOut_var=0.0;
    mnKFsProcessedOut=0;
    mdTimePerMpOut_mean=0.0;
    mdTimePerMpOut_ssd=0.0;
    mdTimePerMpOut_var=0.0;
    mnMPsProcessedOut=0;

    mdCommTime_total=0.0;
    mdCommTime_mean=0.0;
    mdCommTime_ssd=0.0;
    mdCommTime_var=0.0;
    mnCommRuns=0;

    mbFirstMessage=false;
    mVectIndex=0;
    mdTimeThresClient=1.0;
    mdTimeThresServer=0.1;
    mvtTimeStamp.clear();
    mvdLoadIn.clear();
    mvdLoadOut.clear();
    mdSizeOfMsgIn=0;
    mdSizeOfMsgOut=0;
#endif

    #ifdef HACKZ
    mMaxId = make_pair(0,0);
    #endif
}

void Communicator::RunClient()
{
    #ifndef PERFORMANCE
    if(mpCC->mSysState != eSystemState::CLIENT)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }
#endif

    #ifdef STATS
    struct timeval tStart,tNow;
    double dEl;
    #endif

    while(true)
    {        
        {
            if(mbStrictLock)
            {
//                unique_lock<mutex> lock1(mMutexForTracking,defer_lock);
//                unique_lock<mutex> lock2(mMutexForMapping,defer_lock);
//                //unique_lock<mutex> lock3(mpMap->mMutexMapUpdate,defer_lock);

//                lock(lock1,lock2);

//                while(!mpCC->LockComm()){usleep(mpCC->mLockSleep);}
//                if(mVerboseMode == -9) cout << "xxx Comm --> Lock Mapping xxx" << endl;
//                if(mVerboseMode == -9) cout << "LockSleep: " << mpCC->mLockSleep << endl;
                while(!mpCC->LockMapping()){usleep(mpCC->mLockSleep);}
//                if(mVerboseMode == -9) cout << "xxx Comm --> Mapping Locked xxx" << endl;
//                if(mVerboseMode == -9) cout << "xxx Comm --> Lock Tracking xxx" << endl;
                while(!mpCC->LockTracking()){usleep(mpCC->mLockSleep);}
//                if(mVerboseMode == -9) cout << "xxx Comm --> Tracking Locked xxx" << endl;
            }

            #ifdef STATS
            if(mbTotalCommTime)
            {
                gettimeofday(&tStart,NULL);
            }
            if(!mbFirstMessage)
            {
                gettimeofday(&mtStartTotal,NULL);
                gettimeofday(&mtLastStamp,NULL);
            }
            #endif

            if(this->CheckBufferKfOut() || this->CheckBufferMpOut())
            {
                this->PublishMapClient();
            }

            #ifndef MAPFUSION
            if(this->CheckBufferKfIn())
            {
                this->ProcessKfInClient();
            }

            if(this->CheckBufferMpIn())
            {
                this->ProcessMpInClient();
            }            
            #endif

            if(mbStrictLock)
            {
//                mpCC->UnLockComm();
                mpCC->UnLockMapping();
                mpCC->UnLockTracking();
            }
        }

        static int CoutCount = 0;

        //++++++++++ Calculate and show Stats ++++++++++

        #ifdef STATS
        gettimeofday(&tNow,NULL);

        if(mbTotalCommTime)
        {
            dEl = (tNow.tv_sec - tStart.tv_sec) + (tNow.tv_usec - tStart.tv_usec) / 1000000.0;
            mdCommTime_total+=dEl;

            estd::UpdateStats(mdCommTime_mean,mdCommTime_ssd,mnCommRuns,dEl);
        }

//        if(mbFirstMessage)
//        {
//            dEl = (tNow.tv_sec - mtLastStamp.tv_sec) + (tNow.tv_usec - mtLastStamp.tv_usec) / 1000000.0;
//            if(dEl >=mdTimeThresClient)
//            {
//                double dElTotal = (tNow.tv_sec - mtStartTotal.tv_sec) + (tNow.tv_usec - mtStartTotal.tv_usec)  / 1000000.0;

//                mvtTimeStamp.push_back(dElTotal);
//                mvdLoadIn.push_back(static_cast<double>(mdSizeOfMsgIn)/1024.0);
//                mvdLoadOut.push_back(static_cast<double>(mdSizeOfMsgOut)/1024.0);

////                    ++mVectIndex;
//                gettimeofday(&mtLastStamp,NULL);

//                mdSizeOfMsgIn = 0;
//                mdSizeOfMsgOut = 0;
//            }
//        }

        if(CoutCount % mCoutRate == 0)
        {
            unique_lock<mutex> LockComm(mpCC->mMutexCout);

            cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
            cout << "++++++++++++Client " << mClientId << " Stats ++++++++++++++++++++++++++++++++++++++" << endl;

            if(mbTotalCommTime)
            {
                cout << "----- Time spent for comm actions -----" << endl;
                cout << "Total time: " << mdCommTime_total << " sec" << endl;
                cout << "mean per iteration: " << mdCommTime_mean << " sec" << endl;
                mdCommTime_var = mdCommTime_ssd / static_cast<double>(mnCommRuns);
                cout << "var per iteration: " << mdCommTime_var << " sec" << endl;
            }
            else
            {
                cout << "----- KF IN -----" << endl;
                cout << "mean per KF: " << mdTimePerKfIn_mean*1000 << " msec" << endl;
                mdTimePerKfIn_var = mdTimePerKfIn_ssd / static_cast<double>(mnKFsProcessedIn);
                cout << "var: " << mdTimePerKfIn_var*1000000 << " msec^2" << endl;
                cout << "----- MP IN -----" << endl;
                cout << "mean per MP: " << mdTimePerMpIn_mean*1000 << " msec" << endl;
                mdTimePerMpIn_var = mdTimePerMpIn_ssd / static_cast<double>(mnMPsProcessedIn);
                cout << "var: " << mdTimePerMpIn_var*1000000 << " msec^2" << endl;
                cout << "----- KF OUT -----" << endl;
                cout << "mean per KF: " << mdTimePerKfOut_mean*1000 << " msec" << endl;
                mdTimePerKfOut_var = mdTimePerKfOut_ssd / static_cast<double>(mnKFsProcessedOut);
                cout << "var: " << mdTimePerKfOut_var*1000000 << " msec^2" << endl;
                cout << "----- MP Out -----" << endl;
                cout << "mean per MP: " << mdTimePerMpOut_mean*1000 << " msec" << endl;
                mdTimePerMpOut_var = mdTimePerMpOut_ssd / static_cast<double>(mnMPsProcessedOut);
                cout << "var: " << mdTimePerMpOut_var*1000000 << " msec^2" << endl;
            }

            cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;

//            estd::WriteTrafficToFile(mvtTimeStamp,mvdLoadIn,mvdLoadOut,boost::lexical_cast<string>(mClientId));
        }
        #endif

        //++++++++++ Visualization ++++++++++

        #ifdef VISUALIZATION
        static int PubCount = 0;
        ++PubCount;
        if((PubCount % mVisRate) == 0)
        {
            stringstream* sss;
            sss = new stringstream;
            *sss << "CovGraphMap" << mClientId << endl;
            if(mbShowCovGraph) mpMap->PubCovGraphAsMarkerMsg(mpCC->mCovGraphMarkerSize,mpCC->mScaleFactor,mpCC->mNativeOdomFrame,sss->str(),mpCC->mColorR,mpCC->mColorG,mpCC->mColorB);
            if(mbShowMapPoints) mpMap->PubMapPointsAsPclMsg(mpCC->mScaleFactor,mpCC->mNativeOdomFrame);
            sss = new stringstream;
//                *sss << "KeyFramesMap" << mClientId << endl;
            *sss << "comm" << mClientId;
            if(mbShowKFs) mpMap->PubKeyFrames(mpCC->mNativeOdomFrame,sss->str(),mpCC->mColorR,mpCC->mColorG,mpCC->mColorB);
            if(mbShowTraj) mpMap->PubTrajectoryClient(mpCC->mNativeOdomFrame);
            delete sss;
        }
        #endif

        //++++++++++ Debug Info ++++++++++

        #ifndef PERFORMANCE
        if((mVerboseMode == -3) && (CoutCount % mCoutRate == 0))
        {
            cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
            cout << "ClientHandler " << mClientId << " System Type: " << mpCC->mSysState << endl;
            cout << "ClientHandler " << mClientId << " KFs in Map: " << mpMap->GetAllKeyFrames().size() << endl;
            cout << "ClientHandler " << mClientId << " empty KFs: " << mpMap->GetMmpKfBuffer().size() << endl;
            cout << "ClientHandler " << mClientId << " MPs in Map: " << mpMap->GetAllMapPoints().size() << endl;
            cout << "ClientHandler " << mClientId << " erased MPs: " << mpMap->GetMmpErasedMapPoints().size() << endl;
            cout << "ClientHandler " << mClientId << " empty MPs: " << mpMap->GetMmpMpBuffer().size() << endl;
            cout << "---------- IN ----------" << endl;
            cout << "ClientHandler " << mClientId << " received Map msgs: " << mInMapCount << endl;
            cout << "ClientHandler " << mClientId << " received KF msgs: " << mInKfCount << endl;
            cout << "ClientHandler " << mClientId << " received MP msgs: " << mInMpCount << endl;
            cout << "ClientHandler " << mClientId << " different MPs received: " << msRecMPs.size() << endl;
            cout << "ClientHandler " << mClientId << " mlBufferKfIn: " << mlBufferKfIn.size()  << endl;
            cout << "ClientHandler " << mClientId << " mlBufferMpIn: " << mlBufferMpIn.size() << endl;
            cout << "---------- OUT ----------" << endl;
            cout << "ClientHandler " << mClientId << " sent Map msgs: " << mOutMapCount << endl;
            cout << "ClientHandler " << mClientId << " sent KF msgs: " << mOutKfCount << endl;
            cout << "ClientHandler " << mClientId << " sent MP msgs: " << mOutMpCount << endl;
            cout << "ClientHandler " << mClientId << " different MPs sent: " << msSentMPs.size() << endl;
            cout << "ClientHandler " << mClientId << " mlBufferKfOut: " << mlBufferKfOut.size() << endl;
            cout << "ClientHandler " << mClientId << " mlBufferMpOut: " << mlBufferMpOut.size() << endl;
            cout << "ClientHandler " << mClientId<< " total buffered Kfs: " << mOutKfBufferedCount << endl;
            cout << "ClientHandler " << mClientId<< " total buffered MPs: " << mOutMpBufferedCount << endl;
        }
        #endif

        ++CoutCount;

        ResetIfRequested();

        usleep(mCommRate);
    }
}

void Communicator::RunServer()
{
    #ifndef PERFORMANCE
    if(mpCC->mSysState != eSystemState::SERVER)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }
    #endif

    #ifdef STATS
    struct timeval tStart,tNow;
    double dEl;
    #endif

    while(true)
    {
        {
//            unique_lock<mutex> lockMapUpdate(mpMap->mMutexMapUpdate,defer_lock);
//            unique_lock<mutex> lockComm(mpCC->mMutexComm,defer_lock);
//            lock(lockMapUpdate,lockComm);

//            unique_lock<mutex> lockMapUpdate(mpMap->mMutexMapUpdate);
//            if(mVerboseMode == -9) cout << "xxx Comm --> Lock Comm xxx" << endl;
//            if(mVerboseMode == -9) cout << "LockSleep: " << mpCC->mLockSleep << endl;
            while(!mpCC->LockComm()){usleep(mpCC->mLockSleep);}
//            if(mVerboseMode == -9) cout << "xxx Comm --> Comm Locked xxx" << endl;
//            if(mVerboseMode == -9) cout << "xxx Comm --> Lock MapUpdate xxx" << endl;
            while(!mpMap->LockMapUpdate()){usleep(mpCC->mLockSleep);}
//            if(mVerboseMode == -9) cout << "xxx Comm --> MapUpdate Locked xxx" << endl;

            #ifdef STATS
            if(mbTotalCommTime)
            {
                gettimeofday(&tStart,NULL);
            }
            if(!mbFirstMessage)
            {
                gettimeofday(&mtStartTotal,NULL);
                gettimeofday(&mtLastStamp,NULL);
            }
            #endif

            #ifndef PERFORMANCE
            if(mpCC->mbOptActive) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ServerCommunicator::Run(...): Optimization active - Comm should be locked" << endl;
            #endif

//            mpCC->mbCorrectAfterLoop = false;

            #ifndef MAPFUSION
            if(this->CheckBufferKfOut() || this->CheckBufferMpOut())
            {
                this->PublishMapServer();
            }
            #endif

            if(this->CheckBufferKfIn())
            {
                this->ProcessKfInServer();
            }

            if(this->CheckBufferMpIn())
            {
                this->ProcessMpInServer();
            }

//            if(mbLoopClosed) mbLoopClosed = false;
//            if(mpCC->mbCorrectAfterLoop) mpCC->mbCorrectAfterLoop = false;



            mpCC->UnLockComm();
            mpMap->UnLockMapUpdate();
        }

        static int CoutCount = 0;

        //++++++++++ Calculate and show Stats ++++++++++

        #ifdef STATS
        gettimeofday(&tNow,NULL);

        if(mbTotalCommTime)
        {
            dEl = (tNow.tv_sec - tStart.tv_sec) + (tNow.tv_usec - tStart.tv_usec) / 1000000.0;
            mdCommTime_total+=dEl;

            estd::UpdateStats(mdCommTime_mean,mdCommTime_ssd,mnCommRuns,dEl);
        }

//        if(mbFirstMessage)
//        {
//            dEl = (tNow.tv_sec - mtLastStamp.tv_sec) + (tNow.tv_usec - mtLastStamp.tv_usec) / 1000000.0;
//            if(dEl >=mdTimeThresServer)
//            {
////                double dElTotal = (tNow.tv_sec - mtStartTotal.tv_sec) + (tNow.tv_usec - mtStartTotal.tv_usec)  / 1000000.0;

////                mvtTimeStamp.push_back(dElTotal);
////                mvdLoadIn.push_back(static_cast<double>(mdSizeOfMsgIn)/1024.0);
////                mvdLoadOut.push_back(static_cast<double>(mdSizeOfMsgOut)/1024.0);

//////                    ++mVectIndex;

//                mpDatabase->InsertDataMeasurements(mdSizeOfMsgIn,mdSizeOfMsgOut);

//                gettimeofday(&mtLastStamp,NULL);

//                mdSizeOfMsgIn = 0;
//                mdSizeOfMsgOut = 0;
//            }
//        }

        if(mCoutCount % mCoutRate == 0)
        {
            unique_lock<mutex> LockComm(mpCC->mMutexCout);

            cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
            cout << "++++++++++++Client " << mClientId << " Stats ++++++++++++++++++++++++++++++++++++++" << endl;

            if(mbTotalCommTime)
            {
                cout << "----- Time spent for comm actions -----" << endl;
                cout << "Total time: " << mdCommTime_total << " sec" << endl;
                cout << "mean per iteration: " << mdCommTime_mean << " sec" << endl;
                mdCommTime_var = mdCommTime_ssd / static_cast<double>(mnCommRuns);
                cout << "var per iteration: " << mdCommTime_var << " sec" << endl;
            }
            else
            {
                cout << "----- KF IN -----" << endl;
                cout << "mean per KF: " << mdTimePerKfIn_mean*1000 << " msec" << endl;
                mdTimePerKfIn_var = mdTimePerKfIn_ssd / static_cast<double>(mnKFsProcessedIn);
                cout << "var: " << mdTimePerKfIn_var*1000000 << " msec^2" << endl;
                cout << "----- MP IN -----" << endl;
                cout << "mean per KF: " << mdTimePerMpIn_mean*1000 << " msec" << endl;
                mdTimePerMpIn_var = mdTimePerMpIn_ssd / static_cast<double>(mnMPsProcessedIn);
                cout << "var: " << mdTimePerMpIn_var*1000000 << " msec^2" << endl;
                cout << "----- KF OUT -----" << endl;
                cout << "mean per KF: " << mdTimePerKfOut_mean*1000 << " msec" << endl;
                mdTimePerKfOut_var = mdTimePerKfOut_ssd / static_cast<double>(mnKFsProcessedOut);
                cout << "var: " << mdTimePerKfOut_var*1000000 << " msec^2" << endl;
                cout << "----- MP Out -----" << endl;
                cout << "mean per KF: " << mdTimePerMpOut_mean*1000 << " msec" << endl;
                mdTimePerMpOut_var = mdTimePerMpOut_ssd / static_cast<double>(mnMPsProcessedOut);
                cout << "var: " << mdTimePerMpOut_var*1000000 << " msec^2" << endl;
            }

            cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;

//            estd::WriteTrafficToFile(mvtTimeStamp,mvdLoadIn,mvdLoadOut,"S");
        }
        #endif

        //++++++++++ Visualization ++++++++++

        #ifdef VISUALIZATION
        if((mPubCount % mVisRate) == 0)
        {
            if(!mpCC->mbGotMerged)
            {
                stringstream* sss;
                sss = new stringstream;
                *sss << "CovGraphMap" << mClientId << endl;
                if(mbShowCovGraph) mpMap->PubCovGraphAsMarkerMsg(mpCC->mCovGraphMarkerSize,mpCC->mScaleFactor,mpCC->mNativeOdomFrame,sss->str(),mpCC->mColorR,mpCC->mColorG,mpCC->mColorB);
                if(mbShowMapPoints) mpMap->PubMapPointsAsPclMsg(mpCC->mScaleFactor,mpCC->mNativeOdomFrame);
                sss = new stringstream;
    //                *sss << "KeyFramesMap" << mClientId << endl;
                *sss << "comm" << mClientId << endl;
                if(mbShowKFs) mpMap->PubKeyFrames(mpCC->mNativeOdomFrame,sss->str(),mpCC->mColorR,mpCC->mColorG,mpCC->mColorB);
                if(mbShowTraj) mpMap->PubTrajectoryServer(mpCC->mNativeOdomFrame);
                delete sss;
            }
        }
        ++mPubCount;
        #endif

        //++++++++++ Debug Info ++++++++++

        #ifndef PERFORMANCE
        if((mVerboseMode == -3) && (mCoutCount % mCoutRate == 0))
        {
            cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
            cout << "ClientHandler " << mClientId << " System Type: " << mpCC->mSysState << endl;
            cout << "ClientHandler " << mClientId << " KFs in Map: " << mpMap->GetAllKeyFrames().size() << endl;
            cout << "ClientHandler " << mClientId << " empty KFs: " << mpMap->GetMmpKfBuffer().size() << endl;
            cout << "ClientHandler " << mClientId << " MPs in Map: " << mpMap->GetAllMapPoints().size() << endl;
            cout << "ClientHandler " << mClientId << " erased MPs: " << mpMap->GetMmpErasedMapPoints().size() << endl;
            cout << "ClientHandler " << mClientId << " empty MPs: " << mpMap->GetMmpMpBuffer().size() << endl;
            cout << "---------- IN ----------" << endl;
            cout << "ClientHandler " << mClientId << " received Map msgs: " << mInMapCount << endl;
            cout << "ClientHandler " << mClientId << " received KF msgs: " << mInKfCount << endl;
            cout << "ClientHandler " << mClientId << " received MP msgs: " << mInMpCount << endl;
            cout << "ClientHandler " << mClientId << " different MPs received: " << msRecMPs.size() << endl;
            cout << "ClientHandler " << mClientId << " mlBufferKfIn: " << mlBufferKfIn.size()  << endl;
            cout << "ClientHandler " << mClientId << " mlBufferMpIn: " << mlBufferMpIn.size() << endl;
            cout << "---------- OUT ----------" << endl;
            cout << "ClientHandler " << mClientId << " sent Map msgs: " << mOutMapCount << endl;
            cout << "ClientHandler " << mClientId << " sent KF msgs: " << mOutKfCount << endl;
            cout << "ClientHandler " << mClientId << " sent MP msgs: " << mOutMpCount << endl;
            cout << "ClientHandler " << mClientId << " different MPs sent: " << msSentMPs.size() << endl;
            cout << "ClientHandler " << mClientId << " mlBufferKfOut: " << mlBufferKfOut.size() << endl;
            cout << "ClientHandler " << mClientId << " mlBufferMpOut: " << mlBufferMpOut.size() << endl;
            cout << "ClientHandler " << mClientId<< " total buffered Kfs: " << mOutKfBufferedCount << endl;
            cout << "ClientHandler " << mClientId<< " total buffered MPs: " << mOutMpBufferedCount << endl;
        }
        #endif

        ++mCoutCount;

        ResetIfRequested();

        usleep(mCommRate);
    }
}

void Communicator::MapCb(macslam_msgs::macMapConstPtr pMsg)
{
    #ifndef PERFORMANCE
    if(pMsg->MsgCount != (mMsgCountLastMapMsg+1))
    {
        cout << "Client " << mClientId << ": In \"Communicator::MapCb()\": Map message lost" << endl;
        throw estd::infrastructure_ex();
    }
    #endif

    #ifdef STATS
    if(!mbFirstMessage)
        if(pMsg->Keyframes.size() > 0 || pMsg->MapPoints.size() > 0)
            mbFirstMessage = true;
    #endif

    if(pMsg->Keyframes.size() > 0)
    {
        unique_lock<mutex> lock(mMutexBufferKfIn);

        //Keyframes
        for(int idx=0;idx<pMsg->Keyframes.size();++idx)
        {
    //        unique_lock<mutex> lock(mMutexBufferKfIn);

            mlBufferKfIn.push_back(pMsg->Keyframes[idx]);

            #ifndef PERFORMANCE
            ++mInKfCount;
            #endif
        }
    }

    if(pMsg->MapPoints.size() > 0)
    {
        unique_lock<mutex> lock(mMutexBufferMpIn);

        //MapPoints
        for(int idx=0;idx<pMsg->MapPoints.size();++idx)
        {
    //        unique_lock<mutex> lock(mMutexBufferMpIn);

            mlBufferMpIn.push_back(pMsg->MapPoints[idx]);

            #ifndef PERFORMANCE
            ++mInMpCount;
            #endif
        }
    }

    mMsgCountLastMapMsg = pMsg->MsgCount;

    #ifndef PERFORMANCE
    ++mInMapCount;
    #endif
//    mReceivedBytes = mReceivedBytes + sizeof(pMsg);
//    #ifdef STATS
//    mdSizeOfMsgIn = mdSizeOfMsgIn + sizeof(pMsg);
//    #endif
}

void Communicator::ResetCb(std_msgs::BoolConstPtr pMsg)
{
    if(mVerboseMode > 0) cout << "ClientHandler " << mClientId << ": Communicator::ResetCb" << endl;
    if(mbResetRequested)
    {
        if(mVerboseMode > 0) cout << "ClientHandler " << mClientId << ": finished Communicator::ResetCb" << endl;
        return;
    }

    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    if(mVerboseMode > 0) cout << "ClientHandler " << mClientId << ": finished Communicator::ResetCb" << endl;
}

void Communicator::PublishMapCb(std_msgs::BoolConstPtr pMsg) {
	std::unique_lock<std::mutex> lock(Communicator::mMutexExport);
	if((true == pMsg->data) && (false == Communicator::mFlagExported)) {
		// Export map
		std::vector<boost::shared_ptr<KeyFrame>> kfs = mpMap->GetAllKeyFrames();
		std::ofstream output;
		output.open("export_" + to_string(mClientId) + ".txt");

		if(true == output.is_open()) {
			output << "Begin export of map:" << std::endl;
			output << "ID;mapId;timestamp;x;y;z" << std::endl;
			for(boost::shared_ptr<KeyFrame> i : kfs) {
				if((false == i->isBad()) && (false == i->IsEmpty())) {
					std::cout << "test: " << i->mId.first << ";" << i->mId.second << ";" << std::setprecision(19) << i->mTimeStamp << ";" << i->GetPoseInverse().rowRange(0,3).col(3).at<float>(0) << ";" << i->GetPoseInverse().rowRange(0,3).col(3).at<float>(1) << ";" << i->GetPoseInverse().rowRange(0,3).col(3).at<float>(2) << std::endl;

					output << i->mId.first << ";" << i->mId.second << ";" << std::setprecision(19) << i->mTimeStamp << ";" << i->GetPoseInverse().rowRange(0,3).col(3).at<float>(0) << ";" << i->GetPoseInverse().rowRange(0,3).col(3).at<float>(1) << ";" << i->GetPoseInverse().rowRange(0,3).col(3).at<float>(2) << std::endl;
				}
			}
			output.close();
			Communicator::mFlagExported = true;
			std::cout << "Finished export!" << std::endl;
		} else {
			std::cout << "Unalbel to open file!" << std::endl;
		}
	}
}

void Communicator::RequestReset()
{
    cout << "Communicator::RequestReset()" << endl;
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
            {
//                cout << "break ClientCommunicator::RequestReset()" << endl;
                break;
            }
        }
        usleep(3000);
    }
}

void Communicator::ProcessKfInServer()
{
    #ifndef PERFORMANCE
    if(mpCC->mSysState != eSystemState::SERVER)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }
    #endif

    #ifdef STATS
    struct timeval tStart, tNow;
    double dEl;
    #endif

    int ItCount = 0;

    while (this->CheckBufferKfIn() && (ItCount < mKfItBound))
    {
        #ifdef STATS
        if(!mbTotalCommTime)
            gettimeofday(&tStart,NULL);
        #endif

        macslam_msgs::macKeyFrame* pMsg = new macslam_msgs::macKeyFrame();
        {
            unique_lock<mutex> lock(mMutexBufferKfIn);
            *pMsg = mlBufferKfIn.front();
            mlBufferKfIn.pop_front();
        }

        #ifdef STATS
//        mdSizeOfMsgIn = mdSizeOfMsgIn + sizeof(*pMsg);
        #endif

        #ifndef PERFORMANCE
        {
            //consistency check - delete later
            kfptr pCheck0 = mpMap->GetKfPtr(pMsg->mnId,pMsg->mClientId);
            kfptr pCheck1 = mpMap->SearchAndReturnFromKfBuffer(pMsg->mnId,pMsg->mClientId);
            if(pCheck0 && pCheck1)
            {
                cout << "Client " << mClientId << ": In \"ServerCommunicator::ProcessKfInV2()\": KF in map and in KF buffer" << endl;
                throw estd::infrastructure_ex();
            }
        }

        if(pMsg->mClientId != mClientId) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ServerCommunicator::ProcessKfInServer(): Msg KF Client ID != comm client id" << endl;
        #endif

        if(pMsg->bSentOnce)
        {
            //+++++ Update KF+++++
            kfptr pKF = mpMap->GetKfPtr(pMsg->mnId,pMsg->mClientId);

            if(!pKF)
            {
                if(mpMap->IsKfDeleted(pMsg->mnId,pMsg->mClientId))
                    continue;
                else
                {
                    //messages must not get lost (full buffers)
                    cout << "Client " << mClientId << ": In \"ServerCommunicator::ProcessKfIn()\": KF already sent but does not exist in map" << endl;
                    throw estd::infrastructure_ex();
                }
            }

            if(pKF->IsEmpty())
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ServerCommunicator::ProcessKfInV2(): KF already arrived & empty" << endl;
                continue;
            }

            if(pKF->isBad()) continue; //no need to process bad KFs. Anyway, this should not happen - when KF is set to BAD, it is deleted from map

            if(pMsg->mbBad)
            {
                pKF->mbOmitSending = true;
                pKF->SetBadFlag();
                pKF->mbOmitSending = false;
                continue;
            }

            //KF exists, is not bad, empty or deleted and will not be set to BAD by this msg
            pKF->UpdateFromMessageServer(pMsg,mpCC->mg2oS_wcurmap_wclientmap,g2o::Sim3());
//            if(!mpCC->mbCorrectAfterLoop) pKF->UpdateFromMessageServer(pMsg,mpCC->mg2oS_wcurmap_wclientmap,g2o::Sim3());
//            else pKF->UpdateFromMessageServer(pMsg,mpCC->mg2oS_wcurmap_wclientmap,mpCC->mg2oS_loop);
            #ifdef HACKZ
            if(mpCC->mbCorrectAfterLoop && pMsg->mbPoseChanged)
                this->CorrectAfterLC(pKF,mpCC->mg2oS_loop);
            #endif

            pKF->UpdateConnections();

//            mpMapping->InsertKfForLBA(pKF);
        }
        else
        {

            //prepare structures expected by constructor
            std::vector<cv::KeyPoint> vKeys;
            for(int idx=0;idx<pMsg->mvKeys.size();++idx) vKeys.push_back(Converter::fromCvKeyPointMsg(pMsg->mvKeys[idx]));

            std::vector<cv::KeyPoint> vKeysUn;
            for(int idx=0;idx<pMsg->mvKeysUn.size();++idx) vKeysUn.push_back(Converter::fromCvKeyPointMsg(pMsg->mvKeysUn[idx]));

            macslam_msgs::machDescriptor TempDesc = pMsg->mDescriptors[0];
            int iBoundY = static_cast<int>(TempDesc.mDescriptor.size());
            int iBoundX = static_cast<int>(pMsg->mDescriptors.size());
            cv::Mat Descriptors(iBoundX,iBoundY,0);

            for(int idx=0;idx<pMsg->mDescriptors.size();++idx)
            {
                macslam_msgs::machDescriptor DescMsg = pMsg->mDescriptors[idx];
                for(int idy=0;idy<iBoundY;++idy)
                {
                    Descriptors.at<uint8_t>(idx,idy)=DescMsg.mDescriptor[idy];
                }
            }

            std::vector<float> vScaleFactors;
            std::vector<float> vLevelSigma2;
            std::vector<float> vInvLevelSigma2;

            for(int idx=0;idx<pMsg->mvScaleFactors.size();++idx) vScaleFactors.push_back(pMsg->mvScaleFactors[idx]);

            for(int idx=0;idx<pMsg->mvLevelSigma2.size();++idx) vLevelSigma2.push_back(pMsg->mvLevelSigma2[idx]);

            for(int idx=0;idx<pMsg->mvInvLevelSigma2.size();++idx) vInvLevelSigma2.push_back(pMsg->mvInvLevelSigma2[idx]);

            cv::Mat K(3,3,5);
            Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macKeyFrame::_mK_type,float>(K,pMsg->mK);

            //create KeyFrame

            kfptr pKF = mpMap->SearchAndReturnFromKfBuffer(pMsg->mnId,pMsg->mClientId);

            if(pKF)
            {

                if(pMsg->mUniqueId != defid)
                {
                    cout << "Client " << mClientId << " \033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessKfIn(): [create KF from buffer] KF->mUniqueId != defid" << endl;
                    cout << "KF " << pMsg->mnId << "|" << pMsg->mClientId << " KF->mUniqueId: " <<  pMsg->mUniqueId << endl,
                    throw estd::infrastructure_ex();
                }

                *pKF = KeyFrame(pMsg,mpCC->mg2oS_wcurmap_wclientmap,g2o::Sim3(),mpVoc,mpMap,mpDatabase,shared_from_this(),vKeys,vKeysUn,Descriptors,K,vScaleFactors,vLevelSigma2,vInvLevelSigma2,mpCC->mSysState,mpCC->mpUID->GetId());
//                if(!mpCC->mbCorrectAfterLoop) *pKF = KeyFrame(pMsg,mpCC->mg2oS_wcurmap_wclientmap,g2o::Sim3(),mpVoc,mpMap,mpDatabase,shared_from_this(),vKeys,vKeysUn,Descriptors,K,vScaleFactors,vLevelSigma2,vInvLevelSigma2,mpCC->mSysState,mpCC->mpUID->GetId());
//                else *pKF = KeyFrame(pMsg,mpCC->mg2oS_wcurmap_wclientmap,mpCC->mg2oS_loop,mpVoc,mpMap,mpDatabase,shared_from_this(),vKeys,vKeysUn,Descriptors,K,vScaleFactors,vLevelSigma2,vInvLevelSigma2,mpCC->mSysState,mpCC->mpUID->GetId());

                #ifdef HACKZ
                if(mpCC->mbCorrectAfterLoop)
                    this->CorrectAfterLC(pKF,mpCC->mg2oS_loop);
                #endif

//                if(pKF->mnId > mMaxKfId) mMaxKfId = pKF->mnId;

                pKF->UpdateConnections();

                pKF->mdServerTimestamp = ros::Time::now().toNSec();

//                pKF->ShowMyValues();

                mpMap->AddKeyFrame(pKF);
                mpMapping->InsertKeyFrame(pKF);
//                mpMapMatcher->InsertKF(pKF);

                mpMap->DeleteFromKfBuffer(pKF);

                #ifndef PERFORMANCE
                if(pMsg->mbBad) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ServerCommunicator::ProcessKfInV2(): KF arrived first and & mbBad" << endl;
                #endif

                #ifdef HACKZ
                if(!mpCC->mbCorrectAfterLoop && pKF->mId.first > mMaxId.first)
                {
                    mloopCorrectionPoseTcw = pKF->GetPose();
                    mMaxId = pKF->mId;
                }
                #endif
            }
            else
            {
                if(mVerboseMode==5) cout << "create KF" << endl;

                if(pMsg->mUniqueId != defid)
                {
                    cout << "Client " << mClientId << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessKfIn(): [create new KF] KF->mUniqueId != defid" << endl;
                    cout << "KF " << pMsg->mnId << "|" << pMsg->mClientId << " KF->mUniqueId: " <<  pMsg->mUniqueId << endl,
                    throw estd::infrastructure_ex();
                }

                pKF.reset(new KeyFrame(pMsg,mpCC->mg2oS_wcurmap_wclientmap,g2o::Sim3(),mpVoc,mpMap,mpDatabase,shared_from_this(),vKeys,vKeysUn,Descriptors,K,vScaleFactors,vLevelSigma2,vInvLevelSigma2,mpCC->mSysState,mpCC->mpUID->GetId()));
//                if(!mpCC->mbCorrectAfterLoop) pKF.reset(new KeyFrame(pMsg,mpCC->mg2oS_wcurmap_wclientmap,g2o::Sim3(),mpVoc,mpMap,mpDatabase,shared_from_this(),vKeys,vKeysUn,Descriptors,K,vScaleFactors,vLevelSigma2,vInvLevelSigma2,mpCC->mSysState,mpCC->mpUID->GetId()));
//                else pKF.reset(new KeyFrame(pMsg,mpCC->mg2oS_wcurmap_wclientmap,mpCC->mg2oS_loop,mpVoc,mpMap,mpDatabase,shared_from_this(),vKeys,vKeysUn,Descriptors,K,vScaleFactors,vLevelSigma2,vInvLevelSigma2,mpCC->mSysState,mpCC->mpUID->GetId()));
                #ifdef HACKZ
                if(mpCC->mbCorrectAfterLoop)
                    this->CorrectAfterLC(pKF,mpCC->mg2oS_loop);
                #endif

//                if(pKF->mnId > mMaxKfId) mMaxKfId = pKF->mnId;

                pKF->UpdateConnections();

                pKF->mdServerTimestamp = ros::Time::now().toNSec();

//                pKF->ShowMyValues();

                mpMap->AddKeyFrame(pKF);
                mpMapping->InsertKeyFrame(pKF);
//                mpMapMatcher->InsertKF(pKF);

                if(!mbFirstKF)
                {
                    mpMap->mvpKeyFrameOrigins.push_back(pKF);
                    mbFirstKF = true;
                }

                #ifndef PERFORMANCE
                if(pMsg->mbBad) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ServerCommunicator::ProcessKfInV2(): KF arrived first and & mbBad" << endl;
                #endif

                #ifdef HACKZ
                if(!mpCC->mbCorrectAfterLoop && pKF->mId.first > mMaxId.first)
                {
                    mloopCorrectionPoseTcw = pKF->GetPose();
                    mMaxId = pKF->mId;
                }
                #endif
            }
        }

        delete pMsg;
//        if(!mpCC->mbCorrectAfterLoop) ++ItCount;
        ++ItCount;

        #ifdef STATS
        if(!mbTotalCommTime)
        {
            gettimeofday(&tNow,NULL);
            dEl = (tNow.tv_sec - tStart.tv_sec) + (tNow.tv_usec - tStart.tv_usec) / 1000000.0;
            estd::UpdateStats(mdTimePerKfIn_mean,mdTimePerKfIn_ssd,mnKFsProcessedIn,dEl);
        }
        #endif
    }

//    if(mVerboseMode==5) cout << "End ProcessKfIn" << endl;
}

#ifndef MAPFUSION
void Communicator::ProcessKfInClient()
{
    #ifndef PERFORMANCE
    if(mpCC->mSysState != eSystemState::CLIENT)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }
    #endif

    #ifdef STATS
    struct timeval tStart, tNow;
    double dEl;
    #endif

    int ItCount = 0;

    while (this->CheckBufferKfIn() && (ItCount < mKfItBound))
    {
        #ifdef STATS
        if(!mbTotalCommTime)
            gettimeofday(&tStart,NULL);
        #endif

        macslam_msgs::macKeyFrame* pMsg = new macslam_msgs::macKeyFrame();
        {
            unique_lock<mutex> lock(mMutexBufferKfIn);
            *pMsg = mlBufferKfIn.front();
            mlBufferKfIn.pop_front();
        }

        #ifdef STATS
//        mdSizeOfMsgIn = mdSizeOfMsgIn + sizeof(*pMsg);
        #endif

        if(pMsg->mClientId == mClientId)
        {
            if(!pMsg->mbPoseOnly) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessKfInClientV2(): msg client id == comm client id && !mbPoseOnly" << endl;

            kfptr pKF = mpMap->GetKfPtr(pMsg->mnId,pMsg->mClientId);

            if(!pKF)
            {
                if(!mpMap->IsKfDeleted(pMsg->mnId,pMsg->mClientId))
                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessKfInClientV2(): msg client id == comm client id && !pKF from map" << endl;
                continue;
            }

            if(pKF->IsEmpty())
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessKfInClientV2(): KF in map & empty" << endl;
                continue;
            }

            pKF->UpdateFromMessageClient(pMsg);

            if(pMsg->mbBad) pKF->SetBadFlag();

            pKF->SetChangedByServer();
        }
        else
        {
            if(mpMap->IsKfDeleted(pMsg->mnId,pMsg->mClientId))
            {
                continue;
            }

            if(pMsg->mbPoseOnly)
            {
                kfptr pKF = mpMap->GetKfPtr(pMsg->mnId,pMsg->mClientId);

                if(!pKF)
                {
                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessKfInClientV2(): !pKF && !deleted && pMsg->mbPoseOnly" << endl;
                    throw infrastructure_ex();
                }

                #ifndef PERFORMANCE
                {
                    //consistency check - delete later
                    kfptr pCheck0 = mpMap->GetKfPtr(pMsg->mnId,pMsg->mClientId);
                    kfptr pCheck1 = mpMap->SearchAndReturnFromKfBuffer(pMsg->mnId,pMsg->mClientId);
                    if(pCheck0 && pCheck1)
                    {
                        cout << "Client " << mClientId << ": In \"Communicator::ProcessKfInClientV2()\": KF in map and in KF buffer" << endl;
                        throw estd::infrastructure_ex();
                    }
                }
                #endif

                if(pKF->IsEmpty())
                {
                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessKfInClientV2(): KF in map & empty" << endl;
                    continue;
                }

                pKF->UpdateFromMessageClient(pMsg);

                if(pMsg->mbBad) pKF->SetBadFlag();
            }
            else
            {
                #ifndef PERFORMANCE
                {
                    kfptr pKF = mpMap->GetKfPtr(pMsg->mnId,pMsg->mClientId);
                    if(pKF)
                    {
                        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessKfInClientV2(): pKF && !pMsg->mbPoseOnly" << endl;
                        throw infrastructure_ex();
                    }
                }
                #endif

                //prepare structures expected by constructor
                std::vector<cv::KeyPoint> vKeys;
                for(int idx=0;idx<pMsg->mvKeys.size();++idx) vKeys.push_back(Converter::fromCvKeyPointMsg(pMsg->mvKeys[idx]));

                std::vector<cv::KeyPoint> vKeysUn;
                for(int idx=0;idx<pMsg->mvKeysUn.size();++idx) vKeysUn.push_back(Converter::fromCvKeyPointMsg(pMsg->mvKeysUn[idx]));

                macslam_msgs::machDescriptor TempDesc = pMsg->mDescriptors[0];
                int iBoundY = static_cast<int>(TempDesc.mDescriptor.size());
                int iBoundX = static_cast<int>(pMsg->mDescriptors.size());
                cv::Mat Descriptors(iBoundX,iBoundY,0);

                for(int idx=0;idx<pMsg->mDescriptors.size();++idx)
                {
                    macslam_msgs::machDescriptor DescMsg = pMsg->mDescriptors[idx];
                    for(int idy=0;idy<iBoundY;++idy)
                    {
                        Descriptors.at<uint8_t>(idx,idy)=DescMsg.mDescriptor[idy];
                    }
                }

                std::vector<float> vScaleFactors;
                std::vector<float> vLevelSigma2;
                std::vector<float> vInvLevelSigma2;

                for(int idx=0;idx<pMsg->mvScaleFactors.size();++idx) vScaleFactors.push_back(pMsg->mvScaleFactors[idx]);

                for(int idx=0;idx<pMsg->mvLevelSigma2.size();++idx) vLevelSigma2.push_back(pMsg->mvLevelSigma2[idx]);

                for(int idx=0;idx<pMsg->mvInvLevelSigma2.size();++idx) vInvLevelSigma2.push_back(pMsg->mvInvLevelSigma2[idx]);

                cv::Mat K(3,3,5);
                Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macKeyFrame::_mK_type,float>(K,pMsg->mK);

                //create KeyFrame

                kfptr pKF = mpMap->SearchAndReturnFromKfBuffer(pMsg->mnId,pMsg->mClientId);

                if(pKF)
                {
                    if(mVerboseMode==5) cout << "from Buffer" << endl;

                    if(pMsg->mUniqueId == defid)
                    {
                        cout << "Client " << mClientId << " \033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessKfIn(): [create KF from buffer] KF->mUniqueId == defid" << endl;
                        cout << "KF " << pMsg->mnId << "|" << pMsg->mClientId << " KF->mUniqueId: " <<  pMsg->mUniqueId << endl,
                        throw estd::infrastructure_ex();
                    }

//                    if(mpCC->mg2oS_wcurmap_wclientmap != g2o::Sim3())
//                        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessKfInClientV2(): mpCC->mg2oS_wcurmap_wclientmap != g2o::Sim3()" << endl;

//                    cout << "Communicator::ProcessKfInClientV2() --> create foreign KF from empty" << endl;

                    *pKF = KeyFrame(pMsg,mpCC->mg2oS_wcurmap_wclientmap,g2o::Sim3(),mpVoc,mpMap,mpDatabase,shared_from_this(),vKeys,vKeysUn,Descriptors,K,vScaleFactors,vLevelSigma2,vInvLevelSigma2,mpCC->mSysState,pMsg->mUniqueId);
//                    *pKF = KeyFrame(pMsg,mpCC->mg2oS_wcurmap_wclientmap,mpCC->mg2oS_loop,mpVoc,mpMap,mpDatabase,shared_from_this(),vKeys,vKeysUn,Descriptors,K,vScaleFactors,vLevelSigma2,vInvLevelSigma2,mpCC->mSysState,pMsg->mUniqueId);

    //                if(pKF->mnId > mMaxKfId) mMaxKfId = pKF->mnId;

                    pKF->UpdateConnections();

                    pKF->mdServerTimestamp = ros::Time::now().toNSec();

    //                pKF->ShowMyValues();

                    mpMap->AddKeyFrame(pKF);

                    mpMap->DeleteFromKfBuffer(pKF);

                    if(pMsg->mbBad)
                    {
                        pKF->SetBadFlag();
                        cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mClientId << ": In \"Communicator::ProcessKfIn()\": KF arrived first, was empty, is bad" << endl;
                    }
                }
                else
                {
                    #ifndef PERFORMANCE
                    if(pMsg->mUniqueId == defid)
                    {
                        cout << "Client " << mClientId << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessKfIn(): [create new KF] KF->mUniqueId == defid" << endl;
                        cout << "KF " << pMsg->mnId << "|" << pMsg->mClientId << " KF->mUniqueId: " <<  pMsg->mUniqueId << endl,
                        throw estd::infrastructure_ex();
                    }
                    #endif

//                    if(mpCC->mg2oS_wcurmap_wclientmap != g2o::Sim3())
//                        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessKfInClientV2(): mpCC->mg2oS_wcurmap_wclientmap != g2o::Sim3()" << endl;


                    pKF.reset(new KeyFrame(pMsg,mpCC->mg2oS_wcurmap_wclientmap,g2o::Sim3(),mpVoc,mpMap,mpDatabase,shared_from_this(),vKeys,vKeysUn,Descriptors,K,vScaleFactors,vLevelSigma2,vInvLevelSigma2,mpCC->mSysState,pMsg->mUniqueId));
//                    pKF.reset(new KeyFrame(pMsg,mpCC->mg2oS_wcurmap_wclientmap,mpCC->mg2oS_loop,mpVoc,mpMap,mpDatabase,shared_from_this(),vKeys,vKeysUn,Descriptors,K,vScaleFactors,vLevelSigma2,vInvLevelSigma2,mpCC->mSysState,pMsg->mUniqueId));

    //                if(pKF->mnId > mMaxKfId) mMaxKfId = pKF->mnId;

                    pKF->UpdateConnections();

                    pKF->mdServerTimestamp = ros::Time::now().toNSec();

    //                pKF->ShowMyValues();

                    mpMap->AddKeyFrame(pKF);

                    if(pMsg->mbBad)
                    {
                        pKF->SetBadFlag();
                        cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mClientId << ": In \"Communicator::ProcessKfIn()\": KF arrived first, is bad" << endl;
                    }
                }
            }
        }

        delete pMsg;
        ++ItCount;

        #ifdef STATS
        if(!mbTotalCommTime)
        {
            gettimeofday(&tNow,NULL);
            dEl = (tNow.tv_sec - tStart.tv_sec) + (tNow.tv_usec - tStart.tv_usec) / 1000000.0;
            estd::UpdateStats(mdTimePerKfIn_mean,mdTimePerKfIn_ssd,mnKFsProcessedIn,dEl);
        }
        #endif
    }
}
#endif

void Communicator::ProcessMpInServer()
{
    #ifndef PERFORMANCE
    if(mpCC->mSysState != eSystemState::SERVER)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }
    #endif

    #ifdef STATS
    struct timeval tStart, tNow;
    double dEl;
    #endif

    int ItCount = 0;

    while (this->CheckBufferMpIn() && (ItCount < mMpItBound))
    {
        #ifdef STATS
        if(!mbTotalCommTime)
            gettimeofday(&tStart,NULL);
        #endif

        macslam_msgs::macMapPoint* pMsg = new macslam_msgs::macMapPoint();
        {
            unique_lock<mutex> lock(mMutexBufferMpIn);
            *pMsg = mlBufferMpIn.front();
            mlBufferMpIn.pop_front();
        }

        #ifdef STATS
//        mdSizeOfMsgIn = mdSizeOfMsgIn + sizeof(*pMsg);
        #endif

        #ifndef PERFORMANCE
        {
            //consistency check - delete later
            mpptr pCheck0 = mpMap->GetMpPtr(pMsg->mnId,pMsg->mClientId);
            mpptr pCheck1 = mpMap->SearchAndReturnFromMpBuffer(pMsg->mnId,pMsg->mClientId);
            if(pCheck0 && pCheck1)
            {
                cout << "Client " << mClientId << ": In \"ServerCommunicator::ProcessMpInV2()\": MP in map and in MP buffer" << endl;
                throw estd::infrastructure_ex();
            }
        }

        //mbMultiUse writing
        if(pMsg->mClientId != mClientId)
        {
            if(pMsg->mbMultiUse)
            {
                mpptr pMP = mpMap->GetMpPtr(pMsg->mnId,pMsg->mClientId);
                if(pMP)
                {
                    if(!pMP->mbMultiUse)
                    {
                        pMP->mbMultiUse = true;
//                        cout << "+++ Received MultiUse Message +++" << endl;
                    }
                }
                continue;
            }
            else
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ServerCommunicator::ProcessMpInServer(): Msg MP Client ID != comm client id" << endl;
            }
        }


//        if(pMsg->mClientId != mClientId) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ServerCommunicator::ProcessMpInServer(): Msg MP Client ID != comm client id" << endl;
        #endif

//        static size_t createdMPs = 0;
//        static size_t createdEmptyMPs = 0;
//        bool bIsNew = !(msRecMPs.count(pMsg->mnId));
        msRecMPs.insert(pMsg->mnId);

        if(pMsg->bSentOnce)
        {
//            if(bIsNew) cout << "\033[1;31m!!!!! Error !!!!!\033[0m Communicator::ProcessMpIn(): MP new && mbSentOnce TRUE" << endl;

            mpptr pMP = mpMap->GetMpPtr(pMsg->mnId,pMsg->mClientId);

            if(!pMP)
            {
                if(mpMap->IsMpDeleted(pMsg->mnId,pMsg->mClientId))
                    continue;
                else
                {
                    //messages must not get lost (full buffers)
                    cout << "Client " << mClientId << ": In \"ServerCommunicator::ProcessMpIn()\": MP already sent but does not exist in map" << endl;
                    throw estd::infrastructure_ex();
                }
            }

            if(pMP->IsEmpty())
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ServerCommunicator::ProcessMpIn(): MP already arrived & empty" << endl;
                continue;
            }

            if(pMP->isBad()) continue; //no need to process bad MPs. Anyway, this should not happen - when MP is set to BAD, it is deleted from map

            if(pMsg->mbBad)
            {
                pMP->mbOmitSending = true;
                pMP->SetBadFlag();
                pMP->mbOmitSending = false;
                continue;
            }

            //MP exists, is not bad, empty or deleted and will not be set to BAD by this msg
            pMP->UpdateFromMessageServer(pMsg,mpCC->mg2oS_wcurmap_wclientmap,g2o::Sim3());
//            if(!mpCC->mbCorrectAfterLoop) pMP->UpdateFromMessageServer(pMsg,mpCC->mg2oS_wcurmap_wclientmap,g2o::Sim3());
//            else pMP->UpdateFromMessageServer(pMsg,mpCC->mg2oS_wcurmap_wclientmap,mpCC->mg2oS_loop);

            #ifdef HACKZ
            if(mpCC->mbCorrectAfterLoop && pMsg->mbPoseChanged)
                this->CorrectAfterLC(pMP,mpCC->mg2oS_loop);
            #endif
        }
        else
        {
            if(mVerboseMode==-6) cout << "New MP" << endl;

            mpptr pMP = mpMap->SearchAndReturnFromMpBuffer(pMsg->mnId,pMsg->mClientId);

            if(pMP)
            {
                #ifndef PERFORMANCE
                if(pMsg->mUniqueId != defid)
                {
                    cout << "Client " << mClientId << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessMpIn(): [create MP from Buffer] MP->mUniqueId != defid" << endl;
                    cout << "MP " << pMsg->mnId << "|" << pMsg->mClientId << " MP->mUniqueId: " <<  pMsg->mUniqueId << endl,
                    throw estd::infrastructure_ex();
                }
                #endif

                (*pMP) = MapPoint(pMsg,mpCC->mg2oS_wcurmap_wclientmap,g2o::Sim3(),mpMap,shared_from_this(),mpCC->mSysState,mpCC->mpUID->GetId());
//                if(!mpCC->mbCorrectAfterLoop) (*pMP) = MapPoint(pMsg,mpCC->mg2oS_wcurmap_wclientmap,g2o::Sim3(),mpMap,shared_from_this(),mpCC->mSysState,mpCC->mpUID->GetId());
//                else (*pMP) = MapPoint(pMsg,mpCC->mg2oS_wcurmap_wclientmap,mpCC->mg2oS_loop,mpMap,shared_from_this(),mpCC->mSysState,mpCC->mpUID->GetId());

                #ifdef HACKZ
                if(mpCC->mbCorrectAfterLoop)
                    if(!(pMsg->mbBad || pMP->Observations() <=1))
                        this->CorrectAfterLC(pMP,mpCC->mg2oS_loop);
                #endif

//                pMP->ShowMyValues();

                mpMap->AddMapPoint(pMP);

                mpMap->DeleteFromMpBuffer(pMP);

                if(pMsg->mbBad || pMP->Observations() <=1)
                {
                    if(pMsg->mbBad) pMP->mbOmitSending = true;
                    pMP->SetBadFlag();
                    if(pMsg->mbBad) pMP->mbOmitSending = false;
                }
            }
            else
            {
                #ifndef PERFORMANCE
                if(pMsg->mUniqueId != defid)
                {
                    cout << "Client " << mClientId << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessMpIn(): [create new MP] MP->mUniqueId != defid" << endl;
                    cout << "MP " << pMsg->mnId << "|" << pMsg->mClientId << " MP->mUniqueId: " <<  pMsg->mUniqueId << endl,
                    throw estd::infrastructure_ex();
                }
                #endif

                pMP.reset(new MapPoint(pMsg,mpCC->mg2oS_wcurmap_wclientmap,g2o::Sim3(),mpMap,shared_from_this(),mpCC->mSysState,mpCC->mpUID->GetId()));
//                if(!mpCC->mbCorrectAfterLoop) pMP.reset(new MapPoint(pMsg,mpCC->mg2oS_wcurmap_wclientmap,g2o::Sim3(),mpMap,shared_from_this(),mpCC->mSysState,mpCC->mpUID->GetId()));
//                else pMP.reset(new MapPoint(pMsg,mpCC->mg2oS_wcurmap_wclientmap,mpCC->mg2oS_loop,mpMap,shared_from_this(),mpCC->mSysState,mpCC->mpUID->GetId()));

                #ifdef HACKZ
                if(mpCC->mbCorrectAfterLoop)
                    if(!(pMsg->mbBad || pMP->Observations() <=1))
                        this->CorrectAfterLC(pMP,mpCC->mg2oS_loop);
                #endif

//                pMP->ShowMyValues();

                mpMap->AddMapPoint(pMP);

                if(pMsg->mbBad || pMP->Observations() <=1)
                {
                    if(pMsg->mbBad) pMP->mbOmitSending = true;
                    pMP->SetBadFlag();
                    if(pMsg->mbBad) pMP->mbOmitSending = false;
                }
                //insert & delete to have MP in list of deleted MPs. Performance: after set to BAD, MP should not be used anymore --> this could be omitted
            }
        }

        delete pMsg;
//        if(mpCC->mbCorrectAfterLoop) ++ItCount;
        ++ItCount;

        #ifdef STATS
        if(!mbTotalCommTime)
        {
            gettimeofday(&tNow,NULL);
            dEl = (tNow.tv_sec - tStart.tv_sec) + (tNow.tv_usec - tStart.tv_usec) / 1000000.0;
            estd::UpdateStats(mdTimePerMpIn_mean,mdTimePerMpIn_ssd,mnMPsProcessedIn,dEl);
        }
        #endif
    }
}

#ifndef MAPFUSION
void Communicator::ProcessMpInClient()
{
    #ifndef PERFORMANCE
    if(mpCC->mSysState != eSystemState::CLIENT)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }
    #endif

    #ifdef STATS
    struct timeval tStart, tNow;
    double dEl;
    #endif

    int ItCount = 0;

    while (this->CheckBufferMpIn() && (ItCount < mMpItBound))
    {
        #ifdef STATS
        if(!mbTotalCommTime)
            gettimeofday(&tStart,NULL);
        #endif

        macslam_msgs::macMapPoint* pMsg = new macslam_msgs::macMapPoint();
        {
            unique_lock<mutex> lock(mMutexBufferMpIn);
            *pMsg = mlBufferMpIn.front();
            mlBufferMpIn.pop_front();
        }

        #ifdef STATS
//        mdSizeOfMsgIn = mdSizeOfMsgIn + sizeof(*pMsg);
        #endif

        if(pMsg->mClientId == mClientId)
        {            
            #ifndef PERFORMANCE
            if(!pMsg->mbPoseOnly) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessMpInClientV2(): msg client id == comm client id && !mbPoseOnly" << endl;
            #endif

            mpptr pMP = mpMap->GetMpPtr(pMsg->mnId,pMsg->mClientId);

            if(!pMP)
            {
                if(!mpMap->IsMpDeleted(pMsg->mnId,pMsg->mClientId))
                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessMpInClientV2(): msg client id == comm client id && pMP not deleted && !pMP from map" << endl;
                continue;
            }

            if(pMP->IsEmpty())
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessMpInClientV2(): MP in map & empty" << endl;
                continue;
            }

            pMP->UpdateFromMessageClient(pMsg);

            if(pMsg->mbBad) pMP->SetBadFlag();

            pMP->SetChangedByServer();
        }
        else
        {
            if(mpMap->IsMpDeleted(pMsg->mnId,pMsg->mClientId))
            {
                continue;
            }

            if(pMsg->mbPoseOnly)
            {
                mpptr pMP = mpMap->GetMpPtr(pMsg->mnId,pMsg->mClientId);

                if(!pMP)
                {
                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessMpInClientV2(): !pMP && !deleted && pMsg->mbPoseOnly" << endl;
                    throw infrastructure_ex();
                }

                #ifndef PERFORMANCE
                {
                    //consistency check - delete later
                    mpptr pCheck0 = mpMap->GetMpPtr(pMsg->mnId,pMsg->mClientId);
                    mpptr pCheck1 = mpMap->SearchAndReturnFromMpBuffer(pMsg->mnId,pMsg->mClientId);
                    if(pCheck0 && pCheck1)
                    {
                        cout << "Client " << mClientId << ": In \"ServerCommunicator::ProcessMpInClientV2()\": MP in map and in MP buffer" << endl;
                        throw estd::infrastructure_ex();
                    }
                }
                #endif

                if(pMP->IsEmpty())
                {
                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessMpInClientV2(): MP in map & empty" << endl;
                    continue;
                }

                pMP->UpdateFromMessageClient(pMsg);

                if(pMsg->mbBad) pMP->SetBadFlag();
            }
            else
            {
                #ifndef PERFORMANCE
                {
                    mpptr pMP = mpMap->GetMpPtr(pMsg->mnId,pMsg->mClientId);
                    if(pMP)
                    {
                        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessMpInClientV2(): pMP && !pMsg->mbPoseOnly" << endl;
                        throw infrastructure_ex();
                    }
                }
                #endif

                mpptr pMP = mpMap->SearchAndReturnFromMpBuffer(pMsg->mnId,pMsg->mClientId);

                if(pMP)
                {
                    #ifndef PERFORMANCE
                    if(pMsg->mUniqueId == defid)
                    {
                        cout << "Client " << mClientId << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessMpIn(): [create MP from Buffer] MP->mUniqueId 0= defid" << endl;
                        cout << "MP " << pMsg->mnId << "|" << pMsg->mClientId << " MP->mUniqueId: " <<  pMsg->mUniqueId << endl,
                        throw estd::infrastructure_ex();
                    }
                    #endif

//                    if(mpCC->mg2oS_wcurmap_wclientmap != g2o::Sim3())
//                        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessMpInClientV2(): mpCC->mg2oS_wcurmap_wclientmap != g2o::Sim3()" << endl;

                    (*pMP) = MapPoint(pMsg,mpCC->mg2oS_wcurmap_wclientmap,g2o::Sim3(),mpMap,shared_from_this(),mpCC->mSysState,pMsg->mUniqueId);
//                    (*pMP) = MapPoint(pMsg,mpCC->mg2oS_wcurmap_wclientmap,mpCC->mg2oS_loop,mpMap,shared_from_this(),mpCC->mSysState,pMsg->mUniqueId);

    //                pMP->ShowMyValues();

                    mpMap->AddMapPoint(pMP);

                    mpMap->DeleteFromMpBuffer(pMP);

                    if(pMsg->mbBad || pMP->Observations() <=1)
                    {
                        if(pMsg->mbBad) pMP->mbOmitSending = true;
                        pMP->SetBadFlag();
                        if(pMsg->mbBad) pMP->mbOmitSending = false;
                    }
                }
                else
                {
                    #ifndef PERFORMANCE
                    if(pMsg->mUniqueId == defid)
                    {
                        cout << "Client " << mClientId << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessMpIn(): [create new MP] MP->mUniqueId == defid" << endl;
                        cout << "MP " << pMsg->mnId << "|" << pMsg->mClientId << " MP->mUniqueId: " <<  pMsg->mUniqueId << endl,
                        throw estd::infrastructure_ex();
                    }
                    #endif

//                    if(mpCC->mg2oS_wcurmap_wclientmap != g2o::Sim3())
//                        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ProcessMpInClientV2(): mpCC->mg2oS_wcurmap_wclientmap != g2o::Sim3()" << endl;

                    pMP.reset(new MapPoint(pMsg,mpCC->mg2oS_wcurmap_wclientmap,g2o::Sim3(),mpMap,shared_from_this(),mpCC->mSysState,pMsg->mUniqueId));
//                    pMP.reset(new MapPoint(pMsg,mpCC->mg2oS_wcurmap_wclientmap,mpCC->mg2oS_loop,mpMap,shared_from_this(),mpCC->mSysState,pMsg->mUniqueId));

    //                pMP->ShowMyValues();

                    mpMap->AddMapPoint(pMP);

                    if(pMsg->mbBad || pMP->Observations() <=1)
                    {
                        if(pMsg->mbBad) pMP->mbOmitSending = true;
                        pMP->SetBadFlag();
                        if(pMsg->mbBad) pMP->mbOmitSending = false;
                    }
                    //insert & delete to have MP in list of deleted MPs. Performance: after set to BAD, MP should not be used anymore --> this could be omitted
                }
            }
        }

        delete pMsg;
        ++ItCount;

        #ifdef STATS
        if(!mbTotalCommTime)
        {
            gettimeofday(&tNow,NULL);
            dEl = (tNow.tv_sec - tStart.tv_sec) + (tNow.tv_usec - tStart.tv_usec) / 1000000.0;
            estd::UpdateStats(mdTimePerMpIn_mean,mdTimePerMpIn_ssd,mnMPsProcessedIn,dEl);
        }
        #endif
    }
}
#endif

void Communicator::ResetCommunicator()
{
    unique_lock<mutex> lock1(mMutexBufferKfIn);
    unique_lock<mutex> lock2(mMutexBufferMpIn);
    unique_lock<mutex> lock3(mMutexBufferKfOut);
    unique_lock<mutex> lock4(mMutexBufferMpOut);

    usleep(10000); // wait to give msg buffers time to empty

    mlBufferKfIn.clear();
    mlBufferMpIn.clear();
    mlBufferKfOut.clear();
    mlBufferMpOut.clear();

    mbFirstKF = false;
}

void Communicator::ResetDatabase()
{
    vector<kfptr> vpKFs = mpMap->GetAllKeyFrames();

    for(vector<kfptr>::iterator vit=vpKFs.begin();vit!=vpKFs.end();++vit)
    {
        kfptr pKF = *vit;
        mpDatabase->erase(pKF);
    }
}

void Communicator::ResetIfRequested()
 {
     unique_lock<mutex> lock(mMutexReset);

     if(mbResetRequested)
     {
         cout << "ClientCommunicator::ResetIfRequested()" << endl;

         if(mpCC->mSysState == eSystemState::CLIENT)
         {
             this->ResetCommunicator();

             std_msgs::Bool Msg;
             Msg.data = true;
             mPubReset.publish(Msg);

             cout << "ClientCommunicator::ResetIfRequested() -> wait" << endl;
             usleep(3000);
             cout << "ClientCommunicator::ResetIfRequested() -> leave" << endl;
         }
         else if(mpCC->mSysState == eSystemState::SERVER)
         {
             this->ResetCommunicator();
             this->ResetMapping();
             this->ResetMapMatcher();
             this->ResetDatabase();
             this->ResetMap();
         }
         else
         {
             cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ResetIfRequested(): invalid systems state: " << mpCC->mSysState << endl;
             throw infrastructure_ex();
         }

         mbResetRequested = false;
         cout << "ClientHandler " << mClientId << ": finished ServerCommunicator::ResetIfRequested" << endl;
     }
 }

void Communicator::ResetMapping()
 {
     cout << "ClientHandler " << mpCC->mClientId << ": Communicator::ResetIfRequested --> Request Mapping module reset" << endl;
     mpMapping->RequestReset();
     cout << "ClientHandler " << mpCC->mClientId << ": Communicator::ResetIfRequested --> finished Mapping module reset" << endl;
 }

void Communicator::ResetMapMatcher()
 {
     vector<kfptr> vpKFs = mpMap->GetAllKeyFrames();
     mpMapMatcher->EraseKFs(vpKFs);
 }

void Communicator::ResetMap()
{
    mpMap->clear();
}

void Communicator::PassKftoComm(kfptr pKf)
{
    unique_lock<mutex> lock(mMutexBufferKfOut);

    ++mOutKfBufferedCount;

    mlBufferKfOut.push_back(pKf);
    pKf->MarkInOutBuffer();

}

void Communicator::PassMptoComm(mpptr pMp)
{
    unique_lock<mutex> lock(mMutexBufferMpOut);

    ++mOutMpBufferedCount;

    mlBufferMpOut.push_back(pMp);
    pMp->MarkInOutBuffer();

}

void Communicator::DeleteMpFromBuffer(mpptr pMP)
 {
     unique_lock<mutex> lock(mMutexBufferMpOut);

     list<mpptr>::iterator lit = find(mlBufferMpOut.begin(),mlBufferMpOut.end(),pMP);
     if(lit != mlBufferMpOut.end())
     {
         mlBufferMpOut.erase(lit);
         --mOutMpBufferedCount;

         lit = find(mlBufferMpOut.begin(),mlBufferMpOut.end(),pMP);
         if(lit != mlBufferMpOut.end()) cout << "ClientCommunicator::DeleteMpFromBuffer(...): pMP multiple times contained in mlBufferMpOut" << endl;
     }
 //    else cout << "ClientCommunicator::DeleteMpFromBuffer(...): MP searched but not found..." << endl;
 }

#ifndef MAPFUSION
void Communicator::PublishMapServer()
{
    #ifndef PERFORMANCE
    if(mpCC->mSysState != eSystemState::SERVER)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }
    #endif

    #ifdef STATS
    struct timeval tStart, tNow;
    double dEl;
    #endif

    macslam_msgs::macMap msgMap;

    while(this->CheckBufferKfOut())
    {
        #ifdef STATS
        if(!mbTotalCommTime)
            gettimeofday(&tStart,NULL);
        #endif

        kfptr pCurKf;
        {
            unique_lock<mutex> lock(mMutexBufferKfOut);
            pCurKf = mlBufferKfOut.front();

            macslam_msgs::macKeyFrame Msg;

            if(pCurKf->mId.second != mClientId)
            {
                if(pCurKf->SentToClient(mClientId))
                {
                    if(pCurKf->ConvertToPoseMessageServer(Msg,mpCC->mg2oS_wcurmap_wclientmap))
                    {
                        ++mOutKfCount;
                        msgMap.Keyframes.push_back(Msg);
                    }
                    else
                    {
                        pCurKf->UnMarkInOutBuffer();
                    }
                }
                else
                {
                    pCurKf->ConvertToMessageServer(Msg,mpCC->mg2oS_wcurmap_wclientmap);
                    pCurKf->AddInformedClient(mClientId);
                    ++mOutKfCount;
                    msgMap.Keyframes.push_back(Msg);
                }
            }
            else
            {
                if(pCurKf->ConvertToPoseMessageServer(Msg,mpCC->mg2oS_wcurmap_wclientmap))
                {
                    ++mOutKfCount;
                    msgMap.Keyframes.push_back(Msg);
                }
                else
                {
                    pCurKf->UnMarkInOutBuffer();
                }                
            }

            mlBufferKfOut.pop_front();

            #ifdef STATS
//            mdSizeOfMsgOut = mdSizeOfMsgOut + sizeof(Msg);
            #endif

        }
        #ifdef STATS
        if(!mbTotalCommTime)
        {
            gettimeofday(&tNow,NULL);
            dEl = (tNow.tv_sec - tStart.tv_sec) + (tNow.tv_usec - tStart.tv_usec) / 1000000.0;
            estd::UpdateStats(mdTimePerKfOut_mean,mdTimePerKfOut_ssd,mnKFsProcessedOut,dEl);
        }
        #endif
    }

    while(this->CheckBufferMpOut())
    {
        #ifdef STATS
        if(!mbTotalCommTime)
            gettimeofday(&tStart,NULL);
        #endif

//        Eigen::Matrix3d eigR = mpCC->mg2oS_wcurmap_wclientmap.rotation().toRotationMatrix();
//        Eigen::Vector3d eigt = mpCC->mg2oS_wcurmap_wclientmap.translation();
//        cv::Mat Txx = Converter::toCvSE3(eigR,eigt);
//        cv::Mat R = Txx.rowRange(0,3).colRange(0,3);
//        cv::Mat t = Txx.rowRange(0,3).col(3);

//        if(mClientId == 0)
//        {
//            cout << "R: " << R << endl;
//            cout << "t: " << t << endl;
////            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ <<  "mpCC->mg2oS_wcurmap_wclientmap != g2o::Sim3()" << endl;
//        }

        mpptr pCurMp;
        {
            unique_lock<mutex> lock(mMutexBufferMpOut);
            pCurMp = mlBufferMpOut.front();

            macslam_msgs::macMapPoint Msg;

            if(pCurMp->mId.second != mClientId)
            {
                if(pCurMp->SentToClient(mClientId))
                {
                    if(pCurMp->ConvertToPosMessageServer(Msg,mpCC->mg2oS_wcurmap_wclientmap))
                    {
                        ++mOutMpCount;
                        msgMap.MapPoints.push_back(Msg);
                    }
                    else
                    {
                        pCurMp->UnMarkInOutBuffer();
                    }
                }
                else
                {
                    pCurMp->ConvertToMessageServer(Msg,mpCC->mg2oS_wcurmap_wclientmap);
                    pCurMp->AddInformedClient(mClientId);
                    ++mOutMpCount;
                    msgMap.MapPoints.push_back(Msg);
                }

            }
            else
            {
                if(pCurMp->ConvertToPosMessageServer(Msg,mpCC->mg2oS_wcurmap_wclientmap))
                {
                    ++mOutMpCount;
                    msgMap.MapPoints.push_back(Msg);
                }
                else
                {
                    pCurMp->UnMarkInOutBuffer();
                }
            }

            mlBufferMpOut.pop_front();

            #ifdef STATS
//            mdSizeOfMsgOut = mdSizeOfMsgOut + sizeof(Msg);
            #endif
        }        
        #ifdef STATS
        if(!mbTotalCommTime)
        {
            gettimeofday(&tNow,NULL);
            dEl = (tNow.tv_sec - tStart.tv_sec) + (tNow.tv_usec - tStart.tv_usec) / 1000000.0;
            estd::UpdateStats(mdTimePerMpOut_mean,mdTimePerMpOut_ssd,mnMPsProcessedOut,dEl);
        }
        #endif
    }

    if(mVerboseMode==-4) cout << "PubNewData: " << ros::Time::now()  << " #msg: " << msgMap.MsgCount << endl;

    if(!msgMap.Keyframes.empty() || !msgMap.MapPoints.empty())
    {
        ++mOutMapCount;
        msgMap.MsgCount = mOutMapCount;
        mPubMap.publish(msgMap);

        #ifdef STATS
        if(!mbFirstMessage)
            mbFirstMessage = true;

//        mdSizeOfMsgOut = mdSizeOfMsgOut + sizeof(msgMap);
        #endif

//        cout << "Publishing KFs|MPs: " << msgMap.Keyframes.size() << "|" << msgMap.MapPoints.size() << endl;
//        if(msgMap.Keyframes.size() < 3)
//            for(int i = 0;i<msgMap.Keyframes.size();++i)
//            {
//                macslam_msgs::macKeyFrame msg = msgMap.Keyframes[i];
//                cout << "comm" << mClientId << ": Publishing KF " << msg.mnId << "|" << msg.mClientId << endl;
//            }
    }
}
#endif

void Communicator::PublishMapClient()
{
    #ifndef PERFORMANCE
    if(mpCC->mSysState != eSystemState::CLIENT)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }
    #endif

    #ifdef STATS
    struct timeval tStart, tNow;
    double dEl;
    #endif

     macslam_msgs::macMap msgMap;

     while(this->CheckBufferKfOut())
     {
        #ifdef STATS
         if(!mbTotalCommTime)
            gettimeofday(&tStart,NULL);
        #endif

         kfptr pCurKf;
         {
             unique_lock<mutex> lock(mMutexBufferKfOut);
             pCurKf = mlBufferKfOut.front();

             if(pCurKf->mId.second != mClientId)
             {
                 mlBufferKfOut.pop_front();
//                 cout << "\033[1;35m!!! HAZARD !!!\033[0m KF discarded" << endl;
//                 pCurKf->UnMarkInOutBuffer(); // not necessary -- data from other client should not be sent a any point
                 continue;
             }

             ++mOutKfCount;

             macslam_msgs::macKeyFrame Msg;
             pCurKf->ConvertToMessageClient(Msg);

 //            if(pCurKf->mnId == 0 || pCurKf->mnId == 1 || pCurKf->mnId == 9 || pCurKf->mnId == 20) pCurKf->ShowMyValues();

             msgMap.Keyframes.push_back(Msg);

             mlBufferKfOut.pop_front();

            #ifdef STATS
//            mdSizeOfMsgOut = mdSizeOfMsgOut + sizeof(Msg);
            #endif
//             cout << "SizeOf KfMsgs" << sizeof(Msg);
         }
        #ifdef STATS
         if(!mbTotalCommTime)
         {
            gettimeofday(&tNow,NULL);
            dEl = (tNow.tv_sec - tStart.tv_sec) + (tNow.tv_usec - tStart.tv_usec) / 1000000.0;
            estd::UpdateStats(mdTimePerKfOut_mean,mdTimePerKfOut_ssd,mnKFsProcessedOut,dEl);
         }
        #endif
     }

     while(this->CheckBufferMpOut())
     {
        #ifdef STATS
         if(!mbTotalCommTime)
            gettimeofday(&tStart,NULL);
        #endif

         mpptr pCurMp;
         {
             unique_lock<mutex> lock(mMutexBufferMpOut);
             pCurMp = mlBufferMpOut.front();

             if(pCurMp->mId.second != mClientId)
             {
                #ifdef VISUALIZATION
                 //MultiUse Sending
                 if(pCurMp->mbMultiUse)
                 {
                     macslam_msgs::macMapPoint Msg;
                     pCurMp->ConvertForeignPointToMessageClient(Msg);
                     msgMap.MapPoints.push_back(Msg);
//                     cout << "+++ Send MultiUse Message +++" << endl;
                 }
                #endif

                 mlBufferMpOut.pop_front();
                 pCurMp->UnMarkInOutBuffer();
//                 cout << "\033[1;35m!!! HAZARD !!!\033[0m MP discarded" << endl;
//                 pCurMp->UnMarkInOutBuffer(); // not necessary -- data from other client should not be sent a any point
                 continue;
             }

             ++mOutMpCount;

             macslam_msgs::macMapPoint Msg;
             pCurMp->ConvertToMessageClient(Msg);

//             if(!(msSentMPs.count(pCurMp->mId.first))) cout << "send MP -- id|nObs|pCurMp->mbBad: " << pCurMp->mId.first << "|" << pCurMp->Observations() << "|" << (pCurMp->isBad() == true) << endl;
             msSentMPs.insert(pCurMp->mId.first);

             msgMap.MapPoints.push_back(Msg);

             mlBufferMpOut.pop_front();

            #ifdef STATS
//            mdSizeOfMsgOut = mdSizeOfMsgOut + sizeof(Msg);
            #endif

 //            if(pCurMp->mnId == 0 || pCurMp->mnId == 1 || pCurMp->mnId == 17 || pCurMp->mnId == 101) pCurMp->ShowMyValues();
 //            if(pCurMp->mnId == 419 || pCurMp->mnId == 1 || pCurMp->mnId == 17 || pCurMp->mnId == 101) pCurMp->ShowMyObservations();
         }
        #ifdef STATS
         if(!mbTotalCommTime)
         {
            gettimeofday(&tNow,NULL);
            dEl = (tNow.tv_sec - tStart.tv_sec) + (tNow.tv_usec - tStart.tv_usec) / 1000000.0;
            estd::UpdateStats(mdTimePerMpOut_mean,mdTimePerMpOut_ssd,mnMPsProcessedOut,dEl);
         }
         #endif
     }

//     ++mOutMapCount;
//     msgMap.MsgCount = mOutMapCount;

////     cout << "Publishing KFs|MPs: " << msgMap.Keyframes.size() << "|" << msgMap.MapPoints.size() << endl;

//     mPubMap.publish(msgMap);

     if(!msgMap.Keyframes.empty() || !msgMap.MapPoints.empty())
     {
         ++mOutMapCount;
         msgMap.MsgCount = mOutMapCount;

         mPubMap.publish(msgMap);

        #ifdef STATS
         if(!mbFirstMessage)
             mbFirstMessage = true;

//         mdSizeOfMsgOut = mdSizeOfMsgOut + sizeof(msgMap);

//         cout << "SizeOf Msg" << sizeof(msgMap);
//         cout << "SizeOf Msg.KFs" << sizeof(msgMap.Keyframes);
        #endif
     }
}


bool Communicator::CheckBufferKfIn()
{
    unique_lock<mutex> lock(mMutexBufferKfIn);
    return(!mlBufferKfIn.empty());
}

bool Communicator::CheckBufferKfOut()
{
    unique_lock<mutex> lock(mMutexBufferKfOut);
    return(!mlBufferKfOut.empty());
}

bool Communicator::CheckBufferMpIn()
{
    unique_lock<mutex> lock(mMutexBufferMpIn);
    return(!mlBufferMpIn.empty());
}

bool Communicator::CheckBufferMpOut()
{
    unique_lock<mutex> lock(mMutexBufferMpOut);
    return(!mlBufferMpOut.empty());
}

#ifdef HACKZ
void Communicator::CorrectAfterLC(kfptr pKF, g2o::Sim3 g2oS_loop)
{
//    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ServerCommunicator::CorrectAfterLC(): should not be called" << endl;

//    cout << __func__ << __LINE__ << endl;

//    cout << "mpMap->KeyFramesInMap(): " << mpMap->KeyFramesInMap() << endl;
//    cout << "mLoopCorrectionThreshold: " << mLoopCorrectionThreshold << endl;

    if(pKF->IsPoseLocked())
        return;

    if(mpMap->KeyFramesInMap() < mLoopCorrectionThreshold)
        return;

//    cout << __func__ << __LINE__ << endl;

//    cout << "did not return" << endl;

    cv::Mat Tcw = pKF->GetPose();
    cv::Mat T_cref_c = mloopCorrectionPoseTcw * Tcw.inv();
    kfptr pKFref = mpMap->GetKfPtr(mMaxId.first,mMaxId.second);
    if(!pKFref) cout << "\033[1;31m!!!!! ERROR !!!!!\033 " << __func__ << endl;
    cv::Mat Twc_new = pKFref->GetPoseInverse() * T_cref_c;
    pKF->SetPose(Twc_new.inv(),true);

//    pKF->mbOmitSending = true;

//    cv::Mat Tcw = pKF->GetPose();
//    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
//    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
//    g2o::Sim3 g2oS_cw(Converter::toMatrix3d(Rcw),Converter::toVector3d(tcw),1.0);

//    g2oS_cw = g2oS_cw * g2oS_loop;

//    Eigen::Matrix3d eigR = g2oS_cw.rotation().toRotationMatrix();
//    Eigen::Vector3d eigt = g2oS_cw.translation();
//    double s = g2oS_cw.scale();
//    eigt *=(1./s); //[R t/s;0 1]
//    cv::Mat Tcw_new = Converter::toCvSE3(eigR,eigt);
//    pKF->SetPose(Tcw_new,true);

//    pKF->mbOmitSending = false;
}

void Communicator::CorrectAfterLC(mpptr pMP, g2o::Sim3 g2oS_loop)
{
//    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ServerCommunicator::CorrectAfterLC(): should not be called" << endl;

//    cout << __func__ << __LINE__ << endl;

//    cout << "mpMap->KeyFramesInMap(): " << mpMap->KeyFramesInMap() << endl;
//    cout << "mLoopCorrectionThreshold: " << mLoopCorrectionThreshold << endl;

    if(pMP->IsPosLocked())
        return;

    if(mpMap->KeyFramesInMap() < mLoopCorrectionThreshold)
        return;

    kfptr pKFforMP = pMP->GetReferenceKeyFrame();

    if(!pKFforMP) return;
    if(pKFforMP->mId.first <= mMaxId.first) return;

    cv::Mat WorldPos = pMP->GetWorldPos();

//    cout << "WorldPos: " << WorldPos << endl;

    cv::Mat WorldPosHom = cv::Mat::eye(4,1,5);

//    cout << "WorldPosHom: " << WorldPosHom << endl;

//    WorldPosHom.rowRange(0,3).col(0) = WorldPos.rowRange(0,3).col(0);
//    WorldPosHom.rowRange(3,4).col(0) = 0.0f;

    WorldPosHom.at<float>(0,0) = WorldPos.at<float>(0,0);
    WorldPosHom.at<float>(1,0) = WorldPos.at<float>(0,0);
    WorldPosHom.at<float>(2,0) = WorldPos.at<float>(0,0);
    WorldPosHom.at<float>(3,0) = 0.0f;

//    cout << "WorldPosHom: " << WorldPosHom << endl;

    cv::Mat Tcw = pKFforMP->GetPose();
    cv::Mat PosRelHom = Tcw * WorldPosHom;

//    cout << "PosRelHom: " << PosRelHom << endl;

    cv::Mat T_cref_c = mloopCorrectionPoseTcw * Tcw.inv();
    kfptr pKFref = mpMap->GetKfPtr(mMaxId.first,mMaxId.second);
    if(!pKFref) cout << "\033[1;31m!!!!! ERROR !!!!!\033 " << __func__ << endl;
    cv::Mat PosNewHom = pKFref->GetPoseInverse() * T_cref_c * PosRelHom;

//    cout << "PosNewHom: " << PosNewHom << endl;

    pMP->SetWorldPos(PosNewHom.rowRange(0,3).col(0),true);

//    pMP->mbOmitSending = true;

//    cv::Mat P3D_w = pMP->GetWorldPos();
//    Eigen::Matrix<double,3,1> eigP3D_w = Converter::toVector3d(P3D_w);

//    eigP3D_w = g2oS_loop.map(eigP3D_w);

//    cv::Mat P3D_w_new = Converter::toCvMat(eigP3D_w);
//    pMP->SetWorldPos(P3D_w_new,true);

//    pMP->mbOmitSending = false;
}
#endif

} //end ns
