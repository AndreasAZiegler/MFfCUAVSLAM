#include <macslam/server/ServerSystem.h>

int extVerboseMode;

namespace macslam{

ServerSystem::ServerSystem(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, const string &strVocFile)
    : mNh(Nh), mNhPrivate(NhPrivate), mMaxClients(4), mpUID(new estd::UniqueIdDispenser())
{
    //+++++ define checks +++++
    #ifdef PERFORMANCE
    cout << "\033[1;32m xxxxx PERFORMANCE defined xxxxx\033[0m" << endl;
    #endif
    #ifdef STATS
    cout << "\033[1;32m xxxxx STATS defined xxxxx\033[0m" << endl;
    #endif
    #ifdef VISUALIZATION
    cout << "\033[1;32m xxxxx VISUALIZATION defined xxxxx\033[0m" << endl;
    #endif
    #ifdef DEBUG
    cout << "\033[1;32m xxxxx DEBUG defined xxxxx\033[0m" << endl;
    #endif
    #ifdef HACKZ
    cout << "\033[1;32m xxxxx HACKZ defined xxxxx\033[0m" << endl;
    #endif
    #ifdef MAPFUSION
    cout << "\033[1;32m xxxxx MAPFUSION defined xxxxx\033[0m" << endl;
    #endif

    //+++++ load params +++++

    mNhPrivate.param("VerboseMode",mVerboseMode,1);
    mNhPrivate.param("bRecordMode",mbRecordMode,false);
    mNhPrivate.param("NumOfClients",mNumOfClients,0);

    extVerboseMode = mVerboseMode;

    if(mNumOfClients < 1)
    {
        ROS_ERROR_STREAM("In \" System::System(...)\": Num od clients < 1");
        throw estd::infrastructure_ex();
    }

    // Param loading not necessary - all info comes with the messages
//    //+++++ Check settings file +++++

//    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
//    if(!fsSettings.isOpened())
//    {
//       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
//       exit(-1);
//    }

    //+++++ load vocabulary +++++

    this->LoadVocabulary(strVocFile);

    if (mbRecordMode)
    {
        cout << "ENTER to continue..." << endl << endl;
        std::cin.get(); //wait to start bagfile recording
    }

    //+++++ Create KeyFrame Database +++++
    this->InitializeKFDB();
//    mpKFDB.reset(new KeyFrameDatabase(mpVoc));

    //+++++ Create the Map +++++
    this->InitializeMaps();
//    mpMap.reset(new Map());

    //+++++ Create the Comm +++++
//    mpComm.reset(new ServerCommunicator(mNh,mNhPrivate,mpVoc,mpMap,mpKFDB,0));
//    mptCommunicator.reset(new thread(&mcps::ServerCommunicator::Run,mpComm));

}

void ServerSystem::InitializeClients()
{
    //Cannot be called from constructor - shared_from_this() not available

    //Client 0
    mpClient0.reset(new ClientHandler(mNh,mNhPrivate,mpVoc,mpKFDB,mpMap0,0,mpUID,eSystemState::SERVER,mstrSettingsFile));
    mpClient0->InitializeThreads();

    //Client 1
    if(mNumOfClients > 1)
    {
        mpClient1.reset(new ClientHandler(mNh,mNhPrivate,mpVoc,mpKFDB,mpMap1,1,mpUID,eSystemState::SERVER,mstrSettingsFile));
        mpClient1->InitializeThreads();
    }

    //Client 2
    if(mNumOfClients > 2)
    {
        mpClient2.reset(new ClientHandler(mNh,mNhPrivate,mpVoc,mpKFDB,mpMap2,2,mpUID,eSystemState::SERVER,mstrSettingsFile));
        mpClient2->InitializeThreads();
    }

    //Client 3
    if(mNumOfClients > 3)
    {
        mpClient3.reset(new ClientHandler(mNh,mNhPrivate,mpVoc,mpKFDB,mpMap3,3,mpUID,eSystemState::SERVER,mstrSettingsFile));
        mpClient3->InitializeThreads();
    }

    if(mNumOfClients > mMaxClients)
    {
        cout << "\033[1;33m!!! WARN !!!\033[0m Maximum number of clients is " << mMaxClients << endl;
    }

    this->InitializeMapMatcher();
}

void ServerSystem::LoadVocabulary(const string &strVocFile)
{
    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVoc.reset(new ORBVocabulary());
    bool bVocLoad = mpVoc->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;
}

void ServerSystem::InitializeMaps()
{
    mpMap0.reset(new Map(mNh,mNhPrivate,0,eSystemState::SERVER));
    if(mNumOfClients > 1) mpMap1.reset(new Map(mNh,mNhPrivate,1,eSystemState::SERVER)); else mpMap1=nullptr;
    if(mNumOfClients > 2) mpMap2.reset(new Map(mNh,mNhPrivate,2,eSystemState::SERVER)); else mpMap2=nullptr;
    if(mNumOfClients > 3) mpMap3.reset(new Map(mNh,mNhPrivate,3,eSystemState::SERVER)); else mpMap3=nullptr;
}

void ServerSystem::InitializeKFDB()
{
    mpKFDB.reset(new KeyFrameDatabase(mpVoc));
}

void ServerSystem::InitializeMapMatcher()
{
    MapMatchingParams MMParams;

    //load params
    mNhPrivate.param("SolverIterations",MMParams.mSolverIterations,5);
    mNhPrivate.param("MatchesThres",MMParams.mMatchesThres,20);
    mNhPrivate.param("InliersThres",MMParams.mInliersThres,20);
    mNhPrivate.param("TotalMatchesThres",MMParams.mTotalMatchesThres,40);
    mNhPrivate.param("Probability",MMParams.mProbability,0.99);
    mNhPrivate.param("MinInliers",MMParams.mMinInliers,6);
    mNhPrivate.param("MaxIterations",MMParams.mMaxIterations,300);
    mNhPrivate.param("MinHitsForMerge",MMParams.mMinHitsForMerge,3);
    mNhPrivate.param("GBAIterations",MMParams.mGBAIterations,20);
    mNhPrivate.param("StartAfterNumKFs",MMParams.mKFsToSkip,10);
    mNhPrivate.param("LoopLockSleep",MMParams.mLockSleep,1000);

    cout << "Server Map Matcher Params: " << endl;
    cout << "KFs to skip: " << MMParams.mKFsToSkip << endl;
    cout << "Solver Iterations: " << MMParams.mSolverIterations << endl;
    cout << "Matches Threshold: " << MMParams.mMatchesThres << endl;
    cout << "Inliers Threshold: " << MMParams.mInliersThres << endl;
    cout << "Total Matches Threshold: " << MMParams.mTotalMatchesThres << endl;
    cout << "RANSAC Probability: " << MMParams.mProbability << endl;
    cout << "RANSAC minInliers: " << MMParams.mMinInliers << endl;
    cout << "RANSAC maxIterations: " << MMParams.mMaxIterations << endl;
    cout << "Min Hits for Merge: " << MMParams.mMinHitsForMerge << endl;
    cout << "GBA Iterations: " << MMParams.mGBAIterations << endl;
    cout << "LockSleep: " << MMParams.mLockSleep << endl;

    //set up matcher

    mpMapMatcher.reset(new MapMatcher(mNh,mNhPrivate, mpKFDB, mpVoc, mpMap0, mpMap1, mpMap2, mpMap3, MMParams));
    mptMapMatching.reset(new thread(&MapMatcher::Run, mpMapMatcher));

    if(mpClient0) mpClient0->SetMapMatcher(mpMapMatcher);
    if(mpClient1) mpClient1->SetMapMatcher(mpMapMatcher);
    if(mpClient2) mpClient2->SetMapMatcher(mpMapMatcher);
    if(mpClient3) mpClient3->SetMapMatcher(mpMapMatcher);
}

} //end namespace
