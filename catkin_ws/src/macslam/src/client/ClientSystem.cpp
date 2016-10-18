#include <macslam/client/ClientSystem.h>

int extVerboseMode;

namespace macslam{

ClientSystem::ClientSystem(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, const string &strVocFile, const string &strSettingsFile)
    : mNh(Nh), mNhPrivate(NhPrivate), mstrSettingsFile(strSettingsFile), mpUID(new estd::UniqueIdDispenser())
{
    mdElTotalTrack = 0;

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

    std::string TopicNameCamSub;
    int ClientId;

    mNhPrivate.param("VerboseMode",mVerboseMode,1);
    mNhPrivate.param("ClientId",ClientId,-1);
    mNhPrivate.param("bRecordMode",mbRecordMode,false);

    extVerboseMode = mVerboseMode;
    mClientId = static_cast<size_t>(ClientId);

    //+++++ Check settings file +++++

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    //+++++ load vocabulary +++++

    this->LoadVocabulary(strVocFile);

    if (mbRecordMode)
    {
        cout << "ENTER to continue..." << endl << endl;
        std::cin.get(); //wait to start bagfile recording
    }

    //+++++ Create KeyFrame Database +++++
    mpKFDB.reset(new KeyFrameDatabase(mpVoc));

    //+++++ Create the Map +++++
    mpMap.reset(new Map(mNh,mNhPrivate,mClientId,eSystemState::CLIENT));

    //+++++ Initialize Agent +++++
    mpAgent.reset(new ClientHandler(mNh,mNhPrivate,mpVoc,mpKFDB,mpMap,mClientId,mpUID,eSystemState::CLIENT,strSettingsFile));
    mpAgent->InitializeThreads();

    //++++++++++
    if (mVerboseMode > 0) cout << endl << "Clientsystem initialized (Client ID: " << mClientId << ")" << endl;
    if (mVerboseMode > 0) cout << "Input topic: " << TopicNameCamSub << endl;
}

void ClientSystem::LoadVocabulary(const string &strVocFile)
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




} //end namespace
