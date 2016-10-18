#include <macslam/Map.h>

namespace macslam {

Map::Map(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, size_t MapId, eSystemState SysState)
    : mnMaxKFid(0), mnMaxMPid(0),
      mnMaxKFidUnique(0),mnMaxMPidUnique(0),
      mNh(Nh), mNhPrivate(NhPrivate),
      mMapId(MapId),mbOutdated(false),
      mSysState(SysState),
      mbLockMapUpdate(false),mbLockPointCreation(false)
{    
    mVerboseMode = extVerboseMode;

    string SysType;
    if(mSysState == eSystemState::CLIENT)
    {
        SysType = "Client";
    }
    else if(mSysState == eSystemState::SERVER)
    {
        SysType = "Server";
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Map::Map(): invalid systems state: " << mSysState << endl;
        throw infrastructure_ex();
    }

    cout << "set up map" << mMapId << endl;

    std::stringstream* ss;

    ss = new stringstream;
    *ss << "MarkerMap" << SysType << mMapId;
    mPubMarker = mNh.advertise<visualization_msgs::Marker>(ss->str(),10);

    ss = new stringstream;
    *ss << "MapPoints0Map" << SysType << mMapId;
    mPubMapPoints0 = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),1);

    ss = new stringstream;
    *ss << "MapPoints1Map" << SysType << mMapId;
    mPubMapPoints1 = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),1);

    ss = new stringstream;
    *ss << "MapPoints2Map" << SysType << mMapId;
    mPubMapPoints2 = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),1);

    ss = new stringstream;
    *ss << "MapPoints3Map" << SysType << mMapId;
    mPubMapPoints3 = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),1);

    ss = new stringstream;
    *ss << "MapPointsNativeChangedMap" << SysType << mMapId;
    mPubMapPointsx = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),1);

    ss = new stringstream;
    *ss << "MapMarkerArray" << SysType << mMapId;
    mPubMarkerArray = mNh.advertise<visualization_msgs::MarkerArray>(ss->str(),1);

    ss = new stringstream;
    *ss << "MapPointsCol" << SysType << mMapId;
    mPubMapPointsCol = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),1);

    ss = new stringstream;
    *ss << "MapPointsMap" << SysType << mMapId;
    mPubMapPoints = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),1);

    ss = new stringstream;
    *ss << "MapPointsMultiUseMap" << SysType << mMapId;
    mPubMapPointsMultiUse = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),1);

    delete ss;
}

Map::Map(const mapptr &pMapTarget, const mapptr &pMapToFuse)
    : mNh(pMapTarget->mNh),mNhPrivate(pMapTarget->mNhPrivate),
      mMapId(pMapTarget->mMapId),mbOutdated(false),
      mbLockMapUpdate(false),mbLockPointCreation(false)
{
    mVerboseMode = extVerboseMode;

    mSysState = pMapTarget->mSysState;

    string SysType;
    if(mSysState == eSystemState::CLIENT)
    {
        SysType = "Client";
    }
    else if(mSysState == eSystemState::SERVER)
    {
        SysType = "Server";
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Map::Map(): invalid systems state: " << mSysState << endl;
        throw infrastructure_ex();
    }

    std::stringstream* ss;

//    ss = new stringstream;
//    *ss << "CovGraphMap" << SysType << mMapId;
//    mPubMarker = mNh.advertise<visualization_msgs::Marker>(ss->str(),1);

//    ss = new stringstream;
//    *ss << "MapPointsMap" << SysType << mMapId;
//    mPubMapPoints = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),1);

//    ss = new stringstream;
//    *ss << "MapMarkerArray" << SysType << mMapId;
//    mPubMarkerArray = mNh.advertise<visualization_msgs::MarkerArray>(ss->str(),1);

    ss = new stringstream;
    *ss << "MarkerMap" << SysType << mMapId;
    mPubMarker = mNh.advertise<visualization_msgs::Marker>(ss->str(),10);

    ss = new stringstream;
    *ss << "MapPoints0Map" << SysType << mMapId;
    mPubMapPoints0 = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),1);

    ss = new stringstream;
    *ss << "MapPoints1Map" << SysType << mMapId;
    mPubMapPoints1 = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),1);

    ss = new stringstream;
    *ss << "MapPoints2Map" << SysType << mMapId;
    mPubMapPoints2 = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),1);

    ss = new stringstream;
    *ss << "MapPoints3Map" << SysType << mMapId;
    mPubMapPoints3 = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),1);

    ss = new stringstream;
    *ss << "MapPointsNativeChangedMap" << SysType << mMapId;
    mPubMapPointsx = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),1);

    ss = new stringstream;
    *ss << "MapMarkerArray" << SysType << mMapId;
    mPubMarkerArray = mNh.advertise<visualization_msgs::MarkerArray>(ss->str(),1);

    ss = new stringstream;
    *ss << "MapPointsMap" << SysType << mMapId;
    mPubMapPoints = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),1);

    ss = new stringstream;
    *ss << "MapPointsMultiUseMap" << SysType << mMapId;
    mPubMapPointsMultiUse = mNh.advertise<sensor_msgs::PointCloud2>(ss->str(),1);

    delete ss;

    //data Map A
    set<size_t> msuAssClientsA = pMapTarget->msuAssClients;
    vector<kfptr> mvpKeyFrameOriginsA = pMapTarget->mvpKeyFrameOrigins;
    std::set<mpptr> mspMapPointsA = pMapTarget->GetMspMapPoints();
    std::set<kfptr> mspKeyFramesA = pMapTarget->GetMspKeyFrames();
    long unsigned int mnMaxKFidA = pMapTarget->GetMaxKFid();
    long unsigned int mnMaxMPidA = pMapTarget->GetMaxMPid();
    long unsigned int mnMaxKFidUniqueA = pMapTarget->GetMaxKFidUnique();
    long unsigned int mnMaxMPidUniqueA = pMapTarget->GetMaxKFidUnique();
    std::map<idpair,mpptr> mmpMapPointsA = pMapTarget->GetMmpMapPoints();
    std::map<idpair,kfptr> mmpKeyFramesA = pMapTarget->GetMmpKeyFrames();
    std::map<idpair,mpptr> mmpErasedMapPointsA = pMapTarget->GetMmpErasedMapPoints();
    std::map<idpair,kfptr> mmpErasedKeyFramesA = pMapTarget->GetMmpErasedKeyFrames();
    map<idpair,kfptr> mmpKfBufferA = pMapTarget->GetMmpKfBuffer();
    map<idpair,mpptr> mmpMpBufferA = pMapTarget->GetMmpMpBuffer();
//    set<chptr> mspClientHandlerA = pMapTarget->GetClientHandlers();
    set<ccptr> spCCA = pMapTarget->GetCCPtrs();

    //data Map B
    set<size_t> msuAssClientsB = pMapToFuse->msuAssClients;
    vector<kfptr> mvpKeyFrameOriginsB = pMapToFuse->mvpKeyFrameOrigins;
    std::set<mpptr> mspMapPointsB = pMapToFuse->GetMspMapPoints();
    std::set<kfptr> mspKeyFramesB = pMapToFuse->GetMspKeyFrames();
    long unsigned int mnMaxKFidB = pMapToFuse->GetMaxKFid();
    long unsigned int mnMaxMPidB = pMapToFuse->GetMaxMPid();
    long unsigned int mnMaxKFidUniqueB = pMapToFuse->GetMaxKFidUnique();
    long unsigned int mnMaxMPidUniqueB = pMapToFuse->GetMaxKFidUnique();
    std::map<idpair,mpptr> mmpMapPointsB = pMapToFuse->GetMmpMapPoints();
    std::map<idpair,kfptr> mmpKeyFramesB = pMapToFuse->GetMmpKeyFrames();
    std::map<idpair,mpptr> mmpErasedMapPointsB = pMapToFuse->GetMmpErasedMapPoints();
    std::map<idpair,kfptr> mmpErasedKeyFramesB = pMapToFuse->GetMmpErasedKeyFrames();
    map<idpair,kfptr> mmpKfBufferB = pMapToFuse->GetMmpKfBuffer();
    map<idpair,mpptr> mmpMpBufferB = pMapToFuse->GetMmpMpBuffer();
//    set<chptr> mspClientHandlerB = pMapToFuse->GetClientHandlers();
    set<ccptr> spCCB = pMapToFuse->GetCCPtrs();

    //fill new map
    mOdomFrame = pMapTarget->mOdomFrame;

    msuAssClients.insert(msuAssClientsA.begin(),msuAssClientsA.end());
    msuAssClients.insert(msuAssClientsB.begin(),msuAssClientsB.end());
    mvpKeyFrameOrigins.insert(mvpKeyFrameOrigins.end(),mvpKeyFrameOriginsA.begin(),mvpKeyFrameOriginsA.end());
    mvpKeyFrameOrigins.insert(mvpKeyFrameOrigins.end(),mvpKeyFrameOriginsB.begin(),mvpKeyFrameOriginsB.end());
    mspMapPoints.insert(mspMapPointsA.begin(),mspMapPointsA.end());
    mspMapPoints.insert(mspMapPointsB.begin(),mspMapPointsB.end());
    mspKeyFrames.insert(mspKeyFramesA.begin(),mspKeyFramesA.end());
    mspKeyFrames.insert(mspKeyFramesB.begin(),mspKeyFramesB.end());
    mnMaxKFid = std::max(mnMaxKFidA,mnMaxKFidB);
    mnMaxMPid = std::max(mnMaxMPidA,mnMaxMPidB);
    mnMaxKFidUnique = std::max(mnMaxKFidUniqueA,mnMaxKFidUniqueB);
    mnMaxMPidUnique = std::max(mnMaxMPidUniqueA,mnMaxMPidUniqueB);
    mmpMapPoints.insert(mmpMapPointsA.begin(),mmpMapPointsA.end());
    mmpMapPoints.insert(mmpMapPointsB.begin(),mmpMapPointsB.end());
    mmpKeyFrames.insert(mmpKeyFramesA.begin(),mmpKeyFramesA.end());
    mmpKeyFrames.insert(mmpKeyFramesB.begin(),mmpKeyFramesB.end());
    mmpErasedMapPoints.insert(mmpErasedMapPointsA.begin(),mmpErasedMapPointsA.end());
    mmpErasedMapPoints.insert(mmpErasedMapPointsB.begin(),mmpErasedMapPointsB.end());
    mmpErasedKeyFrames.insert(mmpErasedKeyFramesA.begin(),mmpErasedKeyFramesA.end());
    mmpErasedKeyFrames.insert(mmpErasedKeyFramesB.begin(),mmpErasedKeyFramesB.end());
    mmpKfBuffer.insert(mmpKfBufferA.begin(),mmpKfBufferA.end());
    mmpKfBuffer.insert(mmpKfBufferB.begin(),mmpKfBufferB.end());
    mmpMpBuffer.insert(mmpMpBufferA.begin(),mmpMpBufferA.end());
    mmpMpBuffer.insert(mmpMpBufferB.begin(),mmpMpBufferB.end());
//    mspClientHandler.insert(mspClientHandlerA.begin(),mspClientHandlerA.end());
//    mspClientHandler.insert(mspClientHandlerB.begin(),mspClientHandlerB.end());
    mspCC.insert(spCCA.begin(),spCCA.end());
    mspCC.insert(spCCB.begin(),spCCB.end());

    for(set<ccptr>::const_iterator sit = mspCC.begin();sit!=mspCC.end();++sit)
    {
        ccptr pCC = *sit;
        mspComm.insert(pCC->mpCH->GetCommPtr());
    }

    #ifdef DEBUG
    //----- check containers -----

        //TARGET CONTAINER

    std::vector<kfptr> vpKeyFramesTarget = pMapTarget->GetAllKeyFrames();

    for(std::vector<kfptr>::iterator vit = vpKeyFramesTarget.begin();vit!=vpKeyFramesTarget.end();++vit)
    {
        kfptr pKFi = *vit;

        set<kfptr>::iterator sit = this->mspKeyFrames.find(pKFi);
        if(sit == mspKeyFrames.end()) cout << "\033[1;31m!!!!! MFCError#1 !!!!!\033[0m Map::Map(map A,map B): KF in target map but not in fused map" << endl;
        else
        {
            kfptr pKFj = *sit;

            if(pKFi->isBad())
            {
                if(pKFj->isBad())
                {
                    //everything OK...
                }
                else
                {
                    cout << "\033[1;31m!!!!! MFCError#2 !!!!!\033[0m Map::Map(map A,map B): KF in target map BAD but OK in fused map" << endl;
                }
            }
            else
            {
                if(pKFj->isBad())
                {
                    cout << "\033[1;31m!!!!! MFCError#3 !!!!!\033[0m Map::Map(map A,map B): KF in target map OK but BAD in fused map" << endl;
                }
                else
                {
                    set<kfptr> spKfConi = pKFi->GetConnectedKeyFrames();
                    set<kfptr> spKfConj = pKFj->GetConnectedKeyFrames();

                    if(spKfConi.size() == spKfConj.size())
                    {
                        for(set<kfptr>::iterator sit2 = spKfConi.begin();sit2!=spKfConi.end();++sit2)
                        {
                            kfptr pKFConi = *sit2;
                            set<kfptr>::iterator sit3 = spKfConj.find(pKFConi);
                            if(sit3 == mspKeyFrames.end()) cout << "\033[1;31m!!!!! MFCError#4 !!!!!\033[0m Map::Map(map A,map B): connected KF for target map member KF not in fused map member KF" << endl;
                            else
                            {
                                kfptr pKFConj = *sit3;

                                if(pKFConi->isBad())
                                {
                                    if(pKFConj->isBad())
                                    {
                                        //everything OK...
                                    }
                                    else
                                    {
                                        cout << "\033[1;31m!!!!! MFCError#5 !!!!!\033[0m Map::Map(map A,map B): connected KF for target map member KF BAD but connected KF for fused map member KF OK" << endl;
                                    }
                                }
                                else
                                {
                                    if(pKFConj->isBad())
                                    {
                                        cout << "\033[1;31m!!!!! MFCError#6 !!!!!\033[0m Map::Map(map A,map B): connected KF for target map member KF OK but connected KF for fused map member KF BAD" << endl;
                                    }
                                    else
                                    {
                                        //everything OK...
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        cout << "\033[1;31m!!!!! MFCError#7 !!!!!\033[0m Map::Map(map A,map B): num of connected KFs not equal for target map and fused map" << endl;
                    }
                }
            }
        }
    }

        //TOFUSE CONTAINER

    std::vector<kfptr> vpKeyFramesToFuse = pMapToFuse->GetAllKeyFrames();

    for(std::vector<kfptr>::iterator vit = vpKeyFramesToFuse.begin();vit!=vpKeyFramesToFuse.end();++vit)
    {
        kfptr pKFi = *vit;

        set<kfptr>::iterator sit = this->mspKeyFrames.find(pKFi);
        if(sit == mspKeyFrames.end()) cout << "\033[1;31m!!!!! MFCError#8 !!!!!\033[0m Map::Map(map A,map B): KF in ToFuse map but not in fused map" << endl;
        else
        {
            kfptr pKFj = *sit;

            if(pKFi->isBad())
            {
                if(pKFj->isBad())
                {
                    //everything OK...
                }
                else
                {
                    cout << "\033[1;31m!!!!! MFCError#9 !!!!!\033[0m Map::Map(map A,map B): KF in ToFuse map BAD but OK in fused map" << endl;
                }
            }
            else
            {
                if(pKFj->isBad())
                {
                    cout << "\033[1;31m!!!!! MFCError#10 !!!!!\033[0m Map::Map(map A,map B): KF in ToFuse map OK but BAD in fused map" << endl;
                }
                else
                {
                    set<kfptr> spKfConi = pKFi->GetConnectedKeyFrames();
                    set<kfptr> spKfConj = pKFj->GetConnectedKeyFrames();

                    if(spKfConi.size() == spKfConj.size())
                    {
                        for(set<kfptr>::iterator sit2 = spKfConi.begin();sit2!=spKfConi.end();++sit2)
                        {
                            kfptr pKFConi = *sit2;
                            set<kfptr>::iterator sit3 = spKfConj.find(pKFConi);
                            if(sit3 == mspKeyFrames.end()) cout << "\033[1;31m!!!!! MFCError#11 !!!!!\033[0m Map::Map(map A,map B): connected KF for ToFuse map member KF not in fused map member KF" << endl;
                            else
                            {
                                kfptr pKFConj = *sit3;

                                if(pKFConi->isBad())
                                {
                                    if(pKFConj->isBad())
                                    {
                                        //everything OK...
                                    }
                                    else
                                    {
                                        cout << "\033[1;31m!!!!! MFCError#12 !!!!!\033[0m Map::Map(map A,map B): connected KF for ToFuse map member KF BAD but connected KF for fused map member KF OK" << endl;
                                    }
                                }
                                else
                                {
                                    if(pKFConj->isBad())
                                    {
                                        cout << "\033[1;31m!!!!! MFCError#13 !!!!!\033[0m Map::Map(map A,map B): connected KF for ToFuse map member KF OK but connected KF for fused map member KF BAD" << endl;
                                    }
                                    else
                                    {
                                        //everything OK...
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        cout << "\033[1;31m!!!!! MFCError#14 !!!!!\033[0m Map::Map(map A,map B): num of connected KFs not euqal for ToFuse map and fused map" << endl;
                    }
                }
            }
        }
    }
    #endif


    //----------------------------

    cout << "\033[1;33m!!! WARN !!!\033[0m Usage of \"Map::Map(const mapptr &pMapA, const mapptr &pMapB)\" --> do not forget to call \"Map::UpdateAssociatedData()\" afterwards (cannot be called by this constructor)" << endl;
}

void Map::UpdateAssociatedData()
{    
//    set<commptr> spComm;
//    for(set<ccptr>::const_iterator sit = mspCC.begin();sit!=mspCC.end();++sit)
//    {
//        ccptr pCC = *sit;
//        spComm.insert(pCC->mpCH->GetCommPtr());
//    }

    //replace associated maps
    for(set<kfptr>::iterator sit = mspKeyFrames.begin();sit!=mspKeyFrames.end();++sit)
    {
        kfptr pKF = *sit;
        pKF->ReplaceMap(this->shared_from_this());
        for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
        {
            commptr pComm = *sit2;
            pKF->AddCommPtr(pComm);
            pComm->PassKftoComm(pKF);
        }
    }

    for(set<mpptr>::iterator sit = mspMapPoints.begin();sit!=mspMapPoints.end();++sit)
    {
        mpptr pMP = *sit;
        pMP->ReplaceMap(this->shared_from_this());
        for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
        {
            commptr pComm = *sit2;
            pMP->AddCommPtr(pComm);
            pComm->PassMptoComm(pMP);
        }
    }

    for(map<idpair,kfptr>::iterator mit = mmpKfBuffer.begin();mit!=mmpKfBuffer.end();++mit)
    {
        (mit->second)->ReplaceMap(this->shared_from_this());
    }

    for(map<idpair,mpptr>::iterator mit = mmpMpBuffer.begin();mit!=mmpMpBuffer.end();++mit)
    {
        (mit->second)->ReplaceMap(this->shared_from_this());
    }
}

Map& Map::operator=(Map& rhs)
{
    mVerboseMode = rhs.mVerboseMode;

    msuAssClients = rhs.msuAssClients;
    mvpKeyFrameOrigins = rhs.mvpKeyFrameOrigins;
    mspMapPoints = rhs.mspMapPoints;
    mspKeyFrames = rhs.mspKeyFrames;
    mnMaxKFid = rhs.mnMaxKFid;
    mnMaxMPid = rhs.mnMaxMPid;
    mmpMapPoints = rhs.mmpMapPoints;
    mmpKeyFrames = rhs.mmpKeyFrames;
    mmpErasedMapPoints = rhs.mmpErasedMapPoints;
    mmpErasedKeyFrames = rhs.mmpErasedKeyFrames;
    mmpKfBuffer = rhs.mmpKfBuffer;
    mmpMapPoints = rhs.mmpMpBuffer;
}

void Map::AddKeyFrame(kfptr pKF)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - AddKeyFrame " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMap);

    if(mSysState == eSystemState::CLIENT)
    {
        if(mspKeyFrames.count(pKF)) cout << "!!!!! WARNING: adding KF to map that already exists !!!!!!" << endl;
        else
        {
            if(mspComm.size() != 1)
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  Map::AddKeyFrame: more than one Comm ptr " << endl;
            }
            commptr pComm = *(mspComm.begin());
            pComm->PassKftoComm(pKF);
//            mpComm->PassKftoComm(pKF);
        }
    }
    else if(mSysState == eSystemState::SERVER)
    {
        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            pKF->AddCommPtr(*sit);
//            commptr pComm = *sit;
//            if(pKF->mId.second != pComm->GetClientId())
//                pComm->PassKftoComm(pKF);
        }
    }

    mspKeyFrames.insert(pKF);


    if(pKF->mId.first>mnMaxKFid)
        mnMaxKFid=pKF->mId.first;
    if(pKF->mUniqueId>mnMaxKFidUnique)
        mnMaxKFidUnique=pKF->mUniqueId;

//    idpair idp = make_pair(pKF->mnId,pKF->mClientId);
//    mmpKeyFrames[idp] = pKF; //Performance: not nice, but fast...
    mmpKeyFrames[pKF->mId] = pKF; //Performance: not nice, but fast...

//    if(mspKeyFrames.size() % 10 == 0) cout << "Keyframes in map: " << mspKeyFrames.size() << endl;
}

void Map::AddMapPoint(mpptr pMP)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - AddMapPoint " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMap);

    if(mSysState == eSystemState::CLIENT)
    {
        if(mspMapPoints.count(pMP)) cout << "!!!!! WARNING: adding MP to map that already exists !!!!!!" << endl;
        else
        {
            if(mspComm.size() != 1)
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  Map::AddMapPoint: more than one Comm ptr " << endl;
            }

            commptr pComm = *(mspComm.begin());
            pComm->PassMptoComm(pMP);
//            mpComm->PassMptoComm(pMP);
        }
    }
    else if(mSysState == eSystemState::SERVER)
    {
        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            pMP->AddCommPtr(*sit);
//            commptr pComm = *sit;
//            if(pMP->mId.second != pComm->GetClientId())
//                pComm->PassMptoComm(pMP);
        }
    }

//    if((*(mspCC.begin()))->mbUseImgCol)
//    {
//        pMP->mRGB = pMP->GetColorRGB();
//        pMP->mGray = pMP->GetColor();
//    }

    mspMapPoints.insert(pMP);

    if(pMP->mId.first>mnMaxMPid)
        mnMaxMPid=pMP->mId.first;
    if(pMP->mUniqueId>mnMaxMPidUnique)
        mnMaxMPidUnique=pMP->mUniqueId;

//    idpair idp = make_pair(pMP->mnId,pMP->mClientId);
//    mmpMapPoints[idp] = pMP; //Performance: not nice, but fast...
    mmpMapPoints[pMP->mId] = pMP; //Performance: not nice, but fast...
}

void Map::EraseMapPoint(mpptr pMP)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - EraseMapPoint " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

//    pMP->EraseInOutBuffer();

//    idpair idp = make_pair(pMP->mnId,pMP->mClientId);
    std::map<idpair,mpptr>::iterator mit = mmpMapPoints.find(pMP->mId);
    if(mit != mmpMapPoints.end()) mmpMapPoints.erase(mit);

    unique_lock<mutex> lock2(mMutexErased);
    mmpErasedMapPoints[pMP->mId] = pMP;

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(kfptr pKF)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - EraseKeyFrame " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

//    idpair idp = make_pair(pKF->mnId,pKF->mClientId);
    std::map<idpair,kfptr>::iterator mit = mmpKeyFrames.find(pKF->mId);
    if(mit != mmpKeyFrames.end()) mmpKeyFrames.erase(mit);

    unique_lock<mutex> lock2(mMutexErased);
    mmpErasedKeyFrames[pKF->mId] = pKF;

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<mpptr> &vpMPs)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - SetReferenceMapPoints " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

vector<Map::kfptr> Map::GetAllKeyFrames()
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - GetAllKeyFrames " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMap);
    return vector<kfptr>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<Map::mpptr> Map::GetAllMapPoints()
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - GetAllMapPoints " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMap);
    return vector<mpptr>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - MapPointsInMap " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - KeyFramesInMap " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<Map::mpptr> Map::GetReferenceMapPoints()
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - GetReferenceMapPoints " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - GetMaxKFid " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

long unsigned int Map::GetMaxMPid()
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - GetMaxMPid " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMap);
    return mnMaxMPid;
}

long unsigned int Map::GetMaxKFidUnique()
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - GetMaxKFidUnique " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFidUnique;
}

long unsigned int Map::GetMaxMPidUnique()
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - GetMaxMPidUnique " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMap);
    return mnMaxMPidUnique;
}

void Map::clear()
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - clear " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    mspMapPoints.clear();
    mspKeyFrames.clear();
//    mspErasedMapPoints.clear();
//    mspErasedKeyFrames.clear();
    mmpMapPoints.clear();
    mmpKeyFrames.clear();
    mmpErasedMapPoints.clear();
    mmpErasedKeyFrames.clear();
    mnMaxKFid = 0;
    mnMaxMPid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
//    mspCC.clear();
}

Map::kfptr Map::GetKfPtr(size_t KfId, size_t ClientId) //Performance: find a better implementation for this method
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - GetKfPtr " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMap);

    idpair idp = make_pair(KfId,ClientId);
    std::map<idpair,kfptr>::iterator mit = mmpKeyFrames.find(idp);
    if(mit != mmpKeyFrames.end()) return mit->second;
    else return nullptr;
}

Map::mpptr Map::GetMpPtr(size_t MpId, size_t ClientId) //Performance: find a better implementation for this method
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - GetMpPtr " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMap);

    idpair idp = make_pair(MpId,ClientId);
    std::map<idpair,mpptr>::iterator mit = mmpMapPoints.find(idp);
    if(mit != mmpMapPoints.end()) return mit->second;
    else return nullptr;
}


bool Map::IsKfDeleted(size_t KfId, size_t ClientId)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - IsKfDeleted " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexErased);

    idpair idp = make_pair(KfId,ClientId);
    std::map<idpair,kfptr>::iterator mit = mmpErasedKeyFrames.find(idp);
    if(mit != mmpErasedKeyFrames.end()) return true;
    else return false;
}

bool Map::IsMpDeleted(size_t MpId, size_t ClientId)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - IsMpDeleted " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexErased);

    idpair idp = make_pair(MpId,ClientId);
    std::map<idpair,mpptr>::iterator mit = mmpErasedMapPoints.find(idp);
    if(mit != mmpErasedMapPoints.end()) return true;
    else return false;
}

Map::kfptr Map::GetFromKfBuffer(size_t KfId, size_t ClientId)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - GetFromKfBuffer " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexKfBuffer);

    idpair idp = make_pair(KfId,ClientId);
    map<idpair,kfptr>::iterator mit = mmpKfBuffer.find(idp);
    if(mit!=mmpKfBuffer.end()) return mit->second;
    else
    {
        kfptr pKF{new KeyFrame(false,this->shared_from_this(),mSysState)};
        mmpKfBuffer[idp] = pKF;
        return pKF;
    }
}

bool Map::IsInKfBuffer(size_t KfId, size_t ClientId)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - IsInKfBuffer " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexKfBuffer);

    idpair idp = make_pair(KfId,ClientId);
    map<idpair,kfptr>::iterator mit = mmpKfBuffer.find(idp);
    if(mit!=mmpKfBuffer.end()) return true;
    else return false;
}

Map::kfptr Map::SearchAndReturnFromKfBuffer(size_t KfId, size_t ClientId)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - SearchAndReturnFromKfBuffer " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexKfBuffer);

    idpair idp = make_pair(KfId,ClientId);
    map<idpair,kfptr>::iterator mit = mmpKfBuffer.find(idp);
    if(mit!=mmpKfBuffer.end()) return mit->second;
    else return nullptr;
}

void Map::DeleteFromKfBuffer(kfptr pKF)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - DeleteFromKfBuffer " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexKfBuffer);

//    idpair idp = make_pair(pKF->mnId,pKF->mClientId);
    map<idpair,kfptr>::iterator mit = mmpKfBuffer.find(pKF->mId);
    if(mit!=mmpKfBuffer.end()) mmpKfBuffer.erase(mit);

    unique_lock<mutex> lock2(mMutexErased);
    mmpErasedKeyFrames[pKF->mId] = pKF;
}

Map::mpptr Map::GetFromMpBuffer(size_t MpId, size_t ClientId)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - GetFromMpBuffer " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMpBuffer);

    idpair idp = make_pair(MpId,ClientId);
    map<idpair,mpptr>::iterator mit = mmpMpBuffer.find(idp);
    if(mit!=mmpMpBuffer.end()) return mit->second;
    else
    {
//        mpptr pMP{new MapPointBase<Types>(false,this->shared_from_this())};
        mpptr pMP{new MapPoint(false,this->shared_from_this(),mSysState)};
        mmpMpBuffer[idp] = pMP;
        return pMP;
    }
}

bool Map::IsInMpBuffer(size_t MpId, size_t ClientId)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - IsInMpBuffer " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMpBuffer);

    idpair idp = make_pair(MpId,ClientId);
    map<idpair,mpptr>::iterator mit = mmpMpBuffer.find(idp);
    if(mit!=mmpMpBuffer.end()) return true;
    else return false;
}

Map::mpptr Map::SearchAndReturnFromMpBuffer(size_t MpId, size_t ClientId)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - SearchAndReturnFromMpBuffer " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMpBuffer);

    idpair idp = make_pair(MpId,ClientId);
    map<idpair,mpptr>::iterator mit = mmpMpBuffer.find(idp);
    if(mit!=mmpMpBuffer.end()) return mit->second;
    else return nullptr;
}

void Map::DeleteFromMpBuffer(mpptr pMP)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - DeleteFromMpBuffer " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(mMutexMpBuffer);

//    idpair idp = make_pair(pMP->mnId,pMP->mClientId);
    map<idpair,mpptr>::iterator mit = mmpMpBuffer.find(pMP->mId);
    if(mit!=mmpMpBuffer.end()) mmpMpBuffer.erase(mit);

//    unique_lock<mutex> lock2(mMutexErased);
//    mmpErasedMapPoints[pMP->mId] = pMP;
}

void Map::AddCCPtr(ccptr pCC)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - AddCCPtr " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(this->mMutexCC);
    mspCC.insert(pCC);

    mspComm.insert(pCC->mpCH->GetCommPtr());
}

set<Map::ccptr> Map::GetCCPtrs()
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - GetCCPtrs " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    unique_lock<mutex> lock(this->mMutexCC);
    return mspCC;
}

sensor_msgs::PointCloud2 Map::GetMapPointsAsPclMsg(double ScaleFactor, string FrameId)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - GetMapPointsAsPclMsg " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    sensor_msgs::PointCloud2 pclMsg;
    pcl::PointCloud<pcl::PointXYZ> pclCloud;

    for(set<mpptr>::iterator sit=mspMapPoints.begin();sit!=mspMapPoints.end();++sit)
    {
        mpptr pMPi = *sit;
        if(pMPi->isBad()) continue;

        pcl::PointXYZ p;
        cv::Mat Tworld = pMPi->GetWorldPos();
//        cout << "Point Pos: "<< Tworld << endl;
        p.x = (ScaleFactor)*((double)(Tworld.at<float>(0,0)));
        p.y = (ScaleFactor)*((double)(Tworld.at<float>(0,1)));
        p.z = (ScaleFactor)*((double)(Tworld.at<float>(0,2)));
        pclCloud.points.push_back(p);
    }

    pcl::toROSMsg(pclCloud,pclMsg);

    pclMsg.header.frame_id = FrameId;
    pclMsg.header.stamp = ros::Time::now();

    return pclMsg;
}

visualization_msgs::Marker Map::GetCovGraphAsMarkerMsg(double MarkerSize, double ScaleFactor, string FrameId, string ns, int Id, double r, double g, double b)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - GetCovGraphAsMarkerMsg " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    visualization_msgs::Marker CovMsg;

    CovMsg.header.frame_id = FrameId;
    CovMsg.header.stamp = ros::Time::now();
    CovMsg.ns = ns;
    CovMsg.type = visualization_msgs::Marker::LINE_LIST;
    CovMsg.color.r = r;
    CovMsg.color.g = g;
    CovMsg.color.b = b;
    CovMsg.color.a = 1.0;
    CovMsg.action = visualization_msgs::Marker::ADD;
    CovMsg.scale.x = MarkerSize;
    CovMsg.id = Id;

//    size_t MaxVal = max(mnMaxKFid,mspKeyFrames.size())+1; //num of KFs is not necessarily equal to highest ID
    size_t MaxVal;

    if(mSysState == eSystemState::CLIENT)
//        size_t MaxVal = 2*mnMaxKFid + 1; //no commback --> works
        MaxVal = max(mnMaxKFid,mspKeyFrames.size())+1; //num of KFs is not necessarily equal to highest ID
    else if(mSysState == eSystemState::SERVER)
        MaxVal = 2*mnMaxKFidUnique + 1; //sometimes for client, KF is not already entered in map --> KF->mUniqueId > mnMaxKFidUnique
    vector<vector<bool>> CovMat(MaxVal,vector<bool>(MaxVal,false));

    size_t ConId,KfId;
    size_t count = 0;

    for(set<kfptr>::iterator sit=mspKeyFrames.begin();sit!=mspKeyFrames.end();++sit)
    {
        kfptr pKFi = *sit;
        if(pKFi->IsEmpty()) continue;

        set<kfptr> vConKFs = pKFi->GetConnectedKeyFrames();

        for(set<kfptr>::iterator sit=vConKFs.begin();sit!=vConKFs.end();++sit)
        {
            kfptr pKFcon = *sit;
            if(pKFcon->isBad()) continue;

            if(mSysState == eSystemState::CLIENT)
            {
                if(pKFi->mVisId == -1) pKFi->mVisId = count++;
                if(pKFcon->mVisId == -1) pKFcon->mVisId = count++;

                KfId = pKFi->mVisId;
                ConId = pKFcon->mVisId;
            }
            else if(mSysState == eSystemState::SERVER)
            {
                KfId = pKFi->mUniqueId;
                ConId = pKFcon->mUniqueId;
            }

            if(max(KfId,ConId) > (MaxVal-1)) //starts with 0 -> CovMat[MaxVal][MaxVal] does not exist. Yet, this should not happen...
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  Map::GetCovGraphAsMarkerMsg(...): KF ID out of bounds" << endl;
                cout << "MaxVal: " << MaxVal << endl;
                cout << "pKFi->Id: " << pKFi->mId.first << "|" << pKFi->mId.second << "|" << pKFi->mUniqueId << endl;
                cout << "pKFcon->Id: " << pKFcon->mId.first << "|" << pKFcon->mId.second << "|" << pKFcon->mUniqueId << endl;
                continue;
            }

            if(CovMat[KfId][ConId]==true || CovMat[ConId][KfId]==true) continue;

            cv::Mat T1 = pKFi->GetPoseInverse();
            cv::Mat T2 = pKFcon->GetPoseInverse();

            geometry_msgs::Point p1;
            geometry_msgs::Point p2;

            p1.x = ScaleFactor*((double)(T1.at<float>(0,3)));
            p1.y = ScaleFactor*((double)(T1.at<float>(1,3)));
            p1.z = ScaleFactor*((double)(T1.at<float>(2,3)));

            p2.x = ScaleFactor*((double)(T2.at<float>(0,3)));
            p2.y = ScaleFactor*((double)(T2.at<float>(1,3)));
            p2.z = ScaleFactor*((double)(T2.at<float>(2,3)));

            CovMsg.points.push_back(p1);
            CovMsg.points.push_back(p2);

            CovMat[KfId][ConId]=true;
            CovMat[ConId][KfId]=true;
        }
    }

    if(mSysState == eSystemState::CLIENT)
    {
        for(set<kfptr>::iterator sit=mspKeyFrames.begin();sit!=mspKeyFrames.end();++sit)
        {
            kfptr pKFi = *sit;
            if(pKFi->IsEmpty()) continue;

            if(pKFi->mVisId != -1) pKFi->mVisId = -1;

            set<kfptr> vConKFs = pKFi->GetConnectedKeyFrames();

            for(set<kfptr>::iterator sit=vConKFs.begin();sit!=vConKFs.end();++sit)
            {
                kfptr pKFcon = *sit;
                if(pKFcon->isBad()) continue;

                if(pKFcon->mVisId != -1) pKFcon->mVisId = -1;
            }
        }
    }

    return CovMsg;
}

void Map::PubCovGraphAsMarkerMsg(double MarkerSize, double ScaleFactor, string FrameId, string ns, double r, double g, double b)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - PubCovGraphAsMarkerMsg " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    std::stringstream* ss;

    string frame;
    if(FrameId == "empty") frame = mOdomFrame;
    else frame = FrameId;

    string myns;
    if(ns != "empty") myns = ns;
    else
    {
        ss = new stringstream;
        *ss << "CovGraphNative" << mMapId;
        ns = ss->str();
        delete ss;
    }

    visualization_msgs::Marker MarkerMsg = this->GetCovGraphAsMarkerMsg(MarkerSize,ScaleFactor,frame,myns,0,r,g,b);
    mPubMarker.publish(MarkerMsg);
}

void Map::PubMapPointsAsPclMsg(double ScaleFactor, string FrameId)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - PubMapPointsAsPclMsg " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

//    this->ClearMapPoints();

    ccptr pCC = *(mspCC.begin());

    std::stringstream* ss;

    string frame;
    if(FrameId == "empty") frame = mOdomFrame;
    else frame = FrameId;

//    sensor_msgs::PointCloud2 pclMsg = this->GetMapPointsAsPclMsg(ScaleFactor,frame);

    vector<pcl::PointCloud<pcl::PointXYZ>> vClouds;
    for(int i=0;i<5;++i) //msg five for changed native points
    {
        pcl::PointCloud<pcl::PointXYZ> pclCloud;
        pclCloud.clear();
        vClouds.push_back(pclCloud);
    }

    pcl::PointCloud<pcl::PointXYZRGB> pclCloudRGB;

    pcl::PointCloud<pcl::PointXYZ> CloudMultiUse;

    for(set<mpptr>::iterator sit=mspMapPoints.begin();sit!=mspMapPoints.end();++sit)
    {
        mpptr pMPi = *sit;
        if(pMPi->isBad()) continue;

        pcl::PointXYZ p;
        cv::Mat Tworld = pMPi->GetWorldPos();
//        cout << "Point Pos: "<< Tworld << endl;
        p.x = (ScaleFactor)*((double)(Tworld.at<float>(0,0)));
        p.y = (ScaleFactor)*((double)(Tworld.at<float>(0,1)));
        p.z = (ScaleFactor)*((double)(Tworld.at<float>(0,2)));

        if(pMPi->mId.second < 0 || pMPi->mId.second > 3)
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Map::PubMapPointsAsPclMsg(): pMPi->mId.second not in [0,3] -- mClientId: " << pMPi->mId.second << endl;
            throw infrastructure_ex();
        }

//        if(!pMPi->ChangedByServer())
//            vClouds[pMPi->mId.second].points.push_back(p);
//        else
//            vClouds[4].points.push_back(p);

        #ifdef HACKZ
        //if((p.x > 60 || p.y > 8) && pCC->mSysState == eSystemState::SERVER) continue;
        #endif

        if(pMPi->mbMultiUse)
            CloudMultiUse.points.push_back(p);
        else
            vClouds[pMPi->mId.second].points.push_back(p);

        if(pCC->mbUseImgCol){
            pcl::PointXYZRGB pcol;
//            uint8_t gray = pMPi->GetColor();
//            uint32_t rgb = ((uint32_t)gray << 16 | (uint32_t)gray << 8 | (uint32_t)gray);
//            uint32_t rgb = pMPi->GetColorRGB();
//            uint32_t rgb = pMPi->mRGB;
            uint32_t rgb = pMPi->mGray;
            pcol.rgb = *reinterpret_cast<float*>(&rgb);
            pcol.x = p.x;
            pcol.y = p.y;
            pcol.z = p.z;
            pclCloudRGB.points.push_back(pcol);
        }
    }

    if(!vClouds[0].points.empty())
    {
        sensor_msgs::PointCloud2 pclMsg;
        pcl::toROSMsg(vClouds[0],pclMsg);
        pclMsg.header.frame_id = FrameId;
        pclMsg.header.stamp = ros::Time::now();
        mPubMapPoints0.publish(pclMsg);
//        cout << "publishing points client 0: " << pclMsg.data.size() << endl;
    }

    if(!vClouds[1].points.empty())
    {
        sensor_msgs::PointCloud2 pclMsg;
        pcl::toROSMsg(vClouds[1],pclMsg);
        pclMsg.header.frame_id = FrameId;
        pclMsg.header.stamp = ros::Time::now();
        mPubMapPoints1.publish(pclMsg);
//        cout << "publishing points client 1: " << pclMsg.data.size() << endl;

    }

    if(!vClouds[2].points.empty())
    {
        sensor_msgs::PointCloud2 pclMsg;
        pcl::toROSMsg(vClouds[2],pclMsg);
        pclMsg.header.frame_id = FrameId;
        pclMsg.header.stamp = ros::Time::now();
        mPubMapPoints2.publish(pclMsg);
    }

    if(!vClouds[3].points.empty())
    {
        sensor_msgs::PointCloud2 pclMsg;
        pcl::toROSMsg(vClouds[3],pclMsg);
        pclMsg.header.frame_id = FrameId;
        pclMsg.header.stamp = ros::Time::now();
        mPubMapPoints3.publish(pclMsg);
    }

//    if(!vClouds[4].points.empty())
//    {
//        sensor_msgs::PointCloud2 pclMsg;
//        pcl::toROSMsg(vClouds[4],pclMsg);
//        pclMsg.header.frame_id = FrameId;
//        pclMsg.header.stamp = ros::Time::now();
//        mPubMapPointsx.publish(pclMsg);
////        cout << "publishing points native changed: " << pclMsg.data.size() << endl;
//    }

    if(!CloudMultiUse.points.empty())
    {
        sensor_msgs::PointCloud2 pclMsg;
        pcl::toROSMsg(CloudMultiUse,pclMsg);
        pclMsg.header.frame_id = FrameId;
        pclMsg.header.stamp = ros::Time::now();
        mPubMapPointsMultiUse.publish(pclMsg);
//        cout << "publishing multiuse pcl with points: " << CloudMultiUse.points.size() << endl;
//        cout << "publishing multiuse pcl with points: " << pclMsg.data.size() << endl;
    }

    if(pCC->mbUseImgCol){
        sensor_msgs::PointCloud2 pclMsg;
        pcl::toROSMsg(pclCloudRGB,pclMsg);
        pclMsg.header.frame_id = FrameId;
        pclMsg.header.stamp = ros::Time::now();
        mPubMapPointsCol.publish(pclMsg);
    }
}

void Map::PubMapPointsAsPclMsgV2(double ScaleFactor, string FrameId)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - PubMapPointsAsPclMsg " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    ccptr pCC = *(mspCC.begin());

    std::stringstream* ss;

    string frame;
    if(FrameId == "empty") frame = mOdomFrame;
    else frame = FrameId;

//    sensor_msgs::PointCloud2 pclMsg = this->GetMapPointsAsPclMsg(ScaleFactor,frame);

    pcl::PointCloud<pcl::PointXYZ> Cloud;
    pcl::PointCloud<pcl::PointXYZ> CloudMultiUse;

    for(set<mpptr>::iterator sit=mspMapPoints.begin();sit!=mspMapPoints.end();++sit)
    {
        mpptr pMPi = *sit;
        if(pMPi->isBad()) continue;

        pcl::PointXYZ p;
        cv::Mat Tworld = pMPi->GetWorldPos();
//        cout << "Point Pos: "<< Tworld << endl;
        p.x = (ScaleFactor)*((double)(Tworld.at<float>(0,0)));
        p.y = (ScaleFactor)*((double)(Tworld.at<float>(0,1)));
        p.z = (ScaleFactor)*((double)(Tworld.at<float>(0,2)));

        if(pMPi->mId.second < 0 || pMPi->mId.second > 3)
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Map::PubMapPointsAsPclMsg(): pMPi->mId.second not in [0,3] -- mClientId: " << pMPi->mId.second << endl;
            throw infrastructure_ex();
        }

//        if(pMPi->mbMultiUse) cout << "#x#x#x#x#x#x#x#" << endl;

        if(pMPi->mbMultiUse)
            CloudMultiUse.points.push_back(p);
        else
            Cloud.points.push_back(p);
    }

    if(!Cloud.points.empty())
    {
        sensor_msgs::PointCloud2 pclMsg;
        pcl::toROSMsg(Cloud,pclMsg);
        pclMsg.header.frame_id = FrameId;
        pclMsg.header.stamp = ros::Time::now();
        mPubMapPoints.publish(pclMsg);
//        cout << "publishing pcl with points: " << pclMsg.data.size() << endl;
    }

    if(!CloudMultiUse.points.empty())
    {
        sensor_msgs::PointCloud2 pclMsg;
        pcl::toROSMsg(CloudMultiUse,pclMsg);
        pclMsg.header.frame_id = FrameId;
        pclMsg.header.stamp = ros::Time::now();
        mPubMapPointsMultiUse.publish(pclMsg);
//        cout << "publishing multiuse pcl with points: " << CloudMultiUse.points.size() << endl;
//        cout << "publishing multiuse pcl with points: " << pclMsg.data.size() << endl;
    }
}

void Map::PubKeyFrames(string FrameId, string ns, double r, double g, double b)
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - PubKeyFrames " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    vector<visualization_msgs::Marker> vMsgs;
    ccptr pCC = *(mspCC.begin());
    std::stringstream* ss;

//    visualization_msgs::MarkerArray KFs;

    for(int i=0;i<5;++i) //msg five for changed native points
    {
        visualization_msgs::Marker KeyFrames;

        KeyFrames.header.frame_id = FrameId;
        KeyFrames.header.stamp = ros::Time::now();

        ss = new stringstream;
        *ss << ns << "KFs" << i << "Map" << mMapId;
        KeyFrames.ns = ss->str();

//            KeyFrames.id=pCC->mClientId;
        KeyFrames.id=0;        
        KeyFrames.type = visualization_msgs::Marker::LINE_LIST;
        KeyFrames.scale.x=pCC->mCamLineSize;
        KeyFrames.pose.orientation.w=1.0;
        KeyFrames.action=visualization_msgs::Marker::ADD;

        if(i == 0)
        {
            KeyFrames.color.r = pCC->mCols.mc0.mr;
            KeyFrames.color.g = pCC->mCols.mc0.mg;
            KeyFrames.color.b = pCC->mCols.mc0.mb;
        }

        if(i == 1)
        {
            KeyFrames.color.r = pCC->mCols.mc1.mr;
            KeyFrames.color.g = pCC->mCols.mc1.mg;
            KeyFrames.color.b = pCC->mCols.mc1.mb;
        }

        if(i== 2)
        {
            KeyFrames.color.r = pCC->mCols.mc2.mr;
            KeyFrames.color.g = pCC->mCols.mc2.mg;
            KeyFrames.color.b = pCC->mCols.mc2.mb;
        }

        if(i == 3)
        {
            KeyFrames.color.r = pCC->mCols.mc3.mr;
            KeyFrames.color.g = pCC->mCols.mc3.mg;
            KeyFrames.color.b = pCC->mCols.mc3.mb;
        }

        if(i == 4)
        {
            KeyFrames.color.r = pCC->mCols.mcx.mr;
            KeyFrames.color.g = pCC->mCols.mcx.mg;
            KeyFrames.color.b = pCC->mCols.mcx.mb;
        }

        KeyFrames.color.a = 1.0;

        vMsgs.push_back(KeyFrames);
    }


    float fScale = static_cast<float>(pCC->mScaleFactor);
    float d = static_cast<float>(pCC->mCamSize);

    for(set<kfptr>::const_iterator sit=mspKeyFrames.begin();sit!=mspKeyFrames.end();++sit)
    {
        if(((*sit)->IsEmpty()) || ((*sit)->isBad())) continue;

        kfptr pKFi = *sit;
        size_t ClientId = pKFi->mId.second;

//        {
//            //KF as sphere
//            visualization_msgs::Marker KF;
//            KF.header.frame_id = FrameId;
//            KF.header.stamp = ros::Time::now();
//            KF.id = pKFi->mId.first;
//            KF.ns = "CovKFs";
//            KF.type = visualization_msgs::Marker::SPHERE;
//            KF.scale.x=pCC->mMarkerSphereDiameter;
//            KF.scale.y=pCC->mMarkerSphereDiameter;
//            KF.scale.z=pCC->mMarkerSphereDiameter;
//            KF.pose.orientation.w=1.0;
//            KF.action=visualization_msgs::Marker::ADD;
//            KF.color.r = 0.0f;
//            KF.color.g = 1.0f;
//            KF.color.b = 0.0f;
//            KF.color.a = 1.0f;
////            cv::Mat p = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
//            cv::Mat T = pKFi->GetPoseInverse();
////            p = T*p;
//            geometry_msgs::Point msgs_p;
//            msgs_p.x=(fScale)*T.at<float>(0,3);
//            msgs_p.y=(fScale)*T.at<float>(1,3);
//            msgs_p.z=(fScale)*T.at<float>(2,3);
//            KF.pose.position = msgs_p;
//            KFs.markers.push_back(KF);
//        }


        //Camera is a pyramid. Define in camera coordinate system
        cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
        cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
        cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
        cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
        cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

        cv::Mat Twc = pKFi->GetPoseInverse();
        cv::Mat ow = pKFi->GetCameraCenter();
        cv::Mat p1w = Twc*p1;
        cv::Mat p2w = Twc*p2;
        cv::Mat p3w = Twc*p3;
        cv::Mat p4w = Twc*p4;

        geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
        msgs_o.x=(fScale)*ow.at<float>(0);
        msgs_o.y=(fScale)*ow.at<float>(1);
        msgs_o.z=(fScale)*ow.at<float>(2);
        msgs_p1.x=(fScale)*p1w.at<float>(0);
        msgs_p1.y=(fScale)*p1w.at<float>(1);
        msgs_p1.z=(fScale)*p1w.at<float>(2);
        msgs_p2.x=(fScale)*p2w.at<float>(0);
        msgs_p2.y=(fScale)*p2w.at<float>(1);
        msgs_p2.z=(fScale)*p2w.at<float>(2);
        msgs_p3.x=(fScale)*p3w.at<float>(0);
        msgs_p3.y=(fScale)*p3w.at<float>(1);
        msgs_p3.z=(fScale)*p3w.at<float>(2);
        msgs_p4.x=(fScale)*p4w.at<float>(0);
        msgs_p4.y=(fScale)*p4w.at<float>(1);
        msgs_p4.z=(fScale)*p4w.at<float>(2);

//        if(!pKFi->ChangedByServer())
//        {
            vMsgs[ClientId].points.push_back(msgs_o);
            vMsgs[ClientId].points.push_back(msgs_p1);
            vMsgs[ClientId].points.push_back(msgs_o);
            vMsgs[ClientId].points.push_back(msgs_p2);
            vMsgs[ClientId].points.push_back(msgs_o);
            vMsgs[ClientId].points.push_back(msgs_p3);
            vMsgs[ClientId].points.push_back(msgs_o);
            vMsgs[ClientId].points.push_back(msgs_p4);
            vMsgs[ClientId].points.push_back(msgs_p1);
            vMsgs[ClientId].points.push_back(msgs_p2);
            vMsgs[ClientId].points.push_back(msgs_p2);
            vMsgs[ClientId].points.push_back(msgs_p3);
            vMsgs[ClientId].points.push_back(msgs_p3);
            vMsgs[ClientId].points.push_back(msgs_p4);
            vMsgs[ClientId].points.push_back(msgs_p4);
            vMsgs[ClientId].points.push_back(msgs_p1);
//        }
//        else
//        {
//            vMsgs[4].points.push_back(msgs_o);
//            vMsgs[4].points.push_back(msgs_p1);
//            vMsgs[4].points.push_back(msgs_o);
//            vMsgs[4].points.push_back(msgs_p2);
//            vMsgs[4].points.push_back(msgs_o);
//            vMsgs[4].points.push_back(msgs_p3);
//            vMsgs[4].points.push_back(msgs_o);
//            vMsgs[4].points.push_back(msgs_p4);
//            vMsgs[4].points.push_back(msgs_p1);
//            vMsgs[4].points.push_back(msgs_p2);
//            vMsgs[4].points.push_back(msgs_p2);
//            vMsgs[4].points.push_back(msgs_p3);
//            vMsgs[4].points.push_back(msgs_p3);
//            vMsgs[4].points.push_back(msgs_p4);
//            vMsgs[4].points.push_back(msgs_p4);
//            vMsgs[4].points.push_back(msgs_p1);
//        }
    }

    for(vector<visualization_msgs::Marker>::const_iterator vit = vMsgs.begin();vit!=vMsgs.end();++vit)
    {
        visualization_msgs::Marker msg = *vit;
        if(!msg.points.empty())
            mPubMarker.publish(msg);
    }

//    mPubMarkerArray.publish(KFs);
}

void Map::PubTrajectoryClient(string FrameId)
{
    ccptr pCC = *(mspCC.begin());

    mTraj0.points.clear();

    mTraj0.header.frame_id = FrameId;
    mTraj0.header.stamp = ros::Time::now();
    mTraj0.id = 0;
    mTraj0.ns = "NativeTraj";
    mTraj0.type = visualization_msgs::Marker::LINE_STRIP;
    mTraj0.scale.x = pCC->mCovGraphMarkerSize;
    mTraj0.action = visualization_msgs::Marker::ADD;
    mTraj0.color.a = 1.0;

    if(pCC->mClientId == 0)
    {
        mTraj0.color.r = pCC->mCols.mc0.mr;
        mTraj0.color.g = pCC->mCols.mc0.mg;
        mTraj0.color.b = pCC->mCols.mc0.mb;
    }

    if(pCC->mClientId == 1)
    {
        mTraj0.color.r = pCC->mCols.mc1.mr;
        mTraj0.color.g = pCC->mCols.mc1.mg;
        mTraj0.color.b = pCC->mCols.mc1.mb;
    }

    if(pCC->mClientId == 2)
    {
        mTraj0.color.r = pCC->mCols.mc2.mr;
        mTraj0.color.g = pCC->mCols.mc2.mg;
        mTraj0.color.b = pCC->mCols.mc2.mb;
    }

    if(pCC->mClientId == 3)
    {
        mTraj0.color.r = pCC->mCols.mc3.mr;
        mTraj0.color.g = pCC->mCols.mc3.mg;
        mTraj0.color.b = pCC->mCols.mc3.mb;
    }

    vector<visualization_msgs::Marker> vMsgs;

    std::stringstream* ss;

    for(int i=0;i<4;++i)
    {
        visualization_msgs::Marker KeyFrames;

        KeyFrames.header.frame_id = FrameId;
        KeyFrames.header.stamp = ros::Time::now();

        ss = new stringstream;
        *ss << "KFs" << i << "Map" << mMapId;
        KeyFrames.ns = ss->str();

        KeyFrames.id=0;
        KeyFrames.type = visualization_msgs::Marker::LINE_LIST;
        KeyFrames.scale.x=pCC->mCamLineSize;
        KeyFrames.pose.orientation.w=1.0;
        KeyFrames.action=visualization_msgs::Marker::ADD;

        if(i== 0)
        {
            KeyFrames.color.r = pCC->mCols.mc0.mr;
            KeyFrames.color.g = pCC->mCols.mc0.mg;
            KeyFrames.color.b = pCC->mCols.mc0.mb;
        }

        if(i == 1)
        {
            KeyFrames.color.r = pCC->mCols.mc1.mr;
            KeyFrames.color.g = pCC->mCols.mc1.mg;
            KeyFrames.color.b = pCC->mCols.mc1.mb;
        }

        if(i== 2)
        {
            KeyFrames.color.r = pCC->mCols.mc2.mr;
            KeyFrames.color.g = pCC->mCols.mc2.mg;
            KeyFrames.color.b = pCC->mCols.mc2.mb;
        }

        if(i == 3)
        {
            KeyFrames.color.r = pCC->mCols.mc3.mr;
            KeyFrames.color.g = pCC->mCols.mc3.mg;
            KeyFrames.color.b = pCC->mCols.mc3.mb;
        }

        if(i == 4)
        {
            KeyFrames.color.r = pCC->mCols.mcx.mr;
            KeyFrames.color.g = pCC->mCols.mcx.mg;
            KeyFrames.color.b = pCC->mCols.mcx.mb;
        }

        KeyFrames.color.a = 1.0;

        vMsgs.push_back(KeyFrames);
    }

    float fScale = static_cast<float>(pCC->mScaleFactor);
    float d = static_cast<float>(pCC->mCamSize);

    vector<kfptr> vTraj;
    vTraj.resize(mnMaxKFid+1,nullptr);

    for(set<kfptr>::const_iterator sit=mspKeyFrames.begin();sit!=mspKeyFrames.end();++sit)
    {
        if(((*sit)->IsEmpty()) || ((*sit)->isBad())) continue;

        kfptr pKFi = *sit;
        size_t ClientId = pKFi->mId.second;

        if(ClientId == pCC->mClientId)
        {
            if(pKFi->mId.first > mnMaxKFid)
            {
                cout << "\033[1;33m!!! WARN !!!\033[0m Usage of \"Map::PubTrajectory(...)\" pKFi->mId.first > mnMaxKFid" << endl;
                continue;
            }

            vTraj[pKFi->mId.first] = pKFi;
        }
        else
        {
            //Camera is a pyramid. Define in camera coordinate system
            cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
            cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
            cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
            cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
            cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

            cv::Mat Twc = pKFi->GetPoseInverse();
            cv::Mat ow = pKFi->GetCameraCenter();
            cv::Mat p1w = Twc*p1;
            cv::Mat p2w = Twc*p2;
            cv::Mat p3w = Twc*p3;
            cv::Mat p4w = Twc*p4;

            geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
            msgs_o.x=(fScale)*ow.at<float>(0);
            msgs_o.y=(fScale)*ow.at<float>(1);
            msgs_o.z=(fScale)*ow.at<float>(2);
            msgs_p1.x=(fScale)*p1w.at<float>(0);
            msgs_p1.y=(fScale)*p1w.at<float>(1);
            msgs_p1.z=(fScale)*p1w.at<float>(2);
            msgs_p2.x=(fScale)*p2w.at<float>(0);
            msgs_p2.y=(fScale)*p2w.at<float>(1);
            msgs_p2.z=(fScale)*p2w.at<float>(2);
            msgs_p3.x=(fScale)*p3w.at<float>(0);
            msgs_p3.y=(fScale)*p3w.at<float>(1);
            msgs_p3.z=(fScale)*p3w.at<float>(2);
            msgs_p4.x=(fScale)*p4w.at<float>(0);
            msgs_p4.y=(fScale)*p4w.at<float>(1);
            msgs_p4.z=(fScale)*p4w.at<float>(2);

            vMsgs[ClientId].points.push_back(msgs_o);
            vMsgs[ClientId].points.push_back(msgs_p1);
            vMsgs[ClientId].points.push_back(msgs_o);
            vMsgs[ClientId].points.push_back(msgs_p2);
            vMsgs[ClientId].points.push_back(msgs_o);
            vMsgs[ClientId].points.push_back(msgs_p3);
            vMsgs[ClientId].points.push_back(msgs_o);
            vMsgs[ClientId].points.push_back(msgs_p4);
            vMsgs[ClientId].points.push_back(msgs_p1);
            vMsgs[ClientId].points.push_back(msgs_p2);
            vMsgs[ClientId].points.push_back(msgs_p2);
            vMsgs[ClientId].points.push_back(msgs_p3);
            vMsgs[ClientId].points.push_back(msgs_p3);
            vMsgs[ClientId].points.push_back(msgs_p4);
            vMsgs[ClientId].points.push_back(msgs_p4);
            vMsgs[ClientId].points.push_back(msgs_p1);
        }
    }

    for(int it=0;it<vTraj.size();++it)
    {
        kfptr pKFi = vTraj[it];

        if(!pKFi)
            continue;

        cv::Mat T = pKFi->GetPoseInverse();

        geometry_msgs::Point p;

        p.x = fScale*(T.at<float>(0,3));
        p.y = fScale*(T.at<float>(1,3));
        p.z = fScale*(T.at<float>(2,3));

        mTraj0.points.push_back(p);
    }

    mPubMarker.publish(mTraj0);

    for(vector<visualization_msgs::Marker>::const_iterator vit = vMsgs.begin();vit!=vMsgs.end();++vit)
    {
        visualization_msgs::Marker msg = *vit;
        if(!msg.points.empty())
            mPubMarker.publish(msg);
    }

    delete ss;
}

void Map::PubTrajectoryServer(string FrameId)
{
//    if(mspCC.size() < 2)
//        return;

    ccptr pCC = *(mspCC.begin());
    float fScale = static_cast<float>(pCC->mScaleFactor);

    mTraj0.points.clear();
    mTraj1.points.clear();
    mTraj2.points.clear();
    mTraj3.points.clear();

    mTraj0.header.frame_id = FrameId;
    mTraj0.header.stamp = ros::Time::now();
    mTraj0.id = 0;
    mTraj0.ns = "Traj0";
    mTraj0.type = visualization_msgs::Marker::LINE_STRIP;
    mTraj0.scale.x = pCC->mCovGraphMarkerSize;
    mTraj0.action = visualization_msgs::Marker::ADD;
    mTraj0.color.a = 1.0;
    mTraj0.color.r = pCC->mCols.mc0.mr;
    mTraj0.color.g = pCC->mCols.mc0.mg;
    mTraj0.color.b = pCC->mCols.mc0.mb;

    mTraj1.header.frame_id = FrameId;
    mTraj1.header.stamp = ros::Time::now();
    mTraj1.id = 0;
    mTraj1.ns = "Traj1";
    mTraj1.type = visualization_msgs::Marker::LINE_STRIP;
//    mTraj1.type = visualization_msgs::Marker::LINE_LIST;
    mTraj1.scale.x = pCC->mCovGraphMarkerSize;
    mTraj1.action = visualization_msgs::Marker::ADD;
    mTraj1.color.a = 1.0;
    mTraj1.color.r = pCC->mCols.mc1.mr;
    mTraj1.color.g = pCC->mCols.mc1.mg;
    mTraj1.color.b = pCC->mCols.mc1.mb;

    mTraj2.header.frame_id = FrameId;
    mTraj2.header.stamp = ros::Time::now();
    mTraj2.id = 0;
    mTraj2.ns = "Traj2";
    mTraj2.type = visualization_msgs::Marker::LINE_STRIP;
//    mTraj2.type = visualization_msgs::Marker::LINE_LIST;
    mTraj2.scale.x = pCC->mCovGraphMarkerSize;
    mTraj2.action = visualization_msgs::Marker::ADD;
    mTraj2.color.a = 1.0;
    mTraj2.color.r = pCC->mCols.mc2.mr;
    mTraj2.color.g = pCC->mCols.mc2.mg;
    mTraj2.color.b = pCC->mCols.mc2.mb;

    mTraj3.header.frame_id = FrameId;
    mTraj3.header.stamp = ros::Time::now();
    mTraj3.id = 0;
    mTraj3.ns = "Traj3";
    mTraj3.type = visualization_msgs::Marker::LINE_STRIP;
//    mTraj3.type = visualization_msgs::Marker::SPHERE_LIST;
//    mTraj3.scale.x = pCC->mCovGraphMarkerSize;
    mTraj3.scale.x = pCC->mCovGraphMarkerSize;
//    mTraj3.scale.y = pCC->mCovGraphMarkerSize;
//    mTraj3.scale.z = pCC->mCovGraphMarkerSize;
//    mTraj3.pose.orientation.w = 1.0;
    mTraj3.action = visualization_msgs::Marker::ADD;
    mTraj3.color.a = 1.0;
    mTraj3.color.r = pCC->mCols.mc3.mr;
    mTraj3.color.g = pCC->mCols.mc3.mg;
    mTraj3.color.b = pCC->mCols.mc3.mb;

    vector<vector<kfptr>> vvTraj;
    vector<size_t> vMaxId = vector<size_t>(4,0);

    vvTraj.resize(4,vector<kfptr>(mnMaxKFid+1,nullptr));

    vector<visualization_msgs::Marker> vMsgs;
    vMsgs.resize(4);
    vMsgs[0] = mTraj0;
    vMsgs[1] = mTraj1;
    vMsgs[2] = mTraj2;
    vMsgs[3] = mTraj3;

//    vTraj.resize(mnMaxKFidUnique+1,nullptr);

    for(set<kfptr>::const_iterator sit=mspKeyFrames.begin();sit!=mspKeyFrames.end();++sit)
    {
        if(((*sit)->IsEmpty()) || ((*sit)->isBad())) continue;

        kfptr pKFi = *sit;
        size_t ClientId = pKFi->mId.second;

        if(pKFi->mId.first > mnMaxKFid)
        {
            cout << "\033[1;33m!!! WARN !!!\033[0m Usage of \"Map::PubTrajectoryServer(...)\" pKFi->mId.first > mnMaxKFid" << endl;
            continue;
        }

        if(ClientId == 0)
        {
            vvTraj[0][pKFi->mId.first] = pKFi;
            if(vMaxId[0] < pKFi->mId.first)
                vMaxId[0] = pKFi->mId.first;
        }

        if(ClientId == 1)
        {
            vvTraj[1][pKFi->mId.first] = pKFi;
            if(vMaxId[1] < pKFi->mId.first)
                vMaxId[1] = pKFi->mId.first;
        }

        if(ClientId == 2)
        {
            vvTraj[2][pKFi->mId.first] = pKFi;
            if(vMaxId[2] < pKFi->mId.first)
                vMaxId[2] = pKFi->mId.first;
        }

        if(ClientId == 3)
        {
            vvTraj[3][pKFi->mId.first] = pKFi;
            if(vMaxId[3] < pKFi->mId.first)
                vMaxId[3] = pKFi->mId.first;
        }
    }

    for(int ito = 0;ito<4;++ito)
    {
        vector<kfptr> vTraj = vvTraj[ito];

        if(vTraj.empty())
            continue;

        for(int iti = 0;iti<=vMaxId[ito];++iti)
        {
            kfptr pKFi = vTraj[iti];

            if(!pKFi)
                continue;

            cv::Mat T = pKFi->GetPoseInverse();

            geometry_msgs::Point p;

            p.x = fScale*(T.at<float>(0,3));
            p.y = fScale*(T.at<float>(1,3));
            p.z = fScale*(T.at<float>(2,3));

            vMsgs[ito].points.push_back(p);
        }
    }

    for(vector<visualization_msgs::Marker>::const_iterator vit = vMsgs.begin();vit!=vMsgs.end();++vit)
    {
        visualization_msgs::Marker msg = *vit;
        if(!msg.points.empty())
        {
//            cout  << "publishing trajectory with " << msg.points.size() << " points" << endl;
            mPubMarker.publish(msg);
        }
    }
}

void Map::ClearTrajectories()
{
    visualization_msgs::Marker Traj0,Traj1,Traj2,Traj3;

//    Traj0.header.frame_id = FrameId;
    Traj0.header.stamp = ros::Time::now();
//    Traj0.id = 0;
//    Traj0.ns = "Traj0";
    Traj0.action = 3;

//    Traj1.header.frame_id = FrameId;
    Traj1.header.stamp = ros::Time::now();
//    Traj1.id = 0;
//    Traj1.ns = "Traj1";
    Traj1.action = 3;

//    Traj2.header.frame_id = FrameId;
    Traj2.header.stamp = ros::Time::now();
//    Traj2.id = 0;
//    Traj2.ns = "Traj2";
    Traj2.action = 3;

//    Traj3.header.frame_id = FrameId;
    Traj3.header.stamp = ros::Time::now();
//    Traj3.id = 0;
//    Traj3.ns = "Traj3";
    Traj3.action = 3;

    mPubMarker.publish(Traj0);
}

//void Map::ClearKeyFrames(string FrameId, string ns)
void Map::ClearKeyFrames()
{
    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - ClearKeyFrames " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

//    visualization_msgs::MarkerArray MArrayMsg;

    visualization_msgs::Marker MarkerMsg;

//    MarkerMsg.header.frame_id = FrameId;
    MarkerMsg.header.frame_id = "world";
    MarkerMsg.header.stamp = ros::Time::now();
//    MarkerMsg.ns = ns;
    MarkerMsg.id = 0;
    MarkerMsg.action = 3;

//    std::stringstream* ss;

//    for(int i=0;i<4;++i)
//    {
//        ss = new stringstream;
//        *ss << "KFs" << i << "Map" << mMapId;
//        MarkerMsg.ns = ss->str();

//        mPubMarker.publish(MarkerMsg);
//    }

//    delete ss;

    mPubMarker.publish(MarkerMsg);


//    MArrayMsg.markers.push_back(MarkerMsg);

//    mPubMarkerArray.publish(MArrayMsg);
}

void Map::ClearMapPoints()
{
    //not nice, but clouds do not seem to offer good method to clear screen, liek e.g. markers

    #ifndef PERFORMANCE
    if(mbOutdated)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m  use of outdated map - PubMapPointsAsPclMsg " << endl;
        cout << "Associated clients:" << endl;
        for(set<size_t>::const_iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit << endl;

        throw estd::infrastructure_ex();
    }
    #endif

    pcl::PointCloud<pcl::PointXYZ> pclCloud;

    for(set<mpptr>::iterator sit=mspMapPoints.begin();sit!=mspMapPoints.end();++sit)
    {
        pcl::PointXYZ p;
        p.x = 0.0;
        p.y = 0.0;
        p.z = 0.0;
        pclCloud.points.push_back(p);
    }

    sensor_msgs::PointCloud2 pclMsg;
    pcl::toROSMsg(pclCloud,pclMsg);
    pclMsg.header.frame_id = "world";
    pclMsg.header.stamp = ros::Time::now();
    mPubMapPoints0.publish(pclMsg);
    mPubMapPoints1.publish(pclMsg);
    mPubMapPoints2.publish(pclMsg);
    mPubMapPoints3.publish(pclMsg);
    mPubMapPointsMultiUse.publish(pclMsg);
}

int Map::GetLockSleep()
{
    ccptr pCC = *(mspCC.begin());return pCC->mLockSleep;
}

}
