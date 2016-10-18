#include <macslam/MapPoint.h>

namespace macslam {

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const cv::Mat &Pos, kfptr pRefKF, mapptr pMap, size_t ClientId, commptr pComm, eSystemState SysState, size_t UniqueId)
    : mFirstKfId(pRefKF->mId), mFirstFrame(pRefKF->mFrameId), mUniqueId(UniqueId),
      nObs(0), mTrackReferenceForFrame(defpair),mLastFrameSeen(defpair),
      mpReplaced(nullptr), mfMinDistance(0), mfMaxDistance(0),
      mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
      mpMap(pMap),
//      mpComm(pComm),
      mbIsEmpty(false), mbPoseLock(false),mbPoseChanged(false), mbSentOnce(false),mbInOutBuffer(false),mbDescriptorChanged(false),mbNormalAndDepthChanged(false),
      mLoopPointForKF_LC(defpair), mCorrectedByKF_LC(defpair),mCorrectedReference_LC(-1),
      mLoopPointForKF_MM(defpair), mCorrectedByKF_MM(defpair),mCorrectedReference_MM(-1),
      mBAGlobalForKF(defpair),mBALocalForKF(defpair),
      mFuseCandidateForKF(defpair),
      mSysState(SysState),mbDoNotReplace(false),mbOmitSending(false),
      mbChangedByServer(false),mbLoopCorrected(false),mbMultiUse(false)
//      mbDescriptorSet(false),mbNormalAndDepthSet(false)
{
    mspComm.insert(pComm);

    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
//    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    while(!mpMap->LockPointCreation()){usleep(mpMap->GetLockSleep());}
    mId=make_pair(nNextId++,ClientId);
    mpMap->UnLockPointCreation();

    if(mUniqueId == 0 && !mId.first == 0) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::MapPoint(..): mUniqueId not set" << endl;
}

MapPoint::MapPoint(const cv::Mat &Pos, mapptr pMap, frameptr pFrame, const int &idxF, commptr pComm, eSystemState SysState, size_t UniqueId)
    : mFirstKfId(defpair),mFirstFrame(pFrame->mId), mUniqueId(UniqueId),
      nObs(0), mTrackReferenceForFrame(defpair), mLastFrameSeen(defpair),
      mpReplaced(nullptr), mfMinDistance(0), mfMaxDistance(0),
      mpRefKF(nullptr), mnVisible(1), mnFound(1), mbBad(false),
      mpMap(pMap),
//      mpComm(pComm),
      mbIsEmpty(false), mbPoseLock(false),mbPoseChanged(false), mbSentOnce(false),mbInOutBuffer(false),mbDescriptorChanged(false),mbNormalAndDepthChanged(false),
      mLoopPointForKF_LC(defpair), mCorrectedByKF_LC(defpair),mCorrectedReference_LC(-1),
      mLoopPointForKF_MM(defpair), mCorrectedByKF_MM(defpair),mCorrectedReference_MM(-1),
      mBAGlobalForKF(defpair),mBALocalForKF(defpair),
      mFuseCandidateForKF(defpair),
      mSysState(SysState),mbDoNotReplace(false),mbOmitSending(false),
      mbChangedByServer(false),mbLoopCorrected(false),mbMultiUse(false)
//      mbDescriptorSet(false),mbNormalAndDepthSet(false)
{
    mspComm.insert(pComm);

    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    //cout << "MapPoint() -> dist: " << dist << endl;
    const int level = pFrame->mvKeysUn[idxF].octave;
    //cout << "MapPoint() -> level: " << level << endl;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
//    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    while(!mpMap->LockPointCreation()){usleep(mpMap->GetLockSleep());}
    mId=make_pair(nNextId++,pFrame->mId.second);
    mpMap->UnLockPointCreation();

    if(mUniqueId == 0 && !mId.first == 0) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::MapPoint(..): mUniqueId not set" << endl;
}

MapPoint::MapPoint(bool NonUsedValue, mapptr pMap, eSystemState SysState) //the "NonUsedValue" is for safety, to not accidentally construct emtpy objects
    : mpMap(pMap),
//      mpComm(nullptr),
      mId(defpair), mFirstKfId(defpair), mUniqueId(defid),
      mbIsEmpty(true),mbBad(true), mbPoseLock(false), nObs(0),
      mnFound(0),mnVisible(0),
      mSysState(SysState),mbDoNotReplace(false),mbOmitSending(false),
      mbChangedByServer(false),mbLoopCorrected(false),mbMultiUse(false)
{
    //pass map to MP to be able to erase itself from MP buffer
}

MapPoint& MapPoint::operator=(MapPoint&& rhs)
{
      mpMap = rhs.mpMap;
//      mpComm = rhs.mpComm;
      mspComm = rhs.mspComm;
      mbSentOnce = move(rhs.mbSentOnce); mId = move(rhs.mId); mFirstKfId = move(rhs.mFirstKfId);mUniqueId = move(rhs.mUniqueId),
      mWorldPos = move(rhs.mWorldPos);mObservations = rhs.mObservations;mNormalVector = move(rhs.mNormalVector);mDescriptor = move(rhs.mDescriptor);
      mpRefKF = rhs.mpRefKF;mpReplaced = rhs.mpReplaced;mnVisible = move(rhs.mnVisible);mnFound = move(rhs.mnFound);mbBad = move(rhs.mbBad);mfMinDistance = move(rhs.mfMinDistance);mfMaxDistance = move(rhs.mfMaxDistance);
      mbIsEmpty = move(rhs.mbIsEmpty);
      mbPoseLock = move(rhs.mbPoseLock);
      mObservationsLock = rhs.mObservationsLock;
      nObs = move(rhs.nObs);

      mLoopPointForKF_LC=move(rhs.mLoopPointForKF_LC);
      mCorrectedByKF_LC=move(rhs.mCorrectedByKF_LC);
      mCorrectedReference_LC=move(rhs.mCorrectedReference_LC);
//      mBAGlobalForKF_LC=move(rhs.mBAGlobalForKF_LC);

      mLoopPointForKF_MM=move(rhs.mLoopPointForKF_MM);
      mCorrectedByKF_MM=move(rhs.mCorrectedByKF_MM);
      mCorrectedReference_MM=move(rhs.mCorrectedReference_MM);

//      mBAGlobalForKF_MM=move(rhs.mBAGlobalForKF_MM);
      mBAGlobalForKF=move(rhs.mBAGlobalForKF);
//      mnLoopPointForKFClientId = move(rhs.mnLoopPointForKFClientId);
//      mnCorrectedByKFClientId = move(rhs.mnCorrectedByKFClientId);
//      mnCorrectedReferenceClientId = move(rhs.mnCorrectedReferenceClientId);
      mSysState = rhs.mSysState;
      mbDoNotReplace = move(rhs.mbDoNotReplace);
      mbOmitSending = move(rhs.mbOmitSending);

//      mbDescriptorSet = move(rhs.mbDescriptorSet);
//      mbNormalAndDepthSet = move(rhs.mbNormalAndDepthSet);

      mbInOutBuffer = move(rhs.mbInOutBuffer);
      mbChangedByServer = move(rhs.mbChangedByServer);
      mbLoopCorrected = move(rhs.mbLoopCorrected);
      mbMultiUse = move(rhs.mbMultiUse);
}

MapPoint::MapPoint(macslam_msgs::macMapPoint* pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap, g2o::Sim3 mg2oS_loop, mapptr pMap, commptr pComm, eSystemState SysState, size_t UniqueId)
    : mId(make_pair(pMsg->mnId,pMsg->mClientId)), mFirstKfId(make_pair(pMsg->mnFirstKFid,pMsg->mnFirstKfClientId)), mUniqueId(UniqueId),
      nObs(0),mpReplaced(nullptr),
      mpMap(pMap),
//      mpComm(pComm),
      mbIsEmpty(false), mbPoseLock(false),mbPoseChanged(false), mbSentOnce(false),mbInOutBuffer(false),mbDescriptorChanged(false),mbNormalAndDepthChanged(false),
      mLoopPointForKF_LC(defpair), mCorrectedByKF_LC(defpair),mCorrectedReference_LC(defid),
      mLoopPointForKF_MM(defpair), mCorrectedByKF_MM(defpair),mCorrectedReference_MM(defid),
      mBAGlobalForKF(defpair),mBALocalForKF(defpair),
      mFuseCandidateForKF(defpair),
      mSysState(SysState),mbDoNotReplace(false),mbOmitSending(false),
      mbChangedByServer(false),mbLoopCorrected(false),mbMultiUse(false)
//      mbDescriptorSet(true),mbNormalAndDepthSet(true)
{
    mspComm.insert(pComm);

    mbOmitSending = true;

    mbBad = pMsg->mbBad;

    mWorldPos = cv::Mat(3,1,5);

    cv::Mat P3D_wc = cv::Mat(3,1,5); //world client
    Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macMapPoint::_mWorldPos_type,float>(P3D_wc,pMsg->mWorldPos);

    Eigen::Matrix<double,3,1> eigP3D_wc = Converter::toVector3d(P3D_wc);
    Eigen::Matrix<double,3,1> eigP3D_wm = mg2oS_wcurmap_wclientmap.map(eigP3D_wc); //world map

    eigP3D_wm = mg2oS_loop.map(eigP3D_wc);

//    if(!pMsg->mbLoopCorrected)
//    {
//        eigP3D_wm = mg2oS_loop.map(eigP3D_wc);
//        mbLoopCorrected = true;
//    }
//    else
//        mbLoopCorrected = true;

    cv::Mat WorldPos = Converter::toCvMat(eigP3D_wm);
    this->SetWorldPos(WorldPos,false);

    for(int idx=0;idx<pMsg->mObservations_KFIDs.size();++idx)
    {
        kfptr pKF = mpMap->GetKfPtr(pMsg->mObservations_KFIDs[idx],pMsg->mObservations_KFClientIDs[idx]);

//        if (mSysState == eSystemState::CLIENT && pMsg->mObservations_locked[idx])
//            mObservationsLock[pKF]=true;

        if(!pKF)
        {
            if(mpMap->IsKfDeleted(pMsg->mObservations_KFIDs[idx],pMsg->mObservations_KFClientIDs[idx]))
            {
                //don't add deleted KF
//                cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mClientId << ": In \"MapPointBase::MapPoint(...)\": MP wants to add deleted KF" << endl;
                continue;
            }
            pKF = mpMap->GetFromKfBuffer(pMsg->mObservations_KFIDs[idx],pMsg->mObservations_KFClientIDs[idx]);
        }
        mObservations[pKF]=pMsg->mObservations_n[idx];

        ++nObs;
    }

//    if(nObs <= 2) cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mClientId << ": In \"MapPointBase::MapPoint(...)\": MP #" << mnId << " nObs <= 2 after constructor--> better set this point to bad" << endl;

    mNormalVector = cv::Mat(3,1,5);
    Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macMapPoint::_mNormalVector_type,float>(mNormalVector,pMsg->mNormalVector);
    mDescriptor = cv::Mat(1,32,0);
    Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macMapPoint::_mDescriptor_type,uint8_t>(mDescriptor,pMsg->mDescriptor);

//    if(nObs <= 2)

    if(pMsg->mpRefKFId == -1)
    {
        mpRefKF = nullptr;
    }
    else
    {
        size_t id = static_cast<size_t>(pMsg->mpRefKFId);
        size_t cid = static_cast<size_t>(pMsg->mpRefKFClientId);

        kfptr pKF = mpMap->GetKfPtr(id,cid);

        //new strategy: if refKF does not exist yet, use another KF
        if(pKF)
        {
            if(pKF->isBad())
                mpRefKF = nullptr;
            else
                mpRefKF = pKF;
        }
    }

//        if(pKF)
//        {
//            if(pKF->isBad())
//              mpRefKF = nullptr;
//            else
//              mpRefKF = pKF;
//        }
//        else
//        {
//            if(mpMap->IsKfDeleted(pMsg->mpRefKFId,pMsg->mpRefKFClientId))
//            {
//                //don't use deleted KF
////                cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mClientId << ": In \"MapPointBase::MapPoint(...)\": MP wants to use deleted KF as reference" << endl;
//            }
//            else
//            {
//                mpRefKF = mpMap->GetFromKfBuffer(id,cid);
//            }
//        }
//    }

    if(!mpRefKF && nObs > 0) //it ist possible that mObservations is emtpy
    {
        std::map<kfptr,size_t>::const_iterator mitRef = mObservations.begin();
        while(!mpRefKF)
        {
            if(mitRef == mObservations.end()) break;
            if(!(mitRef->first->isBad()))
                mpRefKF=mitRef->first;
            else
                ++mitRef;
        }
    }

//    if(mpRefKF->mnId == 16) cout << "Hit16" << endl;

    if(mpRefKF && mpRefKF->isBad()) cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mId.second << ": In \"MapPointBase::MapPoint(...)\": mpRefKf is bad (may be empty)" << endl;

    if(!mpRefKF)
    {
        cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mId.second << ": In \"MapPointBase::MapPoint(...)\": no mpRefKf available - workaround: set MP bad" << endl;
        cout << "nObs: " << nObs << endl;
        nObs = 0;
    }

    //Danger: Replacement thing is ignored
//    if(pMsg->mpReplaced_Id == -1) mpReplaced = nullptr;
//    else
//    {
//        size_t id = static_cast<size_t>(pMsg->mpReplaced_Id);
//        size_t cid = static_cast<size_t>(pMsg->mpReplaced_ClientId);
//        mpptr pMP = mpMap->GetMpPtr(id,cid);
//        if(pMP) mpReplaced = pMP;
//        else if(!mbBad)
//        {
////            cout << "\033[1;33m!!! WARN !!!\033[0m In MapPoint::UpdateFromMessage(...): pMsg->mpReplaced does not exist and mbBad == false" << endl;
//        }
//    }

    if(mSysState == eSystemState::CLIENT) mbDoNotReplace = true;

    mnVisible = pMsg->mnVisible;
    mnFound = pMsg->mnFound;

    mfMinDistance = pMsg->mfMinDistance;
    mfMaxDistance = pMsg->mfMaxDistance;

    mbMultiUse = pMsg->mbMultiUse;

    mbSentOnce = true; //set at the end of constructor to prevent SendMe() from sending while using constructor

    mbOmitSending = false;

//    if(mbMultiUse) cout << "#+#+#+#+#+#+#+#" << endl;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos, bool bLock)
{
    if(mbPoseLock && mSysState == eSystemState::CLIENT)
        return;

    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);

    if(bLock)
    {
        mbPoseLock = true;
    }

    if(!mbOmitSending && mbSentOnce)
    {
        mbPoseChanged = true;
        SendMe();
    }
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

MapPoint::kfptr MapPoint::GetReferenceKeyFrame()
{
     unique_lock<mutex> lock(mMutexFeatures);
     return mpRefKF;
}

void MapPoint::AddObservation(kfptr pKF, size_t idx, bool bLock)
{
    if(this->IsEmpty())
    {
        cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mId.second << ": In \"MapPointBase::AddObservation(...)\": trying to add observation to EMPTY MP" << endl;
    }
    else
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
            return;

        if(mObservationsLock.count(pKF) && mSysState == eSystemState::CLIENT)
            return;

        mObservations[pKF]=idx;

        nObs++;

        if(bLock)
        {
            mObservationsLock[pKF] = true;
        }

        if(mbSentOnce && !mbOmitSending)
        {
            if(!pKF)
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::AddObservation(...): adding nullptr" << endl;
            }
            else
            {
                mvpNewObs.push_back(pKF);
                mvpNewObsIds.push_back(idx);
                mvbNewObsErase.push_back(false);
            }
        }

        SendMe();
    }
}

void MapPoint::EraseObservation(kfptr pKF, bool bLock)
{
    if(this->IsEmpty())
    {
//        cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mClientId << ": In \"MapPointBase::EraseObservation(...)\": trying to erase observation to EMPTY MP" << endl;
        //ignore -- MP will be overwritten when non-empty constructor ist called
    }
    else
    {
//        cout << "MapPoint::EraseObservation --> mpRefKF->mnId: " << mpRefKF->mnId << endl;
//        cout << "MapPoint::EraseObservation --> pKF->mnId: " << pKF->mnId << endl;
        if(mObservationsLock.count(pKF) && mSysState == eSystemState::CLIENT)
            return;

        bool bBad=false;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            if(mObservations.count(pKF))
            {
                int idx = mObservations[pKF];

                    nObs--;

                mObservations.erase(pKF);

                if(bLock)
                {
                    mObservationsLock[pKF] = true;
                }

                if(mpRefKF==pKF)
                {
                    if(nObs > 0)
                    {
                        mpRefKF = nullptr;
                        std::map<kfptr,size_t>::const_iterator mitRef = mObservations.begin();
                        while(!mpRefKF)
                        {
                            if(mitRef == mObservations.end()) break;
                            if(!(mitRef->first->isBad()))
                                mpRefKF=mitRef->first;
                            else
                                ++mitRef;
                        }
                    }
//                        mpRefKF=mObservations.begin()->first;
                    else
                        mpRefKF=nullptr;
                }

//                if(mpRefKF->isBad() && !mpRefKF->IsEmpty()) cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mClientId << ": In \"MapPointBase::EraseObservation(...)\": mpRefKf is bad" << endl;

                // If only 2 observations or less, discard point
                if(nObs<=2)
                    bBad=true;
            }
        }

        if(bBad)
            SetBadFlag();

        if(mbSentOnce && !mbOmitSending)
        {
            mvpNewObs.push_back(pKF);
            mvpNewObsIds.push_back(defid);
            mvbNewObsErase.push_back(true);
            SendMe();
        }



        if(mpRefKF && mpRefKF->isBad() && !mbBad)
        {
            cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mId.second << ": In \"MapPointBase::EraseObservation(...)\": mpRefKf is bad (may be empty)" << endl;
            cout << "MP: " << this->mId.first << endl;
            cout << "mpRefKF: " << mpRefKF->mId.first << "|" << mpRefKF->mId.second;
        }

        if(!mpRefKF)
        {
            cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mId.second << ": In \"MapPointBase::EraseObservation(...)\": no mpRefKf available - workaround: set MP bad" << endl;
            cout << "mbBad true? " << (mbBad == true) << endl;
            if(!mbBad) SetBadFlag();
        }
    }
}

map<MapPoint::kfptr, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
//    cout << " Client " << mClientId << ": \"MapPointBase::SetBadFlag(...)\" MP #" << mnId << endl;

    unique_lock<mutex> lockMap(mMapMutex);

    map<kfptr,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<kfptr,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        kfptr pKF = mit->first;
        if(pKF->IsEmpty()) continue;
        pKF->EraseMapPointMatch(mit->second);
    }

    if(this->IsEmpty())
    {
        mpMap->DeleteFromMpBuffer(this->shared_from_this());
        mpMap->EraseMapPoint(this->shared_from_this());
    }
    else
        mpMap->EraseMapPoint(this->shared_from_this());

    if(!mbOmitSending && mbSentOnce)
    {
        SendMe();
    }
}

MapPoint::mpptr MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(mpptr pMP)
{
//    if(pMP->mnId==this->mnId && pMP->mClientId==this->mClientId) //ID-Tag
    if(pMP->mId==this->mId) //ID-Tag
        return;

    unique_lock<mutex> lockMap(mMapMutex);

    int nvisible, nfound;
    map<kfptr,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<kfptr,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        kfptr pKF = mit->first;

        if(pKF->IsEmpty()) continue;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    if(this->IsEmpty())
    {
        mpMap->DeleteFromMpBuffer(this->shared_from_this());
        mpMap->EraseMapPoint(this->shared_from_this());
    }
    else
        mpMap->EraseMapPoint(this->shared_from_this());

    if(!mbOmitSending && mbSentOnce)
    {
        SendMe();
    }
}

void MapPoint::ReplaceAndLock(mpptr pMP)
{
//    if(pMP->mnId==this->mnId && pMP->mClientId==this->mClientId) //ID-Tag
    if(pMP->mId==this->mId) //ID-Tag
        return;

    unique_lock<mutex> lockMap(mMapMutex);

    int nvisible, nfound;
    map<kfptr,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<kfptr,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        kfptr pKF = mit->first;

        if(pKF->IsEmpty()) continue;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP,true);
            pMP->AddObservation(pKF,mit->second,true);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second,true);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    if(this->IsEmpty())
    {
        mpMap->DeleteFromMpBuffer(this->shared_from_this());
        mpMap->EraseMapPoint(this->shared_from_this());
    }
    else
        mpMap->EraseMapPoint(this->shared_from_this());

    SendMe();
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;

    if(mbSentOnce)
    {
        mnVisibleAdded += n;
    }

    if(!mbOmitSending && mbSentOnce)
    {
        SendMe();
    }
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;

    if(mbSentOnce)
    {
        mnFoundAdded += n;
    }

    if(!mbOmitSending && mbSentOnce)
    {
        SendMe();
    }
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(kfptr pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);

    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(kfptr pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<kfptr,size_t> observations;
    kfptr pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }

    if(!mpRefKF) cout <<  "Client " << mId.second << " MP " << mId.first << ": \033[1;35m!!!!! HAZARD !!!!!\033[0m MapPointBase::UpdateNormalAndDepth(...): mpRefKf is nullptr" << endl;

    if(pRefKF->IsEmpty()) return;

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<kfptr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        kfptr pKF = mit->first;

        if(pKF->isBad()) continue; //safety first, could also be empty...

        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }

//    mbNormalAndDepthSet = true;
    mbNormalAndDepthChanged = true;

    if(!mbOmitSending && mbSentOnce)
    {
        SendMe();
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, const float &logScaleFactor)
{
    float ratio;
    {
        unique_lock<mutex> lock3(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    return ceil(log(ratio)/logScaleFactor);
}

void MapPoint::SendMe()
{
//    if(!mbDescriptorSet || !mbNormalAndDepthSet) return;
//    if(mbSentOnce && !this->IsInOutBuffer()) mpComm->PassMptoComm(this->shared_from_this());

    if(mspComm.empty())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::SendMe(): no Comm ptrs" << endl;
//        throw infrastructure_ex();
        cout << "bad?: " << (mbBad == true) << endl;
        cout << "empty?: " << (mbIsEmpty == true) << endl;
        return;
    }

//    cout << "MapPoint::SendMe(): #commptrs: " << mspComm.size() << endl;

    if(mbSentOnce && !this->IsInOutBuffer())
    {
        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            commptr pComm = *sit;
            pComm->PassMptoComm(this->shared_from_this());
        }
    }
}

void MapPoint::SendMeMultiUse()
{
    cout << __func__ << __LINE__ << endl;

    if(mspComm.empty())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::SendMe(): no Comm ptrs" << endl;
//        throw infrastructure_ex();
        cout << "bad?: " << (mbBad == true) << endl;
        cout << "empty?: " << (mbIsEmpty == true) << endl;
        return;
    }

//    cout << "MapPoint::SendMe(): #commptrs: " << mspComm.size() << endl;

    cout << "mbSentOnce: " << mbSentOnce << endl;
    cout << "this->IsInOutBuffer(): " << this->IsInOutBuffer() << endl;

    if(mbSentOnce && !this->IsInOutBuffer())
    {
        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            commptr pComm = *sit;
            cout << "pass to comm" << endl;
            pComm->PassMptoComm(this->shared_from_this());
        }
    }
}

void MapPoint::ShowMyValues()
{
    cout << "******* MP VALUES *******" << endl;

    cout << "mnId: " << mId.first << endl;
    cout << "mClientId: " << mId.second << endl;
    cout << "mnFirstKFid: " << mFirstKfId.first << endl;
    cout << "mnFirstKfClientId: " << mFirstKfId.second << endl;
    cout << "mbBad: " << mbBad << endl;
    cout << "mbSentOnce: " << mbSentOnce << endl;

    cout << "mfMinDistance: " << mfMinDistance << endl;
    cout << "mfMaxDistance: " << mfMaxDistance << endl;
    cout << "mnVisible: " << mnVisible << endl;
    cout << "mnFound: " << mnFound << endl;

    cout << "mWorldPos: rows|cols|type: " << mWorldPos.rows << "|" << mWorldPos.cols << "|" << mWorldPos.type() << endl;
    cout << "mWorldPos: " << mWorldPos << endl;

    cout << "mObservations.size(): " << mObservations.size() << endl;

    cout << "mNormalVector: rows|cols|type: " << mNormalVector.rows << "|" << mNormalVector.cols << "|" << mNormalVector.type() << endl;
    cout << "mNormalVector: " << mNormalVector << endl;

    cout << "mDescriptor: rows|cols|type: " << mDescriptor.rows << "|" << mDescriptor.cols << "|" << mDescriptor.type() << endl;
    if(mDescriptor.rows > 0) cout << "mDescriptor[0]: " << static_cast<int>(mDescriptor.at<uint8_t>(0)) << endl;
    if(mDescriptor.rows > 11) cout << "mDescriptor[11]: " <<static_cast<int>(mDescriptor.at<uint8_t>(11)) << endl;

    if(mpRefKF)
    {
        cout << "mpRefKF->mnId: " << mpRefKF->mId.first << endl;
        cout << "mpRefKF->mClientId: " << mpRefKF->mId.second << endl;
    }
    else
    {
        cout << "mpRefKF: " << -1 << endl;
    }

    if(mpReplaced)
    {
        cout << "mpReplaced->mnId: " << mpReplaced->mId.first << endl;
        cout << "mpReplaced->mClientId: " << mpReplaced->mId.second << endl;
    }
    else
    {
        cout << "mpReplaced: " << -1 << endl;
    }
}

void MapPoint::ShowMyObservations()
{
    std::stringstream* ss;
    ss = new stringstream;

    for(std::map<kfptr,size_t>::iterator mit = mObservations.begin();mit!=mObservations.end();++mit)
    {
        *ss << "(" << (mit->first)->mId.first << ";" << (mit->first)->mId.second << ")" << "|" << mit->second << " $ ";
    }

    cout << "MapPoint " << mId.first << "," << mId.second << ": " << ss->str() << endl;

    delete ss;
}

void MapPoint::ReplaceMap(mapptr pNewMap)
{
    unique_lock<mutex> lockMap(mMapMutex);
    mpMap = pNewMap;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<kfptr,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<kfptr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        kfptr pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
//        mbDescriptorSet = true;
        mbDescriptorChanged = true;
    }

    if(!mbOmitSending && mbSentOnce)
    {
        SendMe();
    }
}

void MapPoint::ConvertToMessageClient(macslam_msgs::macMapPoint &Msg)
{
    if(mSysState != eSystemState::CLIENT)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }

    unique_lock<mutex> lock1(mMutexFeatures,defer_lock);
    unique_lock<mutex> lock2(mMutexPos,defer_lock);
    unique_lock<mutex> lock3(mMutexOut,defer_lock);

    lock(lock1,lock2,lock3);

    //check format
    {
//        Msg.mUpdateCount = mUpdateCount;
//        ++mUpdateCount;

//        if(mSysState == eSystemState::CLIENT)
//        {

//        }
//        else if(mSysState == eSystemState::SERVER)
//        {

//        }
//        else
//        {
//            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::ConvertToMessage(): invalid systems state: " << mSysState << endl;
//            throw infrastructure_ex();
//        }

        bool bCheckSize = (mWorldPos.rows != 3) || (mWorldPos.cols != 1) || (mNormalVector.rows != 3) || (mNormalVector.cols != 1) || (mDescriptor.rows != 1) || (mDescriptor.cols != 32);

        if(bCheckSize)
        {
            cout << "In \"MapPoint::ConvertToMessage(...)\": member matrix SIZE not as expected in ROS message" << endl;
            cout << "Id: " << mId.first << "|" << mId.second << endl;
            cout << "mWorldPos.rows: " << mWorldPos.rows << " -- expected: 3" << endl;
            cout << "mWorldPos.cols: " << mWorldPos.cols << " -- expected: 1" << endl;
            cout << "mNormalVector.rows: " << mNormalVector.rows << " -- expected: 3" << endl;
            cout << "mNormalVector.cols: " << mNormalVector.cols << " -- expected: 1" << endl;
            cout << "mDescriptor.rows: " << mDescriptor.rows << " -- expected: 1" << endl;
            cout << "mDescriptor.rows: " << mDescriptor.cols << " -- expected: 32" << endl;
            throw estd::infrastructure_ex();
        }

        bool bCheckTypes = (mDescriptor.type() != 0);

        if(bCheckTypes)
        {
            cout << "In \"KeyFrame::ConvertToMessage(...)\": member matrix TYPES not as expected in ROS message" << endl;
            throw estd::infrastructure_ex();
        }
    }

    Msg.bSentOnce = mbSentOnce;
    Msg.mbBad = mbBad;

    Msg.mnId = mId.first;
    Msg.mClientId = mId.second;
    Msg.mUniqueId = mUniqueId;
    Msg.mnFirstKFid = mFirstKfId.first;
    Msg.mnFirstKfClientId = mFirstKfId.second;

    Msg.mbPoseOnly = false;

//    Msg.mbLoopCorrected = mbLoopCorrected;

    Msg.mbMultiUse = mbMultiUse;
//    if(mbMultiUse) cout << "xxx send Msg with mbMultiUse TRUE xxx" << endl;

    if(mbSentOnce)
    {
        if(mbPoseChanged)
        {
            Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mWorldPos_type,float>(mWorldPos,Msg.mWorldPos);
            Msg.mbPoseChanged = true;
            mbPoseChanged = false;
        }

        for(int idx = 0; idx<mvpNewObs.size();++idx)
        {
            Msg.mObservations_KFIDs.push_back(mvpNewObs[idx]->mId.first);
            Msg.mObservations_KFClientIDs.push_back(mvpNewObs[idx]->mId.second);
            Msg.mObservations_n.push_back(mvpNewObsIds[idx]);
            Msg.mObservations_erase.push_back(mvbNewObsErase[idx]);
        }

        Msg.mnVisible = mnVisibleAdded;
        Msg.mnFound = mnFoundAdded;

        mvpNewObs.clear();
        mvpNewObsIds.clear();
        mvbNewObsErase.clear();
        mnVisibleAdded = 0;
        mnFoundAdded = 0;

        if(mbDescriptorChanged)
        {
            Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mDescriptor_type,uint8_t>(mDescriptor,Msg.mDescriptor);
            Msg.mbDescriptorChanged = true;
            mbDescriptorChanged = false;
        }

        if(mbNormalAndDepthChanged)
        {
            Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mNormalVector_type,float>(mNormalVector,Msg.mNormalVector);
            Msg.mfMinDistance = mfMinDistance;
            Msg.mfMaxDistance = mfMaxDistance;
            Msg.mbNormalAndDepthChanged = true;
            mbNormalAndDepthChanged = false;
        }
    }
    else
    {
        Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mWorldPos_type,float>(mWorldPos,Msg.mWorldPos);
        Msg.mbPoseChanged = true;

        for(std::map<kfptr,size_t>::const_iterator mit=mObservations.begin();mit!=mObservations.end();++mit)
        {
            Msg.mObservations_KFIDs.push_back(mit->first->mId.first);
            Msg.mObservations_KFClientIDs.push_back(mit->first->mId.second);
            Msg.mObservations_n.push_back(mit->second);
//            Msg.mObservations_n.push_back(mObservationsLock[mit->first]);
        }

        if(!mvpNewObs.empty() && mSysState==eSystemState::CLIENT) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::ConvertToMessage(...): mvpNewMapPoints not empty" << endl;

        Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mNormalVector_type,float>(mNormalVector,Msg.mNormalVector);

        Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mDescriptor_type,uint8_t>(mDescriptor,Msg.mDescriptor);

        if(mpRefKF)
        {
            Msg.mpRefKFId = static_cast<int64>(mpRefKF->mId.first);
            Msg.mpRefKFClientId = static_cast<int64>(mpRefKF->mId.second);
        }
        else
        {
            cout <<  "Client " << mId.second << " MP " << mId.first << ": \033[1;35m!!!!! HAZARD !!!!!\033[0m MapPoint::ConvertToMessage(...): mpRefKf is nullptr" << endl;
            Msg.mpRefKFId = static_cast<int64>(-1);
            Msg.mpRefKFClientId = static_cast<int64>(-1);
        }

        Msg.mnVisible = static_cast<int16_t>(mnVisible);
        Msg.mnFound = static_cast<int16_t>(mnFound);

        Msg.mbBad = mbBad;

        if(mpReplaced)
        {
            Msg.mpReplaced_Id = static_cast<int64>(mpReplaced->mId.first);
            Msg.mpReplaced_ClientId = static_cast<int64>(mpReplaced->mId.second);
        }
        else
        {
            Msg.mpReplaced_Id = static_cast<int64>(-1);
            Msg.mpReplaced_ClientId = static_cast<int64>(-1);
        }

        Msg.mfMinDistance = mfMinDistance;
        Msg.mfMaxDistance = mfMaxDistance;

        mbSentOnce = true;

        mvpNewObs.clear();
        mvpNewObsIds.clear();
        mvbNewObsErase.clear();
        mnVisibleAdded = 0;
        mnFoundAdded = 0;
        mbPoseChanged = false;
    }

    mbInOutBuffer = false;
}

void MapPoint::ConvertForeignPointToMessageClient(macslam_msgs::macMapPoint &Msg)
{
    if(mSysState != eSystemState::CLIENT)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }

    unique_lock<mutex> lock1(mMutexFeatures,defer_lock);
    unique_lock<mutex> lock2(mMutexPos,defer_lock);
    unique_lock<mutex> lock3(mMutexOut,defer_lock);

    lock(lock1,lock2,lock3);

    Msg.mnId = mId.first;
    Msg.mClientId = mId.second;

    Msg.mbMultiUse = mbMultiUse;
}

void MapPoint::ConvertToMessageServer(macslam_msgs::macMapPoint &Msg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    if(mSysState != eSystemState::SERVER)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }

    unique_lock<mutex> lock1(mMutexFeatures,defer_lock);
    unique_lock<mutex> lock2(mMutexPos,defer_lock);
    unique_lock<mutex> lock3(mMutexOut,defer_lock);

    lock(lock1,lock2,lock3);

    {
        bool bCheckSize = (mWorldPos.rows != 3) || (mWorldPos.cols != 1) || (mNormalVector.rows != 3) || (mNormalVector.cols != 1) || (mDescriptor.rows != 1) || (mDescriptor.cols != 32);

        if(bCheckSize)
        {
            cout << "In \"MapPoint::ConvertToMessage(...)\": member matrix SIZE not as expected in ROS message" << endl;
            cout << "Id: " << mId.first << "|" << mId.second << endl;
            cout << "mWorldPos.rows: " << mWorldPos.rows << " -- expected: 3" << endl;
            cout << "mWorldPos.cols: " << mWorldPos.cols << " -- expected: 1" << endl;
            cout << "mNormalVector.rows: " << mNormalVector.rows << " -- expected: 3" << endl;
            cout << "mNormalVector.cols: " << mNormalVector.cols << " -- expected: 1" << endl;
            cout << "mDescriptor.rows: " << mDescriptor.rows << " -- expected: 1" << endl;
            cout << "mDescriptor.rows: " << mDescriptor.cols << " -- expected: 32" << endl;
            throw estd::infrastructure_ex();
        }

        bool bCheckTypes = (mDescriptor.type() != 0);

        if(bCheckTypes)
        {
            cout << "In \"KeyFrame::ConvertToMessage(...)\": member matrix TYPES not as expected in ROS message" << endl;
            throw estd::infrastructure_ex();
        }
    }

    Msg.bSentOnce = mbSentOnce;
    Msg.mbBad = mbBad;

    Msg.mnId = mId.first;
    Msg.mClientId = mId.second;
    Msg.mUniqueId = mUniqueId;
    Msg.mnFirstKFid = mFirstKfId.first;
    Msg.mnFirstKfClientId = mFirstKfId.second;

    Msg.mbPoseOnly = false;

//    Msg.mbLoopCorrected = mbLoopCorrected;

    Msg.mbMultiUse = mbMultiUse;

//    Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mWorldPos_type,float>(mWorldPos,Msg.mWorldPos);

//    cv::Mat P3D_wc = cv::Mat(3,1,5); //world client
//    Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macMapPoint::_mWorldPos_type,float>(P3D_wc,pMsg->mWorldPos);

    Eigen::Matrix<double,3,1> eigP3D_wm = Converter::toVector3d(mWorldPos);
    Eigen::Matrix<double,3,1> eigP3D_wc = (mg2oS_wcurmap_wclientmap.inverse()).map(eigP3D_wm); //client map

    cv::Mat ClientPos = Converter::toCvMat(eigP3D_wc);

    Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mWorldPos_type,float>(ClientPos,Msg.mWorldPos);

//    Msg.mbPoseChanged = true;
    Msg.mbPoseChanged = mbPoseChanged;


    for(std::map<kfptr,size_t>::const_iterator mit=mObservations.begin();mit!=mObservations.end();++mit)
    {
        Msg.mObservations_KFIDs.push_back(mit->first->mId.first);
        Msg.mObservations_KFClientIDs.push_back(mit->first->mId.second);
        Msg.mObservations_n.push_back(mit->second);
//            Msg.mObservations_n.push_back(mObservationsLock[mit->first]);
    }

//    if(!mvpNewObs.empty() && mSysState==eSystemState::CLIENT) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::ConvertToMessage(...): mvpNewMapPoints not empty" << endl;

    Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mNormalVector_type,float>(mNormalVector,Msg.mNormalVector);

    Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mDescriptor_type,uint8_t>(mDescriptor,Msg.mDescriptor);

    if(mpRefKF)
    {
        Msg.mpRefKFId = static_cast<int64>(mpRefKF->mId.first);
        Msg.mpRefKFClientId = static_cast<int64>(mpRefKF->mId.second);
    }
    else
    {
        cout <<  "Client " << mId.second << " MP " << mId.first << ": \033[1;35m!!!!! HAZARD !!!!!\033[0m MapPoint::ConvertToMessage(...): mpRefKf is nullptr" << endl;
        Msg.mpRefKFId = static_cast<int64>(-1);
        Msg.mpRefKFClientId = static_cast<int64>(-1);
    }

    Msg.mnVisible = static_cast<int16_t>(mnVisible);
    Msg.mnFound = static_cast<int16_t>(mnFound);

    Msg.mbBad = mbBad;

    if(mpReplaced)
    {
        Msg.mpReplaced_Id = static_cast<int64>(mpReplaced->mId.first);
        Msg.mpReplaced_ClientId = static_cast<int64>(mpReplaced->mId.second);
    }
    else
    {
        Msg.mpReplaced_Id = static_cast<int64>(-1);
        Msg.mpReplaced_ClientId = static_cast<int64>(-1);
    }

    Msg.mfMinDistance = mfMinDistance;
    Msg.mfMaxDistance = mfMaxDistance;

    mbSentOnce = true;

//    mvpNewObs.clear();
//    mvpNewObsIds.clear();
//    mvbNewObsErase.clear();
//    mnVisibleAdded = 0;
//    mnFoundAdded = 0;

    mbPoseChanged = false;
    mbInOutBuffer = false;
}

bool MapPoint::ConvertToPosMessageServer(macslam_msgs::macMapPoint &Msg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    if(mSysState != eSystemState::SERVER)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }

    unique_lock<mutex> lock2(mMutexPos,defer_lock);
    unique_lock<mutex> lock3(mMutexOut,defer_lock);

    if(!mbPoseChanged) return false;

    Msg.mnId = mId.first;
    Msg.mClientId = mId.second;
    Msg.mUniqueId = mUniqueId;
    Msg.mbPoseOnly = true;

//    Msg.mbLoopCorrected = mbLoopCorrected;

    Msg.mbMultiUse = mbMultiUse;

//    cv::Mat P3D_wc = cv::Mat(3,1,5); //world client
//    Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macMapPoint::_mWorldPos_type,float>(P3D_wc,pMsg->mWorldPos);

    Eigen::Matrix<double,3,1> eigP3D_wm = Converter::toVector3d(mWorldPos);
    Eigen::Matrix<double,3,1> eigP3D_wc = (mg2oS_wcurmap_wclientmap.inverse()).map(eigP3D_wm); //client map

    cv::Mat ClientPos = Converter::toCvMat(eigP3D_wc);

    Converter::CvMatToMsgArrayFixedSize<macslam_msgs::macMapPoint::_mWorldPos_type,float>(ClientPos,Msg.mWorldPos);

    mbPoseChanged = false;

    mbInOutBuffer = false;

    return true;
}

void MapPoint::UpdateFromMessageServer(macslam_msgs::macMapPoint *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap, g2o::Sim3 mg2oS_loop)
{
    if(mSysState != eSystemState::SERVER)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }

    mbOmitSending = true;

//    if(mUniqueId == -1 && pMsg->mUniqueId != -1)
//    {
//        mUniqueId = pMsg->mUniqueId;
//        cout << "MapPoint::UpdateFromMessage --> set mUniqueId" << endl;
//    }

    bool bBad=false;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lockMap(mMapMutex);

        if(pMsg->mbPoseChanged)
        {
            unique_lock<mutex> lock2(mGlobalMutex);
            unique_lock<mutex> lock(mMutexPos);

            cv::Mat P3D_wc = cv::Mat(3,1,5); //world client
            Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macMapPoint::_mWorldPos_type,float>(P3D_wc,pMsg->mWorldPos);

            Eigen::Matrix<double,3,1> eigP3D_wc = Converter::toVector3d(P3D_wc);
            Eigen::Matrix<double,3,1> eigP3D_wm = mg2oS_wcurmap_wclientmap.map(eigP3D_wc); //world map

//            eigP3D_wm = mg2oS_loop.map(eigP3D_wc);

//            if(!pMsg->mbLoopCorrected)
//            {
//                eigP3D_wm = mg2oS_loop.map(eigP3D_wc);
//                mbLoopCorrected = true;
//            }

            cv::Mat WorldPos = Converter::toCvMat(eigP3D_wm);

            if(!mbPoseLock)
                WorldPos.copyTo(mWorldPos);

//            mbPoseLock = true;
        }

        if(pMsg->mbDescriptorChanged)
        {
            mDescriptor = cv::Mat(1,32,0);
            Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macMapPoint::_mDescriptor_type,uint8_t>(mDescriptor,pMsg->mDescriptor);
        }

        if(pMsg->mbNormalAndDepthChanged)
        {
            mNormalVector = cv::Mat(3,1,5);
            Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macMapPoint::_mNormalVector_type,float>(mNormalVector,pMsg->mNormalVector);

            mfMaxDistance = pMsg->mfMaxDistance;
            mfMinDistance = pMsg->mfMinDistance;
        }

//        if(pMsg->mbPoseChanged && !mbPoseLock)
//        {
////            cv::Mat newWorldPos = cv::Mat(3,1,5);
////            Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macMapPoint::_mWorldPos_type,float>(newWorldPos,pMsg->mWorldPos);
////            this->SetWorldPos(newWorldPos);

//            cv::Mat P3D_wc = cv::Mat(3,1,5); //world client
//            Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macMapPoint::_mWorldPos_type,float>(P3D_wc,pMsg->mWorldPos);

//            Eigen::Matrix<double,3,1> eigP3D_wc = Converter::toVector3d(P3D_wc);
//            Eigen::Matrix<double,3,1> eigP3D_wm = mg2oS_wcurmap_wclientmap.map(eigP3D_wc); //world map

//            cv::Mat WorldPos = Converter::toCvMat(eigP3D_wm);
//            this->SetWorldPos(WorldPos,false);
//        }

        for(int idx=0;idx<pMsg->mObservations_KFIDs.size();++idx)
        {            
            kfptr pKF = mpMap->GetKfPtr(pMsg->mObservations_KFIDs[idx],pMsg->mObservations_KFClientIDs[idx]);

            if(!pKF)
            {
                if(mpMap->IsMpDeleted(pMsg->mObservations_KFIDs[idx],pMsg->mObservations_KFClientIDs[idx]))
                {
                    continue; //don't add deleted KF
                }

                pKF = mpMap->GetFromKfBuffer(pMsg->mObservations_KFIDs[idx],pMsg->mObservations_KFClientIDs[idx]);
            }

            std::map<kfptr,bool>::iterator mit = mObservationsLock.find(pKF);
            if(mit != mObservationsLock.end())
            {
                if(mit->second) continue; //KF locked by server
            }

            if(!pMsg->mObservations_erase[idx])
            {
                //add observation, not locked by server
                if(!mObservations.count(pKF))
                {
                    mObservations[pKF]=pMsg->mObservations_n[idx];
                    nObs++;
                }
            }
            else
            {
                //delete observation, not locked
    //            bool bBad=false;
                {
                    if(mObservations.count(pKF))
                    {
                        int idx = mObservations[pKF];
            //            if(pKF->mvuRight[idx]>=0)
            //                nObs-=2;
            //            else
                            nObs--;

                        mObservations.erase(pKF);

                        if(mpRefKF==pKF && nObs > 0) //it ist possible that mObservations is emtpy
                        {
                            if(nObs > 0)
                            {
                                mpRefKF = nullptr;
                                std::map<kfptr,size_t>::const_iterator mitRef = mObservations.begin();
                                while(!mpRefKF)
                                {
                                    if(mitRef == mObservations.end()) break;
                                    if(!(mitRef->first->isBad()))
                                        mpRefKF=mitRef->first;
                                    else
                                        ++mitRef;
                                }
                            }
//                                mpRefKF=mObservations.begin()->first;
                            else
                                mpRefKF=nullptr;
                        }

//                        if(mpRefKF && mpRefKF->isBad() && !mpRefKF->IsEmpty()) cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mClientId << ": In \"MapPoint::UpdateFromMessageV2(...)\": mpRefKf is bad" << endl;

                        // If only 2 observations or less, discard point
    //                    if(nObs<=2)
    //                        bBad=true;
                    }
                }

    //            if(bBad)
    //                SetBadFlag();
            }

        }

        mnVisible+=pMsg->mnVisible;
        mnFound+=pMsg->mnFound;

        if(nObs<=1)
            bBad=true;
    }

    if(!mbMultiUse)
        mbMultiUse = pMsg->mbMultiUse;

    mbOmitSending = false;

    if(bBad)
        SetBadFlag();

    if(mpRefKF && mpRefKF->isBad() && !mbBad)
    {
        cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mId.second << ": In \"MapPoint::UpdateFromMessageV2(...)\": mpRefKf is bad (may be empty)" << endl;
        cout << "MP: " << this->mId.first << endl;
        cout << "mpRefKF: " << mpRefKF->mId.first << "|" << mpRefKF->mId.second << endl;
                    cout << "mpRefKf empty?: " << mpRefKF->IsEmpty() << endl;
    }

    if(!mpRefKF)
    {
        cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mId.second << ": In \"MapPoint::UpdateFromMessageV2(...)\": no mpRefKf available - workaround: set MP bad" << endl;
        cout << "mbBad true? " << (mbBad == true) << endl;
        if(!mbBad) SetBadFlag();
    }

//    mbLoopCorrected = pMsg->mbLoopCorrected;

//    if(mbMultiUse) cout << "#+#+#+#+#+#+#+#" << endl;
}

void MapPoint::UpdateFromMessageClient(macslam_msgs::macMapPoint *pMsg)
{
    if(mSysState != eSystemState::CLIENT)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m System Type Mismatch" << endl;
        throw infrastructure_ex();
    }

    cv::Mat P3D_wc = cv::Mat(3,1,5); //world client
    Converter::MsgArrayFixedSizeToCvMat<macslam_msgs::macMapPoint::_mWorldPos_type,float>(P3D_wc,pMsg->mWorldPos);
    P3D_wc.copyTo(P3D_wc);

    mbPoseChanged == false;

//    mbLoopCorrected = pMsg->mbLoopCorrected;

    if(!mbMultiUse)
        mbMultiUse = pMsg->mbMultiUse;

//    for(int idx=0;idx<pMsg->mObservations_KFIDs.size();++idx)
//    {
//        kfptr pKF = mpMap->GetKfPtr(pMsg->mObservations_KFIDs[idx],pMsg->mObservations_KFClientIDs[idx]);

//        if (mSysState == eSystemState::CLIENT && pMsg->mObservations_locked[idx])
//            mObservationsLock[pKF]=true;

//        if(!pKF)
//        {
//            if(mpMap->IsKfDeleted(pMsg->mObservations_KFIDs[idx],pMsg->mObservations_KFClientIDs[idx]))
//            {
//                //don't add deleted KF
////                cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mClientId << ": In \"MapPointBase::MapPoint(...)\": MP wants to add deleted KF" << endl;
//                continue;
//            }
//            pKF = mpMap->GetFromKfBuffer(pMsg->mObservations_KFIDs[idx],pMsg->mObservations_KFClientIDs[idx]);
//        }
//        mObservations[pKF]=pMsg->mObservations_n[idx];

//        ++nObs;

//        if(pMsg->mpRefKFId == -1)
//        {
//            mpRefKF = nullptr;
//        }
//        else
//        {
//            size_t id = static_cast<size_t>(pMsg->mpRefKFId);
//            size_t cid = static_cast<size_t>(pMsg->mpRefKFClientId);

//            kfptr pKF = mpMap->GetKfPtr(id,cid);

//            //new strategy: if refKF does not exist yet, use another KF
//            if(pKF)
//            {
//                if(pKF->isBad())
//                    mpRefKF = nullptr;
//                else
//                    mpRefKF = pKF;
//            }
//        }

//        if(!mpRefKF && nObs > 0) //it ist possible that mObservations is emtpy
//        {
//            std::map<kfptr,size_t>::const_iterator mitRef = mObservations.begin();
//            while(!mpRefKF)
//            {
//                if(mitRef == mObservations.end()) break;
//                if(!(mitRef->first->isBad()))
//                    mpRefKF=mitRef->first;
//                else
//                    ++mitRef;
//            }
//        }

//        mnVisible = pMsg->mnVisible;
//        mnFound = pMsg->mnFound;
//    }
}

void MapPoint::EraseInOutBuffer()
{
//    if(!mbSentOnce && mbInOutBuffer)
//    {
//        mpComm->DeleteMpFromBuffer(shared_from_this());
//        mbInOutBuffer = false;
//    }

    if(mspComm.empty())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::EraseInOutBuffer(...): no Comm ptrs" << endl;
        throw infrastructure_ex();
    }

    if(mbSentOnce && !this->IsInOutBuffer())
    {
        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            commptr pComm = *sit;
            pComm->DeleteMpFromBuffer(shared_from_this());
            mbInOutBuffer = false;
        }
    }
}

uint32_t MapPoint::GetColor()
{
    if(mpRefKF)
        return mpRefKF->GetMPColors(shared_from_this());

    kfptr pKF = mObservations.begin()->first;
    if(pKF)
      return mpRefKF->GetMPColors(shared_from_this());

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    return rgb;
}

uint32_t MapPoint::GetColorRGB()
{
    if(mpRefKF)
        return mpRefKF->GetMPColorsRGB(shared_from_this());

    kfptr pKF = mObservations.begin()->first;
    if(pKF)
      return mpRefKF->GetMPColorsRGB(shared_from_this());

    cout << "\033[1;33m!!! WARN !!!\033[0m In \"MapPoint::GetColorRGB(...)\": no reference color" << endl;

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    return rgb;
}

} //end ns
