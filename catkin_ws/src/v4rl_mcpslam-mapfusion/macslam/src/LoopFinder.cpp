#include <macslam/LoopFinder.h>

namespace macslam {

LoopFinder::LoopFinder(ccptr pCC, dbptr pDB, vocptr pVoc, mapptr pMap, LoopFinderParams LoopParams)
    : mpCC(pCC), mpKFDB(pDB), mpVoc(pVoc), mpMap(pMap), mParams(LoopParams),
//      mLastLoopKFid(0),mLastLoopKFCLientid(0),
      mbResetRequested(false),
      mbFixScale(false),mKFcount(0),
      mLoopRate(LoopParams.mLoopRate),
      mLockSleep(LoopParams.mLoopLockSleep)
{
    mVerboseMode = extVerboseMode;

    stringstream* ss;
    ss = new stringstream;
    *ss << "LoopMarker" << mpCC->mClientId;
    string PubTopicName = ss->str();
    delete ss;


    mPubMarker = mNh.advertise<visualization_msgs::Marker>(PubTopicName,10);

    if(!mpCC || !mpKFDB || !mpVoc || !mpMap)
    {
        cout << ": In \"LoopFinder::LoopFinder(...)\": nullptr ex" << endl;
        throw estd::infrastructure_ex();
    }

    cout << "Server Loop Finder Params: " << endl;
    cout << "Loop Rate: " << mLoopRate << endl;
    cout << "Loop Lock Sleep: " << mLockSleep << endl;
    cout << "Solver Iterations: " << mParams.mSolverIterations << endl;
    cout << "Matches Threshold: " << mParams.mMatchesThres << endl;
    cout << "Inliers Threshold: " << mParams.mInliersThres << endl;
    cout << "Total Matches Threshold: " << mParams.mTotalMatchesThres << endl;
    cout << "RANSAC Probability: " << mParams.mProbability << endl;
    cout << "RANSAC minInliers: " << mParams.mMinInliers << endl;
    cout << "RANSAC maxIterations: " << mParams.mMaxIterations << endl;
    cout << "Min Hits for Merge: " << mParams.mMinHitsForMerge << endl;
    cout << "GBA Iterations: " << mParams.mGBAIterations << endl;

    mpCC->mNhPrivate.param("KFNewLoopThres",mKFNewLoopThres,10);
    cout << "Loop Closure Module Client " << mpCC->mClientId << " : mKFNewLoopThres: " << mKFNewLoopThres << endl;

    #ifdef HACKZ
    mStartCount = 0;
    #endif
}

void LoopFinder::Run()
{
    while(1)
    {
        if(CheckKfQueue())
        {
//            unique_lock<mutex> lockOpt(mpCC->mMutexPlaceRec);
            while(!mpCC->LockPlaceRec()){usleep(mpCC->mLockSleep);}

//            cout << "search for loop" << endl;

            // Detect loop candidates and check covisibility consistency
            bool bDetect = DetectLoop();
            if(bDetect)
            {
                // Compute similarity transformation [sR|t]
                // In the stereo/RGBD case s=1
                bool bSim3 = ComputeSim3();
                if(bSim3)
                {
                    // Perform loop fusion and pose graph optimization
                    CorrectLoop();
                }
            }

            mpCC->UnLockPlaceRec();
        }

        ResetIfRequested();

        usleep(mLoopRate);
    }
}

bool LoopFinder::DetectLoop()
{
    if(mVerboseMode == -8) cout << "LoopFinder::DetectLoop()" << endl;

    {
        unique_lock<mutex> lock(mMutexKfInQueue);
        mpCurrentKF = mlKfInQueue.front();
        mlKfInQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    ++mKFcount;

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
//    if(mpCurrentKF->mnId<mLastLoopKFid+10)
//    if(mpCurrentKF->mnId<10)
//    if(mpMap->GetAllKeyFrames().size() < 10 || mKFcount < 10)
    if(mpMap->GetAllKeyFrames().size() < 10 || mKFcount < mKFNewLoopThres)
    {
        mpCurrentKF->SetErase();
        return false;
    }

    #ifdef HACKZ
    ++mStartCount;
    if(mStartCount < 70)
    {
        mpCurrentKF->SetErase();
        return false;
    }
    #endif

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    const vector<kfptr> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        kfptr pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = mpVoc->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;
    }

    // Query the database imposing the minimum score
    vector<kfptr> vpCandidateKFs = mpKFDB->DetectLoopCandidates(mpCurrentKF, minScore*0.8);

//    cout << __func__ << " --> vpCandidateKFs.size(): " << vpCandidateKFs.size() << endl;

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mvConsistentGroups.clear(); //Danger: Why deleting the found consistent groups in this case?
//        cout << "\033[1;33m!!! WARN !!!\033[0m In \"MapMatcher::DetectLoop()\": Why deleting the found consistent groups when no loop candidates found?" << endl;
        mpCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups; //pair <set<KF*>,int> --> int counts consistent groups found for this group
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    //mvConsistentGroups stores the last found consistent groups.

    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        kfptr pCandidateKF = vpCandidateKFs[i];

        set<kfptr> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);
        //group with candidate and connected KFs

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            set<kfptr> sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for(set<kfptr>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    //KF found that is contained in candidate's group and comparison group
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0); //For "ConsistentGroup" the "int" is initialized with 0
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    mvConsistentGroups = vCurrentConsistentGroups;

    if(mvpEnoughConsistentCandidates.empty())
    {
//        cout << "FALSE - not enough consistent candidates" << endl;
        mpCurrentKF->SetErase();
        return false;
    }
    else
    {
//        cout << "TRUE - search SIM3" << endl;
        return true;
    }

//    cout << "FALSE - end of algorithm" << endl;
    mpCurrentKF->SetErase();
    return false;
}

bool LoopFinder::ComputeSim3()
{
    if(mVerboseMode == -8) cout << "LoopFinder::ComputeSim3()" << endl;

    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<mpptr> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    if(mVerboseMode == -8) cout << "MapMatcher::ComputeSim3(): #candidates: " << nInitialCandidates << endl;

    for(int i=0; i<nInitialCandidates; i++)
    {
        kfptr pKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->SetNotErase();

        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            cout << "MapMatcher::ComputeSim3(): bad KF: " << endl;
            continue;
        }

        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);

        if(mVerboseMode == -8) cout << "MapMatcher::ComputeSim3(): SearchByBoW -> nmatches: " << nmatches << endl;

//        if(nmatches<20)
        if(nmatches<mParams.mMatchesThres)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);

//            cout << "MapMatcher::ComputeSim3() --> vvpMapPointMatches.size(): " << vvpMapPointMatches[i].size() << endl;

//            pSolver->SetRansacParameters(0.99,20,300);
            pSolver->SetRansacParameters(mParams.mProbability,mParams.mMinInliers,mParams.mMaxIterations);
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;
    }

    if(mVerboseMode == -8) cout << "MapMatcher::ComputeSim3(): #candidates with enough matches: " << nCandidates << endl;

    bool bMatch = false;

    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            kfptr pKF = mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
//            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);
            cv::Mat Scm  = pSolver->iterate(mParams.mSolverIterations,bNoMore,vbInliers,nInliers);

//            if(mVerboseMode == -8) cout << "bNoMore" << bNoMore << endl;

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
                if(mVerboseMode == -8) cout << "MapMatcher::ComputeSim3(): discard KF: " << endl;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                vector<mpptr> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<mpptr>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();
                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);

//                if(mVerboseMode == -8) cout << "MapMatcher::ComputeSim3(): vpMapPointMatches before Sim3 opt: " << vpMapPointMatches.size() << endl;

                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
//                cout << "mpCurrentKF->mId" << mpCurrentKF->mId.first << "|" << mpCurrentKF->mId.second << "  ->getPoseInverse: " << mpCurrentKF->GetPoseInverse() << endl;
//                cout << "pKF->mId" << pKF->mId.first << "|" << pKF->mId.second << "  ->getPoseInverse: " << pKF->GetPoseInverse() << endl;
//                cout << "R: " << R << " t: " << t << " s: " << s << endl;
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);

                if(mVerboseMode == -8) cout << "MapMatcher::ComputeSim3(): nInliers after opt: " << nInliers << endl;

                // If optimization is succesful stop ransacs and continue
//                if(nInliers>=20)
                if(nInliers>=mParams.mInliersThres)
                {
                    bMatch = true;
                    mpMatchedKF = pKF;
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);

                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
            }
        }
    }

    if(mVerboseMode == -8) cout << "MapMatcher::ComputeSim3(): After RANSAC: bMatch: " << bMatch << endl;

    if(!bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    vector<kfptr> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();
    for(vector<kfptr>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        kfptr pKF = *vit;
        vector<mpptr> vpMapPoints = pKF->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            mpptr pMP = vpMapPoints[i];
            if(pMP)
            {
//                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId && pMP->mnLoopPointForKFClientId!=mpCurrentKF->mClientId) //ID Tag
                if(!pMP->isBad() && pMP->mLoopPointForKF_LC!=mpCurrentKF->mId) //ID Tag
                {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->mLoopPointForKF_LC = mpCurrentKF->mId;
//                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
//                    pMP->mnLoopPointForKFClientId=mpCurrentKF->mClientId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    if(mVerboseMode == -8) cout << "MapMatcher::ComputeSim3(): nTotalMatches: " << nTotalMatches << endl;

//    if(nTotalMatches>=40)
    if(nTotalMatches>=mParams.mTotalMatchesThres)
    {
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    else
    {
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }
}

void LoopFinder::CorrectLoop()
{
    cout << "\033[1;32m!!! LOOP FOUND !!!\033[0m" << endl;
    #ifdef VISUALIZATION
    this->PublishLoopEdges();
    #endif

    set<idpair> sChangedKFs;

    bool b0 = false;
    bool b1 = false;
    bool b2 = false;
    bool b3 = false;

    unique_lock<mutex> lockComm0, lockComm1, lockComm2, lockComm3;
    unique_lock<mutex> lockMapping0, lockMapping1, lockMapping2, lockMapping3;

    set<ccptr> spCC = mpMap->GetCCPtrs();
    if(spCC.size() != mpMap->msuAssClients.size()) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": spCC.size() != mpMap->msuAssClients.size()" << endl;

    cout << "--- Lock threads" << endl;

    for(set<ccptr>::iterator sit = spCC.begin();sit!=spCC.end();++sit)
    {
        ccptr pCC = *sit;

        if(pCC->mClientId > 3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId out of bounds (" << pCC->mClientId << ")" << endl;
        if(!(mpMap->msuAssClients.count(pCC->mClientId))) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId in pCC but not in msuAssClients" << endl;
        switch(pCC->mClientId)
        {
            case(static_cast<size_t>(0)):
                if(b0) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                b0 = true;
                while(!pCC->LockComm()){usleep(mLockSleep);}
                while(!pCC->LockMapping()){usleep(mLockSleep);}
//                lockComm0 = unique_lock<mutex>(pCC->mMutexComm);
//                lockMapping0 = unique_lock<mutex>(pCC->mMutexMapping);
                break;
            case(static_cast<size_t>(1)):
                if(b1) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                b1 = true;
                while(!pCC->LockComm()){usleep(mLockSleep);}
                while(!pCC->LockMapping()){usleep(mLockSleep);}
//                lockComm1 = unique_lock<mutex>(pCC->mMutexComm);
//                lockMapping1 = unique_lock<mutex>(pCC->mMutexMapping);
                break;
            case(static_cast<size_t>(2)):
                if(b2) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                b2 = true;
                while(!pCC->LockComm()){usleep(mLockSleep);}
                while(!pCC->LockMapping()){usleep(mLockSleep);}
//                lockComm2 = unique_lock<mutex>(pCC->mMutexComm);
//                lockMapping2 = unique_lock<mutex>(pCC->mMutexMapping);
                break;
            case(static_cast<size_t>(3)):
                if(b3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                b3 = true;
                while(!pCC->LockComm()){usleep(mLockSleep);}
                while(!pCC->LockMapping()){usleep(mLockSleep);}
//                lockComm3 = unique_lock<mutex>(pCC->mMutexComm);
//                lockMapping3 = unique_lock<mutex>(pCC->mMutexMapping);
                break;
            default: cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId out of bounds" << endl;
        }
    }

    for(set<ccptr>::iterator sit = spCC.begin();sit!=spCC.end();++sit)
    {
        ccptr pCC = *sit;
        pCC->mbOptActive = true;
    }

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();

    cv::Mat Rcur = Twc.rowRange(0,3).colRange(0,3);
    cv::Mat tcur = Twc.rowRange(0,3).col(3);
    g2o::Sim3 g2oScur(Converter::toMatrix3d(Rcur),Converter::toVector3d(tcur),1.0);
    g2o::Sim3 g2oS_loop = g2oScur*mg2oScw;

    mKFcount = 0;

    // Get Map Mutex
//    unique_lock<mutex> lockMap(mpMap->mMutexMapUpdate);
    while(!mpMap->LockMapUpdate()){usleep(mpCC->mLockSleep);}

    cout << "--- transform KFs/MPs" << endl;

    for(vector<kfptr>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        kfptr pKFi = *vit;

        cv::Mat Tiw = pKFi->GetPose();

        if(pKFi!=mpCurrentKF)
        {
            cv::Mat Tic = Tiw*Twc;
            cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
            cv::Mat tic = Tic.rowRange(0,3).col(3);
            g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
            g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
            //Pose corrected with the Sim3 of the loop closure
            CorrectedSim3[pKFi]=g2oCorrectedSiw;
        }

        cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
        cv::Mat tiw = Tiw.rowRange(0,3).col(3);
        g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
        //Pose without correction
        NonCorrectedSim3[pKFi]=g2oSiw;
    }

    // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
    for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
    {
        kfptr pKFi = mit->first;
        g2o::Sim3 g2oCorrectedSiw = mit->second;
        g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

        g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

        vector<mpptr> vpMPsi = pKFi->GetMapPointMatches();
        for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
        {
            mpptr pMPi = vpMPsi[iMP];
            if(!pMPi)
                continue;
            if(pMPi->isBad())
                continue;
//            if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId && pMPi->mnCorrectedByKFClientId == mpCurrentKF->mClientId) //ID Tag
            if(pMPi->mCorrectedByKF_LC==mpCurrentKF->mId) //ID Tag
                continue;

            // Project with non-corrected pose and project back with corrected pose
            cv::Mat P3Dw = pMPi->GetWorldPos();
            Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
            Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

            cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
            pMPi->SetWorldPos(cvCorrectedP3Dw,true);
//            pMPi->mnCorrectedByKF = mpCurrentKF->mnId; //ID Tag
//            pMPi->mnCorrectedByKFClientId = mpCurrentKF->mClientId; //ID Tag
//            pMPi->mnCorrectedReference = pKFi->mnId; //ID Tag
//            pMPi->mnCorrectedReferenceClientId = pKFi->mClientId; //ID Tag
            pMPi->mCorrectedByKF_LC = mpCurrentKF->mId;
            pMPi->mCorrectedReference_LC = mpCurrentKF->mUniqueId;
            pMPi->UpdateNormalAndDepth();
        }

        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
        double s = g2oCorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

        pKFi->SetPose(correctedTiw,true);
        sChangedKFs.insert(pKFi->mId);

        // Make sure connections are updated
        pKFi->UpdateConnections();
    }

    cout << "--- update matched MPs" << endl;

    // Start Loop Fusion
    // Update matched map points and replace if duplicated
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
        {
            mpptr pLoopMP = mvpCurrentMatchedPoints[i];
            mpptr pCurMP = mpCurrentKF->GetMapPoint(i);
            if(pCurMP)
                pCurMP->ReplaceAndLock(pLoopMP);
            else
            {
                mpCurrentKF->AddMapPoint(pLoopMP,i,true);
                pLoopMP->AddObservation(mpCurrentKF,i,true);
                pLoopMP->ComputeDistinctiveDescriptors();
            }
        }
    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3,mvpLoopMapPoints);

    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<kfptr, set<kfptr> > LoopConnections;

    for(vector<kfptr>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        kfptr pKFi = *vit;
        vector<kfptr> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        pKFi->UpdateConnections();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        for(vector<kfptr>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for(vector<kfptr>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    cout << "--- optimize Ess grapgh" << endl;

    // Optimize graph
    Optimizer::OptimizeEssentialGraphLoopClosure(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

//    mpMap->PubCovGraphAsMarkerMsg(mpCC->mCovGraphMarkerSize,mpCC->mScaleFactor,mpCC->mNativeOdomFrame);
//    cout << "Essential Graph optimized" << endl;
//    cin.get();

    // Add loop edge
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);

    cout << "Starting Global Bundle Adjustment" << endl;

//    size_t nLoopKF = mpCurrentKF->mnId;
    idpair nLoopKF = mpCurrentKF->mId;

    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    //mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

    struct timeval tStart,tNow;
    double dEl;
    gettimeofday(&tStart,NULL);

    Optimizer::MapFusionGBA(mpMap,mpCC->mClientId,mParams.mGBAIterations,&mbStopGBA,nLoopKF,false);
    cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mpCC->mClientId << ": In \"LoopFinder::CorrectLoop()\": fix GBA loop KF num when fixing id tags" << endl;

    cout << "Global Bundle Adjustment finished" << endl;
    cout << "Updating map ..." << endl;

    // Correct keyframes starting at map first keyframe
    list<kfptr> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());

    while(!lpKFtoCheck.empty())
    {
        kfptr pKF = lpKFtoCheck.front();
        const set<kfptr> sChilds = pKF->GetChilds();
        cv::Mat Twc = pKF->GetPoseInverse();
        for(set<kfptr>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
        {
            kfptr pChild = *sit;
//            if(pChild->mnBAGlobalForKF!=nLoopKF)
            if(pChild->mBAGlobalForKF!=nLoopKF)
            {
                cv::Mat Tchildc = pChild->GetPose()*Twc;
                pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
//                pChild->mnBAGlobalForKF=nLoopKF;
                pChild->mBAGlobalForKF=nLoopKF;

            }
            lpKFtoCheck.push_back(pChild);
        }

        pKF->mTcwBefGBA = pKF->GetPose();
        pKF->SetPose(pKF->mTcwGBA,true);
        pKF->mbLoopCorrected = true;
        sChangedKFs.insert(pKF->mId);
        lpKFtoCheck.pop_front();
    }

    // Correct MapPoints
    const vector<mpptr> vpMPs = mpMap->GetAllMapPoints();

    for(size_t i=0; i<vpMPs.size(); i++)
    {
        mpptr pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

//        if(pMP->mnBAGlobalForKF==nLoopKF)
        if(pMP->mBAGlobalForKF==nLoopKF)
        {
            // If optimized by Global BA, just update
            pMP->SetWorldPos(pMP->mPosGBA,true);
            pMP->mbLoopCorrected = true;
        }
        else
        {
            // Update according to the correction of its reference keyframe
            kfptr pRefKF = pMP->GetReferenceKeyFrame();

            if(!pRefKF)
            {
                cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": pRefKf is nullptr" << endl;
                continue;
            }

//            if(pRefKF->mnBAGlobalForKF!=nLoopKF)
            if(pRefKF->mBAGlobalForKF!=nLoopKF)
                continue;

            // Map to non-corrected camera
            cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
            cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
            cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

            // Backproject using corrected camera
            cv::Mat Twc = pRefKF->GetPoseInverse();
            cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
            cv::Mat twc = Twc.rowRange(0,3).col(3);

            pMP->SetWorldPos(Rwc*Xc+twc,true);
            pMP->mbLoopCorrected = true;
        }
    }

    gettimeofday(&tNow,NULL);
    dEl = (tNow.tv_sec - tStart.tv_sec) + (tNow.tv_usec - tStart.tv_usec) / 1000000.0;
    cout << "Optimization Time: Agents|KFs|MPs|Time: " << mpMap->GetCCPtrs().size() << "|" << mpMap->GetAllKeyFrames().size() << "|" << mpMap->GetAllMapPoints().size() << "|" << dEl << endl;

    cout << "Map updated!" << endl;

    for(set<ccptr>::iterator sit = spCC.begin();sit!=spCC.end();++sit)
    {
        ccptr pCC = *sit;
        pCC->UnLockComm();
        pCC->UnLockMapping();
        pCC->mbOptActive = false;

        #ifdef HACKZ
        if(pCC->mClientId == mpCurrentKF->mId.second)
        {
            cout << "Updating mg2oS_loop of client " << pCC->mClientId << endl;
            pCC->mg2oS_loop = g2oS_loop;
            pCC->mbCorrectAfterLoop = true;

            cv::Mat T = Converter::toCvMat(g2oS_loop);
            cout << "Transformation: \n" << T << endl;
        }
        #endif
    }

    mpMap->UnLockMapUpdate();

    cout << "Changed KF poses: " << sChangedKFs.size() << endl;

    cout << "\033[1;32m!!! LOOP CLOSED !!!\033[0m" << endl;
    #ifdef VISUALIZATION
    this->ClearLoopEdges();
    #endif

//    mpMap->PubCovGraphAsMarkerMsg(mpCC->mCovGraphMarkerSize,mpCC->mScaleFactor,mpCC->mNativeOdomFrame);
//    cout << "Cov Graph after Loop Closure" << endl;
//    cin.get();
}

void LoopFinder::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, std::vector<mpptr> vpLoopMapPoints)
{
    ORBmatcher matcher(0.8);

    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        kfptr pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<mpptr> vpReplacePoints(vpLoopMapPoints.size(),nullptr);
        matcher.Fuse(pKF,cvScw,vpLoopMapPoints,4,vpReplacePoints);

        // Get Map Mutex
//        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        const int nLP = vpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            mpptr pRep = vpReplacePoints[i];
            if(pRep)
            {
                pRep->ReplaceAndLock(vpLoopMapPoints[i]);
            }
        }
    }
}

void LoopFinder::InsertKF(kfptr pKF)
{
    unique_lock<mutex> lock(mMutexKfInQueue);
    mlKfInQueue.push_back(pKF);
}

bool LoopFinder::CheckKfQueue()
{
    unique_lock<mutex> lock(mMutexKfInQueue);
    return (!mlKfInQueue.empty());
}

void LoopFinder::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        if(mVerboseMode > 0)  cout << "ClientHandler " << mpCC->mClientId << ": LoopFinder --> Reset" << endl;

        mlKfInQueue.clear();
//        mLastLoopKFid=0;
//        mLastLoopKFCLientid=0;
        mKFcount=0;

        vector<kfptr> vpKFs = mpMap->GetAllKeyFrames();
        for(vector<kfptr>::iterator vit=vpKFs.begin();vit!=vpKFs.end();++vit)
        {
            kfptr pKF = *vit;
            mpKFDB->erase(pKF);
        }

        mbResetRequested=false;
        if(mVerboseMode > 0)  cout << "ClientHandler " << mpCC->mClientId << ": LoopFinder --> Reset finished" << endl;
    }
}

void LoopFinder::RequestReset()
{
//    cout << "LoopFinder::RequestReset()" << endl;
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
//                cout << "break LoopFinder::RequestReset()" << endl;
                break;
            }
        }
        usleep(mLoopRate);
    }
}

void LoopFinder::PublishLoopEdges()
{
    stringstream* ss;
    ss = new stringstream;
    *ss << "LoopEdgesClient" << mpCC->mClientId << "_red";
    string ns = ss->str();
    delete ss;

    visualization_msgs::Marker MarkerMsg;

//    MarkerMsg.header.frame_id = mpCC->mNativeOdomFrame;
    MarkerMsg.header.frame_id = mpMap->mOdomFrame;
    MarkerMsg.header.stamp = ros::Time::now();
    MarkerMsg.ns = ns;
    MarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    MarkerMsg.color.r = 1.0;
    MarkerMsg.color.g = 0.0;
    MarkerMsg.color.b = 0.0;
    MarkerMsg.color.a = 1.0;
    MarkerMsg.action = visualization_msgs::Marker::ADD;
    MarkerMsg.scale.x = mpCC->mLoopEdgesMarkerSize;
    MarkerMsg.id = 1;

    cv::Mat T1 = mpCurrentKF->GetPoseInverse();
    cv::Mat T2 = mpMatchedKF->GetPoseInverse();

    geometry_msgs::Point p1;
    geometry_msgs::Point p2;

    p1.x = mpCC->mScaleFactor*((double)(T1.at<float>(0,3)));
    p1.y = mpCC->mScaleFactor*((double)(T1.at<float>(1,3)));
    p1.z = mpCC->mScaleFactor*((double)(T1.at<float>(2,3)));

    p2.x = mpCC->mScaleFactor*((double)(T2.at<float>(0,3)));
    p2.y = mpCC->mScaleFactor*((double)(T2.at<float>(1,3)));
    p2.z = mpCC->mScaleFactor*((double)(T2.at<float>(2,3)));

    MarkerMsg.points.push_back(p1);
    MarkerMsg.points.push_back(p2);

    mPubMarker.publish(MarkerMsg);
}

void LoopFinder::ClearLoopEdges()
{
    stringstream* ss;
    ss = new stringstream;
    *ss << "LoopEdgesClient" << mpCC->mClientId << "_red";
    string ns = ss->str();
    delete ss;

    visualization_msgs::Marker MarkerMsg;

//    MarkerMsg.header.frame_id = mpCC->mNativeOdomFrame;
    MarkerMsg.header.frame_id = mpMap->mOdomFrame;
    MarkerMsg.header.stamp = ros::Time::now();
    MarkerMsg.ns = ns;
    MarkerMsg.action = 3;
    MarkerMsg.id = 1;

    mPubMarker.publish(MarkerMsg);
}

}
