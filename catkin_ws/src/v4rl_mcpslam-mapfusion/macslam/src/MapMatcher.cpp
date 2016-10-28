#include <macslam/MapMatcher.h>

namespace macslam {

  MapMatcher::MapMatcher(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, dbptr pDB, vocptr pVoc, mapptr pMap0, mapptr pMap1, mapptr pMap2, mapptr pMap3, MapMatchingParams MMParams)
    : mNh(Nh), mNhPrivate(NhPrivate),
      mpKFDB(pDB), mpVoc(pVoc), mpMap0(pMap0), mpMap1(pMap1), mpMap2(pMap2), mpMap3(pMap3),
      mLastLoopKFid(0), mbResetRequested(false),
      mMMParams(MMParams),
      mbFixScale(false) {
    mNhPrivate.param("MapMatchRate", mMapMatchRate, 5000);

    if(pMap0) {
      mmpMaps[*(pMap0->msuAssClients.begin())] = pMap0;
    }

    if(pMap1) {
      mmpMaps[*(pMap1->msuAssClients.begin())] = pMap1;
    }

    if(pMap2) {
      mmpMaps[*(pMap2->msuAssClients.begin())] = pMap2;
    }

    if(pMap3) {
      mmpMaps[*(pMap3->msuAssClients.begin())] = pMap3;
    }

    if(pMap0) {
      mspMaps.insert(pMap0);
    }

    if(pMap1) {
      mspMaps.insert(pMap1);
    }

    if(pMap2) {
      mspMaps.insert(pMap2);
    }

    if(pMap3) {
      mspMaps.insert(pMap3);
    }

    mPubMarker = mNh.advertise<visualization_msgs::Marker>("MapMatcherMarkers", 10);

    mMatchMatrix =  cv::Mat::zeros(4, 4, 2);

    // Markers
    double LoopMarkerSize;
    mNhPrivate.param("MarkerSizeLoop", LoopMarkerSize, 0.1);
    mNhPrivate.param("ScaleFactor", mScaleFactor, 1.0);

    mMapMatchEdgeMsg.header.frame_id = "world";
    mMapMatchEdgeMsg.header.stamp = ros::Time::now();
    mMapMatchEdgeMsg.ns = "MapMatchEdges_red";
    mMapMatchEdgeMsg.type = visualization_msgs::Marker::LINE_LIST;
    mMapMatchEdgeMsg.color.r = 1.0;
    mMapMatchEdgeMsg.color.g = 0.0;
    mMapMatchEdgeMsg.color.b = 0.0;
    mMapMatchEdgeMsg.color.a = 1.0;
    mMapMatchEdgeMsg.action = visualization_msgs::Marker::ADD;
    mMapMatchEdgeMsg.scale.x = LoopMarkerSize;
    mMapMatchEdgeMsg.id = 1;

    cout << "Map Matcher Initialized" << endl;
  }

  void MapMatcher::Run() {
    double CovGraphMarkerSize;
    mNhPrivate.param("MarkerSizeServer", CovGraphMarkerSize, 0.001);
    MapMergerParams MergerParams(mMMParams.mMinHitsForMerge, mMMParams.mGBAIterations, mScaleFactor, CovGraphMarkerSize);
    mpMapMerger.reset(new MapMerger(MergerParams, shared_from_this(), mNh, mNhPrivate));

    while(1) {
      if(CheckKfQueue()) {
        // Detect loop candidates and check covisibility consistency
        bool bDetect = DetectLoop();

        if(bDetect) {
          // Compute similarity transformation [sR|t]
          // In the stereo/RGBD case s=1
          bool bSim3 = ComputeSim3();

          if(bSim3) {
            // Perform loop fusion and pose graph optimization
            CorrectLoop();
          }
        }
      }

      //        ResetIfRequested();

      usleep(mMapMatchRate);
    }
  }

  bool MapMatcher::DetectLoop() {
    {
      unique_lock<mutex> lock(mMutexKfInQueue);
      mpCurrentKF = mlKfInQueue.front();

      mlKfInQueue.pop_front();
      // Avoid that a keyframe can be erased while it is being process by this thread
      mpCurrentKF->SetNotErase();
    }

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    if(mpCurrentKF->mId.first < mMMParams.mKFsToSkip) {
      mpCurrentKF->SetErase();
      return false;
    }

    mpCurrMap = mpCurrentKF->GetMapptr(); //get map of KF

    if(!mpCurrMap) {
      cout << ": In \"MapMatcher::DetectLoop()\": mpCurrMap is nullptr -> KF not contained in any map" << endl;
      throw estd::infrastructure_ex();
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    const vector<kfptr> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;

    for(size_t i = 0; i < vpConnectedKeyFrames.size(); i++) {
      kfptr pKF = vpConnectedKeyFrames[i];

      if(pKF->isBad()) {
        continue;
      }

      const DBoW2::BowVector &BowVec = pKF->mBowVec;

      float score = mpVoc->score(CurrentBowVec, BowVec);

      if(score < minScore) {
        minScore = score;
      }
    }

    // Query the database imposing the minimum score
    vector<kfptr> vpCandidateKFs = mpKFDB->DetectMapMatchCandidates(mpCurrentKF, minScore, mpCurrMap);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty()) {
      mmvConsistentGroups[mpCurrMap].clear(); //Danger: Why deleting the found consistent groups in this case?
      mpCurrentKF->SetErase();
      return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups; //pair <set<KF*>,int> --> int counts consistent groups found for this group
    vector<bool> vbConsistentGroup(mmvConsistentGroups[mpCurrMap].size(), false);
    //mvConsistentGroups stores the last found consistent groups.

    for(size_t i = 0, iend = vpCandidateKFs.size(); i < iend; i++) {
      kfptr pCandidateKF = vpCandidateKFs[i];

      set<kfptr> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
      spCandidateGroup.insert(pCandidateKF);
      //group with candidate and connected KFs

      bool bEnoughConsistent = false;
      bool bConsistentForSomeGroup = false;

      for(size_t iG = 0, iendG = mmvConsistentGroups[mpCurrMap].size(); iG < iendG; iG++) {
        set<kfptr> sPreviousGroup = mmvConsistentGroups[mpCurrMap][iG].first;

        bool bConsistent = false;

        for(set<kfptr>::iterator sit = spCandidateGroup.begin(), send = spCandidateGroup.end(); sit != send; sit++) {
          if(sPreviousGroup.count(*sit)) {
            //KF found that is contained in candidate's group and comparison group
            bConsistent = true;
            bConsistentForSomeGroup = true;
            break;
          }
        }

        if(bConsistent) {
          int nPreviousConsistency = mmvConsistentGroups[mpCurrMap][iG].second;
          int nCurrentConsistency = nPreviousConsistency + 1;

          if(!vbConsistentGroup[iG]) {
            ConsistentGroup cg = make_pair(spCandidateGroup, nCurrentConsistency);
            vCurrentConsistentGroups.push_back(cg);
            vbConsistentGroup[iG] = true; //this avoid to include the same group more than once
          }

          if(nCurrentConsistency >= mnCovisibilityConsistencyTh && !bEnoughConsistent) {
            mvpEnoughConsistentCandidates.push_back(pCandidateKF);
            bEnoughConsistent = true; //this avoid to insert the same candidate more than once
          }
        }
      }

      // If the group is not consistent with any previous group insert with consistency counter set to zero
      if(!bConsistentForSomeGroup) {
        ConsistentGroup cg = make_pair(spCandidateGroup, 0); //For "ConsistentGroup" the "int" is initialized with 0
        vCurrentConsistentGroups.push_back(cg);
      }
    }

    // Update Covisibility Consistent Groups
    mmvConsistentGroups[mpCurrMap] = vCurrentConsistentGroups;

    // Add Current Keyframe to database
    //    mpKFDB->add(mpCurrentKF);

    if(mvpEnoughConsistentCandidates.empty()) {
      //        cout << "FALSE - not enough consistent candidates" << endl;
      mpCurrentKF->SetErase();
      return false;

    } else {
      //        cout << "TRUE - search SIM3" << endl;
      return true;
    }

    //    cout << "FALSE - end of algorithm" << endl;
    mpCurrentKF->SetErase();
    return false;
  }

  bool MapMatcher::ComputeSim3() {
    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75, true);

    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<mpptr> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates = 0; //candidates with enough matches

    //    cout << "MapMatcher::ComputeSim3(): #candidates: " << nInitialCandidates << endl;

    for(int i = 0; i < nInitialCandidates; i++) {
      kfptr pKF = mvpEnoughConsistentCandidates[i];

      // avoid that local mapping erase it while it is being processed in this thread
      pKF->SetNotErase();

      if(pKF->isBad()) {
        vbDiscarded[i] = true;
        cout << "MapMatcher::ComputeSim3(): bad KF: " << endl;
        continue;
      }

      int nmatches = matcher.SearchByBoW(mpCurrentKF, pKF, vvpMapPointMatches[i]);

      //        if(nmatches<20)
      if(nmatches < mMMParams.mMatchesThres) {
        vbDiscarded[i] = true;
        continue;

      } else {
        Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF, pKF, vvpMapPointMatches[i], mbFixScale);

        //            cout << "MapMatcher::ComputeSim3() --> vvpMapPointMatches.size(): " << vvpMapPointMatches[i].size() << endl;

        //            pSolver->SetRansacParameters(0.99,20,300);
        pSolver->SetRansacParameters(mMMParams.mProbability, mMMParams.mMinInliers, mMMParams.mMaxIterations);
        vpSim3Solvers[i] = pSolver;
      }

      nCandidates++;
    }

    bool bMatch = false;

    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    while(nCandidates > 0 && !bMatch) {
      for(int i = 0; i < nInitialCandidates; i++) {
        if(vbDiscarded[i]) {
          continue;
        }

        kfptr pKF = mvpEnoughConsistentCandidates[i];

        // Perform 5 Ransac Iterations
        vector<bool> vbInliers;
        int nInliers;
        bool bNoMore;

        Sim3Solver* pSolver = vpSim3Solvers[i];
        //            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);
        cv::Mat Scm  = pSolver->iterate(mMMParams.mSolverIterations, bNoMore, vbInliers, nInliers);

        // If Ransac reachs max. iterations discard keyframe
        if(bNoMore) {
          vbDiscarded[i] = true;
          nCandidates--;
        }

        // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
        if(!Scm.empty()) {
          vector<mpptr> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<mpptr>(NULL));

          for(size_t j = 0, jend = vbInliers.size(); j < jend; j++) {
            if(vbInliers[j]) {
              vpMapPointMatches[j] = vvpMapPointMatches[i][j];
            }
          }

          cv::Mat R = pSolver->GetEstimatedRotation();
          cv::Mat t = pSolver->GetEstimatedTranslation();
          const float s = pSolver->GetEstimatedScale();
          matcher.SearchBySim3(mpCurrentKF, pKF, vpMapPointMatches, s, R, t, 7.5);

          g2o::Sim3 gScm(Converter::toMatrix3d(R), Converter::toVector3d(t), s);
          const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);

          // If optimization is succesful stop ransacs and continue
          //                if(nInliers>=20)
          if(nInliers >= mMMParams.mInliersThres) {
            bMatch = true;
            mpMatchedKF = pKF;
            g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()), Converter::toVector3d(pKF->GetTranslation()), 1.0);
            mg2oScw = gScm * gSmw;
            mScw = Converter::toCvMat(mg2oScw);

            mvpCurrentMatchedPoints = vpMapPointMatches;
            break;
          }
        }
      }
    }

    if(!bMatch) {
      for(int i = 0; i < nInitialCandidates; i++) {
        mvpEnoughConsistentCandidates[i]->SetErase();
      }

      mpCurrentKF->SetErase();
      return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    vector<kfptr> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();

    for(vector<kfptr>::iterator vit = vpLoopConnectedKFs.begin(); vit != vpLoopConnectedKFs.end(); vit++) {
      kfptr pKF = *vit;
      vector<mpptr> vpMapPoints = pKF->GetMapPointMatches();

      for(size_t i = 0, iend = vpMapPoints.size(); i < iend; i++) {
        mpptr pMP = vpMapPoints[i];

        if(pMP) {
          //                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId && pMP->mnLoopPointForKFClientId!=mpCurrentKF->mClientId) //ID Tag
          if(!pMP->isBad() && pMP->mLoopPointForKF_MM != mpCurrentKF->mId) { //ID Tag
            mvpLoopMapPoints.push_back(pMP);
            pMP->mLoopPointForKF_MM = mpCurrentKF->mId;
            //                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
            //                    pMP->mnLoopPointForKFClientId=mpCurrentKF->mClientId;
          }
        }
      }
    }

    // Find more matches projecting with the computed Sim3
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints, 10);

    // If enough matches accept Loop
    int nTotalMatches = 0;

    for(size_t i = 0; i < mvpCurrentMatchedPoints.size(); i++) {
      if(mvpCurrentMatchedPoints[i]) {
        nTotalMatches++;
      }
    }

    //    if(nTotalMatches>=40)
    if(nTotalMatches >= mMMParams.mTotalMatchesThres) {
      for(int i = 0; i < nInitialCandidates; i++)
        if(mvpEnoughConsistentCandidates[i] != mpMatchedKF) {
          mvpEnoughConsistentCandidates[i]->SetErase();
        }

      return true;

    } else {
      for(int i = 0; i < nInitialCandidates; i++) {
        mvpEnoughConsistentCandidates[i]->SetErase();
      }

      mpCurrentKF->SetErase();
      return false;
    }
  }

  void MapMatcher::CorrectLoop() {
    cout << "\033[1;32m!!! MAP MATCH FOUND !!!\033[0m" << endl;

    set<size_t> suAssCLientsCurr = mpCurrentKF->GetMapptr()->msuAssClients;
    set<size_t> suAssCLientsMatch = mpMatchedKF->GetMapptr()->msuAssClients;

    for(set<size_t>::iterator sit = suAssCLientsCurr.begin(); sit != suAssCLientsCurr.end(); ++sit) {
      size_t idc = *sit;

      for(set<size_t>::iterator sit2 = suAssCLientsMatch.begin(); sit2 != suAssCLientsMatch.end(); ++sit2) {
        size_t idm = *sit2;

        if(idc == idm) {
          cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::CorrectLoop()\": Associated Clients of matched and current map intersect" << endl;
        }

        mMatchMatrix.at<uint16_t>(idc, idm) = mMatchMatrix.at<uint16_t>(idc, idm) + 1;
        mMatchMatrix.at<uint16_t>(idm, idc) = mMatchMatrix.at<uint16_t>(idm, idc)  + 1;
      }
    }

    cout << "Map Match Matrix:" << endl;
    cout << mMatchMatrix << endl;

    if(mpCurrentKF->mId.second == mpMatchedKF->mId.second) {
      cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::CorrectLoop()\": Matched KFs belong to same client" << endl;
    }

    if(!mpCurrMap->msuAssClients.count(mpCurrentKF->mId.second)) {
      cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::CorrectLoop()\": Current KFs does not belong to current map" << endl;
    }

    if(mpCurrMap->msuAssClients.count(mpMatchedKF->mId.second)) {
      cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::CorrectLoop()\": Matched KFs belongs to current map" << endl;
    }

#ifdef VISUALIZATION
    PublishLoopEdges();
#endif

    cout << "mpCurrentKF: " << mpCurrentKF->mId.first << "|" << mpCurrentKF->mId.second << endl;
    cout << "mpMatchedKF: " << mpMatchedKF->mId.first << "|" << mpMatchedKF->mId.second << endl;

    mapptr pMatchedMap = mpMatchedKF->GetMapptr();

    if(mMMParams.mMinHitsForMerge > 1) {
      cout << "\033[1;33m!!! WARN !!!\033[0m In \"MapMatcher::CorrectLoop()\": map matching / merging algorithm not properly implemented for usin >1 place recognition hit" << endl;
    }

    MapMatchHit MMH(mpCurrentKF, mpMatchedKF, mg2oScw, mvpLoopMapPoints, mvpCurrentMatchedPoints);
    mFoundMatches[mpCurrMap][pMatchedMap].push_back(MMH);
    mFoundMatches[pMatchedMap][mpCurrMap].push_back(MMH);

    if(mFoundMatches[mpCurrMap][pMatchedMap].size() >= mMMParams.mMinHitsForMerge) {
      // Pointer to a Sim3 used by merge maps and global bundle adjustment
      std::shared_ptr<g2o::Sim3> g2oScw = std::make_shared<g2o::Sim3>();

      // Vector with MapMatchHits
      vector<MapMatchHit> vMatches = mFoundMatches[mpCurrMap][pMatchedMap];

      // Merge maps with the first match
      mapptr pMergedMap = mpMapMerger->MergeMaps(mpCurrMap, pMatchedMap, vMatches.back(), g2oScw);
      vMatches.pop_back();
      vMatches.shrink_to_fit();

      // Perform local optimization on the remaining matches
      mpMapMerger->localMapPointFusion(pMergedMap, mpCurrMap, pMatchedMap, vMatches);

      // Perform essential graph optimization
      mpMapMerger->optimizeEssentialGraph(pMergedMap, mpCurrMap, pMatchedMap, vMatches);

      // Perform global bundle adjustment
      mpMapMerger->globalBundleAdjustment(pMergedMap, mpCurrMap, pMatchedMap, vMatches, g2oScw);
    }

    this->ClearLoopEdges();
  }

  void MapMatcher::PublishLoopEdges() {
    mMapMatchEdgeMsg.points.clear();

    tf::StampedTransform Tf_W_Curr, Tf_W_Matched;
    string FrameIdCurr, FrameIdMatched;

    FrameIdCurr = mpCurrentKF->GetMapptr()->mOdomFrame;
    FrameIdMatched = mpMatchedKF->GetMapptr()->mOdomFrame;

    try {
      mTfListen.lookupTransform("world", FrameIdCurr, ros::Time(0), Tf_W_Curr);
      mTfListen.lookupTransform("world", FrameIdMatched, ros::Time(0), Tf_W_Matched);

    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    //    cout << "Tf_W_Curr x|y|z: " << Tf_W_Curr.getOrigin().x() << "|" << Tf_W_Curr.getOrigin().y() << "|" << Tf_W_Curr.getOrigin().z() << endl;
    //    cout << "Tf_W_Matched x|y|z: " << Tf_W_Matched.getOrigin().x() << "|" << Tf_W_Matched.getOrigin().y() << "|" << Tf_W_Matched.getOrigin().z() << endl;

    cv::Mat TCurr = mpCurrentKF->GetPoseInverse();
    cv::Mat TMatch = mpMatchedKF->GetPoseInverse();

    //    cout << "TCurr: " << TCurr << endl;
    //    cout << "TMatch: " << TMatch << endl;

    tf::Point PTfCurr{mScaleFactor*((double)(TCurr.at<float>(0, 3))), mScaleFactor*((double)(TCurr.at<float>(1, 3))), mScaleFactor*((double)(TCurr.at<float>(2, 3)))};
    tf::Point PTfMatch{mScaleFactor*((double)(TMatch.at<float>(0, 3))), mScaleFactor*((double)(TMatch.at<float>(1, 3))), mScaleFactor*((double)(TMatch.at<float>(2, 3)))};

    //    cout << "PTfCurr x|y|z: " << PTfCurr.x() << "|" << PTfCurr.y() << "|" << PTfCurr.z() << endl;
    //    cout << "PTfMatch x|y|z: " << PTfMatch.x() << "|" << PTfMatch.y() << "|" << PTfMatch.z() << endl;

    PTfCurr = Tf_W_Curr * PTfCurr;
    PTfMatch = Tf_W_Matched * PTfMatch;

    geometry_msgs::Point PCurr;
    geometry_msgs::Point PMatch;

    tf::pointTFToMsg(PTfCurr, PCurr);
    tf::pointTFToMsg(PTfMatch, PMatch);

    //    cout << "PCurr x|y|z: " << PCurr.x << "|" << PCurr.y << "|" << PCurr.z << endl;
    //    cout << "PMatch x|y|z: " << PMatch.x << "|" << PMatch.y << "|" << PMatch.z << endl;

    mMapMatchEdgeMsg.points.push_back(PCurr);
    mMapMatchEdgeMsg.points.push_back(PMatch);

    mPubMarker.publish(mMapMatchEdgeMsg);

  }

  void MapMatcher::ClearLoopEdges() {
    mMapMatchEdgeMsg.action = 3;
    mPubMarker.publish(mMapMatchEdgeMsg);
    mMapMatchEdgeMsg.action = visualization_msgs::Marker::ADD;
  }

  void MapMatcher::InsertKF(kfptr pKF) {
    unique_lock<mutex> lock(mMutexKfInQueue);

    mlKfInQueue.push_back(pKF);
  }

  void MapMatcher::EraseKFs(vector<kfptr> vpKFs) {
    unique_lock<mutex> lock(mMutexKfInQueue);

    for(vector<kfptr>::iterator vit = vpKFs.begin(); vit != vpKFs.end(); ++vit) {
      kfptr pKF = *vit;
      std::list<kfptr>::iterator lit = find(mlKfInQueue.begin(), mlKfInQueue.end(), pKF);

      if(lit != mlKfInQueue.end()) {
        mlKfInQueue.erase(lit);
      }
    }
  }

  bool MapMatcher::CheckKfQueue() {
    unique_lock<mutex> lock(mMutexKfInQueue);

    return (!mlKfInQueue.empty());
  }

  void MapMatcher::ResetIfRequested() {
    unique_lock<mutex> lock(mMutexReset);

    if(mbResetRequested) {
      cout << "MapMatcher::ResetIfRequested()" << endl;
      cout << "\033[1;33m!!! WARN !!!\033[0m In \"MapMatcher::ResetIfRequested()\": when one clienthandler requests a reset, all KF, also from other handlers, are flushed" << endl;

      mlKfInQueue.clear();
      mLastLoopKFid = 0;

      mpKFDB->clear();

      mbResetRequested = false;
    }
  }

  void MapMatcher::RequestReset() {
    cout << "MapMatcher::RequestReset()" << endl;
    {
      unique_lock<mutex> lock(mMutexReset);
      mbResetRequested = true;
    }

    while(1) {
      {
        unique_lock<mutex> lock2(mMutexReset);

        if(!mbResetRequested) {
          //                cout << "break ClientCommunicator::RequestReset()" << endl;
          break;
        }
      }
      usleep(mMapMatchRate);
    }
  }

  void MapMatcher::PublishMergedMap(mapptr pMap, set<size_t> suAssClientsC, set<size_t> suAssClientsM) {
    double MarkerSize;
    mNhPrivate.param("MarkerSizeServer", MarkerSize, 0.1);

    visualization_msgs::Marker CovMsgC, CovMsgM, CovMsgCM;

    CovMsgC.header.frame_id = "odomL";
    CovMsgC.header.stamp = ros::Time::now();
    CovMsgC.ns = "CovGraphMergedEdgesCC_yellow";
    CovMsgC.type = visualization_msgs::Marker::LINE_LIST;
    CovMsgC.color.r = 1.0;
    CovMsgC.color.g = 1.0;
    CovMsgC.color.b = 0.0;
    CovMsgC.color.a = 1.0;
    CovMsgC.action = visualization_msgs::Marker::ADD;
    CovMsgC.scale.x = MarkerSize;
    CovMsgC.id = 1;

    CovMsgM.header.frame_id = "odomL";
    CovMsgM.header.stamp = ros::Time::now();
    CovMsgM.ns = "CovGraphMergedEdgesMM_green";
    CovMsgM.type = visualization_msgs::Marker::LINE_LIST;
    CovMsgM.color.r = 0.0;
    CovMsgM.color.g = 1.0;
    CovMsgM.color.b = 0.0;
    CovMsgM.color.a = 1.0;
    CovMsgM.action = visualization_msgs::Marker::ADD;
    CovMsgM.scale.x = MarkerSize;
    CovMsgM.id = 1;

    CovMsgCM.header.frame_id = "odomL";
    CovMsgCM.header.stamp = ros::Time::now();
    CovMsgCM.ns = "CovGraphMergedEdgesCM_red";
    CovMsgCM.type = visualization_msgs::Marker::LINE_LIST;
    CovMsgCM.color.r = 1.0;
    CovMsgCM.color.g = 0.0;
    CovMsgCM.color.b = 0.0;
    CovMsgCM.color.a = 1.0;
    CovMsgCM.action = visualization_msgs::Marker::ADD;
    CovMsgCM.scale.x = MarkerSize;
    CovMsgCM.id = 1;

    CovMsgC.points.clear();
    CovMsgM.points.clear();
    CovMsgCM.points.clear();

    vector<kfptr> vpKFs = pMap->GetAllKeyFrames();
    size_t MaxVal = pMap->GetMaxKFidUnique() + 1;
    vector<vector<bool>> CovMat(MaxVal, vector<bool>(MaxVal, false));

    for(vector<kfptr>::iterator vit = vpKFs.begin(); vit != vpKFs.end(); ++vit) {
      if(((*vit)->IsEmpty()) || ((*vit)->isBad())) {
        continue;
      }

      kfptr pKF = *vit;
      set<kfptr> vConKFs = pKF->GetConnectedKeyFrames();


      for(set<kfptr>::iterator sit = vConKFs.begin(); sit != vConKFs.end(); ++sit) {
        if((*sit)->isBad()) {
          continue;
        }

        kfptr pCon = *sit;

        if(pKF->mId.first >= MaxVal) {
          cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapMatcher::PublishMergedMap(...): KF ID >= MaxVal" << endl;
        }

        size_t CurIdKf, CurIdCon;
        CurIdKf = pKF->mUniqueId;
        CurIdCon = pCon->mUniqueId;

        if(max(pKF->mUniqueId, pCon->mUniqueId) > (MaxVal - 1)) { //starts with 0 -> CovMat[MaxVal][MaxVal] does not exist. Yet, this should not happen...
          cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapMatcher::PublishMergedMap(...): KF ID out of bounds" << endl;
          continue;
        }

        if(CovMat[CurIdKf][CurIdCon] == true || CovMat[CurIdCon][CurIdKf] == true) {
          continue;
        }

        cv::Mat T1 = pKF->GetPoseInverse();
        cv::Mat T2 = pCon->GetPoseInverse();

        geometry_msgs::Point p1;
        geometry_msgs::Point p2;

        p1.x = mScaleFactor * ((double)(T1.at<float>(0, 3)));
        p1.y = mScaleFactor * ((double)(T1.at<float>(1, 3)));
        p1.z = mScaleFactor * ((double)(T1.at<float>(2, 3)));

        p2.x = mScaleFactor * ((double)(T2.at<float>(0, 3)));
        p2.y = mScaleFactor * ((double)(T2.at<float>(1, 3)));
        p2.z = mScaleFactor * ((double)(T2.at<float>(2, 3)));

        if(!(suAssClientsC.count(pKF->mId.second)) && !(suAssClientsM.count(pKF->mId.second))) {
          cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::PublishMergedMap(...)\": pKF: ClientId neither IDC nor IDM" << endl;
        }

        if(!(suAssClientsC.count(pCon->mId.second)) && !(suAssClientsM.count(pCon->mId.second))) {
          cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::PublishMergedMap(...)\": pCon: ClientId neither IDC nor IDM" << endl;
        }

        if(suAssClientsC.count(pKF->mId.second) && suAssClientsC.count(pCon->mId.second)) {
          CovMsgC.points.push_back(p1);
          CovMsgC.points.push_back(p2);

        } else if(suAssClientsM.count(pKF->mId.second) && suAssClientsM.count(pCon->mId.second)) {
          CovMsgM.points.push_back(p1);
          CovMsgM.points.push_back(p2);

        } else {
          CovMsgCM.points.push_back(p1);
          CovMsgCM.points.push_back(p2);
        }

        CovMat[CurIdKf][CurIdCon] = true;
        CovMat[CurIdCon][CurIdKf] = true;
      }
    }

    cout << "Publishing Merged Cov Graph Eges CC with " << CovMsgC.points.size() << " points" << endl;
    cout << "Publishing Merged Cov Graph MM with " << CovMsgM.points.size() << " points" << endl;
    cout << "Publishing Merged Cov Graph CM with " << CovMsgCM.points.size() << " points" << endl;

    mPubMarker.publish(CovMsgC);
    mPubMarker.publish(CovMsgM);
    mPubMarker.publish(CovMsgCM);
  }

}
