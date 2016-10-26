#include <macslam/MapMerger.h>

namespace macslam {

  MapMerger::MapMerger(MapMergerParams Params, matchptr pMatcher, ros::NodeHandle Nh, ros::NodeHandle NhPrivate)
    : mMyParams(Params), bIsBusy(false), mpMatcher(pMatcher),
      mNh(Nh), mNhPrivate(NhPrivate) {
    if(!mpMatcher) {
      ROS_ERROR_STREAM("In \" MapMerger::MapMerger()\": nullptr passed");
      throw estd::infrastructure_ex();
    }

    mPubMarker = mNh.advertise<visualization_msgs::Marker>("MapMergingMarker", 10);
    mPubMarkerArray = mNh.advertise<visualization_msgs::MarkerArray>("MapMergingMarkerArrays", 10);
  }

  MapMerger::mapptr MapMerger::MergeMaps(mapptr pMapCurr, mapptr pMapMatch, MapMatchHit vMatchHit, std::shared_ptr<g2o::Sim3> g2oScw_end) {
    this->SetBusy();

    bool b0 = false;
    bool b1 = false;
    bool b2 = false;
    bool b3 = false;

    set<ccptr> spCCC = pMapCurr->GetCCPtrs();
    set<ccptr> spCCM = pMapMatch->GetCCPtrs();

    if(spCCC.size() != pMapCurr->msuAssClients.size()) {
      cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": spCCC.size() != pMapCurr->msuAssClients.size()" << endl;
      cout << "Map id: " << pMapCurr->mMapId << endl;
      cout << "Associated client IDs:" << endl;

      for(set<size_t>::const_iterator sit = pMapCurr->msuAssClients.begin(); sit != pMapCurr->msuAssClients.end(); ++sit) {
        cout << *sit << endl;
      }

      cout << "Associated pCCs:" << endl;

      for(set<ccptr>::const_iterator sit = spCCC.begin(); sit != spCCC.end(); ++sit) {
        cout << (*sit)->mClientId << endl;
      }

      throw estd::infrastructure_ex();
    }

    if(spCCM.size() != pMapMatch->msuAssClients.size()) {
      cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": spCCM.size() != pMapMatch->msuAssClients.size()" << endl;
      cout << "Map id: " << pMapMatch->mMapId << endl;
      cout << "Associated client IDs:" << endl;

      for(set<size_t>::const_iterator sit = pMapMatch->msuAssClients.begin(); sit != pMapMatch->msuAssClients.end(); ++sit) {
        cout << *sit << endl;
      }

      cout << "Associated pCCs:" << endl;

      for(set<ccptr>::const_iterator sit = spCCM.begin(); sit != spCCM.end(); ++sit) {
        cout << (*sit)->mClientId << endl;
      }

      throw estd::infrastructure_ex();
    }

    for(set<ccptr>::iterator sit = spCCC.begin(); sit != spCCC.end(); ++sit) {
      ccptr pCC = *sit;

      cout << "spCCC: pCC->mCLientId: " << pCC->mClientId << endl;

      if(pCC->mClientId > 3) {
        cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId out of bounds (" << pCC->mClientId << ")" << endl;
      }

      if(!(pMapCurr->msuAssClients.count(pCC->mClientId))) {
        cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId in pCC but not in msuAssClients" << endl;
      }

      switch(pCC->mClientId) {
      case(static_cast<size_t>(0)):
        if(b0) {
          cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId found twice" << endl;
        }

        b0 = true;

        while(!pCC->LockComm()) {
          usleep(pCC->mLockSleep);
        }

        while(!pCC->LockMapping()) {
          usleep(pCC->mLockSleep);
        }

        while(!pCC->LockPlaceRec()) {
          usleep(pCC->mLockSleep);
        }

        break;

      case(static_cast<size_t>(1)):
        if(b1) {
          cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId found twice" << endl;
        }

        b1 = true;

        while(!pCC->LockComm()) {
          usleep(pCC->mLockSleep);
        }

        while(!pCC->LockMapping()) {
          usleep(pCC->mLockSleep);
        }

        while(!pCC->LockPlaceRec()) {
          usleep(pCC->mLockSleep);
        }

        break;

      case(static_cast<size_t>(2)):
        if(b2) {
          cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId found twice" << endl;
        }

        b2 = true;

        while(!pCC->LockComm()) {
          usleep(pCC->mLockSleep);
        }

        while(!pCC->LockMapping()) {
          usleep(pCC->mLockSleep);
        }

        while(!pCC->LockPlaceRec()) {
          usleep(pCC->mLockSleep);
        }

        break;

      case(static_cast<size_t>(3)):
        if(b3) {
          cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId found twice" << endl;
        }

        b3 = true;

        while(!pCC->LockComm()) {
          usleep(pCC->mLockSleep);
        }

        while(!pCC->LockMapping()) {
          usleep(pCC->mLockSleep);
        }

        while(!pCC->LockPlaceRec()) {
          usleep(pCC->mLockSleep);
        }

        break;

      default:
        cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId out of bounds" << endl;
      }
    }

    for(set<ccptr>::iterator sit = spCCM.begin(); sit != spCCM.end(); ++sit) {
      ccptr pCC = *sit;

      cout << "spCCM: pCC->mCLientId: " << pCC->mClientId << endl;

      if(pCC->mClientId > 3) {
        cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId out of bounds (" << pCC->mClientId << ")" << endl;
      }

      if(!(pMapMatch->msuAssClients.count(pCC->mClientId))) {
        cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId in pCC but not in msuAssClients" << endl;
      }

      switch(pCC->mClientId) {
      case(static_cast<size_t>(0)):
        if(b0) {
          cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId found twice" << endl;
        }

        b0 = true;

        while(!pCC->LockComm()) {
          usleep(pCC->mLockSleep);
        }

        while(!pCC->LockMapping()) {
          usleep(pCC->mLockSleep);
        }

        while(!pCC->LockPlaceRec()) {
          usleep(pCC->mLockSleep);
        }

        break;

      case(static_cast<size_t>(1)):
        if(b1) {
          cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId found twice" << endl;
        }

        b1 = true;

        while(!pCC->LockComm()) {
          usleep(pCC->mLockSleep);
        }

        while(!pCC->LockMapping()) {
          usleep(pCC->mLockSleep);
        }

        while(!pCC->LockPlaceRec()) {
          usleep(pCC->mLockSleep);
        }

        break;

      case(static_cast<size_t>(2)):
        if(b2) {
          cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId found twice" << endl;
        }

        b2 = true;

        while(!pCC->LockComm()) {
          usleep(pCC->mLockSleep);
        }

        while(!pCC->LockMapping()) {
          usleep(pCC->mLockSleep);
        }

        while(!pCC->LockPlaceRec()) {
          usleep(pCC->mLockSleep);
        }

        break;

      case(static_cast<size_t>(3)):
        if(b3) {
          cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId found twice" << endl;
        }

        b3 = true;

        while(!pCC->LockComm()) {
          usleep(pCC->mLockSleep);
        }

        while(!pCC->LockMapping()) {
          usleep(pCC->mLockSleep);
        }

        while(!pCC->LockPlaceRec()) {
          usleep(pCC->mLockSleep);
        }

        break;

      default:
        cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId out of bounds" << endl;
      }
    }

    //Lock all mutexes
    cout << "lock Map mutexes" << endl;

    while(!pMapCurr->LockMapUpdate()) {
      usleep(pMapCurr->GetLockSleep());
    }

    while(!pMapMatch->LockMapUpdate()) {
      usleep(pMapMatch->GetLockSleep());
    }

    cout << "locked" << endl;

    for(set<ccptr>::iterator sit = spCCC.begin(); sit != spCCC.end(); ++sit) {
      (*sit)->mbOptActive = true;
    }


    for(set<ccptr>::iterator sit = spCCM.begin(); sit != spCCM.end(); ++sit) {
      (*sit)->mbOptActive = true;
    }

    /*
    if(vMatchHits.size() < mMyParams.mMinHits)
    {
        cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps\": not enough matches between maps" << endl;
        this->SetIdle();
        return nullptr;
    }
    */

    if(pMapCurr == nullptr || pMapMatch == nullptr) {
      cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps\": at least one map is nullptr" << endl;
      this->SetIdle();
      return nullptr;
    }

    cout << "mMyParams.mMinHits: " << mMyParams.mMinHits << endl;
    cout << "mMyParams.mGBAIterations: " << mMyParams.mGBAIterations << endl;

    //create new map
    cout << "Constructor" << endl;
    mapptr pFusedMap{new Map(pMapMatch, pMapCurr)};
    cout << "Lock Map Update" << endl;

    while(!pFusedMap->LockMapUpdate()) {
      usleep(pFusedMap->GetLockSleep());
    }

    cout << "Update Associated Data" << endl;
    pFusedMap->UpdateAssociatedData();

    //check sizes
    cout << "Map A: KFs|MPs: " << pMapCurr->GetAllKeyFrames().size() << "|" << pMapCurr->GetAllMapPoints().size() << endl;
    cout << "Map B: KFs|MPs: " << pMapMatch->GetAllKeyFrames().size() << "|" << pMapMatch->GetAllMapPoints().size() << endl;
    cout << "Fused Map: KFs|MPs: " << pFusedMap->GetAllKeyFrames().size() << "|" << pFusedMap->GetAllKeyFrames().size() << endl;

    size_t IdC = vMatchHit.mpKFCurr->mId.second;
    size_t IdM = vMatchHit.mpKFMatch->mId.second;

    set<size_t> suAssClientsC = pMapCurr->msuAssClients;
    set<size_t> suAssClientsM = pMapMatch->msuAssClients;

#ifdef VISUALIZATION
    mpMatcher->PublishMergedMap(pFusedMap, suAssClientsC, suAssClientsM);
#endif

    g2o::Sim3 g2oS_wm_wc; //world match - world curr

    //optimize
    idpair nLoopKf;

    kfptr pKFCur = vMatchHit.mpKFCurr;
    kfptr pKFMatch = vMatchHit.mpKFMatch;
    g2o::Sim3 g2oScw = vMatchHit.mg2oScw;
    std::vector<mpptr> vpCurrentMatchedPoints = vMatchHit.mvpCurrentMatchedPoints;
    std::vector<mpptr> vpLoopMapPoints = vMatchHit.mvpLoopMapPoints;

    vector<kfptr> vpKeyFramesCurr = pMapCurr->GetAllKeyFrames();

    if(IdC != pKFCur->mId.second || IdM != pKFMatch->mId.second) {
      std::cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps\": client ID mismatch" << endl;
    }

    // Ensure current keyframe is updated
    pKFCur->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = pKFCur->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(pKFCur);

    nLoopKf = pKFCur->mId;

    KeyFrameAndPose CorrectedSim3;
    KeyFrameAndPose NonCorrectedSim3;
    CorrectedSim3[pKFCur] = g2oScw;
    cv::Mat Twc = pKFCur->GetPoseInverse();

    {
      cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
      cv::Mat twc = Twc.rowRange(0, 3).col(3);
      g2o::Sim3 g2oSwc(Converter::toMatrix3d(Rwc), Converter::toVector3d(twc), 1.0);
      g2oS_wm_wc = (g2oScw.inverse()) * (g2oSwc.inverse());
    }

    KeyFrameAndPose CorrectedSim3All;
    KeyFrameAndPose NonCorrectedSim3All;
    CorrectedSim3All[pKFCur] = g2oScw;

    for(vector<kfptr>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end(); vit != vend; vit++) {
      kfptr pKFi = *vit;

      cv::Mat Tiw = pKFi->GetPose();

      if(pKFi != pKFCur) {
        cv::Mat Tic = Tiw * Twc;
        cv::Mat Ric = Tic.rowRange(0, 3).colRange(0, 3);
        cv::Mat tic = Tic.rowRange(0, 3).col(3);
        g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric), Converter::toVector3d(tic), 1.0);
        g2o::Sim3 g2oCorrectedSiw = g2oSic * g2oScw;
        //Pose corrected with the Sim3 of the loop closure
        CorrectedSim3[pKFi] = g2oCorrectedSiw;
      }

      cv::Mat Riw = Tiw.rowRange(0, 3).colRange(0, 3);
      cv::Mat tiw = Tiw.rowRange(0, 3).col(3);
      g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw), Converter::toVector3d(tiw), 1.0);
      //Pose without correction
      NonCorrectedSim3[pKFi] = g2oSiw;
    }

    for(vector<kfptr>::iterator vit = vpKeyFramesCurr.begin(); vit != vpKeyFramesCurr.end(); ++vit) {
      kfptr pKFi = *vit;

      KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKFi);

      if(it != CorrectedSim3.end()) {
        CorrectedSim3All[pKFi] = it->second;
        NonCorrectedSim3All[pKFi] = NonCorrectedSim3All[pKFi];

        KeyFrameAndPose::const_iterator it2 = NonCorrectedSim3.find(pKFi);

        if(it2 == NonCorrectedSim3.end()) {
          cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps\": Siw for KF in CorrectedSim3 but not in NonCorrectedSim3" << endl;
        }

      } else {
        cv::Mat Tiw = pKFi->GetPose();
        //CorrectedSim3All
        cv::Mat Tic = Tiw * Twc;
        cv::Mat Ric = Tic.rowRange(0, 3).colRange(0, 3);
        cv::Mat tic = Tic.rowRange(0, 3).col(3);
        g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric), Converter::toVector3d(tic), 1.0);
        g2o::Sim3 g2oCorrectedSiw = g2oSic * g2oScw;
        //Pose corrected with the Sim3 of the loop closure
        CorrectedSim3All[pKFi] = g2oCorrectedSiw;
        //NonCorrectedSim3All
        cv::Mat Riw = Tiw.rowRange(0, 3).colRange(0, 3);
        cv::Mat tiw = Tiw.rowRange(0, 3).col(3);
        g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw), Converter::toVector3d(tiw), 1.0);
        //Pose without correction
        NonCorrectedSim3All[pKFi] = g2oSiw;
      }
    }

    // Correct MapPoints and KeyFrames of current map
    for(KeyFrameAndPose::iterator mit = CorrectedSim3All.begin(), mend = CorrectedSim3All.end(); mit != mend; mit++) {
      kfptr pKFi = mit->first;
      g2o::Sim3 g2oCorrectedSiw = mit->second;
      g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

      g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

      vector<mpptr> vpMPsi = pKFi->GetMapPointMatches();

      for(size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; iMP++) {
        mpptr pMPi = vpMPsi[iMP];

        if(!pMPi) {
          continue;
        }

        if(pMPi->isBad()) {
          continue;
        }

        if(pMPi->mCorrectedByKF_MM == pKFCur->mId) { //ID Tag
          continue;
        }

        // Project with non-corrected pose and project back with corrected pose
        cv::Mat P3Dw = pMPi->GetWorldPos();
        Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        pMPi->SetWorldPos(cvCorrectedP3Dw, true);
        pMPi->mCorrectedByKF_MM = pKFCur->mId;
        pMPi->mCorrectedReference_MM = pKFCur->mUniqueId;
        pMPi->UpdateNormalAndDepth();
      }

      // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
      Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
      Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
      double s = g2oCorrectedSiw.scale();

      eigt *= (1. / s); //[R t/s;0 1]

      cv::Mat correctedTiw = Converter::toCvSE3(eigR, eigt);

      pKFi->SetPose(correctedTiw, true);

      // Make sure connections are updated
      pKFi->UpdateConnections();
    }

    //            cout << "\033[1;33m!!! WARN !!!\033[0m In \"MapMatcher::MergeMaps(...)\": skipped else branch, use only one match" << endl;

#ifdef VISUALIZATION
    mpMatcher->PublishMergedMap(pFusedMap, suAssClientsC, suAssClientsM);
#endif
    cout << "fused map after transformation" << endl;
    //        cin.get();

    // Start Loop Fusion
    // Update matched map points and replace if duplicated
    for(size_t i = 0; i < vpCurrentMatchedPoints.size(); i++) {
      if(vpCurrentMatchedPoints[i]) {
        mpptr pLoopMP = vpCurrentMatchedPoints[i];
        mpptr pCurMP = pKFCur->GetMapPoint(i);

        if(pCurMP) {
          //                    cout << "is pLoopMP nullptr?: " << (!pLoopMP) << endl;
          pCurMP->ReplaceAndLock(pLoopMP);

        } else {
          pKFCur->AddMapPoint(pLoopMP, i, true); //lock this MapPoint
          pLoopMP->AddObservation(pKFCur, i, true);
          pLoopMP->ComputeDistinctiveDescriptors();
        }
      }
    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3, vpLoopMapPoints);

#ifdef VISUALIZATION
    mpMatcher->PublishMergedMap(pFusedMap, suAssClientsC, suAssClientsM);
    cout << "loop connections and KFs" << endl;
#endif

    *g2oScw_end = g2oS_wm_wc;

    return(pFusedMap);
  }

  void MapMerger::localMatchOptimization(mapptr mpMap, vector<MapMatchHit> vMatchHits) {
    cout << "\033[1;32m!!! LOOP FOUND !!!\033[0m" << endl;
#ifdef VISUALIZATION
    /// \todo Rewrite PublishLoopEdges for this purpose.
    //this->PublishLoopEdges();
#endif

		KeyFrameAndPose CorrectedSim3Vec[vMatchHits.size()];
		// Loop through all matches
		int lv = 0;
		for(auto i : vMatchHits) {
			/// \todo Loop through all matches and not only use current key frame of the first match, use according data structure
			//boost::shared_ptr<KeyFrame> mpCurrentKF = vMatchHits.front().mpKFCurr;
			boost::shared_ptr<KeyFrame> mpCurrentKF = i.mpKFCurr;

			KeyFrameAndPose CorrectedSim3;
			KeyFrameAndPose NonCorrectedSim3;
			/// \todo Use transformation of the according match
			//CorrectedSim3[mpCurrentKF] = vMatchHits.front().mg2oScw;
			CorrectedSim3[mpCurrentKF] = i.mg2oScw;
			cv::Mat Twc = mpCurrentKF->GetPoseInverse();

			cv::Mat Rcur = Twc.rowRange(0, 3).colRange(0, 3);
			cv::Mat tcur = Twc.rowRange(0, 3).col(3);
			g2o::Sim3 g2oScur(Converter::toMatrix3d(Rcur), Converter::toVector3d(tcur), 1.0);
			/// \todo Use transformation of the according match
			//g2o::Sim3 g2oS_loop = g2oScur * vMatchHits.front().mg2oScw;
			g2o::Sim3 g2oS_loop = g2oScur * i.mg2oScw;

			cout << "--- transform KFs/MPs" << endl;

			// Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
			std::vector<boost::shared_ptr<KeyFrame>> mvpCurrentConnectedKFs = i.mpKFCurr->GetVectorCovisibleKeyFrames();
			mvpCurrentConnectedKFs.push_back(i.mpKFCurr);

			for(vector<kfptr>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end(); vit != vend; vit++) {
				kfptr pKFi = *vit;

				cv::Mat Tiw = pKFi->GetPose();

				if(pKFi != mpCurrentKF) {
					cv::Mat Tic = Tiw * Twc;
					cv::Mat Ric = Tic.rowRange(0, 3).colRange(0, 3);
					cv::Mat tic = Tic.rowRange(0, 3).col(3);
					g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric), Converter::toVector3d(tic), 1.0);
					/// \todo Use transformation of the according match
					g2o::Sim3 g2oCorrectedSiw = g2oSic * i.mg2oScw;
					//g2o::Sim3 g2oCorrectedSiw = g2oSic * vMatchHits.front().mg2oScw;
					//Pose corrected with the Sim3 of the loop closure
					CorrectedSim3[pKFi] = g2oCorrectedSiw;
					CorrectedSim3Vec[lv][pKFi] = g2oCorrectedSiw;
				}

				cv::Mat Riw = Tiw.rowRange(0, 3).colRange(0, 3);
				cv::Mat tiw = Tiw.rowRange(0, 3).col(3);
				g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw), Converter::toVector3d(tiw), 1.0);
				//Pose without correction
				NonCorrectedSim3[pKFi] = g2oSiw;
			}
			++lv;

			// Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
			for(KeyFrameAndPose::iterator mit = CorrectedSim3.begin(), mend = CorrectedSim3.end(); mit != mend; mit++) {
				kfptr pKFi = mit->first;
				g2o::Sim3 g2oCorrectedSiw = mit->second;
				g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

				g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

				vector<mpptr> vpMPsi = pKFi->GetMapPointMatches();

				for(size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; iMP++) {
					mpptr pMPi = vpMPsi[iMP];

					if(!pMPi) {
						continue;
					}

					if(pMPi->isBad()) {
						continue;
					}

					//            if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId && pMPi->mnCorrectedByKFClientId == mpCurrentKF->mClientId) //ID Tag
					if(pMPi->mCorrectedByKF_LC == mpCurrentKF->mId) { //ID Tag
						continue;
					}

					// Project with non-corrected pose and project back with corrected pose
					cv::Mat P3Dw = pMPi->GetWorldPos();
					Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
					Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

					cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
					pMPi->SetWorldPos(cvCorrectedP3Dw, true);
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

				eigt *= (1. / s); //[R t/s;0 1]

				cv::Mat correctedTiw = Converter::toCvSE3(eigR, eigt);

				pKFi->SetPose(correctedTiw, true);

				// Make sure connections are updated
				pKFi->UpdateConnections();
			}
		}

    cout << "--- update matched MPs" << endl;

    // Start Loop Fusion
    // Update matched map points and replace if duplicated
    /// \todo Use the matched points of the according match
    // Loop through all matches
    for(auto j : vMatchHits) {
      for(size_t i = 0; i < j.mvpCurrentMatchedPoints.size(); i++) {
      //for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        /// \todo Use the matched points of the according match
        if(j.mvpCurrentMatchedPoints[i]) {
        //if(mvpCurrentMatchedPoints[i])
          /// \todo Use the matched points of the according match
          mpptr pLoopMP = j.mvpCurrentMatchedPoints[i];
          //mpptr pLoopMP = mvpCurrentMatchedPoints[i];
          mpptr pCurMP = j.mpKFCurr->GetMapPoint(i);

					if(pCurMP) {
						pCurMP->ReplaceAndLock(pLoopMP);

					} else {
						j.mpKFCurr->AddMapPoint(pLoopMP, i, true);
						pLoopMP->AddObservation(j.mpKFCurr, i, true);
						pLoopMP->ComputeDistinctiveDescriptors();
					}
				}
			}
		}

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    /// \todo Use the matched points of the according match
    lv = 0;
    for(auto i : vMatchHits) {
      SearchAndFuse(CorrectedSim3Vec[lv], i.mvpLoopMapPoints);
      //SearchAndFuse(CorrectedSim3, mvpLoopMapPoints);
      ++lv;
    }

		for(auto i : vMatchHits) {
			std::vector<boost::shared_ptr<KeyFrame>> mvpCurrentConnectedKFs = i.mpKFCurr->GetVectorCovisibleKeyFrames();
			mvpCurrentConnectedKFs.push_back(i.mpKFCurr);

			// After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
			map<kfptr, set<kfptr> > LoopConnections;

			for(vector<kfptr>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end(); vit != vend; vit++) {
				kfptr pKFi = *vit;
				vector<kfptr> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

				// Update connections. Detect new links.
				pKFi->UpdateConnections();
				LoopConnections[pKFi] = pKFi->GetConnectedKeyFrames();

				for(vector<kfptr>::iterator vit_prev = vpPreviousNeighbors.begin(), vend_prev = vpPreviousNeighbors.end(); vit_prev != vend_prev; vit_prev++) {
					LoopConnections[pKFi].erase(*vit_prev);
				}

				for(vector<kfptr>::iterator vit2 = mvpCurrentConnectedKFs.begin(), vend2 = mvpCurrentConnectedKFs.end(); vit2 != vend2; vit2++) {
					LoopConnections[pKFi].erase(*vit2);
				}
			}
		}

    // Perform local bundle adjustment
    /*
    bool mbStopLBA = false;

		std::vector<kfptr> pKFs;
		for(auto i : vMatchHits) {
			pKFs.push_back(i.mpKFCurr);
		}

		Optimizer::LocalBundleAdjustment(pKFs, &mbStopLBA, mpMap, mpMap->mMapId);
		*/

  }

  void MapMerger::optimizeEssentialGraph(mapptr pFusedMap, mapptr pMapCurr, mapptr pMapMatch, vector<MapMatchHit> vMatchHits) {
    kfptr pKFCur = vMatchHits.back().mpKFCurr;
    kfptr pKFMatch = vMatchHits.back().mpKFMatch;

    set<size_t> suAssClientsC = pMapCurr->msuAssClients;
    set<size_t> suAssClientsM = pMapMatch->msuAssClients;

    std::vector<kfptr> mvpCurrentConnectedKFs = pKFCur->GetVectorCovisibleKeyFrames();

    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<kfptr, set<kfptr>> LoopConnections;

    for(vector<kfptr>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end(); vit != vend; vit++) {
      kfptr pKFi = *vit;
      vector<kfptr> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

      // Update connections. Detect new links.
      pKFi->UpdateConnections();
      LoopConnections[pKFi] = pKFi->GetConnectedKeyFrames();

      for(vector<kfptr>::iterator vit_prev = vpPreviousNeighbors.begin(), vend_prev = vpPreviousNeighbors.end(); vit_prev != vend_prev; vit_prev++) {
        LoopConnections[pKFi].erase(*vit_prev);
      }

      for(vector<kfptr>::iterator vit2 = mvpCurrentConnectedKFs.begin(), vend2 = mvpCurrentConnectedKFs.end(); vit2 != vend2; vit2++) {
        LoopConnections[pKFi].erase(*vit2);
      }
    }

    // Optimize graph
    Optimizer::OptimizeEssentialGraphMapFusionV2(pFusedMap, pKFMatch, pKFCur, LoopConnections, false);

    // Add loop edge
    pKFMatch->AddLoopEdge(pKFCur);
    pKFCur->AddLoopEdge(pKFMatch);

#ifdef VISUALIZATION
    mpMatcher->PublishMergedMap(pFusedMap, suAssClientsC, suAssClientsM);
#endif

    std::cout << "Essential graph optimized" << std::endl;

		cout << "ENTER to continue..." << endl << endl;
		std::cin.get(); //wait to start bagfile recording
	}

  void MapMerger::globalBundleAdjustment(mapptr pFusedMap, mapptr pMapCurr, mapptr pMapMatch, vector<MapMatchHit> vMatchHits, std::shared_ptr<g2o::Sim3> g2oScw) {

    cout << ">>>>> MapMerger::MergeMaps --> Global Bundle Adjustment" << endl;

    set<size_t> suAssClientsC = pMapCurr->msuAssClients;
    set<size_t> suAssClientsM = pMapMatch->msuAssClients;
    kfptr pKFCur = vMatchHits.back().mpKFCurr;
    idpair nLoopKf = pKFCur->mId;

    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;

    struct timeval tStart;
    struct timeval tNow;
    double dEl;
    gettimeofday(&tStart, NULL);

    Optimizer::MapFusionGBA(pFusedMap, pFusedMap->mMapId, mMyParams.mGBAIterations, &mbStopGBA, nLoopKf, false);

    // Correct keyframes starting at map first keyframe
    list<kfptr> lpKFtoCheck(pFusedMap->mvpKeyFrameOrigins.begin(), pFusedMap->mvpKeyFrameOrigins.end());

    while(!lpKFtoCheck.empty()) {
      kfptr pKF = lpKFtoCheck.front();
      const set<kfptr> sChilds = pKF->GetChilds();
      cv::Mat Twc = pKF->GetPoseInverse();

      for(set<kfptr>::const_iterator sit = sChilds.begin(); sit != sChilds.end(); sit++) {
        kfptr pChild = *sit;

        if(pChild->mBAGlobalForKF != nLoopKf) {
          cv::Mat Tchildc = pChild->GetPose() * Twc;
          pChild->mTcwGBA = Tchildc * pKF->mTcwGBA; //*Tcorc*pKF->mTcwGBA;
          pChild->mBAGlobalForKF = nLoopKf;

        }

        lpKFtoCheck.push_back(pChild);
      }

      pKF->mTcwBefGBA = pKF->GetPose();
      pKF->SetPose(pKF->mTcwGBA, true);
      lpKFtoCheck.pop_front();
    }

    // Correct MapPoints
    const vector<mpptr> vpMPs = pFusedMap->GetAllMapPoints();

    for(size_t i = 0; i < vpMPs.size(); i++) {
      mpptr pMP = vpMPs[i];

      if(pMP->isBad()) {
        continue;
      }

      if(pMP->mBAGlobalForKF == nLoopKf) {
        // If optimized by Global BA, just update
        pMP->SetWorldPos(pMP->mPosGBA, true);

      } else {
        // Update according to the correction of its reference keyframe
        kfptr pRefKF = pMP->GetReferenceKeyFrame();

        if(!pRefKF) {
          cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": pRefKf is nullptr" << endl;
          continue;
        }

        //            if(pRefKF->mnBAGlobalForKF!=nLoopKf)
        if(pRefKF->mBAGlobalForKF != nLoopKf) {
          continue;
        }

        // Map to non-corrected camera
        cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0, 3).colRange(0, 3);
        cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0, 3).col(3);
        cv::Mat Xc = Rcw * pMP->GetWorldPos() + tcw;

        // Backproject using corrected camera
        cv::Mat Twc = pRefKF->GetPoseInverse();
        cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
        cv::Mat twc = Twc.rowRange(0, 3).col(3);

        pMP->SetWorldPos(Rwc * Xc + twc, true);
      }
    }

    gettimeofday(&tNow, NULL);
    dEl = (tNow.tv_sec - tStart.tv_sec) + (tNow.tv_usec - tStart.tv_usec) / 1000000.0;
    cout << "Optimization Time: Agents|KFs|MPs|Time: " << pFusedMap->GetCCPtrs().size() << "|" << pFusedMap->GetAllKeyFrames().size() << "|" << pFusedMap->GetAllMapPoints().size() << "|" << dEl << endl;

    cout << "after BA" << endl;
#ifdef VISUALIZATION
    mpMatcher->PublishMergedMap(pFusedMap, suAssClientsC, suAssClientsM);
#endif

    cout << "\033[1;32;41m!!! MAPS MERGED !!!\033[0m" << endl;

    this->SetIdle();


    set<ccptr> spCCC = pMapCurr->GetCCPtrs();
    set<ccptr> spCCF = pFusedMap->GetCCPtrs();

    for(set<ccptr>::iterator sit = spCCF.begin(); sit != spCCF.end(); ++sit) {
      ccptr pCC = *sit;
      chptr pCH = pCC->mpCH;

      if(spCCC.count(pCC)) {
        cout << "pCC found for client id " << pCC->mClientId << endl;
        pCH->ChangeMap(pFusedMap, *g2oScw);
        pCC->mbGotMerged = true;

      } else {
        cout << "g2oS_wm_wc not changed for client id " << pCC->mClientId << endl;
        pCH->ChangeMap(pFusedMap, g2o::Sim3());
      }

      pCC->UnLockComm();
      pCC->UnLockMapping();
      pCC->UnLockPlaceRec();
      pCC->mbOptActive = false;
    }

    pMapCurr->mbOutdated = true;
    pMapMatch->mbOutdated = true;

    pMapCurr->UnLockMapUpdate();
    pMapMatch->UnLockMapUpdate();
    pFusedMap->UnLockMapUpdate();
  }

  void MapMerger::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, std::vector<mpptr> vpLoopMapPoints) {
    ORBmatcher matcher(0.8);

    for(KeyFrameAndPose::const_iterator mit = CorrectedPosesMap.begin(), mend = CorrectedPosesMap.end(); mit != mend; mit++) {
      kfptr pKF = mit->first;

      g2o::Sim3 g2oScw = mit->second;
      cv::Mat cvScw = Converter::toCvMat(g2oScw);

      vector<mpptr> vpReplacePoints(vpLoopMapPoints.size(), nullptr);
      matcher.Fuse(pKF, cvScw, vpLoopMapPoints, 4, vpReplacePoints);

      // Get Map Mutex
      //        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
      const int nLP = vpLoopMapPoints.size();

      for(int i = 0; i < nLP; i++) {
        mpptr pRep = vpReplacePoints[i];

        if(pRep) {
          pRep->ReplaceAndLock(vpLoopMapPoints[i]);
        }
      }
    }
  }

  void MapMerger::SetBusy() {
    unique_lock<mutex> lock(mMutexBusy);
    bIsBusy = true;
  }

  void MapMerger::SetIdle() {
    unique_lock<mutex> lock(mMutexBusy);
    bIsBusy = false;
  }

  bool MapMerger::isBusy() {
    unique_lock<mutex> lock(mMutexBusy);
    return bIsBusy;
  }
}
