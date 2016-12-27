#include <macslam/Mapping.h>

namespace macslam {

  LocalMapping::LocalMapping(ccptr pCC, mapptr pMap, dbptr pDB, int MappingRate, double KfCullingRedundancyThres)
    : mpCC(pCC), mpKFDB(pDB),
      mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
      mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true),
      mMappingRate(MappingRate), mRedundancyThres(KfCullingRedundancyThres),
      mClientId(pCC->mClientId) {
    mVerboseMode = extVerboseMode;
    mAddedKfs = 0;
    mCulledKfs = 0;
  }

  void LocalMapping::RunClient() {

    mbFinished = false;

    while(1) {
      // Tracking will see that Local Mapping is busy
      SetAcceptKeyFrames(false);

      // Check if there are keyframes in the queue
      if(CheckNewKeyFrames()) {
        // Get Communicator Mutex -> Comm cannot publish. Assure no publishing whilst changing data
        //            if(mpComm->mbStrictLock) unique_lock<mutex> lockComm(mpComm->mMutexForMapping);
        if(mVerboseMode == -9) {
          cout << "xxx Mapping --> Lock Mapping xxx" << endl;
        }

        if(mVerboseMode == -9) {
          cout << "LockSleep: " << mpCC->mLockSleep << endl;
        }

        if(mpComm->mbStrictLock) while(!mpCC->LockMapping()) {
            usleep(mpCC->mLockSleep);
          }

        if(mVerboseMode == -9) {
          cout << "xxx Mapping --> Mapping Locked xxx" << endl;
        }

        // BoW conversion and insertion in Map
        ProcessNewKeyFrame();

        // Check recent MapPoints
        MapPointCulling();

        // Triangulate new MapPoints
        CreateNewMapPoints();

        if(!CheckNewKeyFrames()) {
          // Find more matches in neighbor keyframes and fuse point duplications
          SearchInNeighbors();
        }

        mbAbortBA = false;

        if(!CheckNewKeyFrames() && !stopRequested()) {
          // Local BA
          if(mpMap->KeyFramesInMap() > 2) {
            Optimizer::LocalBundleAdjustmentClient(mpCurrentKeyFrame, &mbAbortBA, mpMap, mpComm, mClientId);
          }

          //                // Check redundant local Keyframes
          //                KeyFrameCulling();
        }

        if(mpComm->mbStrictLock) {
          mpCC->UnLockMapping();
        }

      } else if(Stop()) {
        // Safe area to stop
        while(isStopped() && !CheckFinish()) {
          usleep(mMappingRate);
        }

        if(CheckFinish()) {
          break;
        }
      }

      ResetIfRequested();

      // Tracking will see that Local Mapping is busy
      SetAcceptKeyFrames(true);

      if(CheckFinish()) {
        break;
      }

      usleep(mMappingRate);
    }

    SetFinish();
  }

  void LocalMapping::RunServer() {
    while(1) {
      //        unique_lock<mutex> lockMapping(mpCC->mMutexMapping);

      if(mVerboseMode == -9) {
        cout << "xxx Mapping --> Lock Mapping xxx" << endl;
      }

      if(mVerboseMode == -9) {
        cout << "LockSleep: " << mpCC->mLockSleep << endl;
      }

      while(!mpCC->LockMapping()) {
        usleep(mpCC->mLockSleep);
      }

      if(mVerboseMode == -9) {
        cout << "xxx Mapping --> Mapping Locked xxx" << endl;
      }

      if(mpCC->mbOptActive) {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m LocalMapping::Run(...): Optimization active - LocalMapping should be locked" << endl;
      }

      // Check if there are keyframes in the queue
      if(CheckNewKeyFrames()) {
        //            unique_lock<mutex> lockMapUpdate(mpMap->mMutexMapUpdate);
        if(mVerboseMode == -9) {
          cout << "xxx Comm --> Lock MapUpdate xxx" << endl;
        }

        if(mVerboseMode == -9) {
          cout << "LockSleep: " << mpCC->mLockSleep << endl;
        }

        while(!mpMap->LockMapUpdate()) {
          usleep(mpCC->mLockSleep);
        }

        if(mVerboseMode == -9) {
          cout << "xxx Mapping --> MapUpdate Locked xxx" << endl;
        }

        // pop KF from queue
        ProcessNewKeyFrame();



        //Local BA

        //            mbAbortBA=false;

        //            static size_t lbacount = 0;

        //            if(mpMap->KeyFramesInMap()>10 && (lbacount % 1)==0)
        //            {
        ////                cout << "lba start" << endl;
        //                Optimizer::LocalBundleAdjustmentClient(mpCurrentKeyFrame,&mbAbortBA, mpMap,mpComm,mClientId,eSystemState::SERVER);
        ////                cout << "lba end" << endl;
        //            }
        //            ++lbacount;

        //            if(CheckKfsForLBA())
        //            {
        //                kfptr pKfForLBA;
        //                {
        //                    unique_lock<mutex> lock(mMutexKFsForLBA);
        //                    pKfForLBA = mlKfsForLBA.front();
        //                    mlKfsForLBA.pop_front();
        //                }

        //                if(mpMap->KeyFramesInMap()>10)
        //                    Optimizer::LocalBundleAdjustmentClient(pKfForLBA,&mbAbortBA, mpMap,mpComm,mClientId,eSystemState::SERVER);
        //            }

        // Check redundant local Keyframes
        KeyFrameCulling();

        //            cout << "mpCurrentKF->mId" << mpCurrentKeyFrame->mId.first << "|" << mpCurrentKeyFrame->mId.second << "  ->GetPose: " << mpCurrentKeyFrame->GetPose() << endl;
        //            cout << "mpCurrentKF->mId" << mpCurrentKeyFrame->mId.first << "|" << mpCurrentKeyFrame->mId.second << "  ->GetPoseInverse: " << mpCurrentKeyFrame->GetPoseInverse() << endl;
        //            cout << "mpCurrentKF->mId" << mpCurrentKeyFrame->mId.first << "|" << mpCurrentKeyFrame->mId.second << "  ->GetCameraCenter: " << mpCurrentKeyFrame->GetCameraCenter() << endl;

#ifndef MAPFUSION
        mpLoopFinder->InsertKF(mpCurrentKeyFrame);
#endif
        mpMapMatcher->InsertKF(mpCurrentKeyFrame);
        mpKFDB->add(mpCurrentKeyFrame);

        mpMap->UnLockMapUpdate();
      }

      //        cout << "Culled KF IDs: " << endl;
      //        for(int idv = 0; idv < mCulledKfIds.size(); ++idv)
      //            cout << mCulledKfIds[idv];

      ResetIfRequested();

      mpCC->UnLockMapping();

      usleep(mMappingRate);
    }
  }

  void LocalMapping::InsertKeyFrame(kfptr pKF) {
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    //    mbAbortBA=true;

    if(pKF->IsEmpty()) {
      cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Client " << mClientId << ": In \"LocalMapping::ProcessNewKeyFrame()\": empty KF sent to mapping" << endl;
    }

    if(pKF->isBad()) {
      cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mClientId << ": In \"LocalMapping::ProcessNewKeyFrame()\": bad KF sent to mapping" << endl;
    }
  }

  bool LocalMapping::CheckNewKeyFrames() {
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
  }

  void LocalMapping::InsertKfForLBA(kfptr pKF) {
    unique_lock<mutex> lock(mMutexKFsForLBA);
    mlKfsForLBA.push_back(pKF);
    //    mbAbortBA=true;
  }

  bool LocalMapping::CheckKfsForLBA() {
    unique_lock<mutex> lock(mMutexKFsForLBA);
    return(!mlKfsForLBA.empty());
  }

  void LocalMapping::ProcessNewKeyFrame() {
    {
      unique_lock<mutex> lock(mMutexNewKFs);
      mpCurrentKeyFrame = mlNewKeyFrames.front();
      mlNewKeyFrames.pop_front();
    }

    if(mpCC->mSysState == eSystemState::SERVER) {
      ++mAddedKfs;

      if(mpCurrentKeyFrame->isBad()) {
        cout << "\033[1;33m!!! WARN !!!\033[0m Client " << mClientId << ": In \"LocalMapping::ProcessNewKeyFrame()\": bad KF processed by mapping" << endl;
      }

    } else if(mpCC->mSysState == eSystemState::CLIENT) {
      // Compute Bags of Words structures
      mpCurrentKeyFrame->ComputeBoW();

      // Associate MapPoints to the new keyframe and update normal and descriptor
      const vector<mpptr> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

      for(size_t i = 0; i < vpMapPointMatches.size(); i++) {
        mpptr pMP = vpMapPointMatches[i];

        if(pMP) {
          if(!pMP->isBad()) {
            if(!pMP->IsInKeyFrame(mpCurrentKeyFrame)) {
              pMP->AddObservation(mpCurrentKeyFrame, i);
              //                    cout << "LocalMapping::ProcessNewKeyFrame() -> pMP->UpdateNormalAndDepth()" << endl;
              pMP->UpdateNormalAndDepth();
              pMP->ComputeDistinctiveDescriptors();

            } else { // this can only happen for new stereo points inserted by the Tracking
              mlpRecentAddedMapPoints.push_back(pMP);
            }
          }
        }
      }

      // Update links in the Covisibility Graph
      mpCurrentKeyFrame->UpdateConnections();

      // Insert Keyframe in Map
      mpMap->AddKeyFrame(mpCurrentKeyFrame);

    } else {
      cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m LocalMapping::ProcessNewKeyFrame(): invalid systems state: " << mpCC->mSysState << endl;
      throw infrastructure_ex();
    }
  }

  void LocalMapping::KeyFrameCulling() {
    //    unique_lock<mutex> lock1(mpMap->mMutexMapUpdate);

    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<kfptr> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for(vector<kfptr>::iterator vit = vpLocalKeyFrames.begin(), vend = vpLocalKeyFrames.end(); vit != vend; vit++) {
      kfptr pKF = *vit;

      if(0 == pKF->mId.first) {
        continue;
      }

      const vector<mpptr> vpMapPoints = pKF->GetMapPointMatches();

      int nObs = 3;
      const int thObs = nObs;
      int nRedundantObservations = 0;
      int nMPs = 0;

      for(size_t i = 0, iend = vpMapPoints.size(); i < iend; i++) {
        mpptr pMP = vpMapPoints[i];

        if(pMP) {
          if(!pMP->isBad()) {

            /*
            if(!mbMonocular) {
              if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0) {
                continue;
              }
            }
            */

            nMPs++;

            if(pMP->Observations() > thObs) {
              const int &scaleLevel = pKF->mvKeysUn[i].octave;
              const map<kfptr, size_t> observations = pMP->GetObservations();
              int nObs = 0;

              for(map<kfptr, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
                kfptr pKFi = mit->first;

                if(pKFi->isBad()) {
                  continue;
                }

                if(pKFi == pKF) {
                  continue;
                }

                const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                if(scaleLeveli <= scaleLevel + 1) {
                  nObs++;

                  if(nObs >= thObs) {
                    break;
                  }
                }
              }

              if(nObs >= thObs) {
                nRedundantObservations++;
              }
            }
          }
        }
      }

      if(nRedundantObservations > (mRedundancyThres * nMPs)) {
        pKF->SetBadFlag();
        /*
        mCulledKfIds.push_back(pKF->mnId);

        ++mCulledKfs;
        if(mVerboseMode > 0) {
          cout << "Mapping Client " << mClientId <<": Added|Culled KFs: " << mAddedKfs << "|" << mCulledKfs << "--> " << static_cast<double>(mCulledKfs)/static_cast<double>(mAddedKfs) << "%" << endl;
        }

        double ratio = static_cast<double>(nRedundantObservations)/static_cast<double>(nMPs);
        mvCulledKfRatios.push_back(ratio);
        mCulledKfsRatioMean = ((mCulledKfsRatioMean * static_cast<double>(mCulledKfs-1)) + ratio) / static_cast<double>(mCulledKfs);
        mCulledKfRatioStdDev = 0;

				for(vector<double>::iterator vit = mvCulledKfRatios.begin(); vit != mvCulledKfRatios.end(); ++vit) {
					mCulledKfRatioStdDev += (1/static_cast<double>(mCulledKfs))*(*vit)*(*vit);
				}

				mCulledKfRatioStdDev -= mCulledKfsRatioMean*mCulledKfsRatioMean;
				mCulledKfRatioStdDev = sqrt(mCulledKfRatioStdDev);
				if(mVerboseMode > 0) {
					cout << "Mapping Client " << mClientId <<": culled KF stats: Mean|StdDev: " << mCulledKfsRatioMean << "|" << mCulledKfRatioStdDev << endl;
				}
				*/
      }
    }
  }

  void LocalMapping::CreateNewMapPoints() {
    // Retrieve neighbor keyframes in covisibility graph
    //    int nn = 10;
    //    if(mbMonocular)
    int nn = 20;
    const vector<kfptr> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(0.6, false);

    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Tcw1(3, 4, CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0, 3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f * mpCurrentKeyFrame->mfScaleFactor;

    int nnew = 0;

    // Search matches with epipolar restriction and triangulate
    for(size_t i = 0; i < vpNeighKFs.size(); i++) {
      if(i > 0 && CheckNewKeyFrames()) {
        return;
      }

      kfptr pKF2 = vpNeighKFs[i];

      // Check first that baseline is not too short
      cv::Mat Ow2 = pKF2->GetCameraCenter();
      cv::Mat vBaseline = Ow2 - Ow1;
      const float baseline = cv::norm(vBaseline);

      //        if(!mbMonocular)
      //        {
      //            if(baseline<pKF2->mb)
      //            continue;
      //        }
      //        else
      //        {
      const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
      const float ratioBaselineDepth = baseline / medianDepthKF2;

      if(ratioBaselineDepth < 0.01) {
        continue;
      }

      //        }

      // Compute Fundamental Matrix
      cv::Mat F12 = ComputeF12(mpCurrentKeyFrame, pKF2);

      // Search matches that fullfil epipolar constraint
      vector<pair<size_t, size_t> > vMatchedIndices;
      matcher.SearchForTriangulation(mpCurrentKeyFrame, pKF2, F12, vMatchedIndices);

      cv::Mat Rcw2 = pKF2->GetRotation();
      cv::Mat Rwc2 = Rcw2.t();
      cv::Mat tcw2 = pKF2->GetTranslation();
      cv::Mat Tcw2(3, 4, CV_32F);
      Rcw2.copyTo(Tcw2.colRange(0, 3));
      tcw2.copyTo(Tcw2.col(3));

      const float &fx2 = pKF2->fx;
      const float &fy2 = pKF2->fy;
      const float &cx2 = pKF2->cx;
      const float &cy2 = pKF2->cy;
      const float &invfx2 = pKF2->invfx;
      const float &invfy2 = pKF2->invfy;

      // Triangulate each match
      const int nmatches = vMatchedIndices.size();

      for(int ikp = 0; ikp < nmatches; ikp++) {
        const int &idx1 = vMatchedIndices[ikp].first;
        const int &idx2 = vMatchedIndices[ikp].second;

        const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
        //            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
        //            bool bStereo1 = kp1_ur>=0;

        const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
        //            const float kp2_ur = pKF2->mvuRight[idx2];
        //            bool bStereo2 = kp2_ur>=0;

        // Check parallax between rays
        cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.pt.x - cx1) * invfx1, (kp1.pt.y - cy1) * invfy1, 1.0);
        cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.pt.x - cx2) * invfx2, (kp2.pt.y - cy2) * invfy2, 1.0);

        cv::Mat ray1 = Rwc1 * xn1;
        cv::Mat ray2 = Rwc2 * xn2;
        const float cosParallaxRays = ray1.dot(ray2) / (cv::norm(ray1) * cv::norm(ray2));

        float cosParallaxStereo = cosParallaxRays + 1;
        //            float cosParallaxStereo1 = cosParallaxStereo;
        //            float cosParallaxStereo2 = cosParallaxStereo;

        //            if(bStereo1)
        //                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
        //            else if(bStereo2)
        //                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

        //            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

        cv::Mat x3D;

        if(cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 && (cosParallaxRays < 0.9998)) { //(bStereo1 || bStereo2 || cosParallaxRays<0.9998))
          // Linear Triangulation Method
          cv::Mat A(4, 4, CV_32F);
          A.row(0) = xn1.at<float>(0) * Tcw1.row(2) - Tcw1.row(0);
          A.row(1) = xn1.at<float>(1) * Tcw1.row(2) - Tcw1.row(1);
          A.row(2) = xn2.at<float>(0) * Tcw2.row(2) - Tcw2.row(0);
          A.row(3) = xn2.at<float>(1) * Tcw2.row(2) - Tcw2.row(1);

          cv::Mat w, u, vt;
          cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

          x3D = vt.row(3).t();

          if(x3D.at<float>(3) == 0) {
            continue;
          }

          // Euclidean coordinates
          x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);

        }

        //            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
        //            {
        //                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);
        //            }
        //            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
        //            {
        //                x3D = pKF2->UnprojectStereo(idx2);
        //            }
        else {
          continue;  //No stereo and very low parallax
        }

        cv::Mat x3Dt = x3D.t();

        //Check triangulation in front of cameras
        float z1 = Rcw1.row(2).dot(x3Dt) + tcw1.at<float>(2);

        if(z1 <= 0) {
          continue;
        }

        float z2 = Rcw2.row(2).dot(x3Dt) + tcw2.at<float>(2);

        if(z2 <= 0) {
          continue;
        }

        //Check reprojection error in first keyframe
        const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
        const float x1 = Rcw1.row(0).dot(x3Dt) + tcw1.at<float>(0);
        const float y1 = Rcw1.row(1).dot(x3Dt) + tcw1.at<float>(1);
        const float invz1 = 1.0 / z1;

        //            if(!bStereo1)
        //            {
        float u1 = fx1 * x1 * invz1 + cx1;
        float v1 = fy1 * y1 * invz1 + cy1;
        float errX1 = u1 - kp1.pt.x;
        float errY1 = v1 - kp1.pt.y;

        if((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1) {
          continue;
        }

        //            }
        //            else
        //            {
        //                float u1 = fx1*x1*invz1+cx1;
        //                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
        //                float v1 = fy1*y1*invz1+cy1;
        //                float errX1 = u1 - kp1.pt.x;
        //                float errY1 = v1 - kp1.pt.y;
        //                float errX1_r = u1_r - kp1_ur;
        //                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
        //                    continue;
        //            }

        //Check reprojection error in second keyframe
        const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
        const float x2 = Rcw2.row(0).dot(x3Dt) + tcw2.at<float>(0);
        const float y2 = Rcw2.row(1).dot(x3Dt) + tcw2.at<float>(1);
        const float invz2 = 1.0 / z2;
        //            if(!bStereo2)
        //            {
        float u2 = fx2 * x2 * invz2 + cx2;
        float v2 = fy2 * y2 * invz2 + cy2;
        float errX2 = u2 - kp2.pt.x;
        float errY2 = v2 - kp2.pt.y;

        if((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2) {
          continue;
        }

        //            }
        //            else
        //            {
        //                float u2 = fx2*x2*invz2+cx2;
        //                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
        //                float v2 = fy2*y2*invz2+cy2;
        //                float errX2 = u2 - kp2.pt.x;
        //                float errY2 = v2 - kp2.pt.y;
        //                float errX2_r = u2_r - kp2_ur;
        //                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
        //                    continue;
        //            }

        //Check scale consistency
        cv::Mat normal1 = x3D - Ow1;
        float dist1 = cv::norm(normal1);

        cv::Mat normal2 = x3D - Ow2;
        float dist2 = cv::norm(normal2);

        if(dist1 == 0 || dist2 == 0) {
          continue;
        }

        const float ratioDist = dist2 / dist1;
        const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave] / pKF2->mvScaleFactors[kp2.octave];

        /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
            continue;*/
        if(ratioDist * ratioFactor < ratioOctave || ratioDist > ratioOctave * ratioFactor) {
          continue;
        }

        // Triangulation is succesfull
        mpptr pMP{new MapPoint(x3D, mpCurrentKeyFrame, mpMap, mClientId, mpComm, mpCC->mSysState, -1)};

        pMP->AddObservation(mpCurrentKeyFrame, idx1);
        pMP->AddObservation(pKF2, idx2);

        mpCurrentKeyFrame->AddMapPoint(pMP, idx1);
        pKF2->AddMapPoint(pMP, idx2);

        pMP->ComputeDistinctiveDescriptors();

        pMP->UpdateNormalAndDepth();

        mpMap->AddMapPoint(pMP);
        mlpRecentAddedMapPoints.push_back(pMP);

        nnew++;
      }
    }
  }

  void LocalMapping::SearchInNeighbors() {
    // Retrieve neighbor keyframes
    //    int nn = 10;
    //    if(mbMonocular)
    int nn = 20;
    const vector<kfptr> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<kfptr> vpTargetKFs;

    for(vector<kfptr>::const_iterator vit = vpNeighKFs.begin(), vend = vpNeighKFs.end(); vit != vend; vit++) {
      kfptr pKFi = *vit;

      if(pKFi->isBad() || pKFi->mFuseTargetForKF == mpCurrentKeyFrame->mId) {
        continue;
      }

      vpTargetKFs.push_back(pKFi);
      pKFi->mFuseTargetForKF = mpCurrentKeyFrame->mId;

      // Extend to some second neighbors
      const vector<kfptr> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);

      for(vector<kfptr>::const_iterator vit2 = vpSecondNeighKFs.begin(), vend2 = vpSecondNeighKFs.end(); vit2 != vend2; vit2++) {
        kfptr pKFi2 = *vit2;

        if(pKFi2->isBad() || pKFi2->mFuseTargetForKF == mpCurrentKeyFrame->mId || pKFi2->mId == mpCurrentKeyFrame->mId) {
          continue;
        }

        vpTargetKFs.push_back(pKFi2);
      }
    }


    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<mpptr> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(vector<kfptr>::iterator vit = vpTargetKFs.begin(), vend = vpTargetKFs.end(); vit != vend; vit++) {
      kfptr pKFi = *vit;

      matcher.Fuse(pKFi, vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    vector<mpptr> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<kfptr>::iterator vitKF = vpTargetKFs.begin(), vendKF = vpTargetKFs.end(); vitKF != vendKF; vitKF++) {
      kfptr pKFi = *vitKF;

      vector<mpptr> vpMapPointsKFi = pKFi->GetMapPointMatches();

      for(vector<mpptr>::iterator vitMP = vpMapPointsKFi.begin(), vendMP = vpMapPointsKFi.end(); vitMP != vendMP; vitMP++) {
        mpptr pMP = *vitMP;

        if(!pMP) {
          continue;
        }

        if(pMP->isBad() || pMP->mFuseCandidateForKF == mpCurrentKeyFrame->mId) {
          continue;
        }

        pMP->mFuseCandidateForKF = mpCurrentKeyFrame->mId;
        vpFuseCandidates.push_back(pMP);
      }
    }

    matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates);


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i = 0, iend = vpMapPointMatches.size(); i < iend; i++) {
      mpptr pMP = vpMapPointMatches[i];

      if(pMP) {
        if(!pMP->isBad()) {
          pMP->ComputeDistinctiveDescriptors();
          pMP->UpdateNormalAndDepth();
        }
      }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
  }

  cv::Mat LocalMapping::ComputeF12(kfptr &pKF1, kfptr &pKF2) {
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w * R2w.t();
    cv::Mat t12 = -R1w * R2w.t() * t2w + t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv() * t12x * R12 * K2.inv();
  }

  cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v) {
    return (cv::Mat_<float>(3, 3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0, -v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
  }

  void LocalMapping::RequestStop() {
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
  }

  bool LocalMapping::Stop() {
    unique_lock<mutex> lock(mMutexStop);

    if(mbStopRequested && !mbNotStop) {
      mbStopped = true;
      cout << "Local Mapping STOP" << endl;
      return true;
    }

    return false;
  }

  bool LocalMapping::isStopped() {
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
  }

  bool LocalMapping::stopRequested() {
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
  }

  void LocalMapping::Release() {
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinished) {
      return;
    }

    mbStopped = false;
    mbStopRequested = false;
    //    for(list<kfptr>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
    //        delete *lit;
    mlNewKeyFrames.clear(); //at this point, the shared_ptr should only have one instance in this queue -> automatically deleted when list is cleared

    cout << "Local Mapping RELEASE" << endl;
  }

  bool LocalMapping::AcceptKeyFrames() {
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
  }

  void LocalMapping::SetAcceptKeyFrames(bool flag) {
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames = flag;
  }

  bool LocalMapping::SetNotStop(bool flag) {
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped) {
      return false;
    }

    mbNotStop = flag;

    return true;
  }

  void LocalMapping::InterruptBA() {
    mbAbortBA = true;
  }

  void LocalMapping::RequestReset() {

    {
      unique_lock<mutex> lock(mMutexReset);
      mbResetRequested = true;
    }

    while(1) {
      {
        unique_lock<mutex> lock2(mMutexReset);

        if(!mbResetRequested) {
          break;
        }
      }
      usleep(mMappingRate);
    }
  }

  void LocalMapping::ResetIfRequested() {
    unique_lock<mutex> lock(mMutexReset);

    if(mbResetRequested) {
      if(mVerboseMode > 0) {
        cout << "ClientHandler " << mClientId << ": LocalMapping --> Reset" << endl;
      }

      mlNewKeyFrames.clear();
      mlpRecentAddedMapPoints.clear();
#ifndef MAPFUSION

      if(mpLoopFinder) {
        if(mVerboseMode > 0) {
          cout << "ClientHandler " << mpCC->mClientId << ": LocalMapping::ResetIfRequested --> Request Loop Closure module reset" << endl;
        }

        mpLoopFinder->RequestReset();

        if(mVerboseMode > 0) {
          cout << "ClientHandler " << mpCC->mClientId << ": LocalMapping::ResetIfRequested --> finished Loop Closure module reset" << endl;
        }
      }

#endif

      if(mVerboseMode > 0) {
        cout << "ClientHandler " << mClientId << ": LocalMapping --> Resetting finished" << endl;
      }

      mbResetRequested = false;
    }
  }

  void LocalMapping::RequestFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
  }

  bool LocalMapping::CheckFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
  }

  void LocalMapping::SetFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
  }

  bool LocalMapping::isFinished() {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
  }

  void LocalMapping::MapPointCulling() {
    // Check Recent Added MapPoints
    list<mpptr>::iterator lit = mlpRecentAddedMapPoints.begin();
    const idpair nCurrentKFid = mpCurrentKeyFrame->mId;

    int nThObs = 3;
    const int cnThObs = nThObs;

    while(lit != mlpRecentAddedMapPoints.end()) {
      mpptr pMP = *lit;

      if(pMP->isBad()) {
        lit = mlpRecentAddedMapPoints.erase(lit);

      } else if(pMP->GetFoundRatio() < 0.25f ) {
        pMP->SetBadFlag();
        lit = mlpRecentAddedMapPoints.erase(lit);

      } else if(((int)nCurrentKFid.first - (int)pMP->mFirstKfId.first) >= 2 && nCurrentKFid.second != pMP->mFirstFrame.second && pMP->Observations() <= cnThObs) {
        pMP->SetBadFlag();
        lit = mlpRecentAddedMapPoints.erase(lit);

      } else if(((int)nCurrentKFid.first - (int)pMP->mFirstKfId.first) >= 3) {
        lit = mlpRecentAddedMapPoints.erase(lit);

      } else {
        lit++;
      }
    }
  }

} //end ns
