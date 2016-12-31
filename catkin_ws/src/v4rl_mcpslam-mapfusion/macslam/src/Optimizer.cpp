#include <macslam/Optimizer.h>

namespace macslam {


  void Optimizer::GlobalBundleAdjustemntClient(mapptr pMap, size_t ClientId, int nIterations, bool* pbStopFlag, const idpair nLoopKF, const bool bRobust) {
    vector<kfptr> vpKFs = pMap->GetAllKeyFrames();
    vector<mpptr> vpMP = pMap->GetAllMapPoints();
    BundleAdjustmentClient(vpKFs, vpMP, ClientId, nIterations, pbStopFlag, nLoopKF, bRobust);
  }


  void Optimizer::BundleAdjustmentClient(const vector<kfptr> &vpKFs, const vector<mpptr> &vpMP, size_t ClientId,
                                         int nIterations, bool* pbStopFlag, const idpair nLoopKF, const bool bRobust) {
    const idpair zeropair = make_pair(0, ClientId);

    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag) {
      optimizer.setForceStopFlag(pbStopFlag);
    }

    //    long unsigned int maxKFid = 0;

    bool bFixedFrame = false;

    // Set KeyFrame vertices
    for(size_t i = 0; i < vpKFs.size(); i++) {
      kfptr pKF = vpKFs[i];

      if(pKF->isBad()) {
        continue;
      }

      if(pKF->mId.first >= IDRANGE) {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": KF index out of bounds" << endl;
        throw infrastructure_ex();
      }

      g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
      vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
      vSE3->setId(Optimizer::GetID(pKF->mId, true));
      vSE3->setFixed(pKF->mId == zeropair);
      optimizer.addVertex(vSE3);
      //        if(pKF->mnId>maxKFid)
      //            maxKFid=pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    for(size_t i = 0; i < vpMP.size(); i++) {
      mpptr pMP = vpMP[i];

      if(pMP->isBad()) {
        continue;
      }

      if(pMP->mId.first >= IDRANGE) {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": MP index out of bounds" << endl;
        throw infrastructure_ex();
      }

      g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
      vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
      const int id = Optimizer::GetID(pMP->mId, false);
      vPoint->setId(id);
      vPoint->setMarginalized(true);
      optimizer.addVertex(vPoint);

      const map<kfptr, size_t> observations = pMP->GetObservations();

      int nEdges = 0;

      //SET EDGES
      for(map<kfptr, size_t>::const_iterator mit = observations.begin(); mit != observations.end(); mit++) {

        kfptr pKF = mit->first;

        //            if(pKF->isBad() || pKF->mnId>maxKFid)
        if(pKF->isBad()) {
          continue;
        }

        if(pKF->mId.first >= IDRANGE) {
          cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": KF index out of bounds" << endl;
          throw infrastructure_ex();
        }

        nEdges++;

        const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];


        Eigen::Matrix<double, 2, 1> obs;
        obs << kpUn.pt.x, kpUn.pt.y;

        g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(Optimizer::GetID(pKF->mId, true))));
        e->setMeasurement(obs);
        const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

        if(bRobust) {
          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuber2D);
        }

        e->fx = pKF->fx;
        e->fy = pKF->fy;
        e->cx = pKF->cx;
        e->cy = pKF->cy;

        optimizer.addEdge(e);
      }

      if(nEdges == 0) {
        optimizer.removeVertex(vPoint);
        vbNotIncludedMP[i] = true;

      } else {
        vbNotIncludedMP[i] = false;
      }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for(size_t i = 0; i < vpKFs.size(); i++) {
      kfptr pKF = vpKFs[i];

      if(pKF->isBad()) {
        continue;
      }

      g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(Optimizer::GetID(pKF->mId, true)));
      g2o::SE3Quat SE3quat = vSE3->estimate();

      if(nLoopKF == zeropair) {
        pKF->SetPose(Converter::toCvMat(SE3quat), false);

      } else {
        pKF->mTcwGBA.create(4, 4, CV_32F);
        Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
        pKF->mBAGlobalForKF = nLoopKF;
      }
    }

    //Points
    for(size_t i = 0; i < vpMP.size(); i++) {
      if(vbNotIncludedMP[i]) {
        continue;
      }

      mpptr pMP = vpMP[i];

      if(pMP->isBad()) {
        continue;
      }

      g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(Optimizer::GetID(pMP->mId, false)));

      if(nLoopKF == zeropair) {
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()), false);
        //            cout << "Optimizer::BundleAdjustment(...) -> pMP->UpdateNormalAndDepth()" << endl;
        pMP->UpdateNormalAndDepth();

      } else {
        pMP->mPosGBA.create(3, 1, CV_32F);
        Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
        pMP->mBAGlobalForKF = nLoopKF;
      }
    }

  }


  int Optimizer::PoseOptimizationClient(Frame &Frame) {
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences = 0;

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(Frame.mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = Frame.N;

    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    const float deltaMono = sqrt(5.991);

    {
      unique_lock<mutex> lock(MapPoint::mGlobalMutex);

      for(int i = 0; i < N; i++) {
        mpptr pMP = Frame.mvpMapPoints[i];

        if(pMP) {
          // Monocular observation
          nInitialCorrespondences++;
          Frame.mvbOutlier[i] = false;

          Eigen::Matrix<double, 2, 1> obs;
          const cv::KeyPoint &kpUn = Frame.mvKeysUn[i];
          obs << kpUn.pt.x, kpUn.pt.y;

          g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
          e->setMeasurement(obs);
          const float invSigma2 = Frame.mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(deltaMono);

          e->fx = Frame.fx;
          e->fy = Frame.fy;
          e->cx = Frame.cx;
          e->cy = Frame.cy;
          cv::Mat Xw = pMP->GetWorldPos();
          e->Xw[0] = Xw.at<float>(0);
          e->Xw[1] = Xw.at<float>(1);
          e->Xw[2] = Xw.at<float>(2);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }

      }
    }


    if(nInitialCorrespondences < 3) {
      return 0;
    }

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    const int its[4] = {10, 10, 10, 10};

    int nBad = 0;

    for(size_t it = 0; it < 4; it++) {

      vSE3->setEstimate(Converter::toSE3Quat(Frame.mTcw));
      optimizer.initializeOptimization(0);
      optimizer.optimize(its[it]);

      nBad = 0;

      for(size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

        const size_t idx = vnIndexEdgeMono[i];

        if(Frame.mvbOutlier[idx]) {
          e->computeError();
        }

        const float chi2 = e->chi2();

        if(chi2 > chi2Mono[it]) {
          Frame.mvbOutlier[idx] = true;
          e->setLevel(1);
          nBad++;

        } else {
          Frame.mvbOutlier[idx] = false;
          e->setLevel(0);
        }

        if(it == 2) {
          e->setRobustKernel(0);
        }
      }

      if(optimizer.edges().size() < 10) {
        break;
      }
    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    Frame.SetPose(pose);

    return nInitialCorrespondences - nBad;
  }

  void Optimizer::LocalBundleAdjustmentClient(kfptr pKF, bool* pbStopFlag, mapptr pMap, commptr pComm, size_t ClientId, eSystemState SysState) {
    //    const idpair zeropair = make_pair(0,0);

    // Local KeyFrames: First Breath Search from Current Keyframe
    list<kfptr> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mBALocalForKF = pKF->mId;

    const vector<kfptr> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();

    for(int i = 0, iend = vNeighKFs.size(); i < iend; i++) {
      kfptr pKFi = vNeighKFs[i];
      pKFi->mBALocalForKF = pKF->mId;

      if(!pKFi->isBad()) {
        lLocalKeyFrames.push_back(pKFi);
      }
    }

    // Local MapPoints seen in Local KeyFrames
    list<mpptr> lLocalMapPoints;

    for(list<kfptr>::iterator lit = lLocalKeyFrames.begin() , lend = lLocalKeyFrames.end(); lit != lend; lit++) {
      vector<mpptr> vpMPs = (*lit)->GetMapPointMatches();

      for(vector<mpptr>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
        mpptr pMP = *vit;

        if(pMP)
          if(!pMP->isBad())
            if(pMP->mBALocalForKF != pKF->mId) {
              lLocalMapPoints.push_back(pMP);
              pMP->mBALocalForKF = pKF->mId;
            }
      }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<kfptr> lFixedCameras;

    for(list<mpptr>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
      map<kfptr, size_t> observations = (*lit)->GetObservations();

      for(map<kfptr, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
        kfptr pKFi = mit->first;

        if(pKFi->mBALocalForKF != pKF->mId && pKFi->mBAFixedForKF != pKF->mId) {
          pKFi->mBAFixedForKF = pKF->mId;

          if(!pKFi->isBad()) {
            lFixedCameras.push_back(pKFi);
          }
        }
      }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag) {
      optimizer.setForceStopFlag(pbStopFlag);
    }

    //    unsigned long maxKFid = 0;
    //    size_t idrange = 1000000;
    //    size_t maxKfId = 4000000;

    // Set Local KeyFrame vertices
    for(list<kfptr>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
      kfptr pKFi = *lit;

      if(pKFi->mId.first >= IDRANGE) {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": KF index out of bounds" << endl;
        throw infrastructure_ex();
      }

      g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
      vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
      vSE3->setId(Optimizer::GetID(pKFi->mId, true));
      vSE3->setFixed(pKFi->mId.first == 0 && pKFi->mId.second == ClientId);
      optimizer.addVertex(vSE3);
      //        if(pKFi->mnId>maxKFid)
      //            maxKFid=pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for(list<kfptr>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++) {
      kfptr pKFi = *lit;

      if(pKFi->mId.first >= IDRANGE) {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": KF index out of bounds" << endl;
        throw infrastructure_ex();
      }

      g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
      vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
      vSE3->setId(Optimizer::GetID(pKFi->mId, true));
      vSE3->setFixed(true);
      optimizer.addVertex(vSE3);
      //        if(pKFi->mnId>maxKFid)
      //            maxKFid=pKFi->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<kfptr> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<mpptr> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);

    for(list<mpptr>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
      mpptr pMP = *lit;

      if(pMP->mId.first >= IDRANGE) {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": MP index out of bounds" << endl;
        throw infrastructure_ex();
      }

      g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
      vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
      const int id = Optimizer::GetID(pMP->mId, false);
      vPoint->setId(id);
      vPoint->setMarginalized(true);
      optimizer.addVertex(vPoint);

      const map<kfptr, size_t> observations = pMP->GetObservations();

      //Set edges
      for(map<kfptr, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
        kfptr pKFi = mit->first;

        if(pKFi->mId.first >= IDRANGE) {
          cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": KF index out of bounds" << endl;
          throw infrastructure_ex();
        }

        if(!pKFi->isBad()) {
          const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

          // Monocular observation
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(Optimizer::GetID(pKFi->mId, true))));
          e->setMeasurement(obs);
          const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          e->fx = pKFi->fx;
          e->fy = pKFi->fy;
          e->cx = pKFi->cx;
          e->cy = pKFi->cy;

          optimizer.addEdge(e);
          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(pKFi);
          vpMapPointEdgeMono.push_back(pMP);
        }
      }
    }

    if(pbStopFlag)
      if(*pbStopFlag) {
        return;
      }

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore = true;

    if(pbStopFlag)
      if(*pbStopFlag) {
        bDoMore = false;
      }

    if(bDoMore) {

      // Check inlier observations
      for(size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        mpptr pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad()) {
          continue;
        }

        if(e->chi2() > 5.991 || !e->isDepthPositive()) {
          e->setLevel(1);
        }

        e->setRobustKernel(0);
      }

      // Optimize again without the outliers

      optimizer.initializeOptimization(0);
      optimizer.optimize(10);

    }

    vector<pair<kfptr, mpptr> > vToErase;
    vToErase.reserve(vpEdgesMono.size());

    // Check inlier observations
    for(size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
      mpptr pMP = vpMapPointEdgeMono[i];

      if(pMP->isBad()) {
        continue;
      }

      if(e->chi2() > 5.991 || !e->isDepthPositive()) {
        kfptr pKFi = vpEdgeKFMono[i];
        vToErase.push_back(make_pair(pKFi, pMP));
      }
    }

    // Get Map Mutex
    //    unique_lock<mutex> lock(pMap->mMutexMapUpdate);
    if(SysState != eSystemState::SERVER)
      while(!pMap->LockMapUpdate()) {
        usleep(pMap->GetLockSleep());
      }

    if(!vToErase.empty()) {
      for(size_t i = 0; i < vToErase.size(); i++) {
        kfptr pKFi = vToErase[i].first;
        mpptr pMPi = vToErase[i].second;
        pKFi->EraseMapPointMatch(pMPi);
        pMPi->EraseObservation(pKFi);
      }
    }

    // Recover optimized data

    //Keyframes
    for(list<kfptr>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
      kfptr pKF = *lit;
      g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(Optimizer::GetID(pKF->mId, true)));
      g2o::SE3Quat SE3quat = vSE3->estimate();
      pKF->SetPose(Converter::toCvMat(SE3quat), false);
    }

    //Points
    for(list<mpptr>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
      mpptr pMP = *lit;
      g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(Optimizer::GetID(pMP->mId, false)));
      pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()), false);
      //        cout << "Optimizer::LocalBundleAdjustment(...) -> pMP->UpdateNormalAndDepth()" << endl;
      pMP->UpdateNormalAndDepth();
    }

    if(SysState != eSystemState::SERVER) {
      pMap->UnLockMapUpdate();
    }
  }

  void Optimizer::LocalBundleAdjustment(std::vector<kfptr> pKFs, bool* pbStopFlag, mapptr pMap, size_t ClientId, eSystemState SysState) {
    //    const idpair zeropair = make_pair(0,0);

    // Local KeyFrames: First Breath Search from Current Keyframe
    std::vector<std::list<kfptr>> lLocalKeyFrames;

		for(int i = 0, iend = pKFs.size(); i < iend; i++) {
			std::list<kfptr> tmp;
			tmp.push_back(pKFs[i]);
			lLocalKeyFrames.push_back(tmp);
			pKFs[i]->mBALocalForKF = pKFs[i]->mId;
		}

		std::vector< std::vector<kfptr> > vNeighKFs;
		for(auto i : pKFs) {
			vNeighKFs.push_back(i->GetVectorCovisibleKeyFrames());
		}

		//const vector<kfptr> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();

		for(int j = 0, jend = vNeighKFs.size(); j < jend; j++) {
			for(int i = 0, iend = vNeighKFs[j].size(); i < iend; i++) {
			//for(int i = 0, iend = vNeighKFs.size(); i < iend; i++) {
				//kfptr pKFi = vNeighKFs[i];
				kfptr pKFi = vNeighKFs[j][i];

				bool exists = false;
				for(int k = 0, kend = pKFs.size(); k < jend; k++) {
					if(pKFi->mBALocalForKF == pKFs[k]->mId) {
						exists = true;
					}
				}

				if((!pKFi->isBad()) && (false == exists)) {
					pKFi->mBALocalForKF = pKFs[j]->mId;
					//pKFi->mBALocalForKF = pKF->mId;

					lLocalKeyFrames[j].push_back(pKFi);  // Result in duplicated vertexes
				}
			}
		}

    // Local MapPoints seen in Local KeyFrames
    std::vector<std::list<mpptr>> lLocalMapPoints(pKFs.size());

		for(int i = 0, iend = pKFs.size(); i < iend; i++) {
			for(list<kfptr>::iterator lit = lLocalKeyFrames[i].begin() , lend = lLocalKeyFrames[i].end(); lit != lend; lit++) {
				vector<mpptr> vpMPs = (*lit)->GetMapPointMatches();

				for(vector<mpptr>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
					mpptr pMP = *vit;

					if(pMP) {
						if(!pMP->isBad()) {
							// Check if map point is already added
							bool exists = false;
							for(int j = 0, jend = pKFs.size(); j < jend; j++) {
								if(pMP->mBALocalForKF == pKFs[j]->mId) {
									exists = true;
								}
							}

							if(false == exists) {
								lLocalMapPoints[i].push_back(pMP);  // Result in duplicated vertexes
								pMP->mBALocalForKF = pKFs[i]->mId;
							}
						}
					}
				}
			}
		}

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<kfptr> lFixedCameras;

		for(int i = 0, iend = pKFs.size(); i < iend; i++) {
			for(list<mpptr>::iterator lit = lLocalMapPoints[i].begin(), lend = lLocalMapPoints[i].end(); lit != lend; lit++) {
				map<kfptr, size_t> observations = (*lit)->GetObservations();

				for(map<kfptr, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
					kfptr pKFi = mit->first;

					bool exists = false;
					for(int j = 0, jend = pKFs.size(); j < iend; j++) {
						if((pKFi->mBALocalForKF == pKFs[j]->mId) || (pKFi->mBAFixedForKF == pKFs[j]->mId)) {
							exists = true;
						}
					}

					if(false == exists) {
						pKFi->mBAFixedForKF = pKFs[i]->mId;

						if(!pKFi->isBad()) {
							lFixedCameras.push_back(pKFi);
						}
					}
				}
			}
		}

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag) {
      optimizer.setForceStopFlag(pbStopFlag);
    }

    //    unsigned long maxKFid = 0;
    //    size_t idrange = 1000000;
    //    size_t maxKfId = 4000000;

    // Set Local KeyFrame vertices
    for(int i = 0, iend = pKFs.size(); i < iend; i++) {
      for(list<kfptr>::iterator lit = lLocalKeyFrames[i].begin(), lend = lLocalKeyFrames[i].end(); lit != lend; lit++) {
        kfptr pKFi = *lit;

				if(pKFi->mId.first >= IDRANGE) {
					cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": KF index out of bounds" << endl;
					throw infrastructure_ex();
				}

				g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
				vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
				vSE3->setId(Optimizer::GetID(pKFi->mId, true));
				vSE3->setFixed(pKFi->mId.first == 0 && pKFi->mId.second == ClientId);
				optimizer.addVertex(vSE3);
				//        if(pKFi->mnId>maxKFid)
				//            maxKFid=pKFi->mnId;
			}
		}

    // Set Fixed KeyFrame vertices
    for(list<kfptr>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++) {
      kfptr pKFi = *lit;

      if(pKFi->mId.first >= IDRANGE) {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": KF index out of bounds" << endl;
        throw infrastructure_ex();
      }

      g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
      vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
      vSE3->setId(Optimizer::GetID(pKFi->mId, true));
      vSE3->setFixed(true);
      optimizer.addVertex(vSE3);
      //        if(pKFi->mnId>maxKFid)
      //            maxKFid=pKFi->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<kfptr> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<mpptr> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);

		for(int i = 0, iend = pKFs.size(); i < iend; i++) {
			for(list<mpptr>::iterator lit = lLocalMapPoints[i].begin(), lend = lLocalMapPoints[i].end(); lit != lend; lit++) {
				mpptr pMP = *lit;

				if(pMP->mId.first >= IDRANGE) {
					cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": MP index out of bounds" << endl;
					throw infrastructure_ex();
				}

				g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
				vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
				const int id = Optimizer::GetID(pMP->mId, false);
				vPoint->setId(id);
				vPoint->setMarginalized(true);
				optimizer.addVertex(vPoint);

				const map<kfptr, size_t> observations = pMP->GetObservations();

				//Set edges
				for(map<kfptr, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
					kfptr pKFi = mit->first;

					if(pKFi->mId.first >= IDRANGE) {
						cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": KF index out of bounds" << endl;
						throw infrastructure_ex();
					}

					if(!pKFi->isBad()) {
						const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

						// Monocular observation
						Eigen::Matrix<double, 2, 1> obs;
						obs << kpUn.pt.x, kpUn.pt.y;

						g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
						e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(Optimizer::GetID(pKFi->mId, true))));
						e->setMeasurement(obs);
						const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
						e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(thHuberMono);

						e->fx = pKFi->fx;
						e->fy = pKFi->fy;
						e->cx = pKFi->cx;
						e->cy = pKFi->cy;

						optimizer.addEdge(e);
						vpEdgesMono.push_back(e);
						vpEdgeKFMono.push_back(pKFi);
						vpMapPointEdgeMono.push_back(pMP);
					}
				}
			}
		}

    if(pbStopFlag)
      if(*pbStopFlag) {
        return;
      }

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore = true;

    if(pbStopFlag)
      if(*pbStopFlag) {
        bDoMore = false;
      }

    if(bDoMore) {

      // Check inlier observations
      for(size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        mpptr pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad()) {
          continue;
        }

        if(e->chi2() > 5.991 || !e->isDepthPositive()) {
          e->setLevel(1);
        }

        e->setRobustKernel(0);
      }

      // Optimize again without the outliers

      optimizer.initializeOptimization(0);
      optimizer.optimize(10);

    }

    vector<pair<kfptr, mpptr> > vToErase;
    vToErase.reserve(vpEdgesMono.size());

    // Check inlier observations
    for(size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
      mpptr pMP = vpMapPointEdgeMono[i];

      if(pMP->isBad()) {
        continue;
      }

      if(e->chi2() > 5.991 || !e->isDepthPositive()) {
        kfptr pKFi = vpEdgeKFMono[i];
        vToErase.push_back(make_pair(pKFi, pMP));
      }
    }

    // Get Map Mutex
    //    unique_lock<mutex> lock(pMap->mMutexMapUpdate);
    if(SysState != eSystemState::SERVER)
      while(!pMap->LockMapUpdate()) {
        usleep(pMap->GetLockSleep());
      }

    if(!vToErase.empty()) {
      for(size_t i = 0; i < vToErase.size(); i++) {
        kfptr pKFi = vToErase[i].first;
        mpptr pMPi = vToErase[i].second;
        pKFi->EraseMapPointMatch(pMPi);
        pMPi->EraseObservation(pKFi);
      }
    }

    // Recover optimized data

    //Keyframes
    for(int i = 0, iend = pKFs.size(); i < iend; i++) {
      for(list<kfptr>::iterator lit = lLocalKeyFrames[i].begin(), lend = lLocalKeyFrames[i].end(); lit != lend; lit++) {
        kfptr pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(Optimizer::GetID(pKF->mId, true)));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat), false);
      }
    }

    //Points
    for(int i = 0, iend = pKFs.size(); i < iend; i++) {
      for(list<mpptr>::iterator lit = lLocalMapPoints[i].begin(), lend = lLocalMapPoints[i].end(); lit != lend; lit++) {
        mpptr pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(Optimizer::GetID(pMP->mId, false)));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()), false);
        //        cout << "Optimizer::LocalBundleAdjustment(...) -> pMP->UpdateNormalAndDepth()" << endl;
        pMP->UpdateNormalAndDepth();
      }
    }

    if(SysState != eSystemState::SERVER) {
      pMap->UnLockMapUpdate();
    }
  }

  void Optimizer::MapFusionGBA(mapptr pMap, size_t ClientId, int nIterations, bool* pbStopFlag, idpair nLoopKF, const bool bRobust) {
    /*
    int nVertexesCounted = 0;
    int nEdgesCounted = 0;
    */

    //prepare structures

    vector<kfptr> vpKFs = pMap->GetAllKeyFrames();
    vector<mpptr> vpMP = pMap->GetAllMapPoints();

    const idpair zeropair = make_pair(0, ClientId);

    idpair FixedId = (*(pMap->mvpKeyFrameOrigins.begin()))->mId;

    //GBA

    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag) {
      optimizer.setForceStopFlag(pbStopFlag);
    }

    //    long unsigned int maxKFid = 0;
    size_t maxKFid = 0;

    // Set KeyFrame vertices
    cout << "----- Add KFs" << endl;

    for(size_t i = 0; i < vpKFs.size(); i++) {
      kfptr pKF = vpKFs[i];

      if(pKF->isBad()) {
        continue;
      }

      g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
      vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));

      vSE3->setId(pKF->mUniqueId);

      vSE3->setFixed(pKF->mId == FixedId);
      optimizer.addVertex(vSE3);
      //nVertexesCounted++;

      if(pKF->mUniqueId > maxKFid) {
        maxKFid = pKF->mUniqueId;
      }
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    cout << "----- Add MPs" << endl;

    for(size_t i = 0; i < vpMP.size(); i++) {
      mpptr pMP = vpMP[i];

      if(pMP->isBad()) {
        continue;
      }

      g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
      vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));

      const size_t id = pMP->mUniqueId;
      vPoint->setId(id);

      vPoint->setMarginalized(true);
      optimizer.addVertex(vPoint);
      //nVertexesCounted++;

      const map<kfptr, size_t> observations = pMP->GetObservations();

      int nEdges = 0;

      //SET EDGES
      for(map<kfptr, size_t>::const_iterator mit = observations.begin(); mit != observations.end(); mit++) {

        kfptr pKF = mit->first;

        if(pKF->isBad() || pKF->mUniqueId > maxKFid) {
          continue;
        }

        nEdges++;

        const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];


        Eigen::Matrix<double, 2, 1> obs;
        obs << kpUn.pt.x, kpUn.pt.y;

        g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mUniqueId)));
        e->setMeasurement(obs);
        const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

        if(bRobust) {
          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuber2D);
        }

        e->fx = pKF->fx;
        e->fy = pKF->fy;
        e->cx = pKF->cx;
        e->cy = pKF->cy;

        optimizer.addEdge(e);
        //nEdgesCounted++;
      }

      if(nEdges == 0) {
        optimizer.removeVertex(vPoint);
        vbNotIncludedMP[i] = true;

      } else {
        vbNotIncludedMP[i] = false;
      }
    }

    /*
    std::cout << "GBA nVertexes: " << nVertexesCounted << std::endl;
    std::cout << "GBA nEdges: " << nEdgesCounted << std::endl;
    */

    // Optimize!
    cout << "----- Optimize" << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    cout << "----- Recover KFs" << endl;

    for(size_t i = 0; i < vpKFs.size(); i++) {
      kfptr pKF = vpKFs[i];

      if(pKF->isBad()) {
        continue;
      }

      g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mUniqueId));
      g2o::SE3Quat SE3quat = vSE3->estimate();

      if(nLoopKF == zeropair) {
        pKF->SetPose(Converter::toCvMat(SE3quat), true);

      } else {
        pKF->mTcwGBA.create(4, 4, CV_32F);
        Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
        pKF->mBAGlobalForKF = nLoopKF;
      }
    }

    //Points
    cout << "----- Recover MPs" << endl;

    for(size_t i = 0; i < vpMP.size(); i++) {
      if(vbNotIncludedMP[i]) {
        continue;
      }

      mpptr pMP = vpMP[i];

      if(pMP->isBad()) {
        continue;
      }

      g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mUniqueId));

      if(nLoopKF == zeropair) {
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()), true);
        pMP->UpdateNormalAndDepth();

      } else {
        pMP->mPosGBA.create(3, 1, CV_32F);
        Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
        pMP->mBAGlobalForKF = nLoopKF;
      }
    }
  }

  int Optimizer::OptimizeSim3(kfptr pKF1, kfptr pKF2, std::vector<mpptr> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, bool bFixScale) {
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Calibration
    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    // Camera poses
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();

    // Set Sim3 vertex
    g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();
    vSim3->_fix_scale = bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1.at<float>(0, 2);
    vSim3->_principle_point1[1] = K1.at<float>(1, 2);
    vSim3->_focal_length1[0] = K1.at<float>(0, 0);
    vSim3->_focal_length1[1] = K1.at<float>(1, 1);
    vSim3->_principle_point2[0] = K2.at<float>(0, 2);
    vSim3->_principle_point2[1] = K2.at<float>(1, 2);
    vSim3->_focal_length2[0] = K2.at<float>(0, 0);
    vSim3->_focal_length2[1] = K2.at<float>(1, 1);
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int N = vpMatches1.size();
    const vector<mpptr> vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
    vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2 * N);
    vpEdges12.reserve(2 * N);
    vpEdges21.reserve(2 * N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;

    for(int i = 0; i < N; i++) {
      if(!vpMatches1[i]) {
        continue;
      }

      mpptr pMP1 = vpMapPoints1[i];
      mpptr pMP2 = vpMatches1[i];

      const int id1 = 2 * i + 1;
      const int id2 = 2 * (i + 1);

      const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

      if(pMP1 && pMP2) {
        if(!pMP1->isBad() && !pMP2->isBad() && i2 >= 0) {
          g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
          cv::Mat P3D1w = pMP1->GetWorldPos();
          cv::Mat P3D1c = R1w * P3D1w + t1w;
          vPoint1->setEstimate(Converter::toVector3d(P3D1c));
          vPoint1->setId(id1);
          vPoint1->setFixed(true);
          optimizer.addVertex(vPoint1);

          g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
          cv::Mat P3D2w = pMP2->GetWorldPos();
          cv::Mat P3D2c = R2w * P3D2w + t2w;
          vPoint2->setEstimate(Converter::toVector3d(P3D2c));
          vPoint2->setId(id2);
          vPoint2->setFixed(true);
          optimizer.addVertex(vPoint2);

        } else {
          continue;
        }

      } else {
        continue;
      }

      nCorrespondences++;

      // Set edge x1 = S12*X2
      Eigen::Matrix<double, 2, 1> obs1;
      const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
      obs1 << kpUn1.pt.x, kpUn1.pt.y;

      g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
      e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
      e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
      e12->setMeasurement(obs1);
      const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
      e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

      g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
      e12->setRobustKernel(rk1);
      rk1->setDelta(deltaHuber);
      optimizer.addEdge(e12);

      // Set edge x2 = S21*X1
      Eigen::Matrix<double, 2, 1> obs2;
      const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
      obs2 << kpUn2.pt.x, kpUn2.pt.y;

      g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();

      e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
      e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
      e21->setMeasurement(obs2);
      float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
      e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

      g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
      e21->setRobustKernel(rk2);
      rk2->setDelta(deltaHuber);
      optimizer.addEdge(e21);

      vpEdges12.push_back(e12);
      vpEdges21.push_back(e21);
      vnIndexEdge.push_back(i);
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    int nBad = 0;

    for(size_t i = 0; i < vpEdges12.size(); i++) {
      g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
      g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];

      if(!e12 || !e21) {
        continue;
      }

      if(e12->chi2() > th2 || e21->chi2() > th2) {
        size_t idx = vnIndexEdge[i];
        vpMatches1[idx] = static_cast<mpptr>(NULL);
        optimizer.removeEdge(e12);
        optimizer.removeEdge(e21);
        vpEdges12[i] = static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
        vpEdges21[i] = static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
        nBad++;
      }
    }

    int nMoreIterations;

    if(nBad > 0) {
      nMoreIterations = 10;

    } else {
      nMoreIterations = 5;
    }

    if(nCorrespondences - nBad < 10) {
      return 0;
    }

    // Optimize again only with inliers

    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn = 0;

    for(size_t i = 0; i < vpEdges12.size(); i++) {
      g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
      g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];

      if(!e12 || !e21) {
        continue;
      }

      if(e12->chi2() > th2 || e21->chi2() > th2) {
        size_t idx = vnIndexEdge[i];
        vpMatches1[idx] = static_cast<mpptr>(NULL);

      } else {
        nIn++;
      }
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12 = vSim3_recov->estimate();

    return nIn;
  }

  void Optimizer::OptimizeEssentialGraphLoopClosure(mapptr pMap, kfptr pLoopKF, kfptr pCurKF,
      const KeyFrameAndPose &NonCorrectedSim3,
      const KeyFrameAndPose &CorrectedSim3,
      const map<kfptr, set<kfptr> > &LoopConnections, const bool &bFixScale) {
    //    map<idpair,int> mKfToVertexId;
    //    int uKfNextId = 0;

    //    idpair IDPLoopKf = make_pair(pLoopKF->mnId,pLoopKF->mClientId);
    //    idpair IDPCurKf = make_pair(pCurKF->mnId,pCurKF->mClientId);

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const vector<kfptr> vpKFs = pMap->GetAllKeyFrames();
    const vector<mpptr> vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->GetMaxKFidUnique(); //vpKFs.size();

    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid + 1);
    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid + 1);
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid + 1);

    const int minFeat = 100;

    // Set KeyFrame vertices
    for(size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
      kfptr pKF = vpKFs[i];

      if(pKF->isBad()) {
        continue;
      }

      g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

      const size_t nIDi = pKF->mUniqueId;
      //        const int nIDi = uKfNextId;
      //        idpair idp = make_pair(pKF->mnId,pKF->mClientId);
      //        mKfToVertexId[idp] = uKfNextId;
      //        ++uKfNextId;

      KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

      if(it != CorrectedSim3.end()) {
        vScw[nIDi] = it->second;
        VSim3->setEstimate(it->second);

      } else {
        Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
        Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKF->GetTranslation());
        g2o::Sim3 Siw(Rcw, tcw, 1.0);
        vScw[nIDi] = Siw;
        VSim3->setEstimate(Siw);
      }

      if(pKF == pLoopKF) {
        VSim3->setFixed(true);
      }

      VSim3->setId(nIDi);
      VSim3->setMarginalized(false);
      VSim3->_fix_scale = bFixScale;

      optimizer.addVertex(VSim3);

      vpVertices[nIDi] = VSim3;
    }

    set<pair<long unsigned int, long unsigned int> > sInsertedEdges;

    const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

    // Set Loop edges
    for(map<kfptr , set<kfptr > >::const_iterator mit = LoopConnections.begin(), mend = LoopConnections.end(); mit != mend; mit++) {
      kfptr pKF = mit->first;

      if(pKF->isBad()) {
        continue;
      }

      //        idpair idpi = make_pair(pKF->mnId,pKF->mClientId);
      //        {
      //            map<idpair,int>::iterator miti = mKfToVertexId.find(idpi);
      //            if(miti==mKfToVertexId.end())
      //            {
      //                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::OptimizeEssentialGraphLoopClosure(...)\": no nIDi found for KF (Set Loop edges)" << endl;
      //                cout << "KF " << pKF->mnId << "|" << pKF->mClientId << endl;
      //            }
      //        }
      //        const long unsigned int nIDi = static_cast<long unsigned int>(mKfToVertexId[idpi]);
      const size_t nIDi = pKF->mUniqueId;

      const set<kfptr> &spConnections = mit->second;
      const g2o::Sim3 Siw = vScw[nIDi];
      const g2o::Sim3 Swi = Siw.inverse();

      for(set<kfptr>::const_iterator sit = spConnections.begin(), send = spConnections.end(); sit != send; sit++) {
        if((*sit)->isBad()) {
          continue;
        }

        //            idpair idpj = make_pair((*sit)->mnId,(*sit)->mClientId);
        //            {
        //                map<idpair,int>::iterator mitj = mKfToVertexId.find(idpj);
        //                if(mitj==mKfToVertexId.end())
        //                {
        //                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #2: In \"Optimizer::OptimizeEssentialGraphLoopClosure(...)\": no nIDj found for KF (Set Loop edges)" << endl;
        //                    cout << "KF " << (*sit)->mnId << "|" << (*sit)->mClientId << endl;
        //                }
        //            }
        //            const long unsigned int nIDj = static_cast<long unsigned int>(mKfToVertexId[idpj]);
        const size_t nIDj = (*sit)->mUniqueId;

        //            if((idpi!=IDPCurKf || idpj!=IDPLoopKf) && pKF->GetWeight(*sit)<minFeat)
        if((nIDi != pCurKF->mUniqueId || nIDj != pLoopKF->mUniqueId) && pKF->GetWeight(*sit) < minFeat) {
          continue;
        }

        const g2o::Sim3 Sjw = vScw[nIDj];
        const g2o::Sim3 Sji = Sjw * Swi;

        g2o::EdgeSim3* e = new g2o::EdgeSim3();
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
        e->setMeasurement(Sji);

        e->information() = matLambda;

        optimizer.addEdge(e);

        sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
      }
    }

    // Set normal edges
    for(size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
      kfptr pKF = vpKFs[i];

      //        idpair idpi = make_pair(pKF->mnId,pKF->mClientId);
      //        {
      //            map<idpair,int>::iterator miti = mKfToVertexId.find(idpi);
      //            if(miti==mKfToVertexId.end())
      //            {
      //                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #3: In \"Optimizer::OptimizeEssentialGraphLoopClosure(...)\": no nIDi found for KF (Set normal edges)" << endl;
      //                cout << "KF " << pKF->mnId << "|" << pKF->mClientId << endl;
      //            }
      //        }
      //        const int nIDi = mKfToVertexId[idpi];
      const size_t nIDi = pKF->mUniqueId;

      g2o::Sim3 Swi;

      KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

      if(iti != NonCorrectedSim3.end()) {
        Swi = (iti->second).inverse();

      } else {
        Swi = vScw[nIDi].inverse();
      }

      kfptr pParentKF = pKF->GetParent();

      // Spanning tree edge
      if(pParentKF) {
        //            idpair idpj = make_pair(pParentKF->mnId,pParentKF->mClientId);
        //            {
        //                map<idpair,int>::iterator mitj = mKfToVertexId.find(idpj);
        //                if(mitj==mKfToVertexId.end())
        //                {
        //                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #4: In \"Optimizer::OptimizeEssentialGraphLoopClosure(...)\": no nIDj found for KF (Spanning tree edge)" << endl;
        //                    cout << "KF " << pParentKF->mnId << "|" << pParentKF->mClientId << endl;
        //                }
        //            }
        //            int nIDj = mKfToVertexId[idpj];
        const size_t nIDj = pParentKF->mUniqueId;

        g2o::Sim3 Sjw;

        KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

        if(itj != NonCorrectedSim3.end()) {
          Sjw = itj->second;

        } else {
          Sjw = vScw[nIDj];
        }

        g2o::Sim3 Sji = Sjw * Swi;

        g2o::EdgeSim3* e = new g2o::EdgeSim3();
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
        e->setMeasurement(Sji);

        e->information() = matLambda;
        optimizer.addEdge(e);
      }

      // Loop edges
      const set<kfptr> sLoopEdges = pKF->GetLoopEdges();

      for(set<kfptr>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++) {
        kfptr pLKF = *sit;

        //            idpair idpj = make_pair(pLKF->mnId,pLKF->mClientId);
        //            {
        //                map<idpair,int>::iterator mitj = mKfToVertexId.find(idpj);
        //                if(mitj==mKfToVertexId.end())
        //                {
        //                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #5: In \"Optimizer::OptimizeEssentialGraphLoopClosure(...)\": no nIDj found for KF (KF Loop edges)" << endl;
        //                    cout << "KF " << pLKF->mnId << "|" << pLKF->mClientId << endl;
        //                }
        //            }
        //            int nIDj = mKfToVertexId[idpj];
        const size_t nIDj = pLKF->mUniqueId;

        if(nIDj < nIDi) {
          g2o::Sim3 Slw;

          KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

          if(itl != NonCorrectedSim3.end()) {
            Slw = itl->second;

          } else {
            Slw = vScw[nIDj];
          }

          g2o::Sim3 Sli = Slw * Swi;
          g2o::EdgeSim3* el = new g2o::EdgeSim3();
          el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
          el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
          el->setMeasurement(Sli);
          el->information() = matLambda;
          optimizer.addEdge(el);
        }
      }

      // Covisibility graph edges
      const vector<kfptr> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);

      for(vector<kfptr>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++) {
        kfptr pKFn = *vit;

        if((*vit)->isBad()) {
          continue;
        }

        if(pKFn && pKFn != pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn)) {
          //                idpair idpj = make_pair(pKFn->mnId,pKFn->mClientId);
          //                {
          //                    map<idpair,int>::iterator mitj = mKfToVertexId.find(idpj);
          //                    if(mitj==mKfToVertexId.end())
          //                    {
          //                        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #6: In \"Optimizer::OptimizeEssentialGraphLoopClosure(...)\": no nIDj found for KF (Covisibility graph edges)" << endl;
          //                        cout << "KF " << pKFn->mnId << "|" << pKFn->mClientId << endl;
          //                    }
          //                }
          //                int nIDj = mKfToVertexId[idpj];
          const size_t nIDj = pKFn->mUniqueId;

          if(!pKFn->isBad() && nIDj < nIDi) {
            if(sInsertedEdges.count(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)))) {
              continue;
            }

            g2o::Sim3 Snw;

            KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

            if(itn != NonCorrectedSim3.end()) {
              Snw = itn->second;

            } else {
              Snw = vScw[nIDj];
            }

            g2o::Sim3 Sni = Snw * Swi;

            g2o::EdgeSim3* en = new g2o::EdgeSim3();
            en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            en->setMeasurement(Sni);
            en->information() = matLambda;
            optimizer.addEdge(en);
          }
        }
      }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i = 0; i < vpKFs.size(); i++) {
      kfptr pKFi = vpKFs[i];

      //        idpair idpi = make_pair(pKFi->mnId,pKFi->mClientId);
      //        {
      //            map<idpair,int>::iterator miti = mKfToVertexId.find(idpi);
      //            if(miti==mKfToVertexId.end())
      //            {
      //                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #7: In \"Optimizer::OptimizeEssentialGraphLoopClosure(...)\": no nIDi found for KF (SE3 Pose Recovering)" << endl;
      //                cout << "KF " << pKFi->mnId << "|" << pKFi->mClientId << endl;
      //            }
      //        }
      //        const int nIDi = mKfToVertexId[idpi];
      const size_t nIDi = pKFi->mUniqueId;

      //        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
      g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(pKFi->mUniqueId));
      g2o::Sim3 CorrectedSiw =  VSim3->estimate();
      vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
      Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
      Eigen::Vector3d eigt = CorrectedSiw.translation();
      double s = CorrectedSiw.scale();

      eigt *= (1. / s); //[R t/s;0 1]

      cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);

      pKFi->SetPose(Tiw, true);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
      mpptr pMP = vpMPs[i];

      if(pMP->isBad()) {
        continue;
      }

      size_t nIDr;

      //        if(pMP->mnCorrectedByKF==pCurKF->mnId && pMP->mnCorrectedByKFClientId==pCurKF->mClientId) //ID Tag
      if(pMP->mCorrectedByKF_LC == pCurKF->mId) { //ID Tag
        //            idpair idpr = make_pair(pMP->mnCorrectedReference,pMP->mnCorrectedReferenceClientId);
        //            {
        //                map<idpair,int>::iterator mitr = mKfToVertexId.find(idpr);
        //                if(mitr==mKfToVertexId.end())
        //                {
        //                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #8: In \"Optimizer::OptimizeEssentialGraphLoopClosure(...)\": no nIDr found for KF (Correct points(if))" << endl;
        //                    cout << "KF " << pMP->mnCorrectedReference << "|" << pMP->mnCorrectedReferenceClientId << endl;
        //                }
        //            }
        //            nIDr = mKfToVertexId[idpr];
        nIDr = pMP->mCorrectedReference_LC;

      } else {
        kfptr pRefKF = pMP->GetReferenceKeyFrame();

        //            idpair idpr = make_pair(pRefKF->mnId,pRefKF->mClientId);
        //            {
        //                map<idpair,int>::iterator mitr = mKfToVertexId.find(idpr);
        //                if(mitr==mKfToVertexId.end())
        //                {
        //                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #9: In \"Optimizer::OptimizeEssentialGraphLoopClosure(...)\": no nIDr found for KF (Correct points(else))" << endl;
        //                    cout << "KF " << pRefKF->mnId << "|" << pRefKF->mClientId << endl;
        //                    cout << "Ref KF bad?: " << pRefKF->isBad() << endl;
        //                }
        //            }
        //            nIDr = mKfToVertexId[idpr];
        nIDr = pRefKF->mUniqueId;
      }

      g2o::Sim3 Srw = vScw[nIDr];
      g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

      cv::Mat P3Dw = pMP->GetWorldPos();
      Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
      Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

      cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
      pMP->SetWorldPos(cvCorrectedP3Dw, true);

      pMP->UpdateNormalAndDepth();
    }

  }


  void Optimizer::OptimizeEssentialGraphMapFusionV2(mapptr pMap, kfptr pLoopKF, kfptr pCurKF,
      const map<kfptr, set<kfptr> > &LoopConnections, const bool &bFixScale) {

    /*
    int nEdgesCounted = 0;
    int nVertexesCounted = 0;
    */

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    //g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const vector<kfptr> vpKFs = pMap->GetAllKeyFrames();
    const vector<mpptr> vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->GetMaxKFidUnique(); //vpKFs.size();

    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid + 1);
    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid + 1);
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid + 1);

    const int minFeat = 100;

    // Set KeyFrame vertices
    for(size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
      kfptr pKF = vpKFs[i];

      if(pKF->isBad()) {
        continue;
      }

      g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

      const size_t nIDi = pKF->mUniqueId;

      Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
      Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKF->GetTranslation());
      g2o::Sim3 Siw(Rcw, tcw, 1.0);
      vScw[nIDi] = Siw;
      VSim3->setEstimate(Siw);

      if(pKF == pLoopKF) {
        VSim3->setFixed(true);
      }

      VSim3->setId(nIDi);
      VSim3->setMarginalized(false);
      VSim3->_fix_scale = bFixScale;

      optimizer.addVertex(VSim3);
      //nVertexesCounted++;

      vpVertices[nIDi] = VSim3;
    }

    set<pair<long unsigned int, long unsigned int> > sInsertedEdges;

    const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

    // Set Loop edges
    for(map<kfptr , set<kfptr > >::const_iterator mit = LoopConnections.begin(), mend = LoopConnections.end(); mit != mend; mit++) {
      kfptr pKF = mit->first;

      if(pKF->isBad()) {
        continue;
      }

      const size_t nIDi = pKF->mUniqueId;

      const set<kfptr> &spConnections = mit->second;
      const g2o::Sim3 Siw = vScw[nIDi];
      const g2o::Sim3 Swi = Siw.inverse();

      for(set<kfptr>::const_iterator sit = spConnections.begin(), send = spConnections.end(); sit != send; sit++) {
        if((*sit)->isBad()) {
          continue;
        }

        const size_t nIDj = (*sit)->mUniqueId;

        if((nIDi != pCurKF->mUniqueId || nIDj != pLoopKF->mUniqueId) && pKF->GetWeight(*sit) < minFeat) {
          continue;
        }

        const g2o::Sim3 Sjw = vScw[nIDj];
        const g2o::Sim3 Sji = Sjw * Swi;

        g2o::EdgeSim3* e = new g2o::EdgeSim3();
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
        e->setMeasurement(Sji);

        e->information() = matLambda;

        optimizer.addEdge(e);
        //nEdgesCounted++;

        sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
      }
    }

    // Set normal edges
    for(size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
      kfptr pKF = vpKFs[i];

      const size_t nIDi = pKF->mUniqueId;

      g2o::Sim3 Swi = vScw[nIDi].inverse();

      kfptr pParentKF = pKF->GetParent();

      // Spanning tree edge
      if(pParentKF) {
        const size_t nIDj = pParentKF->mUniqueId;

        g2o::Sim3 Sjw = vScw[nIDj];

        g2o::Sim3 Sji = Sjw * Swi;

        g2o::EdgeSim3* e = new g2o::EdgeSim3();
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
        e->setMeasurement(Sji);

        e->information() = matLambda;
        optimizer.addEdge(e);
        //nEdgesCounted++;
      }

      // Loop edges
      const set<kfptr> sLoopEdges = pKF->GetLoopEdges();

      for(set<kfptr>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++) {
        kfptr pLKF = *sit;

        const size_t nIDj = pLKF->mUniqueId;

        if(nIDj < nIDi) {
          g2o::Sim3 Slw = vScw[nIDj];

          g2o::Sim3 Sli = Slw * Swi;
          g2o::EdgeSim3* el = new g2o::EdgeSim3();
          el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
          el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
          el->setMeasurement(Sli);
          el->information() = matLambda;
          optimizer.addEdge(el);
          //nEdgesCounted++;
        }
      }

      // Covisibility graph edges
      const vector<kfptr> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);

      for(vector<kfptr>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++) {
        kfptr pKFn = *vit;

        if((*vit)->isBad()) {
          continue;
        }

        if(pKFn && pKFn != pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn)) {
          const size_t nIDj = pKFn->mUniqueId;

          if(!pKFn->isBad() && nIDj < nIDi) {
            if(sInsertedEdges.count(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)))) {
              continue;
            }

            g2o::Sim3 Snw = vScw[nIDj];

            g2o::Sim3 Sni = Snw * Swi;

            g2o::EdgeSim3* en = new g2o::EdgeSim3();
            en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            en->setMeasurement(Sni);
            en->information() = matLambda;
            optimizer.addEdge(en);
            //nEdgesCounted++;
          }
        }
      }
    }

    /*
    std::cout << "Essential graph optimization nVertexes: " << nVertexesCounted << std::endl;
    std::cout << "Essential graph optimization nEdges: " << nEdgesCounted << std::endl;
    */

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i = 0; i < vpKFs.size(); i++) {
      kfptr pKFi = vpKFs[i];

      const size_t nIDi = pKFi->mUniqueId;

      g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(pKFi->mUniqueId));
      g2o::Sim3 CorrectedSiw =  VSim3->estimate();
      vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
      Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
      Eigen::Vector3d eigt = CorrectedSiw.translation();
      double s = CorrectedSiw.scale();

      eigt *= (1. / s); //[R t/s;0 1]

      cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);

      pKFi->SetPose(Tiw, true);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
      mpptr pMP = vpMPs[i];

      if(pMP->isBad()) {
        continue;
      }

      int nIDr;

      if(pMP->mCorrectedByKF_MM == pCurKF->mId) { //ID Tag
        nIDr = pMP->mCorrectedReference_MM;

      } else {
        kfptr pRefKF = pMP->GetReferenceKeyFrame();

        nIDr = pRefKF->mUniqueId;
      }

      g2o::Sim3 Srw = vScw[nIDr];
      g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

      cv::Mat P3Dw = pMP->GetWorldPos();
      Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
      Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

      cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
      pMP->SetWorldPos(cvCorrectedP3Dw, true);

      pMP->UpdateNormalAndDepth();
    }

  }


} // end ns
