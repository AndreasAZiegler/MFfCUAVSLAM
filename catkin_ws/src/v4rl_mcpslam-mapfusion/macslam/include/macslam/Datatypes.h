#ifndef MCPSLAM_DATATYPES_H_
#define MCPSLAM_DATATYPES_H_

//C++
#include <boost/shared_ptr.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>

//ROS
#include <ros/ros.h>

//MAC-SLAM
#include <helper/estd.h>

#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;
using namespace estd;

namespace macslam{

//forward decs
class KeyFrame;
class MapPoint;
//--------------

enum eSystemState{
    NOTYPE=-1,
    CLIENT=0,
    SERVER=1
};

struct LoopFinderParams
{
    LoopFinderParams(int LoopRate = 3000,
                     int SolverIterations = 5,
                     int MatchesThres = 20, int InliersThres = 20, int TotalMatchesThres = 40,
                     double prob = 0.99, int minInliers = 6, int maxIterations = 300,
                     int minHitsForMerge = 3, int GBAIterations = 20, int LoopLockSleep = 1000)
        : mLoopRate(LoopRate),
          mSolverIterations(SolverIterations),
          mMatchesThres(MatchesThres), mInliersThres(InliersThres), mTotalMatchesThres(TotalMatchesThres),
          mProbability(prob), mMinInliers(minInliers), mMaxIterations(maxIterations),
          mMinHitsForMerge(minHitsForMerge), mGBAIterations(GBAIterations), mLoopLockSleep(LoopLockSleep)
        {}
    //Timing
    int mLoopRate;
    int mLoopLockSleep;
    //Loop Closure
    int mSolverIterations;
    int mMatchesThres; //matches that need to be found by SearchByBoW()
    int mInliersThres; //inliers after pose optimization
    int mTotalMatchesThres; //total matches SearchByProjection
    //RANSAC params
    double mProbability;
    int mMinInliers;
    int mMaxIterations;
    //Map Merger
    int mMinHitsForMerge;
    int mGBAIterations;
};

struct MapMergerParams
{
    MapMergerParams(int minHits = 3, int GBAIterations = 20, double ScaleFactor = 1.0, double MarkerSize = 0.1)
        : mMinHits(minHits), mGBAIterations(GBAIterations), mScaleFactor(ScaleFactor),mMarkerSize(MarkerSize) {}
    int mMinHits;
    int mGBAIterations;
    double mScaleFactor;
    double mMarkerSize;
};

struct MapMatchHit
{
public:
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
public:
    MapMatchHit(kfptr pKFCurr = nullptr, kfptr pKFMatch = nullptr, g2o::Sim3 g2oScw = g2o::Sim3(), std::vector<mpptr> vpLoopMapPoints = std::vector<mpptr>(), std::vector<mpptr> vpCurrentMatchedPoints = std::vector<mpptr>())
        : mpKFCurr(pKFCurr), mpKFMatch(pKFMatch), mg2oScw(g2oScw),
          mvpLoopMapPoints(vpLoopMapPoints), mvpCurrentMatchedPoints(vpCurrentMatchedPoints) {}
    kfptr mpKFCurr;
    kfptr mpKFMatch;
    g2o::Sim3 mg2oScw;
    std::vector<mpptr> mvpLoopMapPoints;
    std::vector<mpptr> mvpCurrentMatchedPoints;
};

struct RansacParams{
    RansacParams(double prob = 0.99, int minInliers = 6, int maxIterations = 300)
        : mProbability(prob), mMinInliers(minInliers), mMaxIterations(maxIterations)
         {}
    double mProbability;
    int mMinInliers;
    int mMaxIterations;
};

struct MapMatchingParams
{
    MapMatchingParams(int SolverIterations = 5,
                      int MatchesThres = 20, int InliersThres = 20, int TotalMatchesThres = 40,
                      double prob = 0.99, int minInliers = 6, int maxIterations = 300,
                      int minHitsForMerge = 3, int GBAIterations = 20,
                      int KFsToSkip = 10, int LockSleep = 1000)
        : mSolverIterations(SolverIterations),
          mMatchesThres(MatchesThres), mInliersThres(InliersThres), mTotalMatchesThres(TotalMatchesThres),
          mProbability(prob), mMinInliers(minInliers), mMaxIterations(maxIterations),
          mMinHitsForMerge(minHitsForMerge), mGBAIterations(GBAIterations),
          mKFsToSkip(KFsToSkip), mLockSleep(LockSleep)
          {}
    int mSolverIterations;
    int mMatchesThres; //matches that need to be found by SearchByBoW()
    int mInliersThres; //inliers after pose optimization
    int mTotalMatchesThres; //total matches SearchByProjection
    int mKFsToSkip;
    //RANSAC params
    double mProbability;
    int mMinInliers;
    int mMaxIterations;
    //Map Merger
    int mMinHitsForMerge;
    int mGBAIterations;
    //Timing
    int mLockSleep;
};

} //end namespace

#endif
