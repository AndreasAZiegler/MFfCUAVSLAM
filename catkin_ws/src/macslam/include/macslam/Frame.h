#ifndef MCPSLAM_FRAME_H_
#define MCPSLAM_FRAME_H_

//C++
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>

//MCP SLAM
#include <helper/estd.h>
#include <macslam/Datatypes.h>
#include <macslam/ORBVocabulary.h>
#include <macslam/ORBextractor.h>
#include <macslam/MapPoint.h>
#include <macslam/KeyFrame.h>
#include <macslam/Converter.h>

#include <thirdparty/DBoW2/DBoW2/BowVector.h>
#include <thirdparty/DBoW2/DBoW2/FeatureVector.h>

namespace macslam{

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 75  //64

//forward decs
class MapPoint;
class KeyFrame;
//--------------

class Frame
{
public:
    typedef boost::shared_ptr<Frame> frameptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, extractorptr pExtractor, vocptr pVoc, cv::Mat &K, cv::Mat &distCoef, size_t ClientId);

    // Extract ORB on the image.
    void ExtractORB(const cv::Mat &im);

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(mpptr pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

public:
    // Vocabulary used for relocalization.
    vocptr mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    extractorptr mpORBextractor;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

//    // Stereo baseline multiplied by fx.
//    float mbf;

//    // Stereo baseline in meters.
//    float mb;

//    // Threshold close/far points. Close points are inserted from 1 view.
//    // Far points are inserted as in the monocular case from 2 views.
//    float mThDepth;

    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys;//, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;

//    // Corresponding stereo coordinate and depth for each keypoint.
//    // "Monocular" keypoints have a negative value.
//    std::vector<float> mvuRight;
//    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors;//, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<mpptr> mvpMapPoints;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw;

    // Current and Next Frame id.
    static long unsigned int nNextId;
//    long unsigned int mnId;
//    size_t mClientId; //ID of client that created this frame
    idpair mId;

    // Reference Keyframe.
    kfptr mpReferenceKF;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc
};

} //end namespace


#endif
