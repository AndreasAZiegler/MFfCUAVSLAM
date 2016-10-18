bool bSentOnce
uint32 mUpdateCount

uint32 mnId # unique KF id
uint32 mClientId # client id
uint32 mUniqueId
float64 dTimestamp

#Grid (to speed up feature matching)
    int16 mnGridCols
    int16 mnGridRows
    float32 mfGridElementWidthInv
    float32 mfGridElementHeightInv

#Calibration parameters
    float32 fx
    float32 fy
    float32 cx
    float32 cy
    float32 invfx
    float32 invfy

#Number of KeyPoints
    int16 N

#KeyPoints, stereo coordinate and descriptors (all associated by an index)
    machCvKeyPoint[] mvKeys
    machCvKeyPoint[] mvKeysUn
    machDescriptor[] mDescriptors

#BoW
    uint32[] mBowVec_WordIds
    float64[] mBowVec_WordValues
    uint32[] mFeatVec_NodeIds
    machUIntVec[] mFeatVec_WordIds

#Pose relative to parent (this is computed when bad flag is activated)
    float32[16] mTcp

#Scale
    int8 mnScaleLevels
    float32 mfScaleFactor
    float32 mfLogScaleFactor
    float32[8] mvScaleFactors
    float32[8] mvLevelSigma2
    float32[8] mvInvLevelSigma2

#Image bounds and calibration
    int16 mnMinX
    int16 mnMinY
    int16 mnMaxX
    int16 mnMaxY
    float32[9] mK

#SE3 Pose and camera center
    float32[16] Tcw
    float32[16] Twc
    float32[3] Ow

    bool mbPoseChanged
    bool mbPoseOnly
    bool mbLoopCorrected

    float32[4] Cw #delete later

#MapPoints associated to keypoints
    uint32[] mvpMapPoints_Ids
    uint32[] mvpMapPoints_ClientIds
    uint32[] mvpMapPoints_VectId
    bool[] mvpMapPoints_delete
    bool[] mvpMapPoints_locked

#Grid over the image to speed up feature matching
    #mach2DGrid mGridVGA
    #Grid75x48 mGrid75x48

    uint32[] mConnectedKeyFrameWeights_KFIds
    uint32[] mConnectedKeyFrameWeights_KFClientIds
    int32[] mConnectedKeyFrameWeights_Weights
    uint32[] mvpOrderedConnectedKeyFrames_Ids
    uint32[] mvpOrderedConnectedKeyFrames_ClientIds
    int32[] mvOrderedWeights

#Spanning Tree and Loop Edges
    bool mbFirstConnection
    int64 mpParent_KfId
    int64 mpParent_KfClientId
    int32[] mspChildrens_Ids
    int32[] mspChildrens_ClientIds
    int32[] mspLoopEdges_KfIds
    int32[] mspLoopEdges_KfClientIds

#Bad flags
    bool mbBad
