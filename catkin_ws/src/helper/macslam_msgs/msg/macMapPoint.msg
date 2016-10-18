bool bSentOnce
uint32 mUpdateCount

uint32 mnId # unique KF id
uint32 mClientId # client id
uint32 mUniqueId
int32 mnFirstKFid
uint32 mnFirstKfClientId

#Position in absolute coordinates
     float32[3] mWorldPos
     bool mbPoseChanged
     bool mbPoseOnly
     bool mbLoopCorrected

#Keyframes observing the point and associated index in keyframe
     uint32[] mObservations_KFIDs
     uint32[] mObservations_KFClientIDs
     uint32[] mObservations_n
     bool[] mObservations_erase
     bool[] mObservations_locked

#Mean viewing direction
     float32[3] mNormalVector
     bool mbNormalAndDepthChanged

#Best descriptor to fast matching
     uint8[32] mDescriptor
     # machDescriptor mDescriptor
     bool mbDescriptorChanged

#Reference KeyFrame
     int64 mpRefKFId
     int64 mpRefKFClientId

#Tracking counters
     int16 mnVisible
     int16 mnFound

#Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad
     int64 mpReplaced_Id
     int64 mpReplaced_ClientId

#Scale invariance distances
     float32 mfMinDistance
     float32 mfMaxDistance

#Visualization
     bool mbMultiUse
