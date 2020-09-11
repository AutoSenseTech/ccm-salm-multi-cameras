bool bSentOnce

uint16 mnId # unique KF id
uint8 mClientId # client id
uint32 mUniqueId
float64 dTimestamp
bool mbAck

#Grid (to speed up feature matching)
    int16 mnGridCols
    int16 mnGridRows
    float32[] mvGridElementWidthInv
    float32[] mvGridElementHeightInv

#Calibration parameters
    float32[] vfx
    float32[] vfy
    float32[] vcx
    float32[] vcy
    float32[] vinvfx
    float32[] vinvfy

#Number of jointKeyPoints
    int16 N

#Number of KeyPoints vector
    int16[] mvpkeyPointsNum

#multicamera value
KeyPointsIndexMapPlus[] vKeyPointsIndexMapPlus
Tcamji[] mvTcamji

#KeyPoints, stereo coordinate and descriptors (all associated by an index)
    VCvKeyPoint[] mvKeysMultipleUn
    VDescriptor[] mvpDescriptors

#BKF'S Descriptors (all associated by an index)
    Descriptor[] mDescriptors

#Pose relative to parent
    float32[16] mTcpred
    float32[16] mTcpar

    bool mbPoseChanged

    bool mbServerBA

    float32[16] mT_SC

#Scale
    int8 mnScaleLevels
    float32 mfScaleFactor
    float32 mfLogScaleFactor
    float32[8] mvScaleFactors
    float32[8] mvLevelSigma2
    float32[8] mvInvLevelSigma2

#Image bounds and calibration
    int16[] mvMinX
    int16[] mvMinY
    int16[] mvMaxX
    int16[] mvMaxY
    Vfloat32[] mvpK

#MapPoints associated to keypoints
    uint32[] mvpMapPoints_Ids
    uint8[] mvpMapPoints_ClientIds
    uint16[] mvpMapPoints_VectId

#Related BKFs
    uint16 mpPred_BKfId
    uint8 mpPred_BKfClientId
    uint16 mpPar_BKfId
    uint8 mpPar_BKfClientId

#Bad flags
    bool mbBad

#BF and depth
    float32 mThDepth
    float32[] mvBDepth