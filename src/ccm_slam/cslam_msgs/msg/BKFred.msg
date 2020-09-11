uint16 mnId # unique BKF id
uint8 mClientId # client id
uint32 mUniqueId
bool mbAck

#Poses
    float32[16] mTcpred
    float32[16] mTcpar

    uint16 mpPred_BKfId
    uint8 mpPred_BKfClientId

    uint16 mpPar_BKfId
    uint8 mpPar_BKfClientId

    bool mbServerBA

#Bad flags
    bool mbBad

#BF and depth
    float32 mThDepth
    float32[] mvBDepth