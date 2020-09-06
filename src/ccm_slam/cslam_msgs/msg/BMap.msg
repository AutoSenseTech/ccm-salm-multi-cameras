Header header
uint32 mMsgId

BKF[] BundledKeyframes
BKFred[] BKFUpdates
MP[] MapPoints
MPred[] MPUpdates

#bool bAck
#uint32 Ack #should not be necessary any more
uint16[] vAckBKFs
uint32[] vAckMPs
uint16 WeakAckBKF
uint32 WeakAckMP

uint16 ClosestBKf_Id
uint8 ClosestBKf_ClientId
