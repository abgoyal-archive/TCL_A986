

#ifndef __RTMP_MCU_H__
#define __RTMP_MCU_H__

int RtmpAsicEraseFirmware(struct rt_rtmp_adapter *pAd);

int RtmpAsicLoadFirmware(struct rt_rtmp_adapter *pAd);

int RtmpAsicSendCommandToMcu(struct rt_rtmp_adapter *pAd,
			     u8 Command,
			     u8 Token, u8 Arg0, u8 Arg1);

#endif /* __RTMP_MCU_H__ // */
