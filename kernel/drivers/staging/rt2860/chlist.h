

#ifndef __CHLIST_H__
#define __CHLIST_H__

#include "rtmp_type.h"
#include "rtmp_def.h"

#define ODOR			0
#define IDOR			1
#define BOTH			2

#define BAND_5G         0
#define BAND_24G        1
#define BAND_BOTH       2

struct rt_ch_desp {
	u8 FirstChannel;
	u8 NumOfCh;
	char MaxTxPwr;		/* dBm */
	u8 Geography;	/* 0:out door, 1:in door, 2:both */
	BOOLEAN DfsReq;		/* Dfs require, 0: No, 1: yes. */
};

struct rt_ch_region {
	u8 CountReg[3];
	u8 DfsType;		/* 0: CE, 1: FCC, 2: JAP, 3:JAP_W53, JAP_W56 */
	struct rt_ch_desp ChDesp[10];
};

extern struct rt_ch_region ChRegion[];

struct rt_ch_freq_map {
	u16 channel;
	u16 freqKHz;
};

extern struct rt_ch_freq_map CH_HZ_ID_MAP[];
extern int CH_HZ_ID_MAP_NUM;

#define     MAP_CHANNEL_ID_TO_KHZ(_ch, _khz)					\
		do{													\
			int _chIdx;											\
			for (_chIdx = 0; _chIdx < CH_HZ_ID_MAP_NUM; _chIdx++)\
			{													\
				if ((_ch) == CH_HZ_ID_MAP[_chIdx].channel)			\
				{												\
					(_khz) = CH_HZ_ID_MAP[_chIdx].freqKHz * 1000;	\
					break;										\
				}												\
			}													\
			if (_chIdx == CH_HZ_ID_MAP_NUM)					\
				(_khz) = 2412000;									\
            }while(0)

#define     MAP_KHZ_TO_CHANNEL_ID(_khz, _ch)                 \
		do{													\
			int _chIdx;											\
			for (_chIdx = 0; _chIdx < CH_HZ_ID_MAP_NUM; _chIdx++)\
			{													\
				if ((_khz) == CH_HZ_ID_MAP[_chIdx].freqKHz)			\
				{												\
					(_ch) = CH_HZ_ID_MAP[_chIdx].channel;			\
					break;										\
				}												\
			}													\
			if (_chIdx == CH_HZ_ID_MAP_NUM)					\
				(_ch) = 1;											\
		}while(0)

void BuildChannelListEx(struct rt_rtmp_adapter *pAd);

void BuildBeaconChList(struct rt_rtmp_adapter *pAd,
		       u8 *pBuf, unsigned long *pBufLen);

void N_ChannelCheck(struct rt_rtmp_adapter *pAd);

void N_SetCenCh(struct rt_rtmp_adapter *pAd);

u8 GetCuntryMaxTxPwr(struct rt_rtmp_adapter *pAd, u8 channel);

#endif /* __CHLIST_H__ */
