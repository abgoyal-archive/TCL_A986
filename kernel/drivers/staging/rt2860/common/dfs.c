

#include "../rt_config.h"

BOOLEAN RadarChannelCheck(struct rt_rtmp_adapter *pAd, u8 Ch)
{
	int i;
	BOOLEAN result = FALSE;

	for (i = 0; i < pAd->ChannelListNum; i++) {
		if (Ch == pAd->ChannelList[i].Channel) {
			result = pAd->ChannelList[i].DfsReq;
			break;
		}
	}

	return result;
}
