

#ifndef __SPECTRUM_H__
#define __SPECTRUM_H__

#include "rtmp_type.h"
#include "spectrum_def.h"

char RTMP_GetTxPwr(struct rt_rtmp_adapter *pAd, IN HTTRANSMIT_SETTING HTTxMode);

void MakeMeasurementReqFrame(struct rt_rtmp_adapter *pAd,
			     u8 *pOutBuffer,
			     unsigned long *pFrameLen,
			     u8 TotalLen,
			     u8 Category,
			     u8 Action,
			     u8 MeasureToken,
			     u8 MeasureReqMode,
			     u8 MeasureReqType,
			     u8 NumOfRepetitions);

void EnqueueMeasurementRep(struct rt_rtmp_adapter *pAd,
			   u8 *pDA,
			   u8 DialogToken,
			   u8 MeasureToken,
			   u8 MeasureReqMode,
			   u8 MeasureReqType,
			   u8 ReportInfoLen, u8 *pReportInfo);

void EnqueueTPCReq(struct rt_rtmp_adapter *pAd, u8 *pDA, u8 DialogToken);

void EnqueueTPCRep(struct rt_rtmp_adapter *pAd,
		   u8 *pDA,
		   u8 DialogToken, u8 TxPwr, u8 LinkMargin);

void EnqueueChSwAnn(struct rt_rtmp_adapter *pAd,
		    u8 *pDA, u8 ChSwMode, u8 NewCh);

void PeerSpectrumAction(struct rt_rtmp_adapter *pAd, struct rt_mlme_queue_elem *Elem);

int Set_MeasureReq_Proc(struct rt_rtmp_adapter *pAd, char *arg);

int Set_TpcReq_Proc(struct rt_rtmp_adapter *pAd, char *arg);

int Set_PwrConstraint(struct rt_rtmp_adapter *pAd, char *arg);

void MeasureReqTabInit(struct rt_rtmp_adapter *pAd);

void MeasureReqTabExit(struct rt_rtmp_adapter *pAd);

struct rt_measure_req_entry *MeasureReqLookUp(struct rt_rtmp_adapter *pAd, u8 DialogToken);

struct rt_measure_req_entry *MeasureReqInsert(struct rt_rtmp_adapter *pAd, u8 DialogToken);

void MeasureReqDelete(struct rt_rtmp_adapter *pAd, u8 DialogToken);

void InsertChannelRepIE(struct rt_rtmp_adapter *pAd,
			u8 *pFrameBuf,
			unsigned long *pFrameLen,
			char *pCountry, u8 RegulatoryClass);

void InsertTpcReportIE(struct rt_rtmp_adapter *pAd,
		       u8 *pFrameBuf,
		       unsigned long *pFrameLen,
		       u8 TxPwr, u8 LinkMargin);

void InsertDialogToken(struct rt_rtmp_adapter *pAd,
		       u8 *pFrameBuf,
		       unsigned long *pFrameLen, u8 DialogToken);

void TpcReqTabInit(struct rt_rtmp_adapter *pAd);

void TpcReqTabExit(struct rt_rtmp_adapter *pAd);

void NotifyChSwAnnToPeerAPs(struct rt_rtmp_adapter *pAd,
			    u8 *pRA,
			    u8 *pTA, u8 ChSwMode, u8 Channel);

void RguClass_BuildBcnChList(struct rt_rtmp_adapter *pAd,
			     u8 *pBuf, unsigned long *pBufLen);
#endif /* __SPECTRUM_H__ // */
