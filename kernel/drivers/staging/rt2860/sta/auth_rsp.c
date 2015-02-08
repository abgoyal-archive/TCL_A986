
#include "../rt_config.h"

void AuthRspStateMachineInit(struct rt_rtmp_adapter *pAd,
			     struct rt_state_machine *Sm,
			     IN STATE_MACHINE_FUNC Trans[])
{
	StateMachineInit(Sm, Trans, MAX_AUTH_RSP_STATE, MAX_AUTH_RSP_MSG,
			 (STATE_MACHINE_FUNC) Drop, AUTH_RSP_IDLE,
			 AUTH_RSP_MACHINE_BASE);

	/* column 1 */
	StateMachineSetAction(Sm, AUTH_RSP_IDLE, MT2_PEER_DEAUTH,
			      (STATE_MACHINE_FUNC) PeerDeauthAction);

	/* column 2 */
	StateMachineSetAction(Sm, AUTH_RSP_WAIT_CHAL, MT2_PEER_DEAUTH,
			      (STATE_MACHINE_FUNC) PeerDeauthAction);

}

void PeerAuthSimpleRspGenAndSend(struct rt_rtmp_adapter *pAd,
				 struct rt_header_802_11 * pHdr80211,
				 u16 Alg,
				 u16 Seq,
				 u16 Reason, u16 Status)
{
	struct rt_header_802_11 AuthHdr;
	unsigned long FrameLen = 0;
	u8 *pOutBuffer = NULL;
	int NStatus;

	if (Reason != MLME_SUCCESS) {
		DBGPRINT(RT_DEBUG_TRACE, ("Peer AUTH fail...\n"));
		return;
	}
	/*Get an unused nonpaged memory */
	NStatus = MlmeAllocateMemory(pAd, &pOutBuffer);
	if (NStatus != NDIS_STATUS_SUCCESS)
		return;

	DBGPRINT(RT_DEBUG_TRACE, ("Send AUTH response (seq#2)...\n"));
	MgtMacHeaderInit(pAd, &AuthHdr, SUBTYPE_AUTH, 0, pHdr80211->Addr2,
			 pAd->MlmeAux.Bssid);
	MakeOutgoingFrame(pOutBuffer, &FrameLen, sizeof(struct rt_header_802_11),
			  &AuthHdr, 2, &Alg, 2, &Seq, 2, &Reason, END_OF_ARGS);
	MiniportMMRequest(pAd, 0, pOutBuffer, FrameLen);
	MlmeFreeMemory(pAd, pOutBuffer);
}

void PeerDeauthAction(struct rt_rtmp_adapter *pAd, struct rt_mlme_queue_elem *Elem)
{
	u8 Addr2[MAC_ADDR_LEN];
	u16 Reason;

	if (PeerDeauthSanity(pAd, Elem->Msg, Elem->MsgLen, Addr2, &Reason)) {
		if (INFRA_ON(pAd)
		    && MAC_ADDR_EQUAL(Addr2, pAd->CommonCfg.Bssid)
		    ) {
			DBGPRINT(RT_DEBUG_TRACE,
				 ("AUTH_RSP - receive DE-AUTH from our AP (Reason=%d)\n",
				  Reason));

			RtmpOSWrielessEventSend(pAd, SIOCGIWAP, -1, NULL, NULL,
						0);

			/* send wireless event - for deauthentication */
			if (pAd->CommonCfg.bWirelessEvent)
				RTMPSendWirelessEvent(pAd, IW_DEAUTH_EVENT_FLAG,
						      pAd->MacTab.
						      Content[BSSID_WCID].Addr,
						      BSS0, 0);

			LinkDown(pAd, TRUE);
		}
	} else {
		DBGPRINT(RT_DEBUG_TRACE,
			 ("AUTH_RSP - PeerDeauthAction() sanity check fail\n"));
	}
}
