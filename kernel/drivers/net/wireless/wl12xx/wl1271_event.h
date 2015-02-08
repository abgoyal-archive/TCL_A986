

#ifndef __WL1271_EVENT_H__
#define __WL1271_EVENT_H__


enum {
	RSSI_SNR_TRIGGER_0_EVENT_ID              = BIT(0),
	RSSI_SNR_TRIGGER_1_EVENT_ID              = BIT(1),
	RSSI_SNR_TRIGGER_2_EVENT_ID              = BIT(2),
	RSSI_SNR_TRIGGER_3_EVENT_ID              = BIT(3),
	RSSI_SNR_TRIGGER_4_EVENT_ID              = BIT(4),
	RSSI_SNR_TRIGGER_5_EVENT_ID              = BIT(5),
	RSSI_SNR_TRIGGER_6_EVENT_ID              = BIT(6),
	RSSI_SNR_TRIGGER_7_EVENT_ID              = BIT(7),
	MEASUREMENT_START_EVENT_ID		 = BIT(8),
	MEASUREMENT_COMPLETE_EVENT_ID		 = BIT(9),
	SCAN_COMPLETE_EVENT_ID			 = BIT(10),
	SCHEDULED_SCAN_COMPLETE_EVENT_ID	 = BIT(11),
	AP_DISCOVERY_COMPLETE_EVENT_ID		 = BIT(12),
	PS_REPORT_EVENT_ID			 = BIT(13),
	PSPOLL_DELIVERY_FAILURE_EVENT_ID	 = BIT(14),
	DISCONNECT_EVENT_COMPLETE_ID		 = BIT(15),
	JOIN_EVENT_COMPLETE_ID			 = BIT(16),
	CHANNEL_SWITCH_COMPLETE_EVENT_ID	 = BIT(17),
	BSS_LOSE_EVENT_ID			 = BIT(18),
	REGAINED_BSS_EVENT_ID			 = BIT(19),
	ROAMING_TRIGGER_MAX_TX_RETRY_EVENT_ID	 = BIT(20),
	SOFT_GEMINI_SENSE_EVENT_ID		 = BIT(22),
	SOFT_GEMINI_PREDICTION_EVENT_ID		 = BIT(23),
	SOFT_GEMINI_AVALANCHE_EVENT_ID		 = BIT(24),
	PLT_RX_CALIBRATION_COMPLETE_EVENT_ID	 = BIT(25),
	DBG_EVENT_ID				 = BIT(26),
	HEALTH_CHECK_REPLY_EVENT_ID		 = BIT(27),
	PERIODIC_SCAN_COMPLETE_EVENT_ID		 = BIT(28),
	PERIODIC_SCAN_REPORT_EVENT_ID		 = BIT(29),
	BA_SESSION_TEAR_DOWN_EVENT_ID		 = BIT(30),
	EVENT_MBOX_ALL_EVENT_ID			 = 0x7fffffff,
};

enum {
	EVENT_ENTER_POWER_SAVE_FAIL = 0,
	EVENT_ENTER_POWER_SAVE_SUCCESS,
	EVENT_EXIT_POWER_SAVE_FAIL,
	EVENT_EXIT_POWER_SAVE_SUCCESS,
};

struct event_debug_report {
	u8 debug_event_id;
	u8 num_params;
	__le16 pad;
	__le32 report_1;
	__le32 report_2;
	__le32 report_3;
} __attribute__ ((packed));

#define NUM_OF_RSSI_SNR_TRIGGERS 8

struct event_mailbox {
	__le32 events_vector;
	__le32 events_mask;
	__le32 reserved_1;
	__le32 reserved_2;

	u8 dbg_event_id;
	u8 num_relevant_params;
	__le16 reserved_3;
	__le32 event_report_p1;
	__le32 event_report_p2;
	__le32 event_report_p3;

	u8 number_of_scan_results;
	u8 scan_tag;
	u8 reserved_4[2];
	__le32 compl_scheduled_scan_status;

	__le16 scheduled_scan_attended_channels;
	u8 soft_gemini_sense_info;
	u8 soft_gemini_protective_info;
	s8 rssi_snr_trigger_metric[NUM_OF_RSSI_SNR_TRIGGERS];
	u8 channel_switch_status;
	u8 scheduled_scan_status;
	u8 ps_status;

	u8 reserved_5[29];
} __attribute__ ((packed));

int wl1271_event_unmask(struct wl1271 *wl);
void wl1271_event_mbox_config(struct wl1271 *wl);
int wl1271_event_handle(struct wl1271 *wl, u8 mbox);

#endif
