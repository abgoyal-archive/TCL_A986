

#ifndef __iwl_dev_h__
#define __iwl_dev_h__

#include <linux/pci.h> /* for struct pci_device_id */
#include <linux/kernel.h>
#include <net/ieee80211_radiotap.h>

#include "iwl-eeprom.h"
#include "iwl-csr.h"
#include "iwl-prph.h"
#include "iwl-fh.h"
#include "iwl-debug.h"
#include "iwl-4965-hw.h"
#include "iwl-3945-hw.h"
#include "iwl-agn-hw.h"
#include "iwl-led.h"
#include "iwl-power.h"
#include "iwl-agn-rs.h"

/* configuration for the iwl4965 */
extern struct iwl_cfg iwl4965_agn_cfg;
extern struct iwl_cfg iwl5300_agn_cfg;
extern struct iwl_cfg iwl5100_agn_cfg;
extern struct iwl_cfg iwl5350_agn_cfg;
extern struct iwl_cfg iwl5100_bgn_cfg;
extern struct iwl_cfg iwl5100_abg_cfg;
extern struct iwl_cfg iwl5150_agn_cfg;
extern struct iwl_cfg iwl5150_abg_cfg;
extern struct iwl_cfg iwl6000g2a_2agn_cfg;
extern struct iwl_cfg iwl6000i_2agn_cfg;
extern struct iwl_cfg iwl6000i_2abg_cfg;
extern struct iwl_cfg iwl6000i_2bg_cfg;
extern struct iwl_cfg iwl6000_3agn_cfg;
extern struct iwl_cfg iwl6050_2agn_cfg;
extern struct iwl_cfg iwl6050_2abg_cfg;
extern struct iwl_cfg iwl1000_bgn_cfg;
extern struct iwl_cfg iwl1000_bg_cfg;

struct iwl_tx_queue;

/* CT-KILL constants */
#define CT_KILL_THRESHOLD_LEGACY   110 /* in Celsius */
#define CT_KILL_THRESHOLD	   114 /* in Celsius */
#define CT_KILL_EXIT_THRESHOLD     95  /* in Celsius */

#define IWL_NOISE_MEAS_NOT_AVAILABLE (-127)

#define DEFAULT_RTS_THRESHOLD     2347U
#define MIN_RTS_THRESHOLD         0U
#define MAX_RTS_THRESHOLD         2347U
#define MAX_MSDU_SIZE		  2304U
#define MAX_MPDU_SIZE		  2346U
#define DEFAULT_BEACON_INTERVAL   100U
#define	DEFAULT_SHORT_RETRY_LIMIT 7U
#define	DEFAULT_LONG_RETRY_LIMIT  4U

struct iwl_rx_mem_buffer {
	dma_addr_t page_dma;
	struct page *page;
	struct list_head list;
};

#define rxb_addr(r) page_address(r->page)

/* defined below */
struct iwl_device_cmd;

struct iwl_cmd_meta {
	/* only for SYNC commands, iff the reply skb is wanted */
	struct iwl_host_cmd *source;
	/*
	 * only for ASYNC commands
	 * (which is somewhat stupid -- look at iwl-sta.c for instance
	 * which duplicates a bunch of code because the callback isn't
	 * invoked for SYNC commands, if it were and its result passed
	 * through it would be simpler...)
	 */
	void (*callback)(struct iwl_priv *priv,
			 struct iwl_device_cmd *cmd,
			 struct iwl_rx_packet *pkt);

	/* The CMD_SIZE_HUGE flag bit indicates that the command
	 * structure is stored at the end of the shared queue memory. */
	u32 flags;

	DECLARE_PCI_UNMAP_ADDR(mapping)
	DECLARE_PCI_UNMAP_LEN(len)
};

struct iwl_queue {
	int n_bd;              /* number of BDs in this queue */
	int write_ptr;       /* 1-st empty entry (index) host_w*/
	int read_ptr;         /* last used entry (index) host_r*/
	/* use for monitoring and recovering the stuck queue */
	int last_read_ptr;      /* storing the last read_ptr */
	/* number of time read_ptr and last_read_ptr are the same */
	u8 repeat_same_read_ptr;
	dma_addr_t dma_addr;   /* physical addr for BD's */
	int n_window;	       /* safe queue window */
	u32 id;
	int low_mark;	       /* low watermark, resume queue if free
				* space more than this */
	int high_mark;         /* high watermark, stop queue if free
				* space less than this */
} __attribute__ ((packed));

/* One for each TFD */
struct iwl_tx_info {
	struct sk_buff *skb[IWL_NUM_OF_TBS - 1];
};

#define TFD_TX_CMD_SLOTS 256
#define TFD_CMD_SLOTS 32

struct iwl_tx_queue {
	struct iwl_queue q;
	void *tfds;
	struct iwl_device_cmd **cmd;
	struct iwl_cmd_meta *meta;
	struct iwl_tx_info *txb;
	u8 need_update;
	u8 sched_retry;
	u8 active;
	u8 swq_id;
};

#define IWL_NUM_SCAN_RATES         (2)

struct iwl4965_channel_tgd_info {
	u8 type;
	s8 max_power;
};

struct iwl4965_channel_tgh_info {
	s64 last_radar_time;
};

#define IWL4965_MAX_RATE (33)

struct iwl3945_clip_group {
	/* maximum power level to prevent clipping for each rate, derived by
	 *   us from this band's saturation power in EEPROM */
	const s8 clip_powers[IWL_MAX_RATES];
};

struct iwl3945_channel_power_info {
	struct iwl3945_tx_power tpc;	/* actual radio and DSP gain settings */
	s8 power_table_index;	/* actual (compenst'd) index into gain table */
	s8 base_power_index;	/* gain index for power at factory temp. */
	s8 requested_power;	/* power (dBm) requested for this chnl/rate */
};

struct iwl3945_scan_power_info {
	struct iwl3945_tx_power tpc;	/* actual radio and DSP gain settings */
	s8 power_table_index;	/* actual (compenst'd) index into gain table */
	s8 requested_power;	/* scan pwr (dBm) requested for chnl/rate */
};

struct iwl_channel_info {
	struct iwl4965_channel_tgd_info tgd;
	struct iwl4965_channel_tgh_info tgh;
	struct iwl_eeprom_channel eeprom;	/* EEPROM regulatory limit */
	struct iwl_eeprom_channel ht40_eeprom;	/* EEPROM regulatory limit for
						 * HT40 channel */

	u8 channel;	  /* channel number */
	u8 flags;	  /* flags copied from EEPROM */
	s8 max_power_avg; /* (dBm) regul. eeprom, normal Tx, any rate */
	s8 curr_txpow;	  /* (dBm) regulatory/spectrum/user (not h/w) limit */
	s8 min_power;	  /* always 0 */
	s8 scan_power;	  /* (dBm) regul. eeprom, direct scans, any rate */

	u8 group_index;	  /* 0-4, maps channel to group1/2/3/4/5 */
	u8 band_index;	  /* 0-4, maps channel to band1/2/3/4/5 */
	enum ieee80211_band band;

	/* HT40 channel info */
	s8 ht40_max_power_avg;	/* (dBm) regul. eeprom, normal Tx, any rate */
	u8 ht40_flags;		/* flags copied from EEPROM */
	u8 ht40_extension_channel; /* HT_IE_EXT_CHANNEL_* */

	/* Radio/DSP gain settings for each "normal" data Tx rate.
	 * These include, in addition to RF and DSP gain, a few fields for
	 *   remembering/modifying gain settings (indexes). */
	struct iwl3945_channel_power_info power_info[IWL4965_MAX_RATE];

	/* Radio/DSP gain settings for each scan rate, for directed scans. */
	struct iwl3945_scan_power_info scan_pwr_info[IWL_NUM_SCAN_RATES];
};

#define IWL_TX_FIFO_BK		0
#define IWL_TX_FIFO_BE		1
#define IWL_TX_FIFO_VI		2
#define IWL_TX_FIFO_VO		3
#define IWL_TX_FIFO_UNUSED	-1

#define IWL_MIN_NUM_QUEUES	10

#define IWL_CMD_QUEUE_NUM	4

/* Power management (not Tx power) structures */

enum iwl_pwr_src {
	IWL_PWR_SRC_VMAIN,
	IWL_PWR_SRC_VAUX,
};

#define IEEE80211_DATA_LEN              2304
#define IEEE80211_4ADDR_LEN             30
#define IEEE80211_HLEN                  (IEEE80211_4ADDR_LEN)
#define IEEE80211_FRAME_LEN             (IEEE80211_DATA_LEN + IEEE80211_HLEN)

struct iwl_frame {
	union {
		struct ieee80211_hdr frame;
		struct iwl_tx_beacon_cmd beacon;
		u8 raw[IEEE80211_FRAME_LEN];
		u8 cmd[360];
	} u;
	struct list_head list;
};

#define SEQ_TO_SN(seq) (((seq) & IEEE80211_SCTL_SEQ) >> 4)
#define SN_TO_SEQ(ssn) (((ssn) << 4) & IEEE80211_SCTL_SEQ)
#define MAX_SN ((IEEE80211_SCTL_SEQ) >> 4)

enum {
	CMD_SYNC = 0,
	CMD_SIZE_NORMAL = 0,
	CMD_NO_SKB = 0,
	CMD_SIZE_HUGE = (1 << 0),
	CMD_ASYNC = (1 << 1),
	CMD_WANT_SKB = (1 << 2),
};

#define DEF_CMD_PAYLOAD_SIZE 320

struct iwl_device_cmd {
	struct iwl_cmd_header hdr;	/* uCode API */
	union {
		u32 flags;
		u8 val8;
		u16 val16;
		u32 val32;
		struct iwl_tx_cmd tx;
		struct iwl6000_channel_switch_cmd chswitch;
		u8 payload[DEF_CMD_PAYLOAD_SIZE];
	} __attribute__ ((packed)) cmd;
} __attribute__ ((packed));

#define TFD_MAX_PAYLOAD_SIZE (sizeof(struct iwl_device_cmd))


struct iwl_host_cmd {
	const void *data;
	unsigned long reply_page;
	void (*callback)(struct iwl_priv *priv,
			 struct iwl_device_cmd *cmd,
			 struct iwl_rx_packet *pkt);
	u32 flags;
	u16 len;
	u8 id;
};

#define SUP_RATE_11A_MAX_NUM_CHANNELS  8
#define SUP_RATE_11B_MAX_NUM_CHANNELS  4
#define SUP_RATE_11G_MAX_NUM_CHANNELS  12

struct iwl_rx_queue {
	__le32 *bd;
	dma_addr_t dma_addr;
	struct iwl_rx_mem_buffer pool[RX_QUEUE_SIZE + RX_FREE_BUFFERS];
	struct iwl_rx_mem_buffer *queue[RX_QUEUE_SIZE];
	u32 read;
	u32 write;
	u32 free_count;
	u32 write_actual;
	struct list_head rx_free;
	struct list_head rx_used;
	int need_update;
	struct iwl_rb_status *rb_stts;
	dma_addr_t rb_stts_dma;
	spinlock_t lock;
};

#define IWL_SUPPORTED_RATES_IE_LEN         8

#define MAX_TID_COUNT        9

#define IWL_INVALID_RATE     0xFF
#define IWL_INVALID_VALUE    -1

struct iwl_ht_agg {
	u16 txq_id;
	u16 frame_count;
	u16 wait_for_ba;
	u16 start_idx;
	u64 bitmap;
	u32 rate_n_flags;
#define IWL_AGG_OFF 0
#define IWL_AGG_ON 1
#define IWL_EMPTYING_HW_QUEUE_ADDBA 2
#define IWL_EMPTYING_HW_QUEUE_DELBA 3
	u8 state;
};


struct iwl_tid_data {
	u16 seq_number;
	u16 tfds_in_queue;
	struct iwl_ht_agg agg;
};

struct iwl_hw_key {
	enum ieee80211_key_alg alg;
	int keylen;
	u8 keyidx;
	u8 key[32];
};

union iwl_ht_rate_supp {
	u16 rates;
	struct {
		u8 siso_rate;
		u8 mimo_rate;
	};
};

#define CFG_HT_RX_AMPDU_FACTOR_DEF  (0x3)

#define CFG_HT_MPDU_DENSITY_4USEC   (0x5)
#define CFG_HT_MPDU_DENSITY_DEF CFG_HT_MPDU_DENSITY_4USEC

struct iwl_ht_config {
	/* self configuration data */
	bool is_ht;
	bool is_40mhz;
	bool single_chain_sufficient;
	enum ieee80211_smps_mode smps; /* current smps mode */
	/* BSS related data */
	u8 extension_chan_offset;
	u8 ht_protection;
	u8 non_GF_STA_present;
};

/* QoS structures */
struct iwl_qos_info {
	int qos_active;
	struct iwl_qosparam_cmd def_qos_parm;
};

struct iwl_station_entry {
	struct iwl_addsta_cmd sta;
	struct iwl_tid_data tid[MAX_TID_COUNT];
	u8 used;
	struct iwl_hw_key keyinfo;
	struct iwl_link_quality_cmd *lq;
};

struct iwl_station_priv_common {
	u8 sta_id;
};

struct iwl_station_priv {
	struct iwl_station_priv_common common;
	struct iwl_lq_sta lq_sta;
	atomic_t pending_frames;
	bool client;
	bool asleep;
};

struct iwl_vif_priv {
	u8 ibss_bssid_sta_id;
};

/* one for each uCode image (inst/data, boot/init/runtime) */
struct fw_desc {
	void *v_addr;		/* access by driver */
	dma_addr_t p_addr;	/* access by card's busmaster DMA */
	u32 len;		/* bytes */
};

/* v1/v2 uCode file layout */
struct iwl_ucode_header {
	__le32 ver;	/* major/minor/API/serial */
	union {
		struct {
			__le32 inst_size;	/* bytes of runtime code */
			__le32 data_size;	/* bytes of runtime data */
			__le32 init_size;	/* bytes of init code */
			__le32 init_data_size;	/* bytes of init data */
			__le32 boot_size;	/* bytes of bootstrap code */
			u8 data[0];		/* in same order as sizes */
		} v1;
		struct {
			__le32 build;		/* build number */
			__le32 inst_size;	/* bytes of runtime code */
			__le32 data_size;	/* bytes of runtime data */
			__le32 init_size;	/* bytes of init code */
			__le32 init_data_size;	/* bytes of init data */
			__le32 boot_size;	/* bytes of bootstrap code */
			u8 data[0];		/* in same order as sizes */
		} v2;
	} u;
};


enum iwl_ucode_tlv_type {
	IWL_UCODE_TLV_INVALID		= 0, /* unused */
	IWL_UCODE_TLV_INST		= 1,
	IWL_UCODE_TLV_DATA		= 2,
	IWL_UCODE_TLV_INIT		= 3,
	IWL_UCODE_TLV_INIT_DATA		= 4,
	IWL_UCODE_TLV_BOOT		= 5,
	IWL_UCODE_TLV_PROBE_MAX_LEN	= 6, /* a u32 value */
};

struct iwl_ucode_tlv {
	__le16 type;		/* see above */
	__le16 alternative;	/* see comment */
	__le32 length;		/* not including type/length fields */
	u8 data[0];
} __attribute__ ((packed));

#define IWL_TLV_UCODE_MAGIC	0x0a4c5749

struct iwl_tlv_ucode_header {
	/*
	 * The TLV style ucode header is distinguished from
	 * the v1/v2 style header by first four bytes being
	 * zero, as such is an invalid combination of
	 * major/minor/API/serial versions.
	 */
	__le32 zero;
	__le32 magic;
	u8 human_readable[64];
	__le32 ver;		/* major/minor/API/serial */
	__le32 build;
	__le64 alternatives;	/* bitmask of valid alternatives */
	/*
	 * The data contained herein has a TLV layout,
	 * see above for the TLV header and types.
	 * Note that each TLV is padded to a length
	 * that is a multiple of 4 for alignment.
	 */
	u8 data[0];
};

struct iwl4965_ibss_seq {
	u8 mac[ETH_ALEN];
	u16 seq_num;
	u16 frag_num;
	unsigned long packet_time;
	struct list_head list;
};

struct iwl_sensitivity_ranges {
	u16 min_nrg_cck;
	u16 max_nrg_cck;

	u16 nrg_th_cck;
	u16 nrg_th_ofdm;

	u16 auto_corr_min_ofdm;
	u16 auto_corr_min_ofdm_mrc;
	u16 auto_corr_min_ofdm_x1;
	u16 auto_corr_min_ofdm_mrc_x1;

	u16 auto_corr_max_ofdm;
	u16 auto_corr_max_ofdm_mrc;
	u16 auto_corr_max_ofdm_x1;
	u16 auto_corr_max_ofdm_mrc_x1;

	u16 auto_corr_max_cck;
	u16 auto_corr_max_cck_mrc;
	u16 auto_corr_min_cck;
	u16 auto_corr_min_cck_mrc;

	u16 barker_corr_th_min;
	u16 barker_corr_th_min_mrc;
	u16 nrg_th_cca;
};


#define KELVIN_TO_CELSIUS(x) ((x)-273)
#define CELSIUS_TO_KELVIN(x) ((x)+273)


struct iwl_hw_params {
	u8 max_txq_num;
	u8 dma_chnl_num;
	u16 scd_bc_tbls_size;
	u32 tfd_size;
	u8  tx_chains_num;
	u8  rx_chains_num;
	u8  valid_tx_ant;
	u8  valid_rx_ant;
	u16 max_rxq_size;
	u16 max_rxq_log;
	u32 rx_page_order;
	u32 rx_wrt_ptr_reg;
	u8  max_stations;
	u8  bcast_sta_id;
	u8  ht40_channel;
	u8  max_beacon_itrvl;	/* in 1024 ms */
	u32 max_inst_size;
	u32 max_data_size;
	u32 max_bsm_size;
	u32 ct_kill_threshold; /* value in hw-dependent units */
	u32 ct_kill_exit_threshold; /* value in hw-dependent units */
				    /* for 1000, 6000 series and up */
	u32 calib_init_cfg;
	const struct iwl_sensitivity_ranges *sens;
};


extern void iwl_update_chain_flags(struct iwl_priv *priv);
extern int iwl_set_pwr_src(struct iwl_priv *priv, enum iwl_pwr_src src);
extern const u8 iwl_bcast_addr[ETH_ALEN];
extern int iwl_rxq_stop(struct iwl_priv *priv);
extern void iwl_txq_ctx_stop(struct iwl_priv *priv);
extern int iwl_queue_space(const struct iwl_queue *q);
static inline int iwl_queue_used(const struct iwl_queue *q, int i)
{
	return q->write_ptr >= q->read_ptr ?
		(i >= q->read_ptr && i < q->write_ptr) :
		!(i < q->read_ptr && i >= q->write_ptr);
}


static inline u8 get_cmd_index(struct iwl_queue *q, u32 index, int is_huge)
{
	/*
	 * This is for init calibration result and scan command which
	 * required buffer > TFD_MAX_PAYLOAD_SIZE,
	 * the big buffer at end of command array
	 */
	if (is_huge)
		return q->n_window;	/* must be power of 2 */

	/* Otherwise, use normal size buffers */
	return index & (q->n_window - 1);
}


struct iwl_dma_ptr {
	dma_addr_t dma;
	void *addr;
	size_t size;
};

#define IWL_OPERATION_MODE_AUTO     0
#define IWL_OPERATION_MODE_HT_ONLY  1
#define IWL_OPERATION_MODE_MIXED    2
#define IWL_OPERATION_MODE_20MHZ    3

#define IWL_TX_CRC_SIZE 4
#define IWL_TX_DELIMITER_SIZE 4

#define TX_POWER_IWL_ILLEGAL_VOLTAGE -10000

/* Sensitivity and chain noise calibration */
#define INITIALIZATION_VALUE		0xFFFF
#define IWL4965_CAL_NUM_BEACONS		20
#define IWL_CAL_NUM_BEACONS		16
#define MAXIMUM_ALLOWED_PATHLOSS	15

#define CHAIN_NOISE_MAX_DELTA_GAIN_CODE 3

#define MAX_FA_OFDM  50
#define MIN_FA_OFDM  5
#define MAX_FA_CCK   50
#define MIN_FA_CCK   5

#define AUTO_CORR_STEP_OFDM       1

#define AUTO_CORR_STEP_CCK     3
#define AUTO_CORR_MAX_TH_CCK   160

#define NRG_DIFF               2
#define NRG_STEP_CCK           2
#define NRG_MARGIN             8
#define MAX_NUMBER_CCK_NO_FA 100

#define AUTO_CORR_CCK_MIN_VAL_DEF    (125)

#define CHAIN_A             0
#define CHAIN_B             1
#define CHAIN_C             2
#define CHAIN_NOISE_DELTA_GAIN_INIT_VAL 4
#define ALL_BAND_FILTER			0xFF00
#define IN_BAND_FILTER			0xFF
#define MIN_AVERAGE_NOISE_MAX_VALUE	0xFFFFFFFF

#define NRG_NUM_PREV_STAT_L     20
#define NUM_RX_CHAINS           3

enum iwl4965_false_alarm_state {
	IWL_FA_TOO_MANY = 0,
	IWL_FA_TOO_FEW = 1,
	IWL_FA_GOOD_RANGE = 2,
};

enum iwl4965_chain_noise_state {
	IWL_CHAIN_NOISE_ALIVE = 0,  /* must be 0 */
	IWL_CHAIN_NOISE_ACCUMULATE,
	IWL_CHAIN_NOISE_CALIBRATED,
	IWL_CHAIN_NOISE_DONE,
};

enum iwl4965_calib_enabled_state {
	IWL_CALIB_DISABLED = 0,  /* must be 0 */
	IWL_CALIB_ENABLED = 1,
};


enum iwl_calib {
	IWL_CALIB_XTAL,
	IWL_CALIB_DC,
	IWL_CALIB_LO,
	IWL_CALIB_TX_IQ,
	IWL_CALIB_TX_IQ_PERD,
	IWL_CALIB_BASE_BAND,
	IWL_CALIB_MAX
};

/* Opaque calibration results */
struct iwl_calib_result {
	void *buf;
	size_t buf_len;
};

enum ucode_type {
	UCODE_NONE = 0,
	UCODE_INIT,
	UCODE_RT
};

/* Sensitivity calib data */
struct iwl_sensitivity_data {
	u32 auto_corr_ofdm;
	u32 auto_corr_ofdm_mrc;
	u32 auto_corr_ofdm_x1;
	u32 auto_corr_ofdm_mrc_x1;
	u32 auto_corr_cck;
	u32 auto_corr_cck_mrc;

	u32 last_bad_plcp_cnt_ofdm;
	u32 last_fa_cnt_ofdm;
	u32 last_bad_plcp_cnt_cck;
	u32 last_fa_cnt_cck;

	u32 nrg_curr_state;
	u32 nrg_prev_state;
	u32 nrg_value[10];
	u8  nrg_silence_rssi[NRG_NUM_PREV_STAT_L];
	u32 nrg_silence_ref;
	u32 nrg_energy_idx;
	u32 nrg_silence_idx;
	u32 nrg_th_cck;
	s32 nrg_auto_corr_silence_diff;
	u32 num_in_cck_no_fa;
	u32 nrg_th_ofdm;

	u16 barker_corr_th_min;
	u16 barker_corr_th_min_mrc;
	u16 nrg_th_cca;
};

/* Chain noise (differential Rx gain) calib data */
struct iwl_chain_noise_data {
	u32 active_chains;
	u32 chain_noise_a;
	u32 chain_noise_b;
	u32 chain_noise_c;
	u32 chain_signal_a;
	u32 chain_signal_b;
	u32 chain_signal_c;
	u16 beacon_count;
	u8 disconn_array[NUM_RX_CHAINS];
	u8 delta_gain_code[NUM_RX_CHAINS];
	u8 radio_write;
	u8 state;
};

#define	EEPROM_SEM_TIMEOUT 10		/* milliseconds */
#define EEPROM_SEM_RETRY_LIMIT 1000	/* number of attempts (not time) */

#define IWL_TRAFFIC_ENTRIES	(256)
#define IWL_TRAFFIC_ENTRY_SIZE  (64)

enum {
	MEASUREMENT_READY = (1 << 0),
	MEASUREMENT_ACTIVE = (1 << 1),
};

enum iwl_nvm_type {
	NVM_DEVICE_TYPE_EEPROM = 0,
	NVM_DEVICE_TYPE_OTP,
};

enum iwl_access_mode {
	IWL_OTP_ACCESS_ABSOLUTE,
	IWL_OTP_ACCESS_RELATIVE,
};

enum iwl_pa_type {
	IWL_PA_SYSTEM = 0,
	IWL_PA_INTERNAL = 1,
};

/* interrupt statistics */
struct isr_statistics {
	u32 hw;
	u32 sw;
	u32 sw_err;
	u32 sch;
	u32 alive;
	u32 rfkill;
	u32 ctkill;
	u32 wakeup;
	u32 rx;
	u32 rx_handlers[REPLY_MAX];
	u32 tx;
	u32 unhandled;
};

#ifdef CONFIG_IWLWIFI_DEBUGFS
/* management statistics */
enum iwl_mgmt_stats {
	MANAGEMENT_ASSOC_REQ = 0,
	MANAGEMENT_ASSOC_RESP,
	MANAGEMENT_REASSOC_REQ,
	MANAGEMENT_REASSOC_RESP,
	MANAGEMENT_PROBE_REQ,
	MANAGEMENT_PROBE_RESP,
	MANAGEMENT_BEACON,
	MANAGEMENT_ATIM,
	MANAGEMENT_DISASSOC,
	MANAGEMENT_AUTH,
	MANAGEMENT_DEAUTH,
	MANAGEMENT_ACTION,
	MANAGEMENT_MAX,
};
/* control statistics */
enum iwl_ctrl_stats {
	CONTROL_BACK_REQ =  0,
	CONTROL_BACK,
	CONTROL_PSPOLL,
	CONTROL_RTS,
	CONTROL_CTS,
	CONTROL_ACK,
	CONTROL_CFEND,
	CONTROL_CFENDACK,
	CONTROL_MAX,
};

struct traffic_stats {
	u32 mgmt[MANAGEMENT_MAX];
	u32 ctrl[CONTROL_MAX];
	u32 data_cnt;
	u64 data_bytes;
};
#else
struct traffic_stats {
	u64 data_bytes;
};
#endif

struct iwl_switch_rxon {
	bool switch_in_progress;
	__le16 channel;
};

#define UCODE_TRACE_PERIOD (100)

struct iwl_event_log {
	bool ucode_trace;
	u32 num_wraps;
	u32 next_entry;
	int non_wraps_count;
	int wraps_once_count;
	int wraps_more_count;
};

#define IWL_HOST_INT_TIMEOUT_MAX	(0xFF)
#define IWL_HOST_INT_TIMEOUT_DEF	(0x40)
#define IWL_HOST_INT_TIMEOUT_MIN	(0x0)
#define IWL_HOST_INT_CALIB_TIMEOUT_MAX	(0xFF)
#define IWL_HOST_INT_CALIB_TIMEOUT_DEF	(0x10)
#define IWL_HOST_INT_CALIB_TIMEOUT_MIN	(0x0)

#define IWL_MAX_PLCP_ERR_THRESHOLD_MIN	(0)
#define IWL_MAX_PLCP_ERR_THRESHOLD_DEF	(50)
#define IWL_MAX_PLCP_ERR_LONG_THRESHOLD_DEF	(100)
#define IWL_MAX_PLCP_ERR_EXT_LONG_THRESHOLD_DEF	(200)
#define IWL_MAX_PLCP_ERR_THRESHOLD_MAX	(255)

#define IWL_DELAY_NEXT_FORCE_RF_RESET  (HZ*3)
#define IWL_DELAY_NEXT_FORCE_FW_RELOAD (HZ*5)

/* timer constants use to monitor and recover stuck tx queues in mSecs */
#define IWL_MONITORING_PERIOD  (1000)
#define IWL_ONE_HUNDRED_MSECS   (100)
#define IWL_SIXTY_SECS          (60000)

enum iwl_reset {
	IWL_RF_RESET = 0,
	IWL_FW_RESET,
	IWL_MAX_FORCE_RESET,
};

struct iwl_force_reset {
	int reset_request_count;
	int reset_success_count;
	int reset_reject_count;
	unsigned long reset_duration;
	unsigned long last_force_reset_jiffies;
};

struct iwl_priv {

	/* ieee device used by generic ieee processing code */
	struct ieee80211_hw *hw;
	struct ieee80211_channel *ieee_channels;
	struct ieee80211_rate *ieee_rates;
	struct iwl_cfg *cfg;

	/* temporary frame storage list */
	struct list_head free_frames;
	int frames_count;

	enum ieee80211_band band;
	int alloc_rxb_page;

	void (*rx_handlers[REPLY_MAX])(struct iwl_priv *priv,
				       struct iwl_rx_mem_buffer *rxb);

	struct ieee80211_supported_band bands[IEEE80211_NUM_BANDS];

	/* spectrum measurement report caching */
	struct iwl_spectrum_notification measure_report;
	u8 measurement_status;

	/* ucode beacon time */
	u32 ucode_beacon_time;
	int missed_beacon_threshold;

	/* storing the jiffies when the plcp error rate is received */
	unsigned long plcp_jiffies;

	/* force reset */
	struct iwl_force_reset force_reset[IWL_MAX_FORCE_RESET];

	/* we allocate array of iwl4965_channel_info for NIC's valid channels.
	 *    Access via channel # using indirect index array */
	struct iwl_channel_info *channel_info;	/* channel info array */
	u8 channel_count;	/* # of channels */

	/* thermal calibration */
	s32 temperature;	/* degrees Kelvin */
	s32 last_temperature;

	/* init calibration results */
	struct iwl_calib_result calib_results[IWL_CALIB_MAX];

	/* Scan related variables */
	unsigned long scan_start;
	unsigned long scan_start_tsf;
	void *scan_cmd;
	enum ieee80211_band scan_band;
	struct cfg80211_scan_request *scan_request;
	bool is_internal_short_scan;
	u8 scan_tx_ant[IEEE80211_NUM_BANDS];
	u8 mgmt_tx_ant;

	/* spinlock */
	spinlock_t lock;	/* protect general shared data */
	spinlock_t hcmd_lock;	/* protect hcmd */
	spinlock_t reg_lock;	/* protect hw register access */
	struct mutex mutex;
	struct mutex sync_cmd_mutex; /* enable serialization of sync commands */

	/* basic pci-network driver stuff */
	struct pci_dev *pci_dev;

	/* pci hardware address support */
	void __iomem *hw_base;
	u32  hw_rev;
	u32  hw_wa_rev;
	u8   rev_id;

	/* uCode images, save to reload in case of failure */
	int fw_index;			/* firmware we're trying to load */
	u32 ucode_ver;			/* version of ucode, copy of
					   iwl_ucode.ver */
	struct fw_desc ucode_code;	/* runtime inst */
	struct fw_desc ucode_data;	/* runtime data original */
	struct fw_desc ucode_data_backup;	/* runtime data save/restore */
	struct fw_desc ucode_init;	/* initialization inst */
	struct fw_desc ucode_init_data;	/* initialization data */
	struct fw_desc ucode_boot;	/* bootstrap inst */
	enum ucode_type ucode_type;
	u8 ucode_write_complete;	/* the image write is complete */
	char firmware_name[25];


	struct iwl_rxon_time_cmd rxon_timing;

	/* We declare this const so it can only be
	 * changed via explicit cast within the
	 * routines that actually update the physical
	 * hardware */
	const struct iwl_rxon_cmd active_rxon;
	struct iwl_rxon_cmd staging_rxon;

	struct iwl_switch_rxon switch_rxon;

	/* 1st responses from initialize and runtime uCode images.
	 * 4965's initialize alive response contains some calibration data. */
	struct iwl_init_alive_resp card_alive_init;
	struct iwl_alive_resp card_alive;

	unsigned long last_blink_time;
	u8 last_blink_rate;
	u8 allow_blinking;
	u64 led_tpt;

	u16 active_rate;

	u8 start_calib;
	struct iwl_sensitivity_data sensitivity_data;
	struct iwl_chain_noise_data chain_noise_data;
	__le16 sensitivity_tbl[HD_TABLE_SIZE];

	struct iwl_ht_config current_ht_config;

	/* Rate scaling data */
	u8 retry_rate;

	wait_queue_head_t wait_command_queue;

	int activity_timer_active;

	/* Rx and Tx DMA processing queues */
	struct iwl_rx_queue rxq;
	struct iwl_tx_queue *txq;
	unsigned long txq_ctx_active_msk;
	struct iwl_dma_ptr  kw;	/* keep warm address */
	struct iwl_dma_ptr  scd_bc_tbls;

	u32 scd_base_addr;	/* scheduler sram base address */

	unsigned long status;

	/* counts mgmt, ctl, and data packets */
	struct traffic_stats tx_stats;
	struct traffic_stats rx_stats;

	/* counts interrupts */
	struct isr_statistics isr_stats;

	struct iwl_power_mgr power_data;
	struct iwl_tt_mgmt thermal_throttle;

	struct iwl_notif_statistics statistics;
#ifdef CONFIG_IWLWIFI_DEBUG
	struct iwl_notif_statistics accum_statistics;
	struct iwl_notif_statistics delta_statistics;
	struct iwl_notif_statistics max_delta;
#endif

	/* context information */
	u8 bssid[ETH_ALEN]; /* used only on 3945 but filled by core */
	u8 mac_addr[ETH_ALEN];

	/*station table variables */
	spinlock_t sta_lock;
	int num_stations;
	struct iwl_station_entry stations[IWL_STATION_COUNT];
	struct iwl_wep_key wep_keys[WEP_KEYS_MAX]; /* protected by mutex */
	u8 key_mapping_key;
	unsigned long ucode_key_table;

	/* queue refcounts */
#define IWL_MAX_HW_QUEUES	32
	unsigned long queue_stopped[BITS_TO_LONGS(IWL_MAX_HW_QUEUES)];
	/* for each AC */
	atomic_t queue_stop_count[4];

	/* Indication if ieee80211_ops->open has been called */
	u8 is_open;

	u8 mac80211_registered;

	/* eeprom -- this is in the card's little endian byte order */
	u8 *eeprom;
	int    nvm_device_type;
	struct iwl_eeprom_calib_info *calib_info;

	enum nl80211_iftype iw_mode;

	struct sk_buff *ibss_beacon;

	/* Last Rx'd beacon timestamp */
	u64 timestamp;
	struct ieee80211_vif *vif;

	union {
#if defined(CONFIG_IWL3945) || defined(CONFIG_IWL3945_MODULE)
		struct {
			void *shared_virt;
			dma_addr_t shared_phys;

			struct delayed_work thermal_periodic;
			struct delayed_work rfkill_poll;

			struct iwl3945_notif_statistics statistics;
#ifdef CONFIG_IWLWIFI_DEBUG
			struct iwl3945_notif_statistics accum_statistics;
			struct iwl3945_notif_statistics delta_statistics;
			struct iwl3945_notif_statistics max_delta;
#endif

			u32 sta_supp_rates;
			int last_rx_rssi;	/* From Rx packet statistics */

			/* Rx'd packet timing information */
			u32 last_beacon_time;
			u64 last_tsf;

			/*
			 * each calibration channel group in the
			 * EEPROM has a derived clip setting for
			 * each rate.
			 */
			const struct iwl3945_clip_group clip_groups[5];

		} _3945;
#endif
#if defined(CONFIG_IWLAGN) || defined(CONFIG_IWLAGN_MODULE)
		struct {
			/* INT ICT Table */
			__le32 *ict_tbl;
			void *ict_tbl_vir;
			dma_addr_t ict_tbl_dma;
			dma_addr_t aligned_ict_tbl_dma;
			int ict_index;
			u32 inta;
			bool use_ict;
			/*
			 * reporting the number of tids has AGG on. 0 means
			 * no AGGREGATION
			 */
			u8 agg_tids_count;

			struct iwl_rx_phy_res last_phy_res;
			bool last_phy_res_valid;

			struct completion firmware_loading_complete;
		} _agn;
#endif
	};

	struct iwl_hw_params hw_params;

	u32 inta_mask;

	struct iwl_qos_info qos_data;

	struct workqueue_struct *workqueue;

	struct work_struct restart;
	struct work_struct scan_completed;
	struct work_struct rx_replenish;
	struct work_struct abort_scan;
	struct work_struct beacon_update;
	struct work_struct tt_work;
	struct work_struct ct_enter;
	struct work_struct ct_exit;
	struct work_struct start_internal_scan;

	struct tasklet_struct irq_tasklet;

	struct delayed_work init_alive_start;
	struct delayed_work alive_start;
	struct delayed_work scan_check;

	/* TX Power */
	s8 tx_power_user_lmt;
	s8 tx_power_device_lmt;
	s8 tx_power_lmt_in_half_dbm; /* max tx power in half-dBm format */


#ifdef CONFIG_IWLWIFI_DEBUG
	/* debugging info */
	u32 debug_level; /* per device debugging will override global
			    iwl_debug_level if set */
	u32 framecnt_to_us;
	atomic_t restrict_refcnt;
	bool disable_ht40;
#ifdef CONFIG_IWLWIFI_DEBUGFS
	/* debugfs */
	u16 tx_traffic_idx;
	u16 rx_traffic_idx;
	u8 *tx_traffic;
	u8 *rx_traffic;
	struct dentry *debugfs_dir;
	u32 dbgfs_sram_offset, dbgfs_sram_len;
#endif /* CONFIG_IWLWIFI_DEBUGFS */
#endif /* CONFIG_IWLWIFI_DEBUG */

	struct work_struct txpower_work;
	u32 disable_sens_cal;
	u32 disable_chain_noise_cal;
	u32 disable_tx_power_cal;
	struct work_struct run_time_calib_work;
	struct timer_list statistics_periodic;
	struct timer_list ucode_trace;
	struct timer_list monitor_recover;
	bool hw_ready;

	struct iwl_event_log event_log;
}; /*iwl_priv */

static inline void iwl_txq_ctx_activate(struct iwl_priv *priv, int txq_id)
{
	set_bit(txq_id, &priv->txq_ctx_active_msk);
}

static inline void iwl_txq_ctx_deactivate(struct iwl_priv *priv, int txq_id)
{
	clear_bit(txq_id, &priv->txq_ctx_active_msk);
}

#ifdef CONFIG_IWLWIFI_DEBUG
const char *iwl_get_tx_fail_reason(u32 status);
static inline u32 iwl_get_debug_level(struct iwl_priv *priv)
{
	if (priv->debug_level)
		return priv->debug_level;
	else
		return iwl_debug_level;
}
#else
static inline const char *iwl_get_tx_fail_reason(u32 status) { return ""; }

static inline u32 iwl_get_debug_level(struct iwl_priv *priv)
{
	return iwl_debug_level;
}
#endif


static inline struct ieee80211_hdr *iwl_tx_queue_get_hdr(struct iwl_priv *priv,
							 int txq_id, int idx)
{
	if (priv->txq[txq_id].txb[idx].skb[0])
		return (struct ieee80211_hdr *)priv->txq[txq_id].
				txb[idx].skb[0]->data;
	return NULL;
}


static inline int iwl_is_associated(struct iwl_priv *priv)
{
	return (priv->active_rxon.filter_flags & RXON_FILTER_ASSOC_MSK) ? 1 : 0;
}

static inline int is_channel_valid(const struct iwl_channel_info *ch_info)
{
	if (ch_info == NULL)
		return 0;
	return (ch_info->flags & EEPROM_CHANNEL_VALID) ? 1 : 0;
}

static inline int is_channel_radar(const struct iwl_channel_info *ch_info)
{
	return (ch_info->flags & EEPROM_CHANNEL_RADAR) ? 1 : 0;
}

static inline u8 is_channel_a_band(const struct iwl_channel_info *ch_info)
{
	return ch_info->band == IEEE80211_BAND_5GHZ;
}

static inline u8 is_channel_bg_band(const struct iwl_channel_info *ch_info)
{
	return ch_info->band == IEEE80211_BAND_2GHZ;
}

static inline int is_channel_passive(const struct iwl_channel_info *ch)
{
	return (!(ch->flags & EEPROM_CHANNEL_ACTIVE)) ? 1 : 0;
}

static inline int is_channel_ibss(const struct iwl_channel_info *ch)
{
	return ((ch->flags & EEPROM_CHANNEL_IBSS)) ? 1 : 0;
}

static inline void __iwl_free_pages(struct iwl_priv *priv, struct page *page)
{
	__free_pages(page, priv->hw_params.rx_page_order);
	priv->alloc_rxb_page--;
}

static inline void iwl_free_pages(struct iwl_priv *priv, unsigned long page)
{
	free_pages(page, priv->hw_params.rx_page_order);
	priv->alloc_rxb_page--;
}
#endif				/* __iwl_dev_h__ */
