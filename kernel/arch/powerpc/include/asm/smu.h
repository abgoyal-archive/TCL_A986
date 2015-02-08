
#ifndef _SMU_H
#define _SMU_H

#ifdef __KERNEL__
#include <linux/list.h>
#endif
#include <linux/types.h>



#define SMU_CMD_PARTITION_COMMAND		0x3e
#define   SMU_CMD_PARTITION_LATEST		0x01
#define   SMU_CMD_PARTITION_BASE		0x02
#define   SMU_CMD_PARTITION_UPDATE		0x03


#define SMU_CMD_FAN_COMMAND			0x4a


#define SMU_CMD_BATTERY_COMMAND			0x6f
#define   SMU_CMD_GET_BATTERY_INFO		0x00

#define SMU_CMD_RTC_COMMAND			0x8e
#define   SMU_CMD_RTC_SET_PWRUP_TIMER		0x00 /* i: 7 bytes date */
#define   SMU_CMD_RTC_GET_PWRUP_TIMER		0x01 /* o: 7 bytes date */
#define   SMU_CMD_RTC_STOP_PWRUP_TIMER		0x02
#define   SMU_CMD_RTC_SET_PRAM_BYTE_ACC		0x20 /* i: 1 byte (address?) */
#define   SMU_CMD_RTC_SET_PRAM_AUTOINC		0x21 /* i: 1 byte (data?) */
#define   SMU_CMD_RTC_SET_PRAM_LO_BYTES 	0x22 /* i: 10 bytes */
#define   SMU_CMD_RTC_SET_PRAM_HI_BYTES 	0x23 /* i: 10 bytes */
#define   SMU_CMD_RTC_GET_PRAM_BYTE		0x28 /* i: 1 bytes (address?) */
#define   SMU_CMD_RTC_GET_PRAM_LO_BYTES 	0x29 /* o: 10 bytes */
#define   SMU_CMD_RTC_GET_PRAM_HI_BYTES 	0x2a /* o: 10 bytes */
#define	  SMU_CMD_RTC_SET_DATETIME		0x80 /* i: 7 bytes date */
#define   SMU_CMD_RTC_GET_DATETIME		0x81 /* o: 7 bytes date */

 /*
  * i2c commands
  *
  * To issue an i2c command, first is to send a parameter block to the
  * the SMU. This is a command of type 0x9a with 9 bytes of header
  * eventually followed by data for a write:
  *
  * 0: bus number (from device-tree usually, SMU has lots of busses !)
  * 1: transfer type/format (see below)
  * 2: device address. For combined and combined4 type transfers, this
  *    is the "write" version of the address (bit 0x01 cleared)
  * 3: subaddress length (0..3)
  * 4: subaddress byte 0 (or only byte for subaddress length 1)
  * 5: subaddress byte 1
  * 6: subaddress byte 2
  * 7: combined address (device address for combined mode data phase)
  * 8: data length
  *
  * The transfer types are the same good old Apple ones it seems,
  * that is:
  *   - 0x00: Simple transfer
  *   - 0x01: Subaddress transfer (addr write + data tx, no restart)
  *   - 0x02: Combined transfer (addr write + restart + data tx)
  *
  * This is then followed by actual data for a write.
  *
  * At this point, the OF driver seems to have a limitation on transfer
  * sizes of 0xd bytes on reads and 0x5 bytes on writes. I do not know
  * wether this is just an OF limit due to some temporary buffer size
  * or if this is an SMU imposed limit. This driver has the same limitation
  * for now as I use a 0x10 bytes temporary buffer as well
  *
  * Once that is completed, a response is expected from the SMU. This is
  * obtained via a command of type 0x9a with a length of 1 byte containing
  * 0 as the data byte. OF also fills the rest of the data buffer with 0xff's
  * though I can't tell yet if this is actually necessary. Once this command
  * is complete, at this point, all I can tell is what OF does. OF tests
  * byte 0 of the reply:
  *   - on read, 0xfe or 0xfc : bus is busy, wait (see below) or nak ?
  *   - on read, 0x00 or 0x01 : reply is in buffer (after the byte 0)
  *   - on write, < 0 -> failure (immediate exit)
  *   - else, OF just exists (without error, weird)
  *
  * So on read, there is this wait-for-busy thing when getting a 0xfc or
  * 0xfe result. OF does a loop of up to 64 retries, waiting 20ms and
  * doing the above again until either the retries expire or the result
  * is no longer 0xfe or 0xfc
  *
  * The Darwin I2C driver is less subtle though. On any non-success status
  * from the response command, it waits 5ms and tries again up to 20 times,
  * it doesn't differenciate between fatal errors or "busy" status.
  *
  * This driver provides an asynchronous paramblock based i2c command
  * interface to be used either directly by low level code or by a higher
  * level driver interfacing to the linux i2c layer. The current
  * implementation of this relies on working timers & timer interrupts
  * though, so be careful of calling context for now. This may be "fixed"
  * in the future by adding a polling facility.
  */
#define SMU_CMD_I2C_COMMAND			0x9a
          /* transfer types */
#define   SMU_I2C_TRANSFER_SIMPLE	0x00
#define   SMU_I2C_TRANSFER_STDSUB	0x01
#define   SMU_I2C_TRANSFER_COMBINED	0x02

#define SMU_CMD_POWER_COMMAND			0xaa
#define   SMU_CMD_POWER_RESTART		       	"RESTART"
#define   SMU_CMD_POWER_SHUTDOWN		"SHUTDOWN"
#define   SMU_CMD_POWER_VOLTAGE_SLEW		"VSLEW"

#define SMU_CMD_READ_ADC			0xd8


#define SMU_CMD_MISC_df_COMMAND			0xdf

#define   SMU_CMD_MISC_df_SET_DISPLAY_LIT	0x02

#define   SMU_CMD_MISC_df_NMI_OPTION		0x04

#define   SMU_CMD_MISC_df_DIMM_OFFSET		0x99


#define SMU_CMD_VERSION_COMMAND			0xea
#define   SMU_VERSION_RUNNING			0x00
#define   SMU_VERSION_BASE			0x01
#define   SMU_VERSION_UPDATE			0x02


#define SMU_CMD_SWITCHES			0xdc

/* Switches bits */
#define SMU_SWITCH_CASE_CLOSED			0x01
#define SMU_SWITCH_AC_POWER			0x04
#define SMU_SWITCH_POWER_SWITCH			0x08


#define SMU_CMD_MISC_ee_COMMAND			0xee
#define   SMU_CMD_MISC_ee_GET_DATABLOCK_REC	0x02

#define   SMU_CMD_MISC_ee_GET_WATTS		0x03

#define   SMU_CMD_MISC_ee_LEDS_CTRL		0x04 /* i: 00 (00,01) [00] */
#define   SMU_CMD_MISC_ee_GET_DATA		0x05 /* i: 00 , o: ?? */


#define SMU_CMD_POWER_EVENTS_COMMAND		0x8f

/* SMU_POWER_EVENTS subcommands */
enum {
	SMU_PWR_GET_POWERUP_EVENTS      = 0x00,
	SMU_PWR_SET_POWERUP_EVENTS      = 0x01,
	SMU_PWR_CLR_POWERUP_EVENTS      = 0x02,
	SMU_PWR_GET_WAKEUP_EVENTS       = 0x03,
	SMU_PWR_SET_WAKEUP_EVENTS       = 0x04,
	SMU_PWR_CLR_WAKEUP_EVENTS       = 0x05,

	/*
	 * Get last shutdown cause
	 *
	 * Returns:
	 *   1 byte (signed char): Last shutdown cause. Exact meaning unknown.
	 */
	SMU_PWR_LAST_SHUTDOWN_CAUSE	= 0x07,

	/*
	 * Sets or gets server ID. Meaning or use is unknown.
	 *
	 * Parameters:
	 *   2 (optional): Set server ID (1 byte)
	 *
	 * Returns:
	 *   1 byte (server ID?)
	 */
	SMU_PWR_SERVER_ID		= 0x08,
};

/* Power events wakeup bits */
enum {
	SMU_PWR_WAKEUP_KEY              = 0x01, /* Wake on key press */
	SMU_PWR_WAKEUP_AC_INSERT        = 0x02, /* Wake on AC adapter plug */
	SMU_PWR_WAKEUP_AC_CHANGE        = 0x04,
	SMU_PWR_WAKEUP_LID_OPEN         = 0x08,
	SMU_PWR_WAKEUP_RING             = 0x10,
};



#ifdef __KERNEL__


struct smu_cmd;

struct smu_cmd
{
	/* public */
	u8			cmd;		/* command */
	int			data_len;	/* data len */
	int			reply_len;	/* reply len */
	void			*data_buf;	/* data buffer */
	void			*reply_buf;	/* reply buffer */
	int			status;		/* command status */
	void			(*done)(struct smu_cmd *cmd, void *misc);
	void			*misc;

	/* private */
	struct list_head	link;
};

extern int smu_queue_cmd(struct smu_cmd *cmd);

struct smu_simple_cmd
{
	struct smu_cmd	cmd;
	u8	       	buffer[16];
};

extern int smu_queue_simple(struct smu_simple_cmd *scmd, u8 command,
			    unsigned int data_len,
			    void (*done)(struct smu_cmd *cmd, void *misc),
			    void *misc,
			    ...);

extern void smu_done_complete(struct smu_cmd *cmd, void *misc);

extern void smu_spinwait_cmd(struct smu_cmd *cmd);

static inline void smu_spinwait_simple(struct smu_simple_cmd *scmd)
{
	smu_spinwait_cmd(&scmd->cmd);
}

extern void smu_poll(void);


extern int smu_init(void);
extern int smu_present(void);
struct of_device;
extern struct of_device *smu_get_ofdev(void);


extern void smu_shutdown(void);
extern void smu_restart(void);
struct rtc_time;
extern int smu_get_rtc_time(struct rtc_time *time, int spinwait);
extern int smu_set_rtc_time(struct rtc_time *time, int spinwait);

extern unsigned long smu_cmdbuf_abs;



#define SMU_I2C_READ_MAX	0x1d
#define SMU_I2C_WRITE_MAX	0x15

/* SMU i2c header, exactly matches i2c header on wire */
struct smu_i2c_param
{
	u8	bus;		/* SMU bus ID (from device tree) */
	u8	type;		/* i2c transfer type */
	u8	devaddr;	/* device address (includes direction) */
	u8	sublen;		/* subaddress length */
	u8	subaddr[3];	/* subaddress */
	u8	caddr;		/* combined address, filled by SMU driver */
	u8	datalen;	/* length of transfer */
	u8	data[SMU_I2C_READ_MAX];	/* data */
};

struct smu_i2c_cmd
{
	/* public */
	struct smu_i2c_param	info;
	void			(*done)(struct smu_i2c_cmd *cmd, void *misc);
	void			*misc;
	int			status; /* 1 = pending, 0 = ok, <0 = fail */

	/* private */
	struct smu_cmd		scmd;
	int			read;
	int			stage;
	int			retries;
	u8			pdata[32];
	struct list_head	link;
};

extern int smu_queue_i2c(struct smu_i2c_cmd *cmd);


#endif /* __KERNEL__ */




struct smu_sdbp_header {
	__u8	id;
	__u8	len;
	__u8	version;
	__u8	flags;
};


 /*
 * demangle 16 and 32 bits integer in some SMU partitions
 * (currently, afaik, this concerns only the FVT partition
 * (0x12)
 */
#define SMU_U16_MIX(x)	le16_to_cpu(x);
#define SMU_U32_MIX(x)  ((((x) & 0xff00ff00u) >> 8)|(((x) & 0x00ff00ffu) << 8))


#define SMU_SDB_FVT_ID			0x12

struct smu_sdbp_fvt {
	__u32	sysclk;			/* Base SysClk frequency in Hz for
					 * this operating point. Value need to
					 * be unmixed with SMU_U32_MIX()
					 */
	__u8	pad;
	__u8	maxtemp;		/* Max temp. supported by this
					 * operating point
					 */

	__u16	volts[3];		/* CPU core voltage for the 3
					 * PowerTune modes, a mode with
					 * 0V = not supported. Value need
					 * to be unmixed with SMU_U16_MIX()
					 */
};

#define SMU_SDB_CPUVCP_ID		0x21

struct smu_sdbp_cpuvcp {
	__u16	volt_scale;		/* u4.12 fixed point */
	__s16	volt_offset;		/* s4.12 fixed point */
	__u16	curr_scale;		/* u4.12 fixed point */
	__s16	curr_offset;		/* s4.12 fixed point */
	__s32	power_quads[3];		/* s4.28 fixed point */
};

#define SMU_SDB_CPUDIODE_ID		0x18

struct smu_sdbp_cpudiode {
	__u16	m_value;		/* u1.15 fixed point */
	__s16	b_value;		/* s10.6 fixed point */

};

#define SMU_SDB_SLOTSPOW_ID		0x78

struct smu_sdbp_slotspow {
	__u16	pow_scale;		/* u4.12 fixed point */
	__s16	pow_offset;		/* s4.12 fixed point */
};

#define SMU_SDB_SENSORTREE_ID		0x25

struct smu_sdbp_sensortree {
	__u8	model_id;
	__u8	unknown[3];
};

#define SMU_SDB_CPUPIDDATA_ID		0x17

struct smu_sdbp_cpupiddata {
	__u8	unknown1;
	__u8	target_temp_delta;
	__u8	unknown2;
	__u8	history_len;
	__s16	power_adj;
	__u16	max_power;
	__s32	gp,gr,gd;
};


/* Other partitions without known structures */
#define SMU_SDB_DEBUG_SWITCHES_ID	0x05

#ifdef __KERNEL__
extern const struct smu_sdbp_header *smu_get_sdb_partition(int id,
					unsigned int *size);

/* Get "sdb" partition data from an SMU satellite */
extern struct smu_sdbp_header *smu_sat_get_sdb_partition(unsigned int sat_id,
					int id, unsigned int *size);


#endif /* __KERNEL__ */



struct smu_user_cmd_hdr
{
	__u32		cmdtype;
#define SMU_CMDTYPE_SMU			0	/* SMU command */
#define SMU_CMDTYPE_WANTS_EVENTS	1	/* switch fd to events mode */
#define SMU_CMDTYPE_GET_PARTITION	2	/* retrieve an sdb partition */

	__u8		cmd;			/* SMU command byte */
	__u8		pad[3];			/* padding */
	__u32		data_len;		/* Length of data following */
};

struct smu_user_reply_hdr
{
	__u32		status;			/* Command status */
	__u32		reply_len;		/* Length of data follwing */
};

#endif /*  _SMU_H */
