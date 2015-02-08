
/* sfi.h Simple Firmware Interface */


#ifndef _LINUX_SFI_H
#define _LINUX_SFI_H

/* Table signatures reserved by the SFI specification */
#define SFI_SIG_SYST		"SYST"
#define SFI_SIG_FREQ		"FREQ"
#define SFI_SIG_IDLE		"IDLE"
#define SFI_SIG_CPUS		"CPUS"
#define SFI_SIG_MTMR		"MTMR"
#define SFI_SIG_MRTC		"MRTC"
#define SFI_SIG_MMAP		"MMAP"
#define SFI_SIG_APIC		"APIC"
#define SFI_SIG_XSDT		"XSDT"
#define SFI_SIG_WAKE		"WAKE"
#define SFI_SIG_SPIB		"SPIB"
#define SFI_SIG_I2CB		"I2CB"
#define SFI_SIG_GPEM		"GPEM"
#define SFI_SIG_DEVS		"DEVS"
#define SFI_SIG_GPIO		"GPIO"

#define SFI_SIGNATURE_SIZE	4
#define SFI_OEM_ID_SIZE		6
#define SFI_OEM_TABLE_ID_SIZE	8

#define SFI_SYST_SEARCH_BEGIN		0x000E0000
#define SFI_SYST_SEARCH_END		0x000FFFFF

#define SFI_GET_NUM_ENTRIES(ptable, entry_type) \
	((ptable->header.len - sizeof(struct sfi_table_header)) / \
	(sizeof(entry_type)))
struct sfi_table_header {
	char	sig[SFI_SIGNATURE_SIZE];
	u32	len;
	u8	rev;
	u8	csum;
	char	oem_id[SFI_OEM_ID_SIZE];
	char	oem_table_id[SFI_OEM_TABLE_ID_SIZE];
} __packed;

struct sfi_table_simple {
	struct sfi_table_header		header;
	u64				pentry[1];
} __packed;

/* Comply with UEFI spec 2.1 */
struct sfi_mem_entry {
	u32	type;
	u64	phys_start;
	u64	virt_start;
	u64	pages;
	u64	attrib;
} __packed;

struct sfi_cpu_table_entry {
	u32	apic_id;
} __packed;

struct sfi_cstate_table_entry {
	u32	hint;		/* MWAIT hint */
	u32	latency;	/* latency in ms */
} __packed;

struct sfi_apic_table_entry {
	u64	phys_addr;	/* phy base addr for APIC reg */
} __packed;

struct sfi_freq_table_entry {
	u32	freq_mhz;	/* in MHZ */
	u32	latency;	/* transition latency in ms */
	u32	ctrl_val;	/* value to write to PERF_CTL */
} __packed;

struct sfi_wake_table_entry {
	u64	phys_addr;	/* pointer to where the wake vector locates */
} __packed;

struct sfi_timer_table_entry {
	u64	phys_addr;	/* phy base addr for the timer */
	u32	freq_hz;	/* in HZ */
	u32	irq;
} __packed;

struct sfi_rtc_table_entry {
	u64	phys_addr;	/* phy base addr for the RTC */
	u32	irq;
} __packed;

struct sfi_device_table_entry {
	u8	type;		/* bus type, I2C, SPI or ...*/
#define SFI_DEV_TYPE_SPI	0
#define SFI_DEV_TYPE_I2C	1
#define SFI_DEV_TYPE_UART	2
#define SFI_DEV_TYPE_HSI	3
#define SFI_DEV_TYPE_IPC	4

	u8	host_num;	/* attached to host 0, 1...*/
	u16	addr;
	u8	irq;
	u32	max_freq;
	char	name[16];
} __packed;

struct sfi_gpio_table_entry {
	char	controller_name[16];
	u16	pin_no;
	char	pin_name[16];
} __packed;

struct sfi_spi_table_entry {
	u16	host_num;	/* attached to host 0, 1...*/
	u16	cs;		/* chip select */
	u16	irq_info;
	char	name[16];
	u8	dev_info[10];
} __packed;

struct sfi_i2c_table_entry {
	u16	host_num;
	u16	addr;		/* slave addr */
	u16	irq_info;
	char	name[16];
	u8	dev_info[10];
} __packed;

struct sfi_gpe_table_entry {
	u16	logical_id;	/* logical id */
	u16	phys_id;	/* physical GPE id */
} __packed;

typedef int (*sfi_table_handler) (struct sfi_table_header *table);

#ifdef CONFIG_SFI
extern void __init sfi_init(void);
extern int __init sfi_platform_init(void);
extern void __init sfi_init_late(void);
extern int sfi_table_parse(char *signature, char *oem_id, char *oem_table_id,
				sfi_table_handler handler);

extern int sfi_disabled;
static inline void disable_sfi(void)
{
	sfi_disabled = 1;
}

#else /* !CONFIG_SFI */

static inline void sfi_init(void)
{
}

static inline void sfi_init_late(void)
{
}

#define sfi_disabled	0

static inline int sfi_table_parse(char *signature, char *oem_id,
					char *oem_table_id,
					sfi_table_handler handler)
{
	return -1;
}

#endif /* !CONFIG_SFI */

#endif /*_LINUX_SFI_H*/
