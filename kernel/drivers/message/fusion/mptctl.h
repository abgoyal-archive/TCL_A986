
/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/

#ifndef MPTCTL_H_INCLUDED
#define MPTCTL_H_INCLUDED
/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/



/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
#define MPT_MISCDEV_BASENAME            "mptctl"
#define MPT_MISCDEV_PATHNAME            "/dev/" MPT_MISCDEV_BASENAME

#define MPT_PRODUCT_LENGTH              12

#define MPT_MAGIC_NUMBER	'm'

#define MPTRWPERF		_IOWR(MPT_MAGIC_NUMBER,0,struct mpt_raw_r_w)

#define MPTFWDOWNLOAD		_IOWR(MPT_MAGIC_NUMBER,15,struct mpt_fw_xfer)
#define MPTCOMMAND		_IOWR(MPT_MAGIC_NUMBER,20,struct mpt_ioctl_command)

#if defined(__KERNEL__) && defined(CONFIG_COMPAT)
#define MPTFWDOWNLOAD32		_IOWR(MPT_MAGIC_NUMBER,15,struct mpt_fw_xfer32)
#define MPTCOMMAND32		_IOWR(MPT_MAGIC_NUMBER,20,struct mpt_ioctl_command32)
#endif

#define MPTIOCINFO		_IOWR(MPT_MAGIC_NUMBER,17,struct mpt_ioctl_iocinfo)
#define MPTIOCINFO1		_IOWR(MPT_MAGIC_NUMBER,17,struct mpt_ioctl_iocinfo_rev0)
#define MPTIOCINFO2		_IOWR(MPT_MAGIC_NUMBER,17,struct mpt_ioctl_iocinfo_rev1)
#define MPTTARGETINFO		_IOWR(MPT_MAGIC_NUMBER,18,struct mpt_ioctl_targetinfo)
#define MPTTEST			_IOWR(MPT_MAGIC_NUMBER,19,struct mpt_ioctl_test)
#define MPTEVENTQUERY		_IOWR(MPT_MAGIC_NUMBER,21,struct mpt_ioctl_eventquery)
#define MPTEVENTENABLE		_IOWR(MPT_MAGIC_NUMBER,22,struct mpt_ioctl_eventenable)
#define MPTEVENTREPORT		_IOWR(MPT_MAGIC_NUMBER,23,struct mpt_ioctl_eventreport)
#define MPTHARDRESET		_IOWR(MPT_MAGIC_NUMBER,24,struct mpt_ioctl_diag_reset)
#define MPTFWREPLACE		_IOWR(MPT_MAGIC_NUMBER,25,struct mpt_ioctl_replace_fw)

struct mpt_fw_xfer {
	unsigned int	 iocnum;	/* IOC unit number */
	unsigned int	 fwlen;
	void		__user *bufp;	/* Pointer to firmware buffer */
};

#if defined(__KERNEL__) && defined(CONFIG_COMPAT)
struct mpt_fw_xfer32 {
	unsigned int iocnum;
	unsigned int fwlen;
	u32 bufp;
};
#endif	/*}*/

typedef struct _mpt_ioctl_header {
	unsigned int	 iocnum;	/* IOC unit number */
	unsigned int	 port;		/* IOC port number */
	int		 maxDataSize;	/* Maximum Num. bytes to transfer on read */
} mpt_ioctl_header;

struct mpt_ioctl_diag_reset {
	mpt_ioctl_header hdr;
};


struct mpt_ioctl_pci_info {
	union {
		struct {
			unsigned int  deviceNumber   :  5;
			unsigned int  functionNumber :  3;
			unsigned int  busNumber      : 24;
		} bits;
		unsigned int  asUlong;
	} u;
};

struct mpt_ioctl_pci_info2 {
	union {
		struct {
			unsigned int  deviceNumber   :  5;
			unsigned int  functionNumber :  3;
			unsigned int  busNumber      : 24;
		} bits;
		unsigned int  asUlong;
	} u;
  int segmentID;
};

#define MPT_IOCTL_INTERFACE_SCSI	(0x00)
#define MPT_IOCTL_INTERFACE_FC		(0x01)
#define MPT_IOCTL_INTERFACE_FC_IP	(0x02)
#define MPT_IOCTL_INTERFACE_SAS		(0x03)
#define MPT_IOCTL_VERSION_LENGTH	(32)

struct mpt_ioctl_iocinfo {
	mpt_ioctl_header hdr;
	int		 adapterType;	/* SCSI or FCP */
	int		 port;		/* port number */
	int		 pciId;		/* PCI Id. */
	int		 hwRev;		/* hardware revision */
	int		 subSystemDevice;	/* PCI subsystem Device ID */
	int		 subSystemVendor;	/* PCI subsystem Vendor ID */
	int		 numDevices;		/* number of devices */
	int		 FWVersion;		/* FW Version (integer) */
	int		 BIOSVersion;		/* BIOS Version (integer) */
	char		 driverVersion[MPT_IOCTL_VERSION_LENGTH];	/* Driver Version (string) */
	char		 busChangeEvent;
	char		 hostId;
	char		 rsvd[2];
	struct mpt_ioctl_pci_info2  pciInfo; /* Added Rev 2 */
};

struct mpt_ioctl_iocinfo_rev1 {
	mpt_ioctl_header hdr;
	int		 adapterType;	/* SCSI or FCP */
	int		 port;		/* port number */
	int		 pciId;		/* PCI Id. */
	int		 hwRev;		/* hardware revision */
	int		 subSystemDevice;	/* PCI subsystem Device ID */
	int		 subSystemVendor;	/* PCI subsystem Vendor ID */
	int		 numDevices;		/* number of devices */
	int		 FWVersion;		/* FW Version (integer) */
	int		 BIOSVersion;		/* BIOS Version (integer) */
	char		 driverVersion[MPT_IOCTL_VERSION_LENGTH];	/* Driver Version (string) */
	char		 busChangeEvent;
	char		 hostId;
	char		 rsvd[2];
	struct mpt_ioctl_pci_info  pciInfo; /* Added Rev 1 */
};

struct mpt_ioctl_iocinfo_rev0 {
	mpt_ioctl_header hdr;
	int		 adapterType;	/* SCSI or FCP */
	int		 port;		/* port number */
	int		 pciId;		/* PCI Id. */
	int		 hwRev;		/* hardware revision */
	int		 subSystemDevice;	/* PCI subsystem Device ID */
	int		 subSystemVendor;	/* PCI subsystem Vendor ID */
	int		 numDevices;		/* number of devices */
	int		 FWVersion;		/* FW Version (integer) */
	int		 BIOSVersion;		/* BIOS Version (integer) */
	char		 driverVersion[MPT_IOCTL_VERSION_LENGTH];	/* Driver Version (string) */
	char		 busChangeEvent;
	char		 hostId;
	char		 rsvd[2];
};

struct mpt_ioctl_targetinfo {
	mpt_ioctl_header hdr;
	int		 numDevices;	/* Num targets on this ioc */
	int		 targetInfo[1];
};


struct mpt_ioctl_eventquery {
	mpt_ioctl_header hdr;
	unsigned short	 eventEntries;
	unsigned short	 reserved;
	unsigned int	 eventTypes;
};

struct mpt_ioctl_eventenable {
	mpt_ioctl_header hdr;
	unsigned int	 eventTypes;
};

#ifndef __KERNEL__
typedef struct {
	uint	event;
	uint	eventContext;
	uint	data[2];
} MPT_IOCTL_EVENTS;
#endif

struct mpt_ioctl_eventreport {
	mpt_ioctl_header	hdr;
	MPT_IOCTL_EVENTS	eventData[1];
};

#define MPT_MAX_NAME	32
struct mpt_ioctl_test {
	mpt_ioctl_header hdr;
	u8		 name[MPT_MAX_NAME];
	int		 chip_type;
	u8		 product [MPT_PRODUCT_LENGTH];
};

typedef struct mpt_ioctl_replace_fw {
	mpt_ioctl_header hdr;
	int		 newImageSize;
	u8		 newImage[1];
} mpt_ioctl_replace_fw_t;

struct mpt_ioctl_command {
	mpt_ioctl_header hdr;
	int		timeout;	/* optional (seconds) */
	char		__user *replyFrameBufPtr;
	char		__user *dataInBufPtr;
	char		__user *dataOutBufPtr;
	char		__user *senseDataPtr;
	int		maxReplyBytes;
	int		dataInSize;
	int		dataOutSize;
	int		maxSenseBytes;
	int		dataSgeOffset;
	char		MF[1];
};

#if defined(__KERNEL__) && defined(CONFIG_COMPAT)
struct mpt_ioctl_command32 {
	mpt_ioctl_header hdr;
	int	timeout;
	u32	replyFrameBufPtr;
	u32	dataInBufPtr;
	u32	dataOutBufPtr;
	u32	senseDataPtr;
	int	maxReplyBytes;
	int	dataInSize;
	int	dataOutSize;
	int	maxSenseBytes;
	int	dataSgeOffset;
	char	MF[1];
};
#endif	/*}*/


/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/

#define CPQFCTS_IOC_MAGIC 'Z'
#define HP_IOC_MAGIC 'Z'
#define HP_GETHOSTINFO		_IOR(HP_IOC_MAGIC, 20, hp_host_info_t)
#define HP_GETHOSTINFO1		_IOR(HP_IOC_MAGIC, 20, hp_host_info_rev0_t)
#define HP_GETTARGETINFO	_IOR(HP_IOC_MAGIC, 21, hp_target_info_t)

typedef struct _hp_header {
	unsigned int iocnum;
	unsigned int host;
	unsigned int channel;
	unsigned int id;
	unsigned int lun;
} hp_header_t;

typedef struct _hp_host_info {
	hp_header_t	 hdr;
	u16		 vendor;
	u16		 device;
	u16		 subsystem_vendor;
	u16		 subsystem_id;
	u8		 devfn;
	u8		 bus;
	ushort		 host_no;		/* SCSI Host number, if scsi driver not loaded*/
	u8		 fw_version[16];	/* string */
	u8		 serial_number[24];	/* string */
	u32		 ioc_status;
	u32		 bus_phys_width;
	u32		 base_io_addr;
	u32		 rsvd;
	unsigned int	 hard_resets;		/* driver initiated resets */
	unsigned int	 soft_resets;		/* ioc, external resets */
	unsigned int	 timeouts;		/* num timeouts */
} hp_host_info_t;

typedef struct _hp_host_info_rev0 {
	hp_header_t	 hdr;
	u16		 vendor;
	u16		 device;
	u16		 subsystem_vendor;
	u16		 subsystem_id;
	u8		 devfn;
	u8		 bus;
	ushort		 host_no;		/* SCSI Host number, if scsi driver not loaded*/
	u8		 fw_version[16];	/* string */
	u8		 serial_number[24];	/* string */
	u32		 ioc_status;
	u32		 bus_phys_width;
	u32		 base_io_addr;
	u32		 rsvd;
	unsigned long	 hard_resets;		/* driver initiated resets */
	unsigned long	 soft_resets;		/* ioc, external resets */
	unsigned long	 timeouts;		/* num timeouts */
} hp_host_info_rev0_t;

typedef struct _hp_target_info {
	hp_header_t	 hdr;
	u32 parity_errors;
	u32 phase_errors;
	u32 select_timeouts;
	u32 message_rejects;
	u32 negotiated_speed;
	u8  negotiated_width;
	u8  rsvd[7];				/* 8 byte alignment */
} hp_target_info_t;

#define HP_STATUS_OTHER		1
#define HP_STATUS_OK		2
#define HP_STATUS_FAILED	3

#define HP_BUS_WIDTH_UNK	1
#define HP_BUS_WIDTH_8		2
#define HP_BUS_WIDTH_16		3
#define HP_BUS_WIDTH_32		4

#define HP_DEV_SPEED_ASYNC	2
#define HP_DEV_SPEED_FAST	3
#define HP_DEV_SPEED_ULTRA	4
#define HP_DEV_SPEED_ULTRA2	5
#define HP_DEV_SPEED_ULTRA160	6
#define HP_DEV_SPEED_SCSI1	7
#define HP_DEV_SPEED_ULTRA320	8

/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/


/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/

#endif

