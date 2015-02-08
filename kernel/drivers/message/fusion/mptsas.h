
/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/

#ifndef MPTSAS_H_INCLUDED
#define MPTSAS_H_INCLUDED
/*{-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/

struct mptsas_target_reset_event {
	struct list_head 	list;
	EVENT_DATA_SAS_DEVICE_STATUS_CHANGE sas_event_data;
	u8	target_reset_issued;
	unsigned long	 time_count;
};

enum mptsas_hotplug_action {
	MPTSAS_ADD_DEVICE,
	MPTSAS_DEL_DEVICE,
	MPTSAS_ADD_RAID,
	MPTSAS_DEL_RAID,
	MPTSAS_ADD_PHYSDISK,
	MPTSAS_ADD_PHYSDISK_REPROBE,
	MPTSAS_DEL_PHYSDISK,
	MPTSAS_DEL_PHYSDISK_REPROBE,
	MPTSAS_ADD_INACTIVE_VOLUME,
	MPTSAS_IGNORE_EVENT,
};

struct mptsas_mapping{
	u8			id;
	u8			channel;
};

struct mptsas_device_info {
	struct list_head 	list;
	struct mptsas_mapping	os;	/* operating system mapping*/
	struct mptsas_mapping	fw;	/* firmware mapping */
	u64			sas_address;
	u32			device_info; /* specific bits for devices */
	u16			slot;		/* enclosure slot id */
	u64			enclosure_logical_id; /*enclosure address */
	u8			is_logical_volume; /* is this logical volume */
	/* this belongs to volume */
	u8			is_hidden_raid_component;
	/* this valid when is_hidden_raid_component set */
	u8			volume_id;
	/* cached data for a removed device */
	u8			is_cached;
};

struct mptsas_hotplug_event {
	MPT_ADAPTER		*ioc;
	enum mptsas_hotplug_action event_type;
	u64			sas_address;
	u8			channel;
	u8			id;
	u32			device_info;
	u16			handle;
	u8			phy_id;
	u8			phys_disk_num;		/* hrc - unique index*/
	struct scsi_device	*sdev;
};

struct fw_event_work {
	struct list_head 	list;
	struct delayed_work	 work;
	MPT_ADAPTER	*ioc;
	u32			event;
	u8			retries;
	u8			__attribute__((aligned(4))) event_data[1];
};

struct mptsas_discovery_event {
	struct work_struct	work;
	MPT_ADAPTER		*ioc;
};


struct mptsas_devinfo {
	u16	handle;		/* unique id to address this device */
	u16	handle_parent;	/* unique id to address parent device */
	u16	handle_enclosure; /* enclosure identifier of the enclosure */
	u16	slot;		/* physical slot in enclosure */
	u8	phy_id;		/* phy number of parent device */
	u8	port_id;	/* sas physical port this device
				   is assoc'd with */
	u8	id;		/* logical target id of this device */
	u32	phys_disk_num;	/* phys disk id, for csmi-ioctls */
	u8	channel;	/* logical bus number of this device */
	u64	sas_address;    /* WWN of this device,
				   SATA is assigned by HBA,expander */
	u32	device_info;	/* bitfield detailed info about this device */
};

struct mptsas_portinfo_details{
	u16	num_phys;	/* number of phys belong to this port */
	u64	phy_bitmask; 	/* TODO, extend support for 255 phys */
	struct sas_rphy *rphy;	/* transport layer rphy object */
	struct sas_port *port;	/* transport layer port object */
	struct scsi_target *starget;
	struct mptsas_portinfo *port_info;
};

struct mptsas_phyinfo {
	u16	handle;			/* unique id to address this */
	u8	phy_id; 		/* phy index */
	u8	port_id; 		/* firmware port identifier */
	u8	negotiated_link_rate;	/* nego'd link rate for this phy */
	u8	hw_link_rate; 		/* hardware max/min phys link rate */
	u8	programmed_link_rate;	/* programmed max/min phy link rate */
	u8	sas_port_add_phy;	/* flag to request sas_port_add_phy*/
	struct mptsas_devinfo identify;	/* point to phy device info */
	struct mptsas_devinfo attached;	/* point to attached device info */
	struct sas_phy *phy;		/* transport layer phy object */
	struct mptsas_portinfo *portinfo;
	struct mptsas_portinfo_details * port_details;
};

struct mptsas_portinfo {
	struct list_head list;
	u16		num_phys;	/* number of phys */
	struct mptsas_phyinfo *phy_info;
};

struct mptsas_enclosure {
	u64	enclosure_logical_id;	/* The WWN for the enclosure */
	u16	enclosure_handle;	/* unique id to address this */
	u16	flags;			/* details enclosure management */
	u16	num_slot;		/* num slots */
	u16	start_slot;		/* first slot */
	u8	start_id;		/* starting logical target id */
	u8	start_channel;		/* starting logical channel id */
	u8	sep_id;			/* SEP device logical target id */
	u8	sep_channel;		/* SEP channel logical channel id */
};

/*}-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
#endif
