

#ifndef __rio_map_h__
#define __rio_map_h__


#define MAX_MAP_ENTRY 17
#define	TOTAL_MAP_ENTRIES (MAX_MAP_ENTRY*RIO_SLOTS)
#define	MAX_NAME_LEN 32

struct Map {
	unsigned int HostUniqueNum;	/* Supporting hosts unique number */
	unsigned int RtaUniqueNum;	/* Unique number */
	/*
	 ** The next two IDs must be swapped on big-endian architectures
	 ** when using a v2.04 /etc/rio/config with a v3.00 driver (when
	 ** upgrading for example).
	 */
	unsigned short ID;		/* ID used in the subnet */
	unsigned short ID2;		/* ID of 2nd block of 8 for 16 port */
	unsigned long Flags;		/* Booted, ID Given, Disconnected */
	unsigned long SysPort;		/* First tty mapped to this port */
	struct Top Topology[LINKS_PER_UNIT];	/* ID connected to each link */
	char Name[MAX_NAME_LEN];	/* Cute name by which RTA is known */
};

#define	RTA_BOOTED		0x00000001
#define RTA_NEWBOOT		0x00000010
#define	MSG_DONE		0x00000020
#define	RTA_INTERCONNECT	0x00000040
#define	RTA16_SECOND_SLOT	0x00000080
#define	BEEN_HERE		0x00000100
#define SLOT_TENTATIVE		0x40000000
#define SLOT_IN_USE		0x80000000


#endif
