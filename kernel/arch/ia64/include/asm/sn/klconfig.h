
#ifndef _ASM_IA64_SN_KLCONFIG_H
#define _ASM_IA64_SN_KLCONFIG_H


typedef s32 klconf_off_t;


/* Functions/macros needed to use this structure */

typedef struct kl_config_hdr {
	char		pad[20];
	klconf_off_t	ch_board_info;	/* the link list of boards */
	char		pad0[88];
} kl_config_hdr_t;


#define NODE_OFFSET_TO_LBOARD(nasid,off)        (lboard_t*)(GLOBAL_CAC_ADDR((nasid), (off)))




#define KLCLASS_MASK	0xf0   
#define KLCLASS_NONE	0x00
#define KLCLASS_NODE	0x10             /* CPU, Memory and HUB board */
#define KLCLASS_CPU	KLCLASS_NODE	
#define KLCLASS_IO	0x20             /* BaseIO, 4 ch SCSI, ethernet, FDDI 
					    and the non-graphics widget boards */
#define KLCLASS_ROUTER	0x30             /* Router board */
#define KLCLASS_MIDPLANE 0x40            /* We need to treat this as a board
                                            so that we can record error info */
#define KLCLASS_IOBRICK	0x70		/* IP35 iobrick */
#define KLCLASS_MAX	8		/* Bump this if a new CLASS is added */

#define KLCLASS(_x) ((_x) & KLCLASS_MASK)



#define KLTYPE_MASK	0x0f
#define KLTYPE(_x)      ((_x) & KLTYPE_MASK)

#define KLTYPE_SNIA	(KLCLASS_CPU | 0x1)
#define KLTYPE_TIO	(KLCLASS_CPU | 0x2)

#define KLTYPE_ROUTER     (KLCLASS_ROUTER | 0x1)
#define KLTYPE_META_ROUTER (KLCLASS_ROUTER | 0x3)
#define KLTYPE_REPEATER_ROUTER (KLCLASS_ROUTER | 0x4)

#define KLTYPE_IOBRICK_XBOW	(KLCLASS_MIDPLANE | 0x2)

#define KLTYPE_IOBRICK		(KLCLASS_IOBRICK | 0x0)
#define KLTYPE_NBRICK		(KLCLASS_IOBRICK | 0x4)
#define KLTYPE_PXBRICK		(KLCLASS_IOBRICK | 0x6)
#define KLTYPE_IXBRICK		(KLCLASS_IOBRICK | 0x7)
#define KLTYPE_CGBRICK		(KLCLASS_IOBRICK | 0x8)
#define KLTYPE_OPUSBRICK	(KLCLASS_IOBRICK | 0x9)
#define KLTYPE_SABRICK          (KLCLASS_IOBRICK | 0xa)
#define KLTYPE_IABRICK		(KLCLASS_IOBRICK | 0xb)
#define KLTYPE_PABRICK          (KLCLASS_IOBRICK | 0xc)
#define KLTYPE_GABRICK		(KLCLASS_IOBRICK | 0xd)



#define MAX_COMPTS_PER_BRD 24

typedef struct lboard_s {
	klconf_off_t 	brd_next_any;     /* Next BOARD */
	unsigned char 	struct_type;      /* type of structure, local or remote */
	unsigned char 	brd_type;         /* type+class */
	unsigned char 	brd_sversion;     /* version of this structure */
        unsigned char 	brd_brevision;    /* board revision */
        unsigned char 	brd_promver;      /* board prom version, if any */
 	unsigned char 	brd_flags;        /* Enabled, Disabled etc */
	unsigned char 	brd_slot;         /* slot number */
	unsigned short	brd_debugsw;      /* Debug switches */
	geoid_t		brd_geoid;	  /* geo id */
	partid_t 	brd_partition;    /* Partition number */
        unsigned short 	brd_diagval;      /* diagnostic value */
        unsigned short 	brd_diagparm;     /* diagnostic parameter */
        unsigned char 	brd_inventory;    /* inventory history */
        unsigned char 	brd_numcompts;    /* Number of components */
        nic_t         	brd_nic;          /* Number in CAN */
	nasid_t		brd_nasid;        /* passed parameter */
	klconf_off_t 	brd_compts[MAX_COMPTS_PER_BRD]; /* pointers to COMPONENTS */
	klconf_off_t 	brd_errinfo;      /* Board's error information */
	struct lboard_s *brd_parent;	  /* Logical parent for this brd */
	char            pad0[4];
	unsigned char	brd_confidence;	  /* confidence that the board is bad */
	nasid_t		brd_owner;        /* who owns this board */
	unsigned char 	brd_nic_flags;    /* To handle 8 more NICs */
	char		pad1[24];	  /* future expansion */
	char		brd_name[32];
	nasid_t		brd_next_same_host; /* host of next brd w/same nasid */
	klconf_off_t	brd_next_same;    /* Next BOARD with same nasid */
} lboard_t;

 
typedef struct klinfo_s {                  /* Generic info */
        unsigned char   struct_type;       /* type of this structure */
        unsigned char   struct_version;    /* version of this structure */
        unsigned char   flags;            /* Enabled, disabled etc */
        unsigned char   revision;         /* component revision */
        unsigned short  diagval;          /* result of diagnostics */
        unsigned short  diagparm;         /* diagnostic parameter */
        unsigned char   inventory;        /* previous inventory status */
        unsigned short  partid;		   /* widget part number */
	nic_t 		nic;              /* MUst be aligned properly */
        unsigned char   physid;           /* physical id of component */
        unsigned int    virtid;           /* virtual id as seen by system */
	unsigned char	widid;	          /* Widget id - if applicable */
	nasid_t		nasid;            /* node number - from parent */
	char		pad1;		  /* pad out structure. */
	char		pad2;		  /* pad out structure. */
	void		*data;
        klconf_off_t	errinfo;          /* component specific errors */
        unsigned short  pad3;             /* pci fields have moved over to */
        unsigned short  pad4;             /* klbri_t */
} klinfo_t ;


static inline lboard_t *find_lboard_next(lboard_t * brd)
{
	if (brd && brd->brd_next_any)
		return NODE_OFFSET_TO_LBOARD(NASID_GET(brd), brd->brd_next_any);
        return NULL;
}

#endif /* _ASM_IA64_SN_KLCONFIG_H */
