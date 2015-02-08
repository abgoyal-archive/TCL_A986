

struct LAP {
	u_int8_t	dst;
	u_int8_t	src;
	u_int8_t	type;
};
#define lapShortDDP	1	/* short DDP type */
#define lapDDP		2	/* DDP type */
#define lapKLAP		'K'	/* Kinetics KLAP type */

/* Datagram Delivery Protocol */

struct atDDP {
	u_int16_t	length;
	u_int16_t	checksum;
	u_int16_t	dstNet;
	u_int16_t	srcNet;
	u_int8_t	dstNode;
	u_int8_t	srcNode;
	u_int8_t	dstSkt;
	u_int8_t	srcSkt;
	u_int8_t	type;
};

struct atShortDDP {
	u_int16_t	length;
	u_int8_t	dstSkt;
	u_int8_t	srcSkt;
	u_int8_t	type;
};

#define	ddpMaxWKS	0x7F
#define	ddpMaxData	586
#define	ddpLengthMask	0x3FF
#define	ddpHopShift	10
#define	ddpSize		13	/* size of DDP header (avoid struct padding) */
#define	ddpSSize	5
#define	ddpWKS		128	/* boundary of DDP well known sockets */
#define	ddpRTMP		1	/* RTMP type */
#define	ddpRTMPrequest	5	/* RTMP request type */
#define	ddpNBP		2	/* NBP type */
#define	ddpATP		3	/* ATP type */
#define	ddpECHO		4	/* ECHO type */
#define	ddpIP		22	/* IP type */
#define	ddpARP		23	/* ARP type */
#define ddpEIGRP        88      /* EIGRP over Appletalk */
#define	ddpKLAP		0x4b	/* Kinetics KLAP type */


/* AppleTalk Transaction Protocol */

struct atATP {
	u_int8_t	control;
	u_int8_t	bitmap;
	u_int16_t	transID;
	int32_t userData;
};

#define	atpReqCode	0x40
#define	atpRspCode	0x80
#define	atpRelCode	0xC0
#define	atpXO		0x20
#define	atpEOM		0x10
#define	atpSTS		0x08
#define	atpFlagMask	0x3F
#define	atpControlMask	0xF8
#define	atpMaxNum	8
#define	atpMaxData	578


/* AppleTalk Echo Protocol */

struct atEcho {
	u_int8_t	echoFunction;
	u_int8_t	*echoData;
};

#define echoSkt		4		/* the echoer socket */
#define echoSize	1		/* size of echo header */
#define echoRequest	1		/* echo request */
#define echoReply	2		/* echo request */


/* Name Binding Protocol */

struct atNBP {
	u_int8_t	control;
	u_int8_t	id;
};

struct atNBPtuple {
	u_int16_t	net;
	u_int8_t	node;
	u_int8_t	skt;
	u_int8_t	enumerator;
};

#define	nbpBrRq		0x10
#define	nbpLkUp		0x20
#define	nbpLkUpReply	0x30

#define	nbpNIS		2
#define	nbpTupleMax	15

#define	nbpHeaderSize	2
#define nbpTupleSize	5

#define nbpSkt		2		/* NIS */


/* Routing Table Maint. Protocol */

#define	rtmpSkt		1	/* number of RTMP socket */
#define	rtmpSize	4	/* minimum size */
#define	rtmpTupleSize	3


/* Zone Information Protocol */

struct zipHeader {
	u_int8_t	command;
	u_int8_t	netcount;
};

#define	zipHeaderSize	2
#define	zipQuery	1
#define	zipReply	2
#define	zipTakedown	3
#define	zipBringup	4
#define	ddpZIP		6
#define	zipSkt		6
#define	GetMyZone	7
#define	GetZoneList	8

#define atalk_port(p) \
	(((unsigned)((p) - 16512) < 128) || \
	 ((unsigned)((p) - 200) < 128) || \
	 ((unsigned)((p) - 768) < 128))
