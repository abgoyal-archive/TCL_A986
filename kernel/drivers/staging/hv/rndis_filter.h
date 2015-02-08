


#ifndef _RNDISFILTER_H_
#define _RNDISFILTER_H_

#define __struct_bcount(x)

#include "netvsc.h"

#include "rndis.h"

#define RNDIS_HEADER_SIZE	(sizeof(struct rndis_message) - \
				 sizeof(union rndis_message_container))

#define NDIS_PACKET_TYPE_DIRECTED	0x00000001
#define NDIS_PACKET_TYPE_MULTICAST	0x00000002
#define NDIS_PACKET_TYPE_ALL_MULTICAST	0x00000004
#define NDIS_PACKET_TYPE_BROADCAST	0x00000008
#define NDIS_PACKET_TYPE_SOURCE_ROUTING	0x00000010
#define NDIS_PACKET_TYPE_PROMISCUOUS	0x00000020
#define NDIS_PACKET_TYPE_SMT		0x00000040
#define NDIS_PACKET_TYPE_ALL_LOCAL	0x00000080
#define NDIS_PACKET_TYPE_GROUP		0x00000100
#define NDIS_PACKET_TYPE_ALL_FUNCTIONAL	0x00000200
#define NDIS_PACKET_TYPE_FUNCTIONAL	0x00000400
#define NDIS_PACKET_TYPE_MAC_FRAME	0x00000800


/* Interface */

extern int RndisFilterInit(struct netvsc_driver *driver);

#endif /* _RNDISFILTER_H_ */
