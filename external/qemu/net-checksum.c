

#include "hw/hw.h"
#include "net.h"

#define PROTO_TCP  6
#define PROTO_UDP 17

uint32_t net_checksum_add(int len, uint8_t *buf)
{
    uint32_t sum = 0;
    int i;

    for (i = 0; i < len; i++) {
	if (i & 1)
	    sum += (uint32_t)buf[i];
	else
	    sum += (uint32_t)buf[i] << 8;
    }
    return sum;
}

uint16_t net_checksum_finish(uint32_t sum)
{
    while (sum>>16)
	sum = (sum & 0xFFFF)+(sum >> 16);
    return ~sum;
}

uint16_t net_checksum_tcpudp(uint16_t length, uint16_t proto,
                             uint8_t *addrs, uint8_t *buf)
{
    uint32_t sum = 0;

    sum += net_checksum_add(length, buf);         // payload
    sum += net_checksum_add(8, addrs);            // src + dst address
    sum += proto + length;                        // protocol & length
    return net_checksum_finish(sum);
}

void net_checksum_calculate(uint8_t *data, int length)
{
    int hlen, plen, proto, csum_offset;
    uint16_t csum;

    if ((data[14] & 0xf0) != 0x40)
	return; /* not IPv4 */
    hlen  = (data[14] & 0x0f) * 4;
    plen  = (data[16] << 8 | data[17]) - hlen;
    proto = data[23];

    switch (proto) {
    case PROTO_TCP:
	csum_offset = 16;
	break;
    case PROTO_UDP:
	csum_offset = 6;
	break;
    default:
	return;
    }

    if (plen < csum_offset+2)
	return;

    data[14+hlen+csum_offset]   = 0;
    data[14+hlen+csum_offset+1] = 0;
    csum = net_checksum_tcpudp(plen, proto, data+14+12, data+14+hlen);
    data[14+hlen+csum_offset]   = csum >> 8;
    data[14+hlen+csum_offset+1] = csum & 0xff;
}
