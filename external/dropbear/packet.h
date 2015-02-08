
#ifndef _PACKET_H_

#define _PACKET_H_

#include "includes.h"

void write_packet();
void read_packet();
void decrypt_packet();
void encrypt_packet();

void process_packet();

typedef struct PacketType {
	unsigned char type; /* SSH_MSG_FOO */
	void (*handler)();
} packettype;

#define PACKET_PADDING_OFF 4
#define PACKET_PAYLOAD_OFF 5

#define INIT_READBUF 200

#endif /* _PACKET_H_ */
