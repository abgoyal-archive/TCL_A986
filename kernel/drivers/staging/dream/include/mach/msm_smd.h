

#ifndef __ASM_ARCH_MSM_SMD_H
#define __ASM_ARCH_MSM_SMD_H

typedef struct smd_channel smd_channel_t;

/* warning: notify() may be called before open returns */
int smd_open(const char *name, smd_channel_t **ch, void *priv,
	     void (*notify)(void *priv, unsigned event));

#define SMD_EVENT_DATA 1
#define SMD_EVENT_OPEN 2
#define SMD_EVENT_CLOSE 3

int smd_close(smd_channel_t *ch);

/* passing a null pointer for data reads and discards */
int smd_read(smd_channel_t *ch, void *data, int len);

int smd_write(smd_channel_t *ch, const void *data, int len);

int smd_write_avail(smd_channel_t *ch);
int smd_read_avail(smd_channel_t *ch);

int smd_cur_packet_size(smd_channel_t *ch);

void smd_kick(smd_channel_t *ch);


#if 0
int smd_wait_until_readable(smd_channel_t *ch, int bytes);
int smd_wait_until_writable(smd_channel_t *ch, int bytes);
#endif

typedef enum
{
	SMD_PORT_DS = 0,
	SMD_PORT_DIAG,
	SMD_PORT_RPC_CALL,
	SMD_PORT_RPC_REPLY,
	SMD_PORT_BT,
	SMD_PORT_CONTROL,
	SMD_PORT_MEMCPY_SPARE1,
	SMD_PORT_DATA1,
	SMD_PORT_DATA2,
	SMD_PORT_DATA3,
	SMD_PORT_DATA4,
	SMD_PORT_DATA5,
	SMD_PORT_DATA6,
	SMD_PORT_DATA7,
	SMD_PORT_DATA8,
	SMD_PORT_DATA9,
	SMD_PORT_DATA10,
	SMD_PORT_DATA11,
	SMD_PORT_DATA12,
	SMD_PORT_DATA13,
	SMD_PORT_DATA14,
	SMD_PORT_DATA15,
	SMD_PORT_DATA16,
	SMD_PORT_DATA17,
	SMD_PORT_DATA18,
	SMD_PORT_DATA19,
	SMD_PORT_DATA20,
	SMD_PORT_GPS_NMEA,
	SMD_PORT_BRIDGE_1,
	SMD_PORT_BRIDGE_2,
	SMD_PORT_BRIDGE_3,
	SMD_PORT_BRIDGE_4,
	SMD_PORT_BRIDGE_5,
	SMD_PORT_LOOPBACK,
	SMD_PORT_CS_APPS_MODEM,
	SMD_PORT_CS_APPS_DSP,
	SMD_PORT_CS_MODEM_DSP,
	SMD_NUM_PORTS,
} smd_port_id_type;

#endif
