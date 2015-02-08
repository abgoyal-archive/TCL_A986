
#ifndef QDSP5AUDRECMSGI_H
#define QDSP5AUDRECMSGI_H





#define AUDREC_MSG_CMD_CFG_DONE_MSG	0x0002
#define AUDREC_MSG_CMD_CFG_DONE_MSG_LEN	\
	sizeof(audrec_msg_cmd_cfg_done_msg)


#define AUDREC_MSG_CFG_DONE_TYPE_0_ENA		0x4000
#define AUDREC_MSG_CFG_DONE_TYPE_0_DIS		0x0000

#define AUDREC_MSG_CFG_DONE_TYPE_0_NO_UPDATE	0x0000
#define AUDREC_MSG_CFG_DONE_TYPE_0_UPDATE	0x8000

#define AUDREC_MSG_CFG_DONE_TYPE_1_ENA		0x4000
#define AUDREC_MSG_CFG_DONE_TYPE_1_DIS		0x0000

#define AUDREC_MSG_CFG_DONE_TYPE_1_NO_UPDATE	0x0000
#define AUDREC_MSG_CFG_DONE_TYPE_1_UPDATE	0x8000

typedef struct {
	unsigned short	type_0;
	unsigned short	type_1;
} __attribute__((packed))audrec_msg_cmd_cfg_done_msg;



#define	AUDREC_MSG_CMD_AREC_PARAM_CFG_DONE_MSG		0x0003
#define	AUDREC_MSG_CMD_AREC_PARAM_CFG_DONE_MSG_LEN	\
	sizeof(audrec_msg_cmd_arec_param_cfg_done_msg)

#define	AUDREC_MSG_AREC_PARAM_TYPE_0	0x0000
#define	AUDREC_MSG_AREC_PARAM_TYPE_1	0x0001

typedef struct {
	unsigned short	type;
} __attribute__((packed))audrec_msg_cmd_arec_param_cfg_done_msg;



#define AUDREC_MSG_FATAL_ERR_MSG		0x0004
#define AUDREC_MSG_FATAL_ERR_MSG_LEN	\
	sizeof(audrec_msg_fatal_err_msg)

#define AUDREC_MSG_FATAL_ERR_TYPE_0	0x0000
#define AUDREC_MSG_FATAL_ERR_TYPE_1	0x0001

typedef struct {
	unsigned short	type;
} __attribute__((packed))audrec_msg_fatal_err_msg;


#define AUDREC_MSG_PACKET_READY_MSG		0x0005
#define AUDREC_MSG_PACKET_READY_MSG_LEN	\
	sizeof(audrec_msg_packet_ready_msg)

#define AUDREC_MSG_PACKET_READY_TYPE_0	0x0000
#define AUDREC_MSG_PACKET_READY_TYPE_1	0x0001

typedef struct {
	unsigned short	type;
	unsigned short	pkt_counter_msw;
	unsigned short	pkt_counter_lsw;
	unsigned short	pkt_read_cnt_msw;
	unsigned short	pkt_read_cnt_lsw;
} __attribute__((packed))audrec_msg_packet_ready_msg;

#endif
