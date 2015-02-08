
#ifndef QDSP5AUDPREPROCMSG_H
#define QDSP5AUDPREPROCMSG_H





#define	AUDPREPROC_MSG_CMD_CFG_DONE_MSG	0x0000
#define	AUDPREPROC_MSG_CMD_CFG_DONE_MSG_LEN	\
	sizeof(audpreproc_msg_cmd_cfg_done_msg)

#define	AUDPREPROC_MSG_TYPE_AGC			0x0000
#define	AUDPREPROC_MSG_TYPE_NOISE_REDUCTION	0x0001
#define	AUDPREPROC_MSG_TYPE_IIR_FILTER		0x0002


#define	AUDPREPROC_MSG_STATUS_FLAG_ENA		-1
#define	AUDPREPROC_MSG_STATUS_FLAG_DIS		0x0000

typedef struct {
	unsigned short	type;
	signed short	status_flag;
} __attribute__((packed)) audpreproc_msg_cmd_cfg_done_msg;



#define	AUDPREPROC_MSG_ERROR_MSG_ID		0x0001
#define	AUDPREPROC_MSG_ERROR_MSG_ID_LEN	\
	sizeof(audpreproc_msg_error_msg_id)

#define	AUDPREPROC_MSG_ERR_INDEX_NS		0x0000

typedef struct {
	 unsigned short	err_index;
} __attribute__((packed)) audpreproc_msg_error_msg_id;

#endif
