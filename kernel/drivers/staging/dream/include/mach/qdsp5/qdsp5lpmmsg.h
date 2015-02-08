
#ifndef QDSP5LPMMSGI_H
#define QDSP5LPMMSGI_H



#define	LPM_MSG_IDLE_ACK	0x0000
#define	LPM_MSG_IDLE_ACK_LEN	sizeof(lpm_msg_idle_ack)

typedef struct {
} __attribute__((packed)) lpm_msg_idle_ack;




#define	LPM_MSG_START_ACK	0x0001
#define	LPM_MSG_START_ACK_LEN	sizeof(lpm_msg_start_ack)


typedef struct {
} __attribute__((packed)) lpm_msg_start_ack;



#define	LPM_MSG_DONE		0x0002
#define	LPM_MSG_DONE_LEN	sizeof(lpm_msg_done)

typedef struct {
} __attribute__((packed)) lpm_msg_done;


#endif
