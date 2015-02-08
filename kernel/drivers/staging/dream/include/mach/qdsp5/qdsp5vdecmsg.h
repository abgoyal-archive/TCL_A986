
#ifndef QDSP5VIDDECMSGI_H
#define QDSP5VIDDECMSGI_H



#define	VIDDEC_MSG_SUBF_DONE	0x0000
#define	VIDDEC_MSG_SUBF_DONE_LEN	\
	sizeof(viddec_msg_subf_done)

typedef struct {
	unsigned short	packet_seq_number;
	unsigned short	codec_instance_id;
} __attribute__((packed)) viddec_msg_subf_done;



#define	VIDDEC_MSG_FRAME_DONE	0x0001
#define	VIDDEC_MSG_FRAME_DONE_LEN	\
	sizeof(viddec_msg_frame_done)

typedef struct {
	unsigned short	packet_seq_number;
	unsigned short	codec_instance_id;
} __attribute__((packed)) viddec_msg_frame_done;



#define	VIDDEC_MSG_PP_ENABLE_CMD_DONE	0x0002
#define	VIDDEC_MSG_PP_ENABLE_CMD_DONE_LEN	\
	sizeof(viddec_msg_pp_enable_cmd_done)

typedef struct {
	unsigned short	packet_seq_number;
	unsigned short	codec_instance_id;
} __attribute__((packed)) viddec_msg_pp_enable_cmd_done;




#define	VIDDEC_MSG_PP_FRAME_DONE		0x0003
#define	VIDDEC_MSG_PP_FRAME_DONE_LEN	\
	sizeof(viddec_msg_pp_frame_done)

#define	VIDDEC_MSG_DISP_WORTHY_DISP		0x0000
#define	VIDDEC_MSG_DISP_WORTHY_DISP_NONE	0xFFFF


typedef struct {
	unsigned short	packet_seq_number;
	unsigned short	codec_instance_id;
	unsigned short	display_worthy;
} __attribute__((packed)) viddec_msg_pp_frame_done;


#endif
