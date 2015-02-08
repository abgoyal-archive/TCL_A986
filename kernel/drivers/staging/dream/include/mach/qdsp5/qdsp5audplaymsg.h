
#ifndef QDSP5AUDPLAYMSG_H
#define QDSP5AUDPLAYMSG_H

#define AUDPLAY_MSG_DEC_NEEDS_DATA		0x0001
#define AUDPLAY_MSG_DEC_NEEDS_DATA_MSG_LEN	\
	sizeof(audplay_msg_dec_needs_data)

typedef struct{
	/* reserved*/
	unsigned int dec_id;

	/* The read pointer offset of external memory until which the
	 * bitstream has been DMAed in. */
	unsigned int adecDataReadPtrOffset;

	/* The buffer size of external memory. */
	unsigned int adecDataBufSize;

	unsigned int bitstream_free_len;
	unsigned int bitstream_write_ptr;
	unsigned int bitstarem_buf_start;
	unsigned int bitstream_buf_len;
} __attribute__((packed)) audplay_msg_dec_needs_data;

#define AUDPLAY_MSG_BUFFER_UPDATE 0x0004
#define AUDPLAY_MSG_BUFFER_UPDATE_LEN \
	sizeof(struct audplay_msg_buffer_update)

struct audplay_msg_buffer_update {
	unsigned int buffer_write_count;
	unsigned int num_of_buffer;
	unsigned int buf0_address;
	unsigned int buf0_length;
	unsigned int buf1_address;
	unsigned int buf1_length;
} __attribute__((packed));
#endif /* QDSP5AUDPLAYMSG_H */
