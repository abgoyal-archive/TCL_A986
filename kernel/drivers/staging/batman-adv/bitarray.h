


/* you should choose something big, if you don't want to waste cpu */
#define TYPE_OF_WORD unsigned long
#define WORD_BIT_SIZE (sizeof(TYPE_OF_WORD) * 8)

uint8_t get_bit_status(TYPE_OF_WORD *seq_bits, uint16_t last_seqno,
					   uint16_t curr_seqno);

/* turn corresponding bit on, so we can remember that we got the packet */
void bit_mark(TYPE_OF_WORD *seq_bits, int32_t n);

/* shift the packet array by n places. */
void bit_shift(TYPE_OF_WORD *seq_bits, int32_t n);


char bit_get_packet(TYPE_OF_WORD *seq_bits, int16_t seq_num_diff,
					int8_t set_mark);

/* count the hamming weight, how many good packets did we receive? */
int  bit_packet_count(TYPE_OF_WORD *seq_bits);
