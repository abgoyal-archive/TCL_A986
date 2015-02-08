

#ifndef _TIPC_DBG_H
#define _TIPC_DBG_H


struct print_buf {
	char *buf;
	u32 size;
	char *crs;
	int echo;
};

#define TIPC_PB_MIN_SIZE 64	/* minimum size for a print buffer's array */
#define TIPC_PB_MAX_STR 512	/* max printable string (with trailing NUL) */

void tipc_printbuf_init(struct print_buf *pb, char *buf, u32 size);
void tipc_printbuf_reset(struct print_buf *pb);
int  tipc_printbuf_empty(struct print_buf *pb);
int  tipc_printbuf_validate(struct print_buf *pb);
void tipc_printbuf_move(struct print_buf *pb_to, struct print_buf *pb_from);

int tipc_log_resize(int log_size);

struct sk_buff *tipc_log_resize_cmd(const void *req_tlv_area,
				    int req_tlv_space);
struct sk_buff *tipc_log_dump(void);

#endif
