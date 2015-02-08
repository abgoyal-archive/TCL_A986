

#include "core.h"
#include "config.h"
#include "dbg.h"


static struct print_buf null_buf = { NULL, 0, NULL, 0 };
struct print_buf *const TIPC_NULL = &null_buf;

static struct print_buf cons_buf = { NULL, 0, NULL, 1 };
struct print_buf *const TIPC_CONS = &cons_buf;

static struct print_buf log_buf = { NULL, 0, NULL, 1 };
struct print_buf *const TIPC_LOG = &log_buf;


static char print_string[TIPC_PB_MAX_STR];
static DEFINE_SPINLOCK(print_lock);


#define FORMAT(PTR,LEN,FMT) \
{\
       va_list args;\
       va_start(args, FMT);\
       LEN = vsprintf(PTR, FMT, args);\
       va_end(args);\
       *(PTR + LEN) = '\0';\
}


void tipc_printbuf_init(struct print_buf *pb, char *raw, u32 size)
{
	pb->buf = raw;
	pb->crs = raw;
	pb->size = size;
	pb->echo = 0;

	if (size < TIPC_PB_MIN_SIZE) {
		pb->buf = NULL;
	} else if (raw) {
		pb->buf[0] = 0;
		pb->buf[size - 1] = ~0;
	}
}


void tipc_printbuf_reset(struct print_buf *pb)
{
	if (pb->buf) {
		pb->crs = pb->buf;
		pb->buf[0] = 0;
		pb->buf[pb->size - 1] = ~0;
	}
}


int tipc_printbuf_empty(struct print_buf *pb)
{
	return (!pb->buf || (pb->crs == pb->buf));
}


int tipc_printbuf_validate(struct print_buf *pb)
{
	char *err = "\n\n*** PRINT BUFFER OVERFLOW ***\n\n";
	char *cp_buf;
	struct print_buf cb;

	if (!pb->buf)
		return 0;

	if (pb->buf[pb->size - 1] == 0) {
		cp_buf = kmalloc(pb->size, GFP_ATOMIC);
		if (cp_buf) {
			tipc_printbuf_init(&cb, cp_buf, pb->size);
			tipc_printbuf_move(&cb, pb);
			tipc_printbuf_move(pb, &cb);
			kfree(cp_buf);
			memcpy(pb->buf, err, strlen(err));
		} else {
			tipc_printbuf_reset(pb);
			tipc_printf(pb, err);
		}
	}
	return (pb->crs - pb->buf + 1);
}


void tipc_printbuf_move(struct print_buf *pb_to, struct print_buf *pb_from)
{
	int len;

	/* Handle the cases where contents can't be moved */

	if (!pb_to->buf)
		return;

	if (!pb_from->buf) {
		tipc_printbuf_reset(pb_to);
		return;
	}

	if (pb_to->size < pb_from->size) {
		strcpy(pb_to->buf, "*** PRINT BUFFER MOVE ERROR ***");
		pb_to->buf[pb_to->size - 1] = ~0;
		pb_to->crs = strchr(pb_to->buf, 0);
		return;
	}

	/* Copy data from char after cursor to end (if used) */

	len = pb_from->buf + pb_from->size - pb_from->crs - 2;
	if ((pb_from->buf[pb_from->size - 1] == 0) && (len > 0)) {
		strcpy(pb_to->buf, pb_from->crs + 1);
		pb_to->crs = pb_to->buf + len;
	} else
		pb_to->crs = pb_to->buf;

	/* Copy data from start to cursor (always) */

	len = pb_from->crs - pb_from->buf;
	strcpy(pb_to->crs, pb_from->buf);
	pb_to->crs += len;

	tipc_printbuf_reset(pb_from);
}


void tipc_printf(struct print_buf *pb, const char *fmt, ...)
{
	int chars_to_add;
	int chars_left;
	char save_char;

	spin_lock_bh(&print_lock);

	FORMAT(print_string, chars_to_add, fmt);
	if (chars_to_add >= TIPC_PB_MAX_STR)
		strcpy(print_string, "*** PRINT BUFFER STRING TOO LONG ***");

	if (pb->buf) {
		chars_left = pb->buf + pb->size - pb->crs - 1;
		if (chars_to_add <= chars_left) {
			strcpy(pb->crs, print_string);
			pb->crs += chars_to_add;
		} else if (chars_to_add >= (pb->size - 1)) {
			strcpy(pb->buf, print_string + chars_to_add + 1
			       - pb->size);
			pb->crs = pb->buf + pb->size - 1;
		} else {
			strcpy(pb->buf, print_string + chars_left);
			save_char = print_string[chars_left];
			print_string[chars_left] = 0;
			strcpy(pb->crs, print_string);
			print_string[chars_left] = save_char;
			pb->crs = pb->buf + chars_to_add - chars_left;
		}
	}

	if (pb->echo)
		printk("%s", print_string);

	spin_unlock_bh(&print_lock);
}

#ifdef CONFIG_TIPC_DEBUG


static void print_to_console(char *crs, int len)
{
	int rest = len;

	while (rest > 0) {
		int sz = rest < TIPC_PB_MAX_STR ? rest : TIPC_PB_MAX_STR;
		char c = crs[sz];

		crs[sz] = 0;
		printk((const char *)crs);
		crs[sz] = c;
		rest -= sz;
		crs += sz;
	}
}


static void printbuf_dump(struct print_buf *pb)
{
	int len;

	if (!pb->buf) {
		printk("*** PRINT BUFFER NOT ALLOCATED ***");
		return;
	}

	/* Dump print buffer from char after cursor to end (if used) */

	len = pb->buf + pb->size - pb->crs - 2;
	if ((pb->buf[pb->size - 1] == 0) && (len > 0))
		print_to_console(pb->crs + 1, len);

	/* Dump print buffer from start to cursor (always) */

	len = pb->crs - pb->buf;
	print_to_console(pb->buf, len);
}


void tipc_dump_dbg(struct print_buf *pb, const char *fmt, ...)
{
	int len;

	if (pb == TIPC_CONS)
		return;

	spin_lock_bh(&print_lock);

	FORMAT(print_string, len, fmt);
	printk(print_string);

	printk("\n---- Start of %s log dump ----\n\n",
	       (pb == TIPC_LOG) ? "global" : "local");
	printbuf_dump(pb);
	tipc_printbuf_reset(pb);
	printk("\n---- End of dump ----\n");

	spin_unlock_bh(&print_lock);
}

#endif


int tipc_log_resize(int log_size)
{
	int res = 0;

	spin_lock_bh(&print_lock);
	if (TIPC_LOG->buf) {
		kfree(TIPC_LOG->buf);
		TIPC_LOG->buf = NULL;
	}
	if (log_size) {
		if (log_size < TIPC_PB_MIN_SIZE)
			log_size = TIPC_PB_MIN_SIZE;
		res = TIPC_LOG->echo;
		tipc_printbuf_init(TIPC_LOG, kmalloc(log_size, GFP_ATOMIC),
				   log_size);
		TIPC_LOG->echo = res;
		res = !TIPC_LOG->buf;
	}
	spin_unlock_bh(&print_lock);

	return res;
}


struct sk_buff *tipc_log_resize_cmd(const void *req_tlv_area, int req_tlv_space)
{
	u32 value;

	if (!TLV_CHECK(req_tlv_area, req_tlv_space, TIPC_TLV_UNSIGNED))
		return tipc_cfg_reply_error_string(TIPC_CFG_TLV_ERROR);

	value = ntohl(*(__be32 *)TLV_DATA(req_tlv_area));
	if (value != delimit(value, 0, 32768))
		return tipc_cfg_reply_error_string(TIPC_CFG_INVALID_VALUE
						   " (log size must be 0-32768)");
	if (tipc_log_resize(value))
		return tipc_cfg_reply_error_string(
			"unable to create specified log (log size is now 0)");
	return tipc_cfg_reply_none();
}


struct sk_buff *tipc_log_dump(void)
{
	struct sk_buff *reply;

	spin_lock_bh(&print_lock);
	if (!TIPC_LOG->buf) {
		spin_unlock_bh(&print_lock);
		reply = tipc_cfg_reply_ultra_string("log not activated\n");
	} else if (tipc_printbuf_empty(TIPC_LOG)) {
		spin_unlock_bh(&print_lock);
		reply = tipc_cfg_reply_ultra_string("log is empty\n");
	}
	else {
		struct tlv_desc *rep_tlv;
		struct print_buf pb;
		int str_len;

		str_len = min(TIPC_LOG->size, 32768u);
		spin_unlock_bh(&print_lock);
		reply = tipc_cfg_reply_alloc(TLV_SPACE(str_len));
		if (reply) {
			rep_tlv = (struct tlv_desc *)reply->data;
			tipc_printbuf_init(&pb, TLV_DATA(rep_tlv), str_len);
			spin_lock_bh(&print_lock);
			tipc_printbuf_move(&pb, TIPC_LOG);
			spin_unlock_bh(&print_lock);
			str_len = strlen(TLV_DATA(rep_tlv)) + 1;
			skb_put(reply, TLV_SPACE(str_len));
			TLV_SET(rep_tlv, TIPC_TLV_ULTRA_STRING, NULL, str_len);
		}
	}
	return reply;
}

