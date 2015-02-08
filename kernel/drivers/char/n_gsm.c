

#include <linux/types.h>
#include <linux/major.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/fcntl.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/timer.h>
#include <linux/ctype.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/bitops.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/tty_flip.h>
#include <linux/tty_driver.h>
#include <linux/serial.h>
#include <linux/kfifo.h>
#include <linux/skbuff.h>
#include <linux/gsmmux.h>

static int debug;
module_param(debug, int, 0600);

#define T1	(HZ/10)
#define T2	(HZ/3)
#define N2	3

/* Use long timers for testing at low speed with debug on */
#ifdef DEBUG_TIMING
#define T1	HZ
#define T2	(2 * HZ)
#endif

#define MAX_MRU 512
#define MAX_MTU 512


struct gsm_msg {
	struct gsm_msg *next;
	u8 addr;		/* DLCI address + flags */
	u8 ctrl;		/* Control byte + flags */
	unsigned int len;	/* Length of data block (can be zero) */
	unsigned char *data;	/* Points into buffer but not at the start */
	unsigned char buffer[0];
};


struct gsm_dlci {
	struct gsm_mux *gsm;
	int addr;
	int state;
#define DLCI_CLOSED		0
#define DLCI_OPENING		1	/* Sending SABM not seen UA */
#define DLCI_OPEN		2	/* SABM/UA complete */
#define DLCI_CLOSING		3	/* Sending DISC not seen UA/DM */

	/* Link layer */
	spinlock_t lock;	/* Protects the internal state */
	struct timer_list t1;	/* Retransmit timer for SABM and UA */
	int retries;
	/* Uplink tty if active */
	struct tty_port port;	/* The tty bound to this DLCI if there is one */
	struct kfifo *fifo;	/* Queue fifo for the DLCI */
	struct kfifo _fifo;	/* For new fifo API porting only */
	int adaption;		/* Adaption layer in use */
	u32 modem_rx;		/* Our incoming virtual modem lines */
	u32 modem_tx;		/* Our outgoing modem lines */
	int dead;		/* Refuse re-open */
	/* Flow control */
	int throttled;		/* Private copy of throttle state */
	int constipated;	/* Throttle status for outgoing */
	/* Packetised I/O */
	struct sk_buff *skb;	/* Frame being sent */
	struct sk_buff_head skb_list;	/* Queued frames */
	/* Data handling callback */
	void (*data)(struct gsm_dlci *dlci, u8 *data, int len);
};

/* DLCI 0, 62/63 are special or reseved see gsmtty_open */

#define NUM_DLCI		64


struct gsm_control {
	u8 cmd;		/* Command we are issuing */
	u8 *data;	/* Data for the command in case we retransmit */
	int len;	/* Length of block for retransmission */
	int done;	/* Done flag */
	int error;	/* Error if any */
};


struct gsm_mux {
	struct tty_struct *tty;		/* The tty our ldisc is bound to */
	spinlock_t lock;

	/* Events on the GSM channel */
	wait_queue_head_t event;

	/* Bits for GSM mode decoding */

	/* Framing Layer */
	unsigned char *buf;
	int state;
#define GSM_SEARCH		0
#define GSM_START		1
#define GSM_ADDRESS		2
#define GSM_CONTROL		3
#define GSM_LEN			4
#define GSM_DATA		5
#define GSM_FCS			6
#define GSM_OVERRUN		7
	unsigned int len;
	unsigned int address;
	unsigned int count;
	int escape;
	int encoding;
	u8 control;
	u8 fcs;
	u8 *txframe;			/* TX framing buffer */

	/* Methods for the receiver side */
	void (*receive)(struct gsm_mux *gsm, u8 ch);
	void (*error)(struct gsm_mux *gsm, u8 ch, u8 flag);
	/* And transmit side */
	int (*output)(struct gsm_mux *mux, u8 *data, int len);

	/* Link Layer */
	unsigned int mru;
	unsigned int mtu;
	int initiator;			/* Did we initiate connection */
	int dead;			/* Has the mux been shut down */
	struct gsm_dlci *dlci[NUM_DLCI];
	int constipated;		/* Asked by remote to shut up */

	spinlock_t tx_lock;
	unsigned int tx_bytes;		/* TX data outstanding */
#define TX_THRESH_HI		8192
#define TX_THRESH_LO		2048
	struct gsm_msg *tx_head;	/* Pending data packets */
	struct gsm_msg *tx_tail;

	/* Control messages */
	struct timer_list t2_timer;	/* Retransmit timer for commands */
	int cretries;			/* Command retry counter */
	struct gsm_control *pending_cmd;/* Our current pending command */
	spinlock_t control_lock;	/* Protects the pending command */

	/* Configuration */
	int adaption;		/* 1 or 2 supported */
	u8 ftype;		/* UI or UIH */
	int t1, t2;		/* Timers in 1/100th of a sec */
	int n2;			/* Retry count */

	/* Statistics (not currently exposed) */
	unsigned long bad_fcs;
	unsigned long malformed;
	unsigned long io_error;
	unsigned long bad_size;
	unsigned long unsupported;
};



#define MAX_MUX		4			/* 256 minors */
static struct gsm_mux *gsm_mux[MAX_MUX];	/* GSM muxes */
static spinlock_t gsm_mux_lock;


#define CR			0x02
#define EA			0x01
#define	PF			0x10

/* I is special: the rest are ..*/
#define RR			0x01
#define UI			0x03
#define RNR			0x05
#define REJ			0x09
#define DM			0x0F
#define SABM			0x2F
#define DISC			0x43
#define UA			0x63
#define	UIH			0xEF

/* Channel commands */
#define CMD_NSC			0x09
#define CMD_TEST		0x11
#define CMD_PSC			0x21
#define CMD_RLS			0x29
#define CMD_FCOFF		0x31
#define CMD_PN			0x41
#define CMD_RPN			0x49
#define CMD_FCON		0x51
#define CMD_CLD			0x61
#define CMD_SNC			0x69
#define CMD_MSC			0x71

/* Virtual modem bits */
#define MDM_FC			0x01
#define MDM_RTC			0x02
#define MDM_RTR			0x04
#define MDM_IC			0x20
#define MDM_DV			0x40

#define GSM0_SOF		0xF9
#define GSM1_SOF 		0x7E
#define GSM1_ESCAPE		0x7D
#define GSM1_ESCAPE_BITS	0x20
#define XON			0x11
#define XOFF			0x13

static const struct tty_port_operations gsm_port_ops;


static const u8 gsm_fcs8[256] = {
	0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75,
	0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B,
	0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69,
	0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67,
	0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE, 0xDC, 0x4D,
	0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43,
	0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51,
	0x2A, 0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F,
	0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05,
	0x7E, 0xEF, 0x9D, 0x0C, 0x79, 0xE8, 0x9A, 0x0B,
	0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19,
	0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17,
	0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D,
	0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33,
	0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21,
	0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F,
	0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04, 0x95,
	0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B,
	0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89,
	0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87,
	0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD,
	0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3,
	0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1,
	0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF,
	0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5,
	0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB,
	0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9,
	0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7,
	0xA8, 0x39, 0x4B, 0xDA, 0xAF, 0x3E, 0x4C, 0xDD,
	0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3,
	0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1,
	0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF
};

#define INIT_FCS	0xFF
#define GOOD_FCS	0xCF


static inline u8 gsm_fcs_add(u8 fcs, u8 c)
{
	return gsm_fcs8[fcs ^ c];
}


static inline u8 gsm_fcs_add_block(u8 fcs, u8 *c, int len)
{
	while (len--)
		fcs = gsm_fcs8[fcs ^ *c++];
	return fcs;
}


static int gsm_read_ea(unsigned int *val, u8 c)
{
	/* Add the next 7 bits into the value */
	*val <<= 7;
	*val |= c >> 1;
	/* Was this the last byte of the EA 1 = yes*/
	return c & EA;
}


static u8 gsm_encode_modem(const struct gsm_dlci *dlci)
{
	u8 modembits = 0;
	/* FC is true flow control not modem bits */
	if (dlci->throttled)
		modembits |= MDM_FC;
	if (dlci->modem_tx & TIOCM_DTR)
		modembits |= MDM_RTC;
	if (dlci->modem_tx & TIOCM_RTS)
		modembits |= MDM_RTR;
	if (dlci->modem_tx & TIOCM_RI)
		modembits |= MDM_IC;
	if (dlci->modem_tx & TIOCM_CD)
		modembits |= MDM_DV;
	return modembits;
}


static void gsm_print_packet(const char *hdr, int addr, int cr,
					u8 control, const u8 *data, int dlen)
{
	if (!(debug & 1))
		return;

	printk(KERN_INFO "%s %d) %c: ", hdr, addr, "RC"[cr]);

	switch (control & ~PF) {
	case SABM:
		printk(KERN_CONT "SABM");
		break;
	case UA:
		printk(KERN_CONT "UA");
		break;
	case DISC:
		printk(KERN_CONT "DISC");
		break;
	case DM:
		printk(KERN_CONT "DM");
		break;
	case UI:
		printk(KERN_CONT "UI");
		break;
	case UIH:
		printk(KERN_CONT "UIH");
		break;
	default:
		if (!(control & 0x01)) {
			printk(KERN_CONT "I N(S)%d N(R)%d",
				(control & 0x0E) >> 1, (control & 0xE)>> 5);
		} else switch (control & 0x0F) {
		case RR:
			printk("RR(%d)", (control & 0xE0) >> 5);
			break;
		case RNR:
			printk("RNR(%d)", (control & 0xE0) >> 5);
			break;
		case REJ:
			printk("REJ(%d)", (control & 0xE0) >> 5);
			break;
		default:
			printk(KERN_CONT "[%02X]", control);
		}
	}

	if (control & PF)
		printk(KERN_CONT "(P)");
	else
		printk(KERN_CONT "(F)");

	if (dlen) {
		int ct = 0;
		while (dlen--) {
			if (ct % 8 == 0)
				printk(KERN_CONT "\n    ");
			printk(KERN_CONT "%02X ", *data++);
			ct++;
		}
	}
	printk(KERN_CONT "\n");
}




static int gsm_stuff_frame(const u8 *input, u8 *output, int len)
{
	int olen = 0;
	while (len--) {
		if (*input == GSM1_SOF || *input == GSM1_ESCAPE
		    || *input == XON || *input == XOFF) {
			*output++ = GSM1_ESCAPE;
			*output++ = *input++ ^ GSM1_ESCAPE_BITS;
			olen++;
		} else
			*output++ = *input++;
		olen++;
	}
	return olen;
}

static void hex_packet(const unsigned char *p, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		if (i && (i % 16) == 0)
			printk("\n");
		printk("%02X ", *p++);
	}
	printk("\n");
}


static void gsm_send(struct gsm_mux *gsm, int addr, int cr, int control)
{
	int len;
	u8 cbuf[10];
	u8 ibuf[3];

	switch (gsm->encoding) {
	case 0:
		cbuf[0] = GSM0_SOF;
		cbuf[1] = (addr << 2) | (cr << 1) | EA;
		cbuf[2] = control;
		cbuf[3] = EA;	/* Length of data = 0 */
		cbuf[4] = 0xFF - gsm_fcs_add_block(INIT_FCS, cbuf + 1, 3);
		cbuf[5] = GSM0_SOF;
		len = 6;
		break;
	case 1:
	case 2:
		/* Control frame + packing (but not frame stuffing) in mode 1 */
		ibuf[0] = (addr << 2) | (cr << 1) | EA;
		ibuf[1] = control;
		ibuf[2] = 0xFF - gsm_fcs_add_block(INIT_FCS, ibuf, 2);
		/* Stuffing may double the size worst case */
		len = gsm_stuff_frame(ibuf, cbuf + 1, 3);
		/* Now add the SOF markers */
		cbuf[0] = GSM1_SOF;
		cbuf[len + 1] = GSM1_SOF;
		/* FIXME: we can omit the lead one in many cases */
		len += 2;
		break;
	default:
		WARN_ON(1);
		return;
	}
	gsm->output(gsm, cbuf, len);
	gsm_print_packet("-->", addr, cr, control, NULL, 0);
}


static inline void gsm_response(struct gsm_mux *gsm, int addr, int control)
{
	gsm_send(gsm, addr, 0, control);
}


static inline void gsm_command(struct gsm_mux *gsm, int addr, int control)
{
	gsm_send(gsm, addr, 1, control);
}

/* Data transmission */

#define HDR_LEN		6	/* ADDR CTRL [LEN.2] DATA FCS */


static struct gsm_msg *gsm_data_alloc(struct gsm_mux *gsm, u8 addr, int len,
								u8 ctrl)
{
	struct gsm_msg *m = kmalloc(sizeof(struct gsm_msg) + len + HDR_LEN,
								GFP_ATOMIC);
	if (m == NULL)
		return NULL;
	m->data = m->buffer + HDR_LEN - 1;	/* Allow for FCS */
	m->len = len;
	m->addr = addr;
	m->ctrl = ctrl;
	m->next = NULL;
	return m;
}


static void gsm_data_kick(struct gsm_mux *gsm)
{
	struct gsm_msg *msg = gsm->tx_head;
	int len;
	int skip_sof = 0;

	/* FIXME: We need to apply this solely to data messages */
	if (gsm->constipated)
		return;

	while (gsm->tx_head != NULL) {
		msg = gsm->tx_head;
		if (gsm->encoding != 0) {
			gsm->txframe[0] = GSM1_SOF;
			len = gsm_stuff_frame(msg->data,
						gsm->txframe + 1, msg->len);
			gsm->txframe[len + 1] = GSM1_SOF;
			len += 2;
		} else {
			gsm->txframe[0] = GSM0_SOF;
			memcpy(gsm->txframe + 1 , msg->data, msg->len);
			gsm->txframe[msg->len + 1] = GSM0_SOF;
			len = msg->len + 2;
		}

		if (debug & 4) {
			printk("gsm_data_kick: \n");
			hex_packet(gsm->txframe, len);
		}

		if (gsm->output(gsm, gsm->txframe + skip_sof,
						len - skip_sof) < 0)
			break;
		/* FIXME: Can eliminate one SOF in many more cases */
		gsm->tx_head = msg->next;
		if (gsm->tx_head == NULL)
			gsm->tx_tail = NULL;
		gsm->tx_bytes -= msg->len;
		kfree(msg);
		/* For a burst of frames skip the extra SOF within the
		   burst */
		skip_sof = 1;
	}
}


static void __gsm_data_queue(struct gsm_dlci *dlci, struct gsm_msg *msg)
{
	struct gsm_mux *gsm = dlci->gsm;
	u8 *dp = msg->data;
	u8 *fcs = dp + msg->len;

	/* Fill in the header */
	if (gsm->encoding == 0) {
		if (msg->len < 128)
			*--dp = (msg->len << 1) | EA;
		else {
			*--dp = (msg->len >> 6) | EA;
			*--dp = (msg->len & 127) << 1;
		}
	}

	*--dp = msg->ctrl;
	if (gsm->initiator)
		*--dp = (msg->addr << 2) | 2 | EA;
	else
		*--dp = (msg->addr << 2) | EA;
	*fcs = gsm_fcs_add_block(INIT_FCS, dp , msg->data - dp);
	/* Ugly protocol layering violation */
	if (msg->ctrl == UI || msg->ctrl == (UI|PF))
		*fcs = gsm_fcs_add_block(*fcs, msg->data, msg->len);
	*fcs = 0xFF - *fcs;

	gsm_print_packet("Q> ", msg->addr, gsm->initiator, msg->ctrl,
							msg->data, msg->len);

	/* Move the header back and adjust the length, also allow for the FCS
	   now tacked on the end */
	msg->len += (msg->data - dp) + 1;
	msg->data = dp;

	/* Add to the actual output queue */
	if (gsm->tx_tail)
		gsm->tx_tail->next = msg;
	else
		gsm->tx_head = msg;
	gsm->tx_tail = msg;
	gsm->tx_bytes += msg->len;
	gsm_data_kick(gsm);
}


static void gsm_data_queue(struct gsm_dlci *dlci, struct gsm_msg *msg)
{
	unsigned long flags;
	spin_lock_irqsave(&dlci->gsm->tx_lock, flags);
	__gsm_data_queue(dlci, msg);
	spin_unlock_irqrestore(&dlci->gsm->tx_lock, flags);
}


static int gsm_dlci_data_output(struct gsm_mux *gsm, struct gsm_dlci *dlci)
{
	struct gsm_msg *msg;
	u8 *dp;
	int len, size;
	int h = dlci->adaption - 1;

	len = kfifo_len(dlci->fifo);
	if (len == 0)
		return 0;

	/* MTU/MRU count only the data bits */
	if (len > gsm->mtu)
		len = gsm->mtu;

	size = len + h;

	msg = gsm_data_alloc(gsm, dlci->addr, size, gsm->ftype);
	/* FIXME: need a timer or something to kick this so it can't
	   get stuck with no work outstanding and no buffer free */
	if (msg == NULL)
		return -ENOMEM;
	dp = msg->data;
	switch (dlci->adaption) {
	case 1:	/* Unstructured */
		break;
	case 2:	/* Unstructed with modem bits. Always one byte as we never
		   send inline break data */
		*dp += gsm_encode_modem(dlci);
		len--;
		break;
	}
	WARN_ON(kfifo_out_locked(dlci->fifo, dp , len, &dlci->lock) != len);
	__gsm_data_queue(dlci, msg);
	/* Bytes of data we used up */
	return size;
}


static int gsm_dlci_data_output_framed(struct gsm_mux *gsm,
						struct gsm_dlci *dlci)
{
	struct gsm_msg *msg;
	u8 *dp;
	int len, size;
	int last = 0, first = 0;
	int overhead = 0;

	/* One byte per frame is used for B/F flags */
	if (dlci->adaption == 4)
		overhead = 1;

	/* dlci->skb is locked by tx_lock */
	if (dlci->skb == NULL) {
		dlci->skb = skb_dequeue(&dlci->skb_list);
		if (dlci->skb == NULL)
			return 0;
		first = 1;
	}
	len = dlci->skb->len + overhead;

	/* MTU/MRU count only the data bits */
	if (len > gsm->mtu) {
		if (dlci->adaption == 3) {
			/* Over long frame, bin it */
			kfree_skb(dlci->skb);
			dlci->skb = NULL;
			return 0;
		}
		len = gsm->mtu;
	} else
		last = 1;

	size = len + overhead;
	msg = gsm_data_alloc(gsm, dlci->addr, size, gsm->ftype);

	/* FIXME: need a timer or something to kick this so it can't
	   get stuck with no work outstanding and no buffer free */
	if (msg == NULL)
		return -ENOMEM;
	dp = msg->data;

	if (dlci->adaption == 4) { /* Interruptible framed (Packetised Data) */
		/* Flag byte to carry the start/end info */
		*dp++ = last << 7 | first << 6 | 1;	/* EA */
		len--;
	}
	memcpy(dp, skb_pull(dlci->skb, len), len);
	__gsm_data_queue(dlci, msg);
	if (last)
		dlci->skb = NULL;
	return size;
}


static void gsm_dlci_data_sweep(struct gsm_mux *gsm)
{
	int len;
	/* Priority ordering: We should do priority with RR of the groups */
	int i = 1;

	while (i < NUM_DLCI) {
		struct gsm_dlci *dlci;

		if (gsm->tx_bytes > TX_THRESH_HI)
			break;
		dlci = gsm->dlci[i];
		if (dlci == NULL || dlci->constipated) {
			i++;
			continue;
		}
		if (dlci->adaption < 3)
			len = gsm_dlci_data_output(gsm, dlci);
		else
			len = gsm_dlci_data_output_framed(gsm, dlci);
		if (len < 0)
			return;
		/* DLCI empty - try the next */
		if (len == 0)
			i++;
	}
}


static void gsm_dlci_data_kick(struct gsm_dlci *dlci)
{
	unsigned long flags;

	spin_lock_irqsave(&dlci->gsm->tx_lock, flags);
	/* If we have nothing running then we need to fire up */
	if (dlci->gsm->tx_bytes == 0)
		gsm_dlci_data_output(dlci->gsm, dlci);
	else if (dlci->gsm->tx_bytes < TX_THRESH_LO)
		gsm_dlci_data_sweep(dlci->gsm);
	spin_unlock_irqrestore(&dlci->gsm->tx_lock, flags);
}




static void gsm_control_reply(struct gsm_mux *gsm, int cmd, u8 *data,
					int dlen)
{
	struct gsm_msg *msg;
	msg = gsm_data_alloc(gsm, 0, dlen + 2, gsm->ftype);
	msg->data[0] = (cmd & 0xFE) << 1 | EA;	/* Clear C/R */
	msg->data[1] = (dlen << 1) | EA;
	memcpy(msg->data + 2, data, dlen);
	gsm_data_queue(gsm->dlci[0], msg);
}


static void gsm_process_modem(struct tty_struct *tty, struct gsm_dlci *dlci,
							u32 modem)
{
	int  mlines = 0;
	u8 brk = modem >> 6;

	/* Flow control/ready to communicate */
	if (modem & MDM_FC) {
		/* Need to throttle our output on this device */
		dlci->constipated = 1;
	}
	if (modem & MDM_RTC) {
		mlines |= TIOCM_DSR | TIOCM_DTR;
		dlci->constipated = 0;
		gsm_dlci_data_kick(dlci);
	}
	/* Map modem bits */
	if (modem & MDM_RTR)
		mlines |= TIOCM_RTS | TIOCM_CTS;
	if (modem & MDM_IC)
		mlines |= TIOCM_RI;
	if (modem & MDM_DV)
		mlines |= TIOCM_CD;

	/* Carrier drop -> hangup */
	if (tty) {
		if ((mlines & TIOCM_CD) == 0 && (dlci->modem_rx & TIOCM_CD))
			if (!(tty->termios->c_cflag & CLOCAL))
				tty_hangup(tty);
		if (brk & 0x01)
			tty_insert_flip_char(tty, 0, TTY_BREAK);
	}
	dlci->modem_rx = mlines;
}


static void gsm_control_modem(struct gsm_mux *gsm, u8 *data, int clen)
{
	unsigned int addr = 0;
	unsigned int modem = 0;
	struct gsm_dlci *dlci;
	int len = clen;
	u8 *dp = data;
	struct tty_struct *tty;

	while (gsm_read_ea(&addr, *dp++) == 0) {
		len--;
		if (len == 0)
			return;
	}
	/* Must be at least one byte following the EA */
	len--;
	if (len <= 0)
		return;

	addr >>= 1;
	/* Closed port, or invalid ? */
	if (addr == 0 || addr >= NUM_DLCI || gsm->dlci[addr] == NULL)
		return;
	dlci = gsm->dlci[addr];

	while (gsm_read_ea(&modem, *dp++) == 0) {
		len--;
		if (len == 0)
			return;
	}
	tty = tty_port_tty_get(&dlci->port);
	gsm_process_modem(tty, dlci, modem);
	if (tty) {
		tty_wakeup(tty);
		tty_kref_put(tty);
	}
	gsm_control_reply(gsm, CMD_MSC, data, clen);
}


static void gsm_control_rls(struct gsm_mux *gsm, u8 *data, int clen)
{
	struct tty_struct *tty;
	unsigned int addr = 0 ;
	u8 bits;
	int len = clen;
	u8 *dp = data;

	while (gsm_read_ea(&addr, *dp++) == 0) {
		len--;
		if (len == 0)
			return;
	}
	/* Must be at least one byte following ea */
	len--;
	if (len <= 0)
		return;
	addr >>= 1;
	/* Closed port, or invalid ? */
	if (addr == 0 || addr >= NUM_DLCI || gsm->dlci[addr] == NULL)
		return;
	/* No error ? */
	bits = *dp;
	if ((bits & 1) == 0)
		return;
	/* See if we have an uplink tty */
	tty = tty_port_tty_get(&gsm->dlci[addr]->port);

	if (tty) {
		if (bits & 2)
			tty_insert_flip_char(tty, 0, TTY_OVERRUN);
		if (bits & 4)
			tty_insert_flip_char(tty, 0, TTY_PARITY);
		if (bits & 8)
			tty_insert_flip_char(tty, 0, TTY_FRAME);
		tty_flip_buffer_push(tty);
		tty_kref_put(tty);
	}
	gsm_control_reply(gsm, CMD_RLS, data, clen);
}

static void gsm_dlci_begin_close(struct gsm_dlci *dlci);


static void gsm_control_message(struct gsm_mux *gsm, unsigned int command,
							u8 *data, int clen)
{
	u8 buf[1];
	switch (command) {
	case CMD_CLD: {
		struct gsm_dlci *dlci = gsm->dlci[0];
		/* Modem wishes to close down */
		if (dlci) {
			dlci->dead = 1;
			gsm->dead = 1;
			gsm_dlci_begin_close(dlci);
		}
		}
		break;
	case CMD_TEST:
		/* Modem wishes to test, reply with the data */
		gsm_control_reply(gsm, CMD_TEST, data, clen);
		break;
	case CMD_FCON:
		/* Modem wants us to STFU */
		gsm->constipated = 1;
		gsm_control_reply(gsm, CMD_FCON, NULL, 0);
		break;
	case CMD_FCOFF:
		/* Modem can accept data again */
		gsm->constipated = 0;
		gsm_control_reply(gsm, CMD_FCOFF, NULL, 0);
		/* Kick the link in case it is idling */
		gsm_data_kick(gsm);
		break;
	case CMD_MSC:
		/* Out of band modem line change indicator for a DLCI */
		gsm_control_modem(gsm, data, clen);
		break;
	case CMD_RLS:
		/* Out of band error reception for a DLCI */
		gsm_control_rls(gsm, data, clen);
		break;
	case CMD_PSC:
		/* Modem wishes to enter power saving state */
		gsm_control_reply(gsm, CMD_PSC, NULL, 0);
		break;
		/* Optional unsupported commands */
	case CMD_PN:	/* Parameter negotiation */
	case CMD_RPN:	/* Remote port negotation */
	case CMD_SNC:	/* Service negotation command */
	default:
		/* Reply to bad commands with an NSC */
		buf[0] = command;
		gsm_control_reply(gsm, CMD_NSC, buf, 1);
		break;
	}
}


static void gsm_control_response(struct gsm_mux *gsm, unsigned int command,
							u8 *data, int clen)
{
	struct gsm_control *ctrl;
	unsigned long flags;

	spin_lock_irqsave(&gsm->control_lock, flags);

	ctrl = gsm->pending_cmd;
	/* Does the reply match our command */
	command |= 1;
	if (ctrl != NULL && (command == ctrl->cmd || command == CMD_NSC)) {
		/* Our command was replied to, kill the retry timer */
		del_timer(&gsm->t2_timer);
		gsm->pending_cmd = NULL;
		/* Rejected by the other end */
		if (command == CMD_NSC)
			ctrl->error = -EOPNOTSUPP;
		ctrl->done = 1;
		wake_up(&gsm->event);
	}
	spin_unlock_irqrestore(&gsm->control_lock, flags);
}


static void gsm_control_transmit(struct gsm_mux *gsm, struct gsm_control *ctrl)
{
	struct gsm_msg *msg = gsm_data_alloc(gsm, 0, ctrl->len + 1,
							gsm->ftype|PF);
	if (msg == NULL)
		return;
	msg->data[0] = (ctrl->cmd << 1) | 2 | EA;	/* command */
	memcpy(msg->data + 1, ctrl->data, ctrl->len);
	gsm_data_queue(gsm->dlci[0], msg);
}


static void gsm_control_retransmit(unsigned long data)
{
	struct gsm_mux *gsm = (struct gsm_mux *)data;
	struct gsm_control *ctrl;
	unsigned long flags;
	spin_lock_irqsave(&gsm->control_lock, flags);
	ctrl = gsm->pending_cmd;
	if (ctrl) {
		gsm->cretries--;
		if (gsm->cretries == 0) {
			gsm->pending_cmd = NULL;
			ctrl->error = -ETIMEDOUT;
			ctrl->done = 1;
			spin_unlock_irqrestore(&gsm->control_lock, flags);
			wake_up(&gsm->event);
			return;
		}
		gsm_control_transmit(gsm, ctrl);
		mod_timer(&gsm->t2_timer, jiffies + gsm->t2 * HZ / 100);
	}
	spin_unlock_irqrestore(&gsm->control_lock, flags);
}


static struct gsm_control *gsm_control_send(struct gsm_mux *gsm,
		unsigned int command, u8 *data, int clen)
{
	struct gsm_control *ctrl = kzalloc(sizeof(struct gsm_control),
						GFP_KERNEL);
	unsigned long flags;
	if (ctrl == NULL)
		return NULL;
retry:
	wait_event(gsm->event, gsm->pending_cmd == NULL);
	spin_lock_irqsave(&gsm->control_lock, flags);
	if (gsm->pending_cmd != NULL) {
		spin_unlock_irqrestore(&gsm->control_lock, flags);
		goto retry;
	}
	ctrl->cmd = command;
	ctrl->data = data;
	ctrl->len = clen;
	gsm->pending_cmd = ctrl;
	gsm->cretries = gsm->n2;
	mod_timer(&gsm->t2_timer, jiffies + gsm->t2 * HZ / 100);
	gsm_control_transmit(gsm, ctrl);
	spin_unlock_irqrestore(&gsm->control_lock, flags);
	return ctrl;
}


static int gsm_control_wait(struct gsm_mux *gsm, struct gsm_control *control)
{
	int err;
	wait_event(gsm->event, control->done == 1);
	err = control->error;
	kfree(control);
	return err;
}





static void gsm_dlci_close(struct gsm_dlci *dlci)
{
	del_timer(&dlci->t1);
	if (debug & 8)
		printk("DLCI %d goes closed.\n", dlci->addr);
	dlci->state = DLCI_CLOSED;
	if (dlci->addr != 0) {
		struct tty_struct  *tty = tty_port_tty_get(&dlci->port);
		if (tty) {
			tty_hangup(tty);
			tty_kref_put(tty);
		}
		kfifo_reset(dlci->fifo);
	} else
		dlci->gsm->dead = 1;
	wake_up(&dlci->gsm->event);
	/* A DLCI 0 close is a MUX termination so we need to kick that
	   back to userspace somehow */
}


static void gsm_dlci_open(struct gsm_dlci *dlci)
{
	/* Note that SABM UA .. SABM UA first UA lost can mean that we go
	   open -> open */
	del_timer(&dlci->t1);
	/* This will let a tty open continue */
	dlci->state = DLCI_OPEN;
	if (debug & 8)
		printk("DLCI %d goes open.\n", dlci->addr);
	wake_up(&dlci->gsm->event);
}


static void gsm_dlci_t1(unsigned long data)
{
	struct gsm_dlci *dlci = (struct gsm_dlci *)data;
	struct gsm_mux *gsm = dlci->gsm;

	switch (dlci->state) {
	case DLCI_OPENING:
		dlci->retries--;
		if (dlci->retries) {
			gsm_command(dlci->gsm, dlci->addr, SABM|PF);
			mod_timer(&dlci->t1, jiffies + gsm->t1 * HZ / 100);
		} else
			gsm_dlci_close(dlci);
		break;
	case DLCI_CLOSING:
		dlci->retries--;
		if (dlci->retries) {
			gsm_command(dlci->gsm, dlci->addr, DISC|PF);
			mod_timer(&dlci->t1, jiffies + gsm->t1 * HZ / 100);
		} else
			gsm_dlci_close(dlci);
		break;
	}
}


static void gsm_dlci_begin_open(struct gsm_dlci *dlci)
{
	struct gsm_mux *gsm = dlci->gsm;
	if (dlci->state == DLCI_OPEN || dlci->state == DLCI_OPENING)
		return;
	dlci->retries = gsm->n2;
	dlci->state = DLCI_OPENING;
	gsm_command(dlci->gsm, dlci->addr, SABM|PF);
	mod_timer(&dlci->t1, jiffies + gsm->t1 * HZ / 100);
}


static void gsm_dlci_begin_close(struct gsm_dlci *dlci)
{
	struct gsm_mux *gsm = dlci->gsm;
	if (dlci->state == DLCI_CLOSED || dlci->state == DLCI_CLOSING)
		return;
	dlci->retries = gsm->n2;
	dlci->state = DLCI_CLOSING;
	gsm_command(dlci->gsm, dlci->addr, DISC|PF);
	mod_timer(&dlci->t1, jiffies + gsm->t1 * HZ / 100);
}


static void gsm_dlci_data(struct gsm_dlci *dlci, u8 *data, int len)
{
	/* krefs .. */
	struct tty_port *port = &dlci->port;
	struct tty_struct *tty = tty_port_tty_get(port);
	unsigned int modem = 0;

	if (debug & 16)
		printk("%d bytes for tty %p\n", len, tty);
	if (tty) {
		switch (dlci->adaption)  {
			/* Unsupported types */
			/* Packetised interruptible data */
			case 4:
				break;
			/* Packetised uininterruptible voice/data */
			case 3:
				break;
			/* Asynchronous serial with line state in each frame */
			case 2:
				while (gsm_read_ea(&modem, *data++) == 0) {
					len--;
					if (len == 0)
						return;
				}
				gsm_process_modem(tty, dlci, modem);
			/* Line state will go via DLCI 0 controls only */
			case 1:
			default:
				tty_insert_flip_string(tty, data, len);
				tty_flip_buffer_push(tty);
		}
		tty_kref_put(tty);
	}
}


static void gsm_dlci_command(struct gsm_dlci *dlci, u8 *data, int len)
{
	/* See what command is involved */
	unsigned int command = 0;
	while (len-- > 0) {
		if (gsm_read_ea(&command, *data++) == 1) {
			int clen = *data++;
			len--;
			/* FIXME: this is properly an EA */
			clen >>= 1;
			/* Malformed command ? */
			if (clen > len)
				return;
			if (command & 1)
				gsm_control_message(dlci->gsm, command,
								data, clen);
			else
				gsm_control_response(dlci->gsm, command,
								data, clen);
			return;
		}
	}
}



static struct gsm_dlci *gsm_dlci_alloc(struct gsm_mux *gsm, int addr)
{
	struct gsm_dlci *dlci = kzalloc(sizeof(struct gsm_dlci), GFP_ATOMIC);
	if (dlci == NULL)
		return NULL;
	spin_lock_init(&dlci->lock);
	dlci->fifo = &dlci->_fifo;
	if (kfifo_alloc(&dlci->_fifo, 4096, GFP_KERNEL) < 0) {
		kfree(dlci);
		return NULL;
	}

	skb_queue_head_init(&dlci->skb_list);
	init_timer(&dlci->t1);
	dlci->t1.function = gsm_dlci_t1;
	dlci->t1.data = (unsigned long)dlci;
	tty_port_init(&dlci->port);
	dlci->port.ops = &gsm_port_ops;
	dlci->gsm = gsm;
	dlci->addr = addr;
	dlci->adaption = gsm->adaption;
	dlci->state = DLCI_CLOSED;
	if (addr)
		dlci->data = gsm_dlci_data;
	else
		dlci->data = gsm_dlci_command;
	gsm->dlci[addr] = dlci;
	return dlci;
}

static void gsm_dlci_free(struct gsm_dlci *dlci)
{
	struct tty_struct *tty = tty_port_tty_get(&dlci->port);
	if (tty) {
		tty_vhangup(tty);
		tty_kref_put(tty);
	}
	del_timer_sync(&dlci->t1);
	dlci->gsm->dlci[dlci->addr] = NULL;
	kfifo_free(dlci->fifo);
	kfree(dlci);
}




static void gsm_queue(struct gsm_mux *gsm)
{
	struct gsm_dlci *dlci;
	u8 cr;
	int address;
	/* We have to sneak a look at the packet body to do the FCS.
	   A somewhat layering violation in the spec */

	if ((gsm->control & ~PF) == UI)
		gsm->fcs = gsm_fcs_add_block(gsm->fcs, gsm->buf, gsm->len);
	if (gsm->fcs != GOOD_FCS) {
		gsm->bad_fcs++;
		if (debug & 4)
			printk("BAD FCS %02x\n", gsm->fcs);
		return;
	}
	address = gsm->address >> 1;
	if (address >= NUM_DLCI)
		goto invalid;

	cr = gsm->address & 1;		/* C/R bit */

	gsm_print_packet("<--", address, cr, gsm->control, gsm->buf, gsm->len);

	cr ^= 1 - gsm->initiator;	/* Flip so 1 always means command */
	dlci = gsm->dlci[address];

	switch (gsm->control) {
	case SABM|PF:
		if (cr == 0)
			goto invalid;
		if (dlci == NULL)
			dlci = gsm_dlci_alloc(gsm, address);
		if (dlci == NULL)
			return;
		if (dlci->dead)
			gsm_response(gsm, address, DM);
		else {
			gsm_response(gsm, address, UA);
			gsm_dlci_open(dlci);
		}
		break;
	case DISC|PF:
		if (cr == 0)
			goto invalid;
		if (dlci == NULL || dlci->state == DLCI_CLOSED) {
			gsm_response(gsm, address, DM);
			return;
		}
		/* Real close complete */
		gsm_response(gsm, address, UA);
		gsm_dlci_close(dlci);
		break;
	case UA:
	case UA|PF:
		if (cr == 0 || dlci == NULL)
			break;
		switch (dlci->state) {
		case DLCI_CLOSING:
			gsm_dlci_close(dlci);
			break;
		case DLCI_OPENING:
			gsm_dlci_open(dlci);
			break;
		}
		break;
	case DM:	/* DM can be valid unsolicited */
	case DM|PF:
		if (cr)
			goto invalid;
		if (dlci == NULL)
			return;
		gsm_dlci_close(dlci);
		break;
	case UI:
	case UI|PF:
	case UIH:
	case UIH|PF:
#if 0
		if (cr)
			goto invalid;
#endif
		if (dlci == NULL || dlci->state != DLCI_OPEN) {
			gsm_command(gsm, address, DM|PF);
			return;
		}
		dlci->data(dlci, gsm->buf, gsm->len);
		break;
	default:
		goto invalid;
	}
	return;
invalid:
	gsm->malformed++;
	return;
}



static void gsm0_receive(struct gsm_mux *gsm, unsigned char c)
{
	switch (gsm->state) {
	case GSM_SEARCH:	/* SOF marker */
		if (c == GSM0_SOF) {
			gsm->state = GSM_ADDRESS;
			gsm->address = 0;
			gsm->len = 0;
			gsm->fcs = INIT_FCS;
		}
		break;		/* Address EA */
	case GSM_ADDRESS:
		gsm->fcs = gsm_fcs_add(gsm->fcs, c);
		if (gsm_read_ea(&gsm->address, c))
			gsm->state = GSM_CONTROL;
		break;
	case GSM_CONTROL:	/* Control Byte */
		gsm->fcs = gsm_fcs_add(gsm->fcs, c);
		gsm->control = c;
		gsm->state = GSM_LEN;
		break;
	case GSM_LEN:		/* Length EA */
		gsm->fcs = gsm_fcs_add(gsm->fcs, c);
		if (gsm_read_ea(&gsm->len, c)) {
			if (gsm->len > gsm->mru) {
				gsm->bad_size++;
				gsm->state = GSM_SEARCH;
				break;
			}
			gsm->count = 0;
			gsm->state = GSM_DATA;
		}
		break;
	case GSM_DATA:		/* Data */
		gsm->buf[gsm->count++] = c;
		if (gsm->count == gsm->len)
			gsm->state = GSM_FCS;
		break;
	case GSM_FCS:		/* FCS follows the packet */
		gsm->fcs = c;
		gsm_queue(gsm);
		/* And then back for the next frame */
		gsm->state = GSM_SEARCH;
		break;
	}
}


static void gsm1_receive(struct gsm_mux *gsm, unsigned char c)
{
	if (c == GSM1_SOF) {
		/* EOF is only valid in frame if we have got to the data state
		   and received at least one byte (the FCS) */
		if (gsm->state == GSM_DATA && gsm->count) {
			/* Extract the FCS */
			gsm->count--;
			gsm->fcs = gsm_fcs_add(gsm->fcs, gsm->buf[gsm->count]);
			gsm->len = gsm->count;
			gsm_queue(gsm);
			gsm->state  = GSM_START;
			return;
		}
		/* Any partial frame was a runt so go back to start */
		if (gsm->state != GSM_START) {
			gsm->malformed++;
			gsm->state = GSM_START;
		}
		/* A SOF in GSM_START means we are still reading idling or
		   framing bytes */
		return;
	}

	if (c == GSM1_ESCAPE) {
		gsm->escape = 1;
		return;
	}

	/* Only an unescaped SOF gets us out of GSM search */
	if (gsm->state == GSM_SEARCH)
		return;

	if (gsm->escape) {
		c ^= GSM1_ESCAPE_BITS;
		gsm->escape = 0;
	}
	switch (gsm->state) {
	case GSM_START:		/* First byte after SOF */
		gsm->address = 0;
		gsm->state = GSM_ADDRESS;
		gsm->fcs = INIT_FCS;
		/* Drop through */
	case GSM_ADDRESS:	/* Address continuation */
		gsm->fcs = gsm_fcs_add(gsm->fcs, c);
		if (gsm_read_ea(&gsm->address, c))
			gsm->state = GSM_CONTROL;
		break;
	case GSM_CONTROL:	/* Control Byte */
		gsm->fcs = gsm_fcs_add(gsm->fcs, c);
		gsm->control = c;
		gsm->count = 0;
		gsm->state = GSM_DATA;
		break;
	case GSM_DATA:		/* Data */
		if (gsm->count > gsm->mru ) {	/* Allow one for the FCS */
			gsm->state = GSM_OVERRUN;
			gsm->bad_size++;
		} else
			gsm->buf[gsm->count++] = c;
		break;
	case GSM_OVERRUN:	/* Over-long - eg a dropped SOF */
		break;
	}
}


static void gsm_error(struct gsm_mux *gsm,
				unsigned char data, unsigned char flag)
{
	gsm->state = GSM_SEARCH;
	gsm->io_error++;
}


void gsm_cleanup_mux(struct gsm_mux *gsm)
{
	int i;
	struct gsm_dlci *dlci = gsm->dlci[0];
	struct gsm_msg *txq;

	gsm->dead = 1;

	spin_lock(&gsm_mux_lock);
	for (i = 0; i < MAX_MUX; i++) {
		if (gsm_mux[i] == gsm) {
			gsm_mux[i] = NULL;
			break;
		}
	}
	spin_unlock(&gsm_mux_lock);
	WARN_ON(i == MAX_MUX);

	del_timer_sync(&gsm->t2_timer);
	/* Now we are sure T2 has stopped */
	if (dlci) {
		dlci->dead = 1;
		gsm_dlci_begin_close(dlci);
		wait_event_interruptible(gsm->event,
					dlci->state == DLCI_CLOSED);
	}
	/* Free up any link layer users */
	for (i = 0; i < NUM_DLCI; i++)
		if (gsm->dlci[i])
			gsm_dlci_free(gsm->dlci[i]);
	/* Now wipe the queues */
	for (txq = gsm->tx_head; txq != NULL; txq = gsm->tx_head) {
		gsm->tx_head = txq->next;
		kfree(txq);
	}
	gsm->tx_tail = NULL;
}
EXPORT_SYMBOL_GPL(gsm_cleanup_mux);


int gsm_activate_mux(struct gsm_mux *gsm)
{
	struct gsm_dlci *dlci;
	int i = 0;

	init_timer(&gsm->t2_timer);
	gsm->t2_timer.function = gsm_control_retransmit;
	gsm->t2_timer.data = (unsigned long)gsm;
	init_waitqueue_head(&gsm->event);
	spin_lock_init(&gsm->control_lock);
	spin_lock_init(&gsm->tx_lock);

	if (gsm->encoding == 0)
		gsm->receive = gsm0_receive;
	else
		gsm->receive = gsm1_receive;
	gsm->error = gsm_error;

	spin_lock(&gsm_mux_lock);
	for (i = 0; i < MAX_MUX; i++) {
		if (gsm_mux[i] == NULL) {
			gsm_mux[i] = gsm;
			break;
		}
	}
	spin_unlock(&gsm_mux_lock);
	if (i == MAX_MUX)
		return -EBUSY;

	dlci = gsm_dlci_alloc(gsm, 0);
	if (dlci == NULL)
		return -ENOMEM;
	gsm->dead = 0;		/* Tty opens are now permissible */
	return 0;
}
EXPORT_SYMBOL_GPL(gsm_activate_mux);

void gsm_free_mux(struct gsm_mux *gsm)
{
	kfree(gsm->txframe);
	kfree(gsm->buf);
	kfree(gsm);
}
EXPORT_SYMBOL_GPL(gsm_free_mux);


struct gsm_mux *gsm_alloc_mux(void)
{
	struct gsm_mux *gsm = kzalloc(sizeof(struct gsm_mux), GFP_KERNEL);
	if (gsm == NULL)
		return NULL;
	gsm->buf = kmalloc(MAX_MRU + 1, GFP_KERNEL);
	if (gsm->buf == NULL) {
		kfree(gsm);
		return NULL;
	}
	gsm->txframe = kmalloc(2 * MAX_MRU + 2, GFP_KERNEL);
	if (gsm->txframe == NULL) {
		kfree(gsm->buf);
		kfree(gsm);
		return NULL;
	}
	spin_lock_init(&gsm->lock);

	gsm->t1 = T1;
	gsm->t2 = T2;
	gsm->n2 = N2;
	gsm->ftype = UIH;
	gsm->initiator = 0;
	gsm->adaption = 1;
	gsm->encoding = 1;
	gsm->mru = 64;	/* Default to encoding 1 so these should be 64 */
	gsm->mtu = 64;
	gsm->dead = 1;	/* Avoid early tty opens */

	return gsm;
}
EXPORT_SYMBOL_GPL(gsm_alloc_mux);





static int gsmld_output(struct gsm_mux *gsm, u8 *data, int len)
{
	if (tty_write_room(gsm->tty) < len) {
		set_bit(TTY_DO_WRITE_WAKEUP, &gsm->tty->flags);
		return -ENOSPC;
	}
	if (debug & 4) {
		printk("-->%d bytes out\n", len);
		hex_packet(data, len);
	}
	gsm->tty->ops->write(gsm->tty, data, len);
	return len;
}


static int gsmld_attach_gsm(struct tty_struct *tty, struct gsm_mux *gsm)
{
	int ret;

	gsm->tty = tty_kref_get(tty);
	gsm->output = gsmld_output;
	ret =  gsm_activate_mux(gsm);
	if (ret != 0)
		tty_kref_put(gsm->tty);
	return ret;
}



static void gsmld_detach_gsm(struct tty_struct *tty, struct gsm_mux *gsm)
{
	WARN_ON(tty != gsm->tty);
	gsm_cleanup_mux(gsm);
	tty_kref_put(gsm->tty);
	gsm->tty = NULL;
}

static void gsmld_receive_buf(struct tty_struct *tty, const unsigned char *cp,
			      char *fp, int count)
{
	struct gsm_mux *gsm = tty->disc_data;
	const unsigned char *dp;
	char *f;
	int i;
	char buf[64];
	char flags;

	if (debug & 4) {
		printk("Inbytes %dd\n", count);
		hex_packet(cp, count);
	}

	for (i = count, dp = cp, f = fp; i; i--, dp++) {
		flags = *f++;
		switch (flags) {
		case TTY_NORMAL:
			gsm->receive(gsm, *dp);
			break;
		case TTY_OVERRUN:
		case TTY_BREAK:
		case TTY_PARITY:
		case TTY_FRAME:
			gsm->error(gsm, *dp, flags);
			break;
		default:
			printk(KERN_ERR "%s: unknown flag %d\n",
			       tty_name(tty, buf), flags);
			break;
		}
	}
	/* FASYNC if needed ? */
	/* If clogged call tty_throttle(tty); */
}


static ssize_t gsmld_chars_in_buffer(struct tty_struct *tty)
{
	return 0;
}


static void gsmld_flush_buffer(struct tty_struct *tty)
{
}


static void gsmld_close(struct tty_struct *tty)
{
	struct gsm_mux *gsm = tty->disc_data;

	gsmld_detach_gsm(tty, gsm);

	gsmld_flush_buffer(tty);
	/* Do other clean up here */
	gsm_free_mux(gsm);
}


static int gsmld_open(struct tty_struct *tty)
{
	struct gsm_mux *gsm;

	if (tty->ops->write == NULL)
		return -EINVAL;

	/* Attach our ldisc data */
	gsm = gsm_alloc_mux();
	if (gsm == NULL)
		return -ENOMEM;

	tty->disc_data = gsm;
	tty->receive_room = 65536;

	/* Attach the initial passive connection */
	gsm->encoding = 1;
	return gsmld_attach_gsm(tty, gsm);
}


static void gsmld_write_wakeup(struct tty_struct *tty)
{
	struct gsm_mux *gsm = tty->disc_data;
	unsigned long flags;

	/* Queue poll */
	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	gsm_data_kick(gsm);
	if (gsm->tx_bytes < TX_THRESH_LO) {
		spin_lock_irqsave(&gsm->tx_lock, flags);
		gsm_dlci_data_sweep(gsm);
		spin_unlock_irqrestore(&gsm->tx_lock, flags);
	}
}


static ssize_t gsmld_read(struct tty_struct *tty, struct file *file,
			 unsigned char __user *buf, size_t nr)
{
	return -EOPNOTSUPP;
}


static ssize_t gsmld_write(struct tty_struct *tty, struct file *file,
			   const unsigned char *buf, size_t nr)
{
	int space = tty_write_room(tty);
	if (space >= nr)
		return tty->ops->write(tty, buf, nr);
	set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	return -ENOBUFS;
}


static unsigned int gsmld_poll(struct tty_struct *tty, struct file *file,
							poll_table *wait)
{
	unsigned int mask = 0;
	struct gsm_mux *gsm = tty->disc_data;

	poll_wait(file, &tty->read_wait, wait);
	poll_wait(file, &tty->write_wait, wait);
	if (tty_hung_up_p(file))
		mask |= POLLHUP;
	if (!tty_is_writelocked(tty) && tty_write_room(tty) > 0)
		mask |= POLLOUT | POLLWRNORM;
	if (gsm->dead)
		mask |= POLLHUP;
	return mask;
}

static int gsmld_config(struct tty_struct *tty, struct gsm_mux *gsm,
							struct gsm_config *c)
{
	int need_close = 0;
	int need_restart = 0;

	/* Stuff we don't support yet - UI or I frame transport, windowing */
	if ((c->adaption !=1 && c->adaption != 2) || c->k)
		return -EOPNOTSUPP;
	/* Check the MRU/MTU range looks sane */
	if (c->mru > MAX_MRU || c->mtu > MAX_MTU || c->mru < 8 || c->mtu < 8)
		return -EINVAL;
	if (c->n2 < 3)
		return -EINVAL;
	if (c->encapsulation > 1)	/* Basic, advanced, no I */
		return -EINVAL;
	if (c->initiator > 1)
		return -EINVAL;
	if (c->i == 0 || c->i > 2)	/* UIH and UI only */
		return -EINVAL;
	/*
	 *	See what is needed for reconfiguration
	 */

	/* Timing fields */
	if (c->t1 != 0 && c->t1 != gsm->t1)
		need_restart = 1;
	if (c->t2 != 0 && c->t2 != gsm->t2)
		need_restart = 1;
	if (c->encapsulation != gsm->encoding)
		need_restart = 1;
	if (c->adaption != gsm->adaption)
		need_restart = 1;
	/* Requires care */
	if (c->initiator != gsm->initiator)
		need_close = 1;
	if (c->mru != gsm->mru)
		need_restart = 1;
	if (c->mtu != gsm->mtu)
		need_restart = 1;

	/*
	 *	Close down what is needed, restart and initiate the new
	 *	configuration
	 */

	if (need_close || need_restart) {
		gsm_dlci_begin_close(gsm->dlci[0]);
		/* This will timeout if the link is down due to N2 expiring */
		wait_event_interruptible(gsm->event,
				gsm->dlci[0]->state == DLCI_CLOSED);
		if (signal_pending(current))
			return -EINTR;
	}
	if (need_restart)
		gsm_cleanup_mux(gsm);

	gsm->initiator = c->initiator;
	gsm->mru = c->mru;
	gsm->encoding = c->encapsulation;
	gsm->adaption = c->adaption;

	if (c->i == 1)
		gsm->ftype = UIH;
	else if (c->i == 2)
		gsm->ftype = UI;

	if (c->t1)
		gsm->t1 = c->t1;
	if (c->t2)
		gsm->t2 = c->t2;

	/* FIXME: We need to separate activation/deactivation from adding
	   and removing from the mux array */
	if (need_restart)
		gsm_activate_mux(gsm);
	if (gsm->initiator && need_close)
		gsm_dlci_begin_open(gsm->dlci[0]);
	return 0;
}

static int gsmld_ioctl(struct tty_struct *tty, struct file *file,
		       unsigned int cmd, unsigned long arg)
{
	struct gsm_config c;
	struct gsm_mux *gsm = tty->disc_data;

	switch (cmd) {
	case GSMIOC_GETCONF:
		memset(&c, 0, sizeof(c));
		c.adaption = gsm->adaption;
		c.encapsulation = gsm->encoding;
		c.initiator = gsm->initiator;
		c.t1 = gsm->t1;
		c.t2 = gsm->t2;
		c.t3 = 0;	/* Not supported */
		c.n2 = gsm->n2;
		if (gsm->ftype == UIH)
			c.i = 1;
		else
			c.i = 2;
		printk("Ftype %d i %d\n", gsm->ftype, c.i);
		c.mru = gsm->mru;
		c.mtu = gsm->mtu;
		c.k = 0;
		if (copy_to_user((void *)arg, &c, sizeof(c)))
			return -EFAULT;
		return 0;
	case GSMIOC_SETCONF:
		if (copy_from_user(&c, (void *)arg, sizeof(c)))
			return -EFAULT;
		return gsmld_config(tty, gsm, &c);
	default:
		return n_tty_ioctl_helper(tty, file, cmd, arg);
	}
}


/* Line discipline for real tty */
struct tty_ldisc_ops tty_ldisc_packet = {
	.owner		 = THIS_MODULE,
	.magic           = TTY_LDISC_MAGIC,
	.name            = "n_gsm",
	.open            = gsmld_open,
	.close           = gsmld_close,
	.flush_buffer    = gsmld_flush_buffer,
	.chars_in_buffer = gsmld_chars_in_buffer,
	.read            = gsmld_read,
	.write           = gsmld_write,
	.ioctl           = gsmld_ioctl,
	.poll            = gsmld_poll,
	.receive_buf     = gsmld_receive_buf,
	.write_wakeup    = gsmld_write_wakeup
};


#define TX_SIZE		512

static int gsmtty_modem_update(struct gsm_dlci *dlci, u8 brk)
{
	u8 modembits[5];
	struct gsm_control *ctrl;
	int len = 2;

	if (brk)
		len++;

	modembits[0] = len << 1 | EA;		/* Data bytes */
	modembits[1] = dlci->addr << 2 | 3;	/* DLCI, EA, 1 */
	modembits[2] = gsm_encode_modem(dlci) << 1 | EA;
	if (brk)
		modembits[3] = brk << 4 | 2 | EA;	/* Valid, EA */
	ctrl = gsm_control_send(dlci->gsm, CMD_MSC, modembits, len + 1);
	if (ctrl == NULL)
		return -ENOMEM;
	return gsm_control_wait(dlci->gsm, ctrl);
}

static int gsm_carrier_raised(struct tty_port *port)
{
	struct gsm_dlci *dlci = container_of(port, struct gsm_dlci, port);
	/* Not yet open so no carrier info */
	if (dlci->state != DLCI_OPEN)
		return 0;
	if (debug & 2)
		return 1;
	return dlci->modem_rx & TIOCM_CD;
}

static void gsm_dtr_rts(struct tty_port *port, int onoff)
{
	struct gsm_dlci *dlci = container_of(port, struct gsm_dlci, port);
	unsigned int modem_tx = dlci->modem_tx;
	if (onoff)
		modem_tx |= TIOCM_DTR | TIOCM_RTS;
	else
		modem_tx &= ~(TIOCM_DTR | TIOCM_RTS);
	if (modem_tx != dlci->modem_tx) {
		dlci->modem_tx = modem_tx;
		gsmtty_modem_update(dlci, 0);
	}
}

static const struct tty_port_operations gsm_port_ops = {
	.carrier_raised = gsm_carrier_raised,
	.dtr_rts = gsm_dtr_rts,
};


static int gsmtty_open(struct tty_struct *tty, struct file *filp)
{
	struct gsm_mux *gsm;
	struct gsm_dlci *dlci;
	struct tty_port *port;
	unsigned int line = tty->index;
	unsigned int mux = line >> 6;

	line = line & 0x3F;

	if (mux >= MAX_MUX)
		return -ENXIO;
	/* FIXME: we need to lock gsm_mux for lifetimes of ttys eventually */
	if (gsm_mux[mux] == NULL)
		return -EUNATCH;
	if (line == 0 || line > 61)	/* 62/63 reserved */
		return -ECHRNG;
	gsm = gsm_mux[mux];
	if (gsm->dead)
		return -EL2HLT;
	dlci = gsm->dlci[line];
	if (dlci == NULL)
		dlci = gsm_dlci_alloc(gsm, line);
	if (dlci == NULL)
		return -ENOMEM;
	port = &dlci->port;
	port->count++;
	tty->driver_data = dlci;
	tty_port_tty_set(port, tty);

	dlci->modem_rx = 0;
	/* We could in theory open and close before we wait - eg if we get
	   a DM straight back. This is ok as that will have caused a hangup */
	set_bit(ASYNCB_INITIALIZED, &port->flags);
	/* Start sending off SABM messages */
	gsm_dlci_begin_open(dlci);
	/* And wait for virtual carrier */
	return tty_port_block_til_ready(port, tty, filp);
}

static void gsmtty_close(struct tty_struct *tty, struct file *filp)
{
	struct gsm_dlci *dlci = tty->driver_data;
	if (dlci == NULL)
		return;
	if (tty_port_close_start(&dlci->port, tty, filp) == 0)
		return;
	gsm_dlci_begin_close(dlci);
	tty_port_close_end(&dlci->port, tty);
	tty_port_tty_set(&dlci->port, NULL);
}

static void gsmtty_hangup(struct tty_struct *tty)
{
	struct gsm_dlci *dlci = tty->driver_data;
	tty_port_hangup(&dlci->port);
	gsm_dlci_begin_close(dlci);
}

static int gsmtty_write(struct tty_struct *tty, const unsigned char *buf,
								    int len)
{
	struct gsm_dlci *dlci = tty->driver_data;
	/* Stuff the bytes into the fifo queue */
	int sent = kfifo_in_locked(dlci->fifo, buf, len, &dlci->lock);
	/* Need to kick the channel */
	gsm_dlci_data_kick(dlci);
	return sent;
}

static int gsmtty_write_room(struct tty_struct *tty)
{
	struct gsm_dlci *dlci = tty->driver_data;
	return TX_SIZE - kfifo_len(dlci->fifo);
}

static int gsmtty_chars_in_buffer(struct tty_struct *tty)
{
	struct gsm_dlci *dlci = tty->driver_data;
	return kfifo_len(dlci->fifo);
}

static void gsmtty_flush_buffer(struct tty_struct *tty)
{
	struct gsm_dlci *dlci = tty->driver_data;
	/* Caution needed: If we implement reliable transport classes
	   then the data being transmitted can't simply be junked once
	   it has first hit the stack. Until then we can just blow it
	   away */
	kfifo_reset(dlci->fifo);
	/* Need to unhook this DLCI from the transmit queue logic */
}

static void gsmtty_wait_until_sent(struct tty_struct *tty, int timeout)
{
	/* The FIFO handles the queue so the kernel will do the right
	   thing waiting on chars_in_buffer before calling us. No work
	   to do here */
}

static int gsmtty_tiocmget(struct tty_struct *tty, struct file *filp)
{
	struct gsm_dlci *dlci = tty->driver_data;
	return dlci->modem_rx;
}

static int gsmtty_tiocmset(struct tty_struct *tty, struct file *filp,
	unsigned int set, unsigned int clear)
{
	struct gsm_dlci *dlci = tty->driver_data;
	unsigned int modem_tx = dlci->modem_tx;

	modem_tx &= clear;
	modem_tx |= set;

	if (modem_tx != dlci->modem_tx) {
		dlci->modem_tx = modem_tx;
		return gsmtty_modem_update(dlci, 0);
	}
	return 0;
}


static int gsmtty_ioctl(struct tty_struct *tty, struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	return -ENOIOCTLCMD;
}

static void gsmtty_set_termios(struct tty_struct *tty, struct ktermios *old)
{
	/* For the moment its fixed. In actual fact the speed information
	   for the virtual channel can be propogated in both directions by
	   the RPN control message. This however rapidly gets nasty as we
	   then have to remap modem signals each way according to whether
	   our virtual cable is null modem etc .. */
	tty_termios_copy_hw(tty->termios, old);
}

static void gsmtty_throttle(struct tty_struct *tty)
{
	struct gsm_dlci *dlci = tty->driver_data;
	if (tty->termios->c_cflag & CRTSCTS)
		dlci->modem_tx &= ~TIOCM_DTR;
	dlci->throttled = 1;
	/* Send an MSC with DTR cleared */
	gsmtty_modem_update(dlci, 0);
}

static void gsmtty_unthrottle(struct tty_struct *tty)
{
	struct gsm_dlci *dlci = tty->driver_data;
	if (tty->termios->c_cflag & CRTSCTS)
		dlci->modem_tx |= TIOCM_DTR;
	dlci->throttled = 0;
	/* Send an MSC with DTR set */
	gsmtty_modem_update(dlci, 0);
}

static int gsmtty_break_ctl(struct tty_struct *tty, int state)
{
	struct gsm_dlci *dlci = tty->driver_data;
	int encode = 0;	/* Off */

	if (state == -1)	/* "On indefinitely" - we can't encode this
				    properly */
		encode = 0x0F;
	else if (state > 0) {
		encode = state / 200;	/* mS to encoding */
		if (encode > 0x0F)
			encode = 0x0F;	/* Best effort */
	}
	return gsmtty_modem_update(dlci, encode);
}

static struct tty_driver *gsm_tty_driver;

/* Virtual ttys for the demux */
static const struct tty_operations gsmtty_ops = {
	.open			= gsmtty_open,
	.close			= gsmtty_close,
	.write			= gsmtty_write,
	.write_room		= gsmtty_write_room,
	.chars_in_buffer	= gsmtty_chars_in_buffer,
	.flush_buffer		= gsmtty_flush_buffer,
	.ioctl			= gsmtty_ioctl,
	.throttle		= gsmtty_throttle,
	.unthrottle		= gsmtty_unthrottle,
	.set_termios		= gsmtty_set_termios,
	.hangup			= gsmtty_hangup,
	.wait_until_sent	= gsmtty_wait_until_sent,
	.tiocmget		= gsmtty_tiocmget,
	.tiocmset		= gsmtty_tiocmset,
	.break_ctl		= gsmtty_break_ctl,
};



static int __init gsm_init(void)
{
	/* Fill in our line protocol discipline, and register it */
	int status = tty_register_ldisc(N_GSM0710, &tty_ldisc_packet);
	if (status != 0) {
		printk(KERN_ERR "n_gsm: can't register line discipline (err = %d)\n", status);
		return status;
	}

	gsm_tty_driver = alloc_tty_driver(256);
	if (!gsm_tty_driver) {
		tty_unregister_ldisc(N_GSM0710);
		printk(KERN_ERR "gsm_init: tty allocation failed.\n");
		return -EINVAL;
	}
	gsm_tty_driver->owner	= THIS_MODULE;
	gsm_tty_driver->driver_name	= "gsmtty";
	gsm_tty_driver->name		= "gsmtty";
	gsm_tty_driver->major		= 0;	/* Dynamic */
	gsm_tty_driver->minor_start	= 0;
	gsm_tty_driver->type		= TTY_DRIVER_TYPE_SERIAL;
	gsm_tty_driver->subtype	= SERIAL_TYPE_NORMAL;
	gsm_tty_driver->flags	= TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV
							| TTY_DRIVER_HARDWARE_BREAK;
	gsm_tty_driver->init_termios	= tty_std_termios;
	/* Fixme */
	gsm_tty_driver->init_termios.c_lflag &= ~ECHO;
	tty_set_operations(gsm_tty_driver, &gsmtty_ops);

	spin_lock_init(&gsm_mux_lock);

	if (tty_register_driver(gsm_tty_driver)) {
		put_tty_driver(gsm_tty_driver);
		tty_unregister_ldisc(N_GSM0710);
		printk(KERN_ERR "gsm_init: tty registration failed.\n");
		return -EBUSY;
	}
	printk(KERN_INFO "gsm_init: loaded as %d,%d.\n", gsm_tty_driver->major, gsm_tty_driver->minor_start);
	return 0;
}

static void __exit gsm_exit(void)
{
	int status = tty_unregister_ldisc(N_GSM0710);
	if (status != 0)
		printk(KERN_ERR "n_gsm: can't unregister line discipline (err = %d)\n", status);
	tty_unregister_driver(gsm_tty_driver);
	put_tty_driver(gsm_tty_driver);
	printk(KERN_INFO "gsm_init: unloaded.\n");
}

module_init(gsm_init);
module_exit(gsm_exit);


MODULE_LICENSE("GPL");
MODULE_ALIAS_LDISC(N_GSM0710);
