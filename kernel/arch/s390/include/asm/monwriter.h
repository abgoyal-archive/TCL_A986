

#ifndef _ASM_390_MONWRITER_H
#define _ASM_390_MONWRITER_H

/* mon_function values */
#define MONWRITE_START_INTERVAL	0x00 /* start interval recording */
#define MONWRITE_STOP_INTERVAL	0x01 /* stop interval or config recording */
#define MONWRITE_GEN_EVENT	0x02 /* generate event record */
#define MONWRITE_START_CONFIG	0x03 /* start configuration recording */

/* the header the app uses in its write() data */
struct monwrite_hdr {
	unsigned char mon_function;
	unsigned short applid;
	unsigned char record_num;
	unsigned short version;
	unsigned short release;
	unsigned short mod_level;
	unsigned short datalen;
	unsigned char hdrlen;

} __attribute__((packed));

#endif /* _ASM_390_MONWRITER_H */
