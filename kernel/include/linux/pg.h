

#define PG_MAGIC	'P'
#define PG_RESET	'Z'
#define PG_COMMAND	'C'

#define PG_MAX_DATA	32768

struct pg_write_hdr {

	char	magic;		/* == PG_MAGIC */
	char	func;		/* PG_RESET or PG_COMMAND */
	int     dlen;		/* number of bytes expected to transfer */
	int     timeout;	/* number of seconds before timeout */
	char	packet[12];	/* packet command */

};

struct pg_read_hdr {

	char	magic;		/* == PG_MAGIC */
	char	scsi;		/* "scsi" status == sense key */
	int	dlen;		/* size of device transfer request */
	int     duration;	/* time in seconds command took */
	char    pad[12];	/* not used */

};

/* end of pg.h */
