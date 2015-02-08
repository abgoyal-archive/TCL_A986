

#ifndef pcap_int_h
#define pcap_int_h

#ifdef __cplusplus
extern "C" {
#endif

#include <pcap.h>

#ifdef WIN32
#include <Packet32.h>
#endif /* WIN32 */

#ifdef MSDOS
#include <fcntl.h>
#include <io.h>
#endif

#define SWAPLL(ull)  ((ull & 0xff00000000000000LL) >> 56) | \
                      ((ull & 0x00ff000000000000LL) >> 40) | \
                      ((ull & 0x0000ff0000000000LL) >> 24) | \
                      ((ull & 0x000000ff00000000LL) >> 8)  | \
                      ((ull & 0x00000000ff000000LL) << 8)  | \
                      ((ull & 0x0000000000ff0000LL) << 24) | \
                      ((ull & 0x000000000000ff00LL) << 40) | \
                      ((ull & 0x00000000000000ffLL) << 56)

typedef enum {
	NOT_SWAPPED,
	SWAPPED,
	MAYBE_SWAPPED
} swapped_type_t;

struct pcap_sf {
	FILE *rfile;
	int swapped;
	int hdrsize;
	swapped_type_t lengths_swapped;
	int version_major;
	int version_minor;
	u_char *base;
};

struct pcap_md {
	struct pcap_stat stat;
	/*XXX*/
	int use_bpf;		/* using kernel filter */
	u_long	TotPkts;	/* can't oflow for 79 hrs on ether */
	u_long	TotAccepted;	/* count accepted by filter */
	u_long	TotDrops;	/* count of dropped packets */
	long	TotMissed;	/* missed by i/f during this run */
	long	OrigMissed;	/* missed by i/f before this run */
	char	*device;	/* device name */
#ifdef linux
	int	sock_packet;	/* using Linux 2.0 compatible interface */
	int	timeout;	/* timeout specified to pcap_open_live */
	int	clear_promisc;	/* must clear promiscuous mode when we close */
	int	cooked;		/* using SOCK_DGRAM rather than SOCK_RAW */
	int	ifindex;	/* interface index of device we're bound to */
	int	lo_ifindex;	/* interface index of the loopback device */
	struct pcap *next;	/* list of open promiscuous sock_packet pcaps */
	u_int	packets_read;	/* count of packets read with recvfrom() */
#endif

#ifdef HAVE_DAG_API
#ifdef HAVE_DAG_STREAMS_API
	u_char	*dag_mem_bottom;	/* DAG card current memory bottom pointer */
	u_char	*dag_mem_top;	/* DAG card current memory top pointer */
#else
	void	*dag_mem_base;	/* DAG card memory base address */
	u_int	dag_mem_bottom;	/* DAG card current memory bottom offset */
	u_int	dag_mem_top;	/* DAG card current memory top offset */
#endif /* HAVE_DAG_STREAMS_API */
	int	dag_fcs_bits;	/* Number of checksum bits from link layer */
	int	dag_offset_flags; /* Flags to pass to dag_offset(). */
	int	dag_stream;	/* DAG stream number */
	int	dag_timeout;	/* timeout specified to pcap_open_live.
				 * Same as in linux above, introduce
				 * generally? */
#endif /* HAVE_DAG_API */
};

#ifdef __NetBSD__
#include <sys/param.h>	/* needed to declare __NetBSD_Version__ */
#endif

#if defined(ultrix) || defined(__osf__) || (defined(__NetBSD__) && __NetBSD_Version__ > 106000000)
#define       PCAP_FDDIPAD 3
#endif

struct pcap {
#ifdef WIN32
	ADAPTER *adapter;
	LPPACKET Packet;
	int timeout;
	int nonblock;
#else
	int fd;
	int selectable_fd;
	int send_fd;
#endif /* WIN32 */
	int snapshot;
	int linktype;
	int tzoff;		/* timezone offset */
	int offset;		/* offset for proper alignment */

	int break_loop;		/* flag set to force break from packet-reading loop */

#ifdef PCAP_FDDIPAD
	int fddipad;
#endif

#ifdef MSDOS
        int inter_packet_wait;   /* offline: wait between packets */
        void (*wait_proc)(void); /*          call proc while waiting */
#endif

	struct pcap_sf sf;
	struct pcap_md md;

	/*
	 * Read buffer.
	 */
	int bufsize;
	u_char *buffer;
	u_char *bp;
	int cc;

	/*
	 * Place holder for pcap_next().
	 */
	u_char *pkt;

	/* We're accepting only packets in this direction/these directions. */
	pcap_direction_t direction;

	/*
	 * Methods.
	 */
	int	(*read_op)(pcap_t *, int cnt, pcap_handler, u_char *);
	int	(*inject_op)(pcap_t *, const void *, size_t);
	int	(*setfilter_op)(pcap_t *, struct bpf_program *);
	int	(*setdirection_op)(pcap_t *, pcap_direction_t);
	int	(*set_datalink_op)(pcap_t *, int);
	int	(*getnonblock_op)(pcap_t *, char *);
	int	(*setnonblock_op)(pcap_t *, int, char *);
	int	(*stats_op)(pcap_t *, struct pcap_stat *);
	void	(*close_op)(pcap_t *);

	/*
	 * Placeholder for filter code if bpf not in kernel.
	 */
	struct bpf_program fcode;

	char errbuf[PCAP_ERRBUF_SIZE + 1];
	int dlt_count;
	u_int *dlt_list;

	struct pcap_pkthdr pcap_header;	/* This is needed for the pcap_next_ex() to work */
};


struct pcap_timeval {
    bpf_int32 tv_sec;		/* seconds */
    bpf_int32 tv_usec;		/* microseconds */
};


struct pcap_sf_pkthdr {
    struct pcap_timeval ts;	/* time stamp */
    bpf_u_int32 caplen;		/* length of portion present */
    bpf_u_int32 len;		/* length this packet (off wire) */
};


struct pcap_sf_patched_pkthdr {
    struct pcap_timeval ts;	/* time stamp */
    bpf_u_int32 caplen;		/* length of portion present */
    bpf_u_int32 len;		/* length this packet (off wire) */
    int		index;
    unsigned short protocol;
    unsigned char pkt_type;
};

int	yylex(void);

#ifndef min
#define min(a, b) ((a) > (b) ? (b) : (a))
#endif

/* XXX should these be in pcap.h? */
int	pcap_offline_read(pcap_t *, int, pcap_handler, u_char *);
int	pcap_read(pcap_t *, int cnt, pcap_handler, u_char *);

#ifndef HAVE_STRLCPY
#define strlcpy(x, y, z) \
	(strncpy((x), (y), (z)), \
	 ((z) <= 0 ? 0 : ((x)[(z) - 1] = '\0')), \
	 strlen((y)))
#endif

#include <stdarg.h>

#if !defined(HAVE_SNPRINTF)
#define snprintf pcap_snprintf
extern int snprintf (char *, size_t, const char *, ...);
#endif

#if !defined(HAVE_VSNPRINTF)
#define vsnprintf pcap_vsnprintf
extern int vsnprintf (char *, size_t, const char *, va_list ap);
#endif

#if !defined(WIN32) && !defined(MSDOS)
int	pcap_getnonblock_fd(pcap_t *, char *);
int	pcap_setnonblock_fd(pcap_t *p, int, char *);
#endif

void	pcap_close_common(pcap_t *);

int	pcap_platform_finddevs(pcap_if_t **, char *);
int	add_addr_to_iflist(pcap_if_t **, const char *, u_int, struct sockaddr *,
	    size_t, struct sockaddr *, size_t, struct sockaddr *, size_t,
	    struct sockaddr *, size_t, char *);
int	pcap_add_if(pcap_if_t **, const char *, u_int, const char *, char *);
struct sockaddr *dup_sockaddr(struct sockaddr *, size_t);
int	add_or_find_if(pcap_if_t **, pcap_if_t **, const char *, u_int,
	    const char *, char *);

#ifdef WIN32
char	*pcap_win32strerror(void);
#endif

int	install_bpf_program(pcap_t *, struct bpf_program *);

int	pcap_strcasecmp(const char *, const char *);

#ifdef __cplusplus
}
#endif

#endif
