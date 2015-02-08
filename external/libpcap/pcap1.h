
/* -*- Mode: c; tab-width: 8; indent-tabs-mode: 1; c-basic-offset: 8; -*- */

#ifndef lib_pcap_h
#define lib_pcap_h

#ifdef WIN32
#include <pcap-stdinc.h>
#else /* WIN32 */
#include <sys/types.h>
#include <sys/time.h>
#endif /* WIN32 */

#ifndef PCAP_DONT_INCLUDE_PCAP_BPF_H
#include <pcap-bpf.h>
#endif

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PCAP_VERSION_MAJOR 3
#define PCAP_VERSION_MINOR 0

#define PCAP_ERRBUF_SIZE 256

#if BPF_RELEASE - 0 < 199406
typedef	int bpf_int32;
typedef	u_int bpf_u_int32;
#endif

typedef struct pcap pcap_t;
typedef struct pcap_dumper pcap_dumper_t;
typedef struct pcap_if pcap_if_t;
typedef struct pcap_addr pcap_addr_t;


enum pcap1_info_types {
        PCAP_DATACAPTURE,
	PCAP_TIMESTAMP,
	PCAP_WALLTIME,
	PCAP_TIMESKEW,
	PCAP_PROBEPLACE,              /* aka direction */
	PCAP_COMMENT,                 /* comment */
};

struct pcap1_info_container {
	bpf_u_int32 info_len;         /* in bytes */
	bpf_u_int32 info_type;        /* enum pcap1_info_types */
	unsigned char info_data[0];
};

struct pcap1_info_timestamp {
	struct pcap1_info_container pic;
	bpf_u_int32    nanoseconds;   /* 10^-9 of seconds */
	bpf_u_int32    seconds;       /* seconds since Unix epoch - GMT */
	bpf_u_int16    macroseconds;  /* 16 bits more of MSB of time */
	bpf_u_int16    sigfigs;	      /* accuracy of timestamps - LSB bits */
};	
	
struct pcap1_info_packet {
	struct pcap1_info_container pic;
	bpf_u_int32 caplen;	/* length of portion present */
	bpf_u_int32 len;	/* length this packet (off wire) */
	bpf_u_int32 linktype;	/* data link type (LINKTYPE_*) */
	bpf_u_int32 ifIndex;	/* abstracted interface index */
	unsigned char packet_data[0];
};	

enum pcap1_probe {
	INBOUND  =1,
	OUTBOUND =2,
	FORWARD  =3,
	PREENCAP =4,
	POSTDECAP=5,
};

struct pcap1_info_probe {
	struct pcap1_info_container pic;
	bpf_u_int32                 probeloc;   /* enum pcap1_probe */
        unsigned char               probe_desc[0];
};
	
struct pcap1_info_comment {
	struct pcap1_info_container pic;
        unsigned char               comment[0];
};
	
struct pcap1_packet_header {
	bpf_u_int32 magic;
	u_short     version_major;
	u_short     version_minor;
        bpf_u_int32 block_len;
	struct pcap1_info_container pics[0];
};


struct pcap_stat {
	u_int ps_recv;		/* number of packets received */
	u_int ps_drop;		/* number of packets dropped */
	u_int ps_ifdrop;	/* drops by interface XXX not yet supported */
#ifdef WIN32
	u_int bs_capt;		/* number of packets that reach the application */
#endif /* WIN32 */
};

struct pcap_if {
	struct pcap_if *next;
	char *name;		/* name to hand to "pcap_open_live()" */
	char *description;	/* textual description of interface, or NULL */
	struct pcap_addr *addresses;
	bpf_u_int32 flags;	/* PCAP_IF_ interface flags */
};

#define PCAP_IF_LOOPBACK	0x00000001	/* interface is loopback */

struct pcap_addr {
	struct pcap_addr *next;
	struct sockaddr *addr;		/* address */
	struct sockaddr *netmask;	/* netmask for that address */
	struct sockaddr *broadaddr;	/* broadcast address for that address */
	struct sockaddr *dstaddr;	/* P2P destination address for that address */
};

typedef void (*pcap_handler)(u_char *, const struct pcap_pkthdr *,
			     const u_char *);

char	*pcap_lookupdev(char *);
int	pcap_lookupnet(const char *, bpf_u_int32 *, bpf_u_int32 *, char *);
pcap_t	*pcap_open_live(const char *, int, int, int, char *);
pcap_t	*pcap_open_dead(int, int);
pcap_t	*pcap_open_offline(const char *, char *);
void	pcap_close(pcap_t *);
int	pcap_loop(pcap_t *, int, pcap_handler, u_char *);
int	pcap_dispatch(pcap_t *, int, pcap_handler, u_char *);
const u_char*
	pcap_next(pcap_t *, struct pcap_pkthdr *);
int 	pcap_next_ex(pcap_t *, struct pcap_pkthdr **, const u_char **);
void	pcap_breakloop(pcap_t *);
int	pcap_stats(pcap_t *, struct pcap_stat *);
int	pcap_setfilter(pcap_t *, struct bpf_program *);
int	pcap_getnonblock(pcap_t *, char *);
int	pcap_setnonblock(pcap_t *, int, char *);
void	pcap_perror(pcap_t *, char *);
char	*pcap_strerror(int);
char	*pcap_geterr(pcap_t *);
int	pcap_compile(pcap_t *, struct bpf_program *, char *, int,
	    bpf_u_int32);
int	pcap_compile_nopcap(int, int, struct bpf_program *,
	    char *, int, bpf_u_int32);
void	pcap_freecode(struct bpf_program *);
int	pcap_datalink(pcap_t *);
int	pcap_list_datalinks(pcap_t *, int **);
int	pcap_set_datalink(pcap_t *, int);
int	pcap_datalink_name_to_val(const char *);
const char *pcap_datalink_val_to_name(int);
const char *pcap_datalink_val_to_description(int);
int	pcap_snapshot(pcap_t *);
int	pcap_is_swapped(pcap_t *);
int	pcap_major_version(pcap_t *);
int	pcap_minor_version(pcap_t *);

/* XXX */
FILE	*pcap_file(pcap_t *);
int	pcap_fileno(pcap_t *);

pcap_dumper_t *pcap_dump_open(pcap_t *, const char *);
int	pcap_dump_flush(pcap_dumper_t *);
void	pcap_dump_close(pcap_dumper_t *);
void	pcap_dump(u_char *, const struct pcap_pkthdr *, const u_char *);
FILE	*pcap_dump_file(pcap_dumper_t *);

int	pcap_findalldevs(pcap_if_t **, char *);
void	pcap_freealldevs(pcap_if_t *);

const char *pcap_lib_version(void);

/* XXX this guy lives in the bpf tree */
u_int	bpf_filter(struct bpf_insn *, u_char *, u_int, u_int);
int	bpf_validate(struct bpf_insn *f, int len);
char	*bpf_image(struct bpf_insn *, int);
void	bpf_dump(struct bpf_program *, int);

#ifdef WIN32

int pcap_setbuff(pcap_t *p, int dim);
int pcap_setmode(pcap_t *p, int mode);
int pcap_sendpacket(pcap_t *p, u_char *buf, int size);
int pcap_setmintocopy(pcap_t *p, int size);

#ifdef WPCAP
/* Include file with the wpcap-specific extensions */
#include <Win32-Extensions.h>
#endif

#define MODE_CAPT 0
#define MODE_STAT 1

#else

int	pcap_get_selectable_fd(pcap_t *);

#endif /* WIN32 */

#ifdef __cplusplus
}
#endif

#endif
