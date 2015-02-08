

#ifndef lib_pcap_namedb_h
#define lib_pcap_namedb_h

#ifdef __cplusplus
extern "C" {
#endif

struct pcap_etherent {
	u_char addr[6];
	char name[122];
};
#ifndef PCAP_ETHERS_FILE
#define PCAP_ETHERS_FILE "/etc/ethers"
#endif
struct	pcap_etherent *pcap_next_etherent(FILE *);
u_char *pcap_ether_hostton(const char*);
u_char *pcap_ether_aton(const char *);

bpf_u_int32 **pcap_nametoaddr(const char *);
#ifdef INET6
struct addrinfo *pcap_nametoaddrinfo(const char *);
#endif
bpf_u_int32 pcap_nametonetaddr(const char *);

int	pcap_nametoport(const char *, int *, int *);
int	pcap_nametoportrange(const char *, int *, int *, int *);
int	pcap_nametoproto(const char *);
int	pcap_nametoeproto(const char *);
int	pcap_nametollc(const char *);
#define PROTO_UNDEF		-1

/* XXX move these to pcap-int.h? */
int __pcap_atodn(const char *, bpf_u_int32 *);
int __pcap_atoin(const char *, bpf_u_int32 *);
u_short	__pcap_nametodnaddr(const char *);

#ifdef __cplusplus
}
#endif

#endif
