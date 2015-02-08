
/* @(#) $Header: /tcpdump/master/tcpdump/udp.h,v 1.8.2.2 2007/03/28 07:45:46 hannes Exp $ (LBL) */

struct udphdr {
	u_int16_t	uh_sport;		/* source port */
	u_int16_t	uh_dport;		/* destination port */
	u_int16_t	uh_ulen;		/* udp length */
	u_int16_t	uh_sum;			/* udp checksum */
};

#define TFTP_PORT 69		/*XXX*/
#define KERBEROS_PORT 88	/*XXX*/
#define SUNRPC_PORT 111		/*XXX*/
#define SNMP_PORT 161		/*XXX*/
#define NTP_PORT 123		/*XXX*/
#define SNMPTRAP_PORT 162	/*XXX*/
#define ISAKMP_PORT 500		/*XXX*/
#define SYSLOG_PORT 514         /* rfc3164 */
#define TIMED_PORT 525		/*XXX*/
#define RIP_PORT 520		/*XXX*/
#define LDP_PORT 646
#define AODV_PORT 654		/*XXX*/
#define OLSR_PORT 698           /* rfc3626 */
#define KERBEROS_SEC_PORT 750	/*XXX*/
#define L2TP_PORT 1701		/*XXX*/
#define SIP_PORT 5060
#define ISAKMP_PORT_NATT  4500  /* rfc3948 */
#define ISAKMP_PORT_USER1 7500	/*XXX - nonstandard*/
#define ISAKMP_PORT_USER2 8500	/*XXX - nonstandard*/
#define RX_PORT_LOW 7000	/*XXX*/
#define RX_PORT_HIGH 7009	/*XXX*/
#define NETBIOS_NS_PORT   137
#define NETBIOS_DGRAM_PORT   138
#define CISCO_AUTORP_PORT 496	/*XXX*/
#define RADIUS_PORT 1645
#define RADIUS_NEW_PORT 1812
#define RADIUS_ACCOUNTING_PORT 1646
#define RADIUS_NEW_ACCOUNTING_PORT 1813
#define HSRP_PORT 1985		/*XXX*/
#define LMP_PORT                701 /* rfc4204 */
#define LWRES_PORT		921
#define ZEPHYR_SRV_PORT		2103
#define ZEPHYR_CLT_PORT		2104
#define MPLS_LSP_PING_PORT      3503 /* draft-ietf-mpls-lsp-ping-02.txt */
#define BFD_CONTROL_PORT        3784 /* draft-katz-ward-bfd-v4v6-1hop-00.txt */
#define BFD_ECHO_PORT           3785 /* draft-katz-ward-bfd-v4v6-1hop-00.txt */

#ifdef INET6
#define RIPNG_PORT 521		/*XXX*/
#define DHCP6_SERV_PORT 546	/*XXX*/
#define DHCP6_CLI_PORT 547	/*XXX*/
#endif
