



#define CI_IFACEID	1	/* Interface Identifier */
#define CI_COMPRESSTYPE	2	/* Compression Type     */

typedef struct ipv6cp_options {
    int neg_ifaceid;		/* Negotiate interface identifier? */
    int req_ifaceid;		/* Ask peer to send interface identifier? */
    int accept_local;		/* accept peer's value for iface id? */
    int opt_local;		/* ourtoken set by option */
    int opt_remote;		/* histoken set by option */
    int use_ip;			/* use IP as interface identifier */
#if defined(SOL2) || defined(__linux__)
    int use_persistent;		/* use uniquely persistent value for address */
#endif /* defined(SOL2) */
    int neg_vj;			/* Van Jacobson Compression? */
    u_short vj_protocol;	/* protocol value to use in VJ option */
    eui64_t ourid, hisid;	/* Interface identifiers */
} ipv6cp_options;

extern fsm ipv6cp_fsm[];
extern ipv6cp_options ipv6cp_wantoptions[];
extern ipv6cp_options ipv6cp_gotoptions[];
extern ipv6cp_options ipv6cp_allowoptions[];
extern ipv6cp_options ipv6cp_hisoptions[];

extern struct protent ipv6cp_protent;
