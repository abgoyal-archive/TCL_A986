/*	$NetBSD: isakmp_unity.h,v 1.4 2006/09/09 16:22:09 manu Exp $	*/

/*	$KAME$ */


/* ISAKMP notifies specific to the Unity vendor Id */
/* Sent during xauth if the user types his password too slowly */
#define ISAKMP_NTYPE_UNITY_HEARTBEAT	40500

/* ISAKMP mode config attributes specific to the Unity vendor Id */
#define UNITY_BANNER		28672
#define UNITY_SAVE_PASSWD	28673
#define UNITY_DEF_DOMAIN	28674
#define UNITY_SPLITDNS_NAME	28675
#define UNITY_SPLIT_INCLUDE	28676
#define UNITY_NATT_PORT		28677
#define UNITY_LOCAL_LAN		28678
#define UNITY_PFS		28679
#define UNITY_FW_TYPE		28680
#define UNITY_BACKUP_SERVERS	28681
#define UNITY_DDNS_HOSTNAME	28682

        
struct unity_network {
	struct in_addr addr4;
	struct in_addr mask4;
	char padding[6];
} __attribute__((__packed__));
        
struct unity_netentry {
	struct unity_network    network;       
	struct unity_netentry   *next;
};

int	splitnet_list_add(struct unity_netentry **, struct unity_network *, int *);
void	splitnet_list_free(struct unity_netentry *, int *);
char *	splitnet_list_2str(struct unity_netentry *);

vchar_t *isakmp_unity_req(struct ph1handle *, struct isakmp_data *);
void isakmp_unity_reply(struct ph1handle *, struct isakmp_data *);
