

struct ipfc_header {
	u_char  ipfc_dhost[8];
	u_char  ipfc_shost[8];
};

#define IPFC_HDRLEN 16
