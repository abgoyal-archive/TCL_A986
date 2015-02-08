/*	$NetBSD: isakmp_frag.h,v 1.5 2006/09/18 20:32:40 manu Exp $	*/

/*	Id: isakmp_frag.h,v 1.3 2005/04/09 16:25:24 manubsd Exp */


#define ISAKMP_FRAG_OFF		FLASE   /* = 0 */
#define ISAKMP_FRAG_ON		TRUE    /* = 1 */
#define ISAKMP_FRAG_FORCE	2

/* IKE fragmentation capabilities */
#define VENDORID_FRAG_IDENT 	0x80000000
#define VENDORID_FRAG_BASE 	0x40000000
#define VENDORID_FRAG_AGG 	0x80000000

#define ISAKMP_FRAG_MAXLEN 552

struct isakmp_frag_item {
	int	frag_num;
	int	frag_last;
	struct isakmp_frag_item *frag_next;
	vchar_t *frag_packet;
};

int isakmp_sendfrags(struct ph1handle *, vchar_t *);
unsigned int vendorid_frag_cap(struct isakmp_gen *);
int isakmp_frag_extract(struct ph1handle *, vchar_t *);
vchar_t *isakmp_frag_reassembly(struct ph1handle *);
vchar_t *isakmp_frag_addcap(vchar_t *, int);
