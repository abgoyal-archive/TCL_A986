



struct fddi_header {
	u_char  fddi_fc;		/* frame control */
	u_char  fddi_dhost[6];
	u_char  fddi_shost[6];
};

#define FDDI_HDRLEN 13

/* Useful values for fddi_fc (frame control) field */

#define	FDDIFC_C		0x80		/* Class bit */
#define	FDDIFC_L		0x40		/* Address length bit */
#define	FDDIFC_F		0x30		/* Frame format bits */
#define	FDDIFC_Z		0x0f		/* Control bits */

#define	FDDIFC_VOID		0x40		/* Void frame */
#define	FDDIFC_NRT		0x80		/* Nonrestricted token */
#define	FDDIFC_RT		0xc0		/* Restricted token */
#define	FDDIFC_SMT_INFO		0x41		/* SMT Info */
#define	FDDIFC_SMT_NSA		0x4F		/* SMT Next station adrs */
#define	FDDIFC_MAC_BEACON	0xc2		/* MAC Beacon frame */
#define	FDDIFC_MAC_CLAIM	0xc3		/* MAC Claim frame */
#define	FDDIFC_LLC_ASYNC	0x50		/* Async. LLC frame */
#define	FDDIFC_LLC_SYNC		0xd0		/* Sync. LLC frame */
#define	FDDIFC_IMP_ASYNC	0x60		/* Implementor Async. */
#define	FDDIFC_IMP_SYNC		0xe0		/* Implementor Synch. */
#define FDDIFC_SMT		0x40		/* SMT frame */
#define FDDIFC_MAC		0xc0		/* MAC frame */

#define	FDDIFC_CLFF		0xF0		/* Class/Length/Format bits */
#define	FDDIFC_ZZZZ		0x0F		/* Control bits */
