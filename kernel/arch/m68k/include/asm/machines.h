
#ifndef _SPARC_MACHINES_H
#define _SPARC_MACHINES_H

struct Sun_Machine_Models {
	char *name;
	unsigned char id_machtype;
};

// reduced along with table in arch/m68k/sun3/idprom.c
// sun3 port doesn't need to know about sparc machines.
//#define NUM_SUN_MACHINES   23
#define NUM_SUN_MACHINES  8


#define SM_ARCH_MASK  0xf0
#define SM_SUN3       0x10
#define SM_SUN4       0x20
#define SM_SUN3X      0x40
#define SM_SUN4C      0x50
#define SM_SUN4M      0x70
#define SM_SUN4M_OBP  0x80

#define SM_TYP_MASK   0x0f
/* Sun3 machines */
#define SM_3_160      0x01    /* Sun 3/160 series */
#define SM_3_50       0x02    /* Sun 3/50 series */
#define SM_3_260      0x03    /* Sun 3/260 series */
#define SM_3_110      0x04    /* Sun 3/110 series */
#define SM_3_60       0x07    /* Sun 3/60 series */
#define SM_3_E        0x08    /* Sun 3/E series */

/* Sun3x machines */
#define SM_3_460      0x01    /* Sun 3/460 (460,470,480) series */
#define SM_3_80       0x02    /* Sun 3/80 series */

/* Sun4 machines */
#define SM_4_260      0x01    /* Sun 4/200 series */
#define SM_4_110      0x02    /* Sun 4/100 series */
#define SM_4_330      0x03    /* Sun 4/300 series */
#define SM_4_470      0x04    /* Sun 4/400 series */

/* Sun4c machines                Full Name              - PROM NAME */
#define SM_4C_SS1     0x01    /* Sun4c SparcStation 1   - Sun 4/60  */
#define SM_4C_IPC     0x02    /* Sun4c SparcStation IPC - Sun 4/40  */
#define SM_4C_SS1PLUS 0x03    /* Sun4c SparcStation 1+  - Sun 4/65  */
#define SM_4C_SLC     0x04    /* Sun4c SparcStation SLC - Sun 4/20  */
#define SM_4C_SS2     0x05    /* Sun4c SparcStation 2   - Sun 4/75  */
#define SM_4C_ELC     0x06    /* Sun4c SparcStation ELC - Sun 4/25  */
#define SM_4C_IPX     0x07    /* Sun4c SparcStation IPX - Sun 4/50  */

#define SM_4M_SS60    0x01    /* Sun4m SparcSystem 600                  */
#define SM_4M_SS50    0x02    /* Sun4m SparcStation 10                  */
#define SM_4M_SS40    0x03    /* Sun4m SparcStation 5                   */

/* Sun4d machines -- N/A */
/* Sun4e machines -- N/A */
/* Sun4u machines -- N/A */

#endif /* !(_SPARC_MACHINES_H) */
