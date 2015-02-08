

/*   vim:tw=110:ts=4: */
#ifndef DHF_H
#define DHF_H



#ifdef _WIN32_WCE
#include <windef.h>
#endif

#include "hcf.h"   		 	/* includes HCFCFG.H too */

#ifdef DHF_UIL
#define GET_INFO(pp)  uil_get_info((LTVP)pp)
#define PUT_INFO(pp)  uil_put_info((LTVP)pp)
#else
#define GET_INFO(pp)  hcf_get_info(ifbp, (LTVP)pp)
#define PUT_INFO(pp)  hcf_put_info(ifbp, (LTVP)pp)
#endif


/*---- Defines --------------------------------------------------------------*/
#define CODEMASK				0x0000FFFFL    	/* Codemask for plug records */

/*---- Error numbers --------------------------------------------------------*/

#define DHF_ERR_INCOMP_FW		0x40	/* Image not compatible with NIC */

/*---- Type definitions -----------------------------------------------------*/
/* needed by dhf_wrap.c */

typedef struct {
	LTVP 	ltvp;
	hcf_16	len;
} LTV_INFO_STRUCT , *LTV_INFO_STRUCT_PTR;



typedef struct {
	hcf_32	code;      	/* Code to plug */
	hcf_32	addr;      	/* Address within the memory image to plug it in */
	hcf_32	len;       	/* The # of bytes which are available to store it */
} plugrecord;


#define MAX_DEBUGSTRINGS 		1024
#define MAX_DEBUGSTRING_LEN 	  82

typedef struct {
	hcf_32	id;
	char 	str[MAX_DEBUGSTRING_LEN];
} stringrecord;


#define MAX_DEBUGEXPORTS 		2048
#define MAX_DEBUGEXPORT_LEN 	  12

typedef struct {
	hcf_32	id;
	char 	str[MAX_DEBUGEXPORT_LEN];
} exportrecord;

/* Offsets in memimage array p[] */
#define FWSTRINGS_FUNCTION		0
#define FWEXPORTS_FUNCTION		1

typedef struct {
	char					signature[14+1+1];	/* signature (see DHF.C) + C/LE-Bin/BE-Bin-flag + format version */
	CFG_PROG_STRCT FAR *codep;				/* */
	hcf_32           	 	execution;    		/* Execution address of the firmware */
	void FAR *place_holder_1;
	void FAR  		     	*place_holder_2;
	CFG_RANGE20_STRCT FAR  	*compat;      		/* Pointer to the compatibility info records */
	CFG_IDENTITY_STRCT FAR 	*identity;    		/* Pointer to the identity info records */
	void FAR				*p[2];				/* (Up to 9) pointers for (future) expansion
												 * currently in use:
												 *  - F/W printf information
												 */
} memimage;




EXTERN_C int dhf_download_fw(void *ifbp, memimage *fw);	/* ifbp, ignored when using the UIL */
EXTERN_C int dhf_download_binary(memimage *fw);



/* defined in DHF.C; see there for comments */
EXTERN_C hcf_16 *find_record_in_pda(hcf_16 *pdap, hcf_16 code);

#endif  /* DHF_H */

