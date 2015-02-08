

/*   vim:tw=110:ts=4: */

#include "hcf.h"
#include "hcfdef.h"
#include "dhf.h"
#include "mmd.h"

/* to distinguish MMD from HCF asserts by means of line number */
#undef	FILE_NAME_OFFSET
#define FILE_NAME_OFFSET MMD_FILE_NAME_OFFSET

/*                    12345678901234 */
char signature[14] = "FUPU7D37dhfwci";


#define DL_SIZE 2000

/* CFG_IDENTITY_STRCT   	pri_identity	= { LOF(CFG_IDENTITY_STRCT), CFG_PRI_IDENTITY }; */
CFG_SUP_RANGE_STRCT 	mfi_sup        	= { LOF(CFG_SUP_RANGE_STRCT), CFG_NIC_MFI_SUP_RANGE };
CFG_SUP_RANGE_STRCT 	cfi_sup        	= { LOF(CFG_SUP_RANGE_STRCT), CFG_NIC_CFI_SUP_RANGE };



LTV_INFO_STRUCT ltv_info[] = {
	{ (LTVP)&mfi_sup,			LOF(CFG_SUP_RANGE_STRCT) } ,
	{ (LTVP)&cfi_sup,			LOF(CFG_SUP_RANGE_STRCT) } ,
	{ (LTVP) NULL, 				0 }
};


/***********************************************************************************************************/
/***************************************  PROTOTYPES  ******************************************************/
/***********************************************************************************************************/
static int				check_comp_fw(memimage *fw);


int
check_comp_fw(memimage *fw)
{
CFG_RANGE20_STRCT  		*p;
int   					rc = HCF_SUCCESS;
CFG_RANGE_SPEC_STRCT *i;

	switch (fw->identity->typ) {
	case CFG_FW_IDENTITY:				/* Station F/W */
	case COMP_ID_FW_AP_FAKE:			/* ;?is this useful (used to be:  CFG_AP_IDENTITY) */
		break;
	default:
		MMDASSERT(DO_ASSERT, fw->identity->typ) 	/* unknown/unsupported firmware_type: */
		rc = DHF_ERR_INCOMP_FW;
		return rc; /* ;? how useful is this anyway,
					*  till that is sorted out might as well violate my own single exit principle
					*/
	}
	p = fw->compat;
	i = NULL;
	while (p->len && i == NULL) {					/* check the MFI ranges */
		if (p->typ  == CFG_MFI_ACT_RANGES_STA) {
			i = mmd_check_comp((void *)p, &mfi_sup);
		}
		p++;
	}
	MMDASSERT(i, 0)	/* MFI: NIC Supplier not compatible with F/W image Actor */
	if (i) {
		p = fw->compat;
		i = NULL;
		while (p->len && i == NULL) {			/* check the CFI ranges */
			if (p->typ  == CFG_CFI_ACT_RANGES_STA) {
				 i = mmd_check_comp((void *)p, &cfi_sup);
			}
			p++;
		}
		MMDASSERT(i, 0)	/* CFI: NIC Supplier not compatible with F/W image Actor */
	}
	if (i == NULL) {
		rc = DHF_ERR_INCOMP_FW;
	}
	return rc;
} /* check_comp_fw */








int
dhf_download_binary(memimage *fw)
{
int 			rc = HCF_SUCCESS;
CFG_PROG_STRCT 	*p;
int				i;

	/* validate the image */
	for (i = 0; i < sizeof(signature) && fw->signature[i] == signature[i]; i++)
		; /* NOP */
	if (i != sizeof(signature) 		||
		 fw->signature[i] != 0x01   	||
		 /* test for Little/Big Endian Binary flag */
		 fw->signature[i+1] != (/* HCF_BIG_ENDIAN ? 'B' : */ 'L'))
		rc = DHF_ERR_INCOMP_FW;
	else {					/* Little Endian Binary format */
		fw->codep    = (CFG_PROG_STRCT FAR*)((char *)fw->codep + (hcf_32)fw);
		fw->identity = (CFG_IDENTITY_STRCT FAR*)((char *)fw->identity + (hcf_32)fw);
		fw->compat   = (CFG_RANGE20_STRCT FAR*)((char *)fw->compat + (hcf_32)fw);
		for (i = 0; fw->p[i]; i++)
			fw->p[i] = ((char *)fw->p[i] + (hcf_32)fw);
		p = fw->codep;
		while (p->len) {
			p->host_addr = (char *)p->host_addr + (hcf_32)fw;
			p++;
		}
	}
	return rc;
}   /* dhf_download_binary */


int
dhf_download_fw(void *ifbp, memimage *fw)
{
int 				rc = HCF_SUCCESS;
LTV_INFO_STRUCT_PTR pp = ltv_info;
CFG_PROG_STRCT 		*p = fw->codep;
LTVP 				ltvp;
int					i;

	MMDASSERT(fw != NULL, 0)
	/* validate the image */
	for (i = 0; i < sizeof(signature) && fw->signature[i] == signature[i]; i++)
		; /* NOP */
	if (i != sizeof(signature) 		||
		 fw->signature[i] != 0x01		||
		 /* check for binary image */
		 (fw->signature[i+1] != 'C' && fw->signature[i+1] != (/*HCF_BIG_ENDIAN ? 'B' : */ 'L')))
		 rc = DHF_ERR_INCOMP_FW;

/*	Retrieve all information needed for download from the NIC */
	while ((rc == HCF_SUCCESS) && ((ltvp = pp->ltvp) != NULL)) {
		ltvp->len = pp++->len;	/* Set len to original len. This len is changed to real len by GET_INFO() */
		rc = GET_INFO(ltvp);
		MMDASSERT(rc == HCF_SUCCESS, rc)
		MMDASSERT(rc == HCF_SUCCESS, ltvp->typ)
		MMDASSERT(rc == HCF_SUCCESS, ltvp->len)
	}
	if (rc == HCF_SUCCESS)
		rc = check_comp_fw(fw);
	if (rc == HCF_SUCCESS) {
		while (rc == HCF_SUCCESS && p->len) {
			rc = PUT_INFO(p);
			p++;
		}
	}
	MMDASSERT(rc == HCF_SUCCESS, rc)
	return rc;
}   /* dhf_download_fw */


