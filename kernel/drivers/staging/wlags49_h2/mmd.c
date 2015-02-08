

//   vim:tw=110:ts=4:

#include "hcf.h"				// Needed as long as we do not really sort out the mess
#include "hcfdef.h"				// get CNV_LITTLE_TO_SHORT
#include "mmd.h"				// MoreModularDriver common include file

//to distinguish DHF from HCF asserts by means of line number
#undef	FILE_NAME_OFFSET
#define FILE_NAME_OFFSET DHF_FILE_NAME_OFFSET


CFG_RANGE_SPEC_STRCT*
mmd_check_comp( CFG_RANGES_STRCT *actp, CFG_SUP_RANGE_STRCT *supp )
{

CFG_RANGE_SPEC_BYTE_STRCT  *actq = (CFG_RANGE_SPEC_BYTE_STRCT*)actp->var_rec;
CFG_RANGE_SPEC_BYTE_STRCT  *supq = (CFG_RANGE_SPEC_BYTE_STRCT*)&(supp->variant);
hcf_16	i;
int		act_endian;					//actor endian flag
int		sup_endian;					//supplier endian flag

	act_endian = actp->role == COMP_ROLE_ACT;	//true if native endian				/* 1a */
	sup_endian = supp->bottom < 0x0100;		 	//true if native endian				/* 2a */

#if HCF_ASSERT
	MMDASSERT( supp->len == 6, 																supp->len )
	MMDASSERT( actp->len >= 6 && actp->len%3 == 0, 											actp->len )

	if ( act_endian ) {							//native endian
		MMDASSERT( actp->role == COMP_ROLE_ACT,												actp->role )
		MMDASSERT( 1 <= actp->id && actp->id <= 99,  										actp->id )
	} else {									//non-native endian
		MMDASSERT( actp->role == CNV_END_SHORT(COMP_ROLE_ACT),	 							actp->role )
		MMDASSERT( 1 <= CNV_END_SHORT(actp->id) && CNV_END_SHORT(actp->id) <= 99,				actp->id )
	}
	if ( sup_endian ) {							//native endian
		MMDASSERT( supp->role == COMP_ROLE_SUPL, 											supp->role )
		MMDASSERT( 1 <= supp->id      && supp->id <= 99,  									supp->id )
		MMDASSERT( 1 <= supp->variant && supp->variant <= 99,  								supp->variant )
		MMDASSERT( 1 <= supp->bottom  && supp->bottom <= 99, 								supp->bottom )
		MMDASSERT( 1 <= supp->top     && supp->top <= 99, 		 							supp->top )
		MMDASSERT( supp->bottom <= supp->top,							supp->bottom << 8 | supp->top )
	} else {									//non-native endian
		MMDASSERT( supp->role == CNV_END_SHORT(COMP_ROLE_SUPL), 								supp->role )
		MMDASSERT( 1 <= CNV_END_SHORT(supp->id) && CNV_END_SHORT(supp->id) <= 99,				supp->id )
		MMDASSERT( 1 <= CNV_END_SHORT(supp->variant) && CNV_END_SHORT(supp->variant) <= 99,		supp->variant )
		MMDASSERT( 1 <= CNV_END_SHORT(supp->bottom)  && CNV_END_SHORT(supp->bottom) <=99,		supp->bottom )
		MMDASSERT( 1 <= CNV_END_SHORT(supp->top)     && CNV_END_SHORT(supp->top) <=99,  		supp->top )
		MMDASSERT( CNV_END_SHORT(supp->bottom) <= CNV_END_SHORT(supp->top),	supp->bottom << 8 |	supp->top )
	}
#endif // HCF_ASSERT

#if HCF_BIG_ENDIAN == 0
	act_endian = !act_endian;																		/* 1b*/
	sup_endian = !sup_endian;																		/* 2b*/
#endif // HCF_BIG_ENDIAN

	for ( i = actp->len ; i > 3; actq++, i -= 3 ) {													/* 6 */
		MMDASSERT( actq->variant[act_endian] <= 99, i<<8 | actq->variant[act_endian] )
		MMDASSERT( actq->bottom[act_endian] <= 99 , i<<8 | actq->bottom[act_endian] )
		MMDASSERT( actq->top[act_endian] <= 99	  , i<<8 | actq->top[act_endian] )
		MMDASSERT( actq->bottom[act_endian] <= actq->top[act_endian], i<<8 | actq->bottom[act_endian] )
		if ( actq->variant[act_endian] == supq->variant[sup_endian] &&
			 actq->bottom[act_endian]  <= supq->top[sup_endian] &&
			 actq->top[act_endian]     >= supq->bottom[sup_endian]
		   ) break;
	}
	if ( i <= 3 || supp->len != 6 /*sizeof(CFG_SUP_RANGE_STRCT)/sizeof(hcf_16) - 1 */ ) {
	   actq = NULL;																					/* 8 */
	}
#if HCF_ASSERT
	if ( actq == NULL ) {
		for ( i = 0; i <= supp->len; i += 2 ) {
			MMDASSERT( DO_ASSERT, MERGE_2( ((hcf_16*)supp)[i], ((hcf_16*)supp)[i+1] ) );
		}
		for ( i = 0; i <= actp->len; i += 2 ) {
			MMDASSERT( DO_ASSERT, MERGE_2( ((hcf_16*)actp)[i], ((hcf_16*)actp)[i+1] ) );
		}
	}
#endif // HCF_ASSERT
	return (CFG_RANGE_SPEC_STRCT*)actq;
} // mmd_check_comp

