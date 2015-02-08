

#include <asm/types.h>


u32 bcom_gen_bd_tx_task[] = {
	/* header */
	0x4243544b,
	0x0f040609,
	0x00000000,
	0x00000000,

	/* Task descriptors */
	0x800220e3, /* LCD: idx0 = var0, idx1 = var4; idx1 <= var3; idx0 += inc4, idx1 += inc3 */
	0x13e01010, /*   DRD1A: var4 = var2; FN=0 MORE init=31 WS=0 RS=0 */
	0xb8808264, /*   LCD: idx2 = *idx1, idx3 = var1; idx2 < var9; idx2 += inc4, idx3 += inc4 */
	0x10001308, /*     DRD1A: var4 = idx1; FN=0 MORE init=0 WS=0 RS=0 */
	0x60140002, /*     DRD2A: EU0=0 EU1=0 EU2=0 EU3=2 EXT init=0 WS=2 RS=2 */
	0x0cccfcca, /*     DRD2B1: *idx3 = EU3(); EU3(*idx3,var10)  */
	0xd9190300, /*   LCDEXT: idx2 = idx2; idx2 > var12; idx2 += inc0 */
	0xb8c5e009, /*   LCD: idx3 = *(idx1 + var00000015); ; idx3 += inc1 */
	0x03fec398, /*     DRD1A: *idx0 = *idx3; FN=0 init=31 WS=3 RS=3 */
	0x9919826a, /*   LCD: idx2 = idx2, idx3 = idx3; idx2 > var9; idx2 += inc5, idx3 += inc2 */
	0x0feac398, /*     DRD1A: *idx0 = *idx3; FN=0 TFD INT init=31 WS=1 RS=1 */
	0x99190036, /*   LCD: idx2 = idx2; idx2 once var0; idx2 += inc6 */
	0x60000005, /*     DRD2A: EU0=0 EU1=0 EU2=0 EU3=5 EXT init=0 WS=0 RS=0 */
	0x0c4cf889, /*     DRD2B1: *idx1 = EU3(); EU3(idx2,var9)  */
	0x000001f8, /*   NOP */

	/* VAR[9]-VAR[12] */
	0x40000000,
	0x7fff7fff,
	0x00000000,
	0x40000004,

	/* INC[0]-INC[5] */
	0x40000000,
	0xe0000000,
	0xe0000000,
	0xa0000008,
	0x20000000,
	0x4000ffff,
};

