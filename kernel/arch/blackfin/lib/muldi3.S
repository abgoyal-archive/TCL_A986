

.align 2
.global ___muldi3;
.type ___muldi3, STT_FUNC;

#ifdef CONFIG_ARITHMETIC_OPS_L1
.section .l1.text
#else
.text
#endif


___muldi3:
	/* [SP] technically is part of the caller's frame, but we can
	   use it as scratch space.  */
	A0 = R2.H * R1.L, A1 = R2.L * R1.H (FU) || R3 = [SP + 12];	/* E1 */
	A0 += R3.H * R0.L, A1 += R3.L * R0.H (FU) || [SP] = R4;		/* E1 */
	A0 += A1;							/* E1 */
	R4 = A0.w;
	A0 = R0.l * R3.l (FU);						/* E2 */
	A0 += R2.l * R1.l (FU);						/* E2 */

	A1 = R2.L * R0.L (FU);						/* E4 */
	R3 = A1.w;
	A1 = A1 >> 16;							/* E3c */
	A0 += R2.H * R0.H, A1 += R2.L * R0.H (FU);			/* E2, E3c */
	A1 += R0.L * R2.H (FU);						/* E3c */
	R0 = A1.w;
	A1 = A1 >> 16;							/* E2c */
	A0 += A1;							/* E2c */
	R1 = A0.w;

	/* low(result) = low(E3c):low(E4) */
	R0 = PACK (R0.l, R3.l);
	/* high(result) = E2c + (E1 << 16) */
	R1.h = R1.h + R4.l (NS) || R4 = [SP];
	RTS;

.size ___muldi3, .-___muldi3
