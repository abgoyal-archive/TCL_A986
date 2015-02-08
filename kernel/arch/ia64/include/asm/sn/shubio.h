

#ifndef _ASM_IA64_SN_SHUBIO_H
#define _ASM_IA64_SN_SHUBIO_H

#define HUB_WIDGET_ID_MAX	0xf
#define IIO_NUM_ITTES		7
#define HUB_NUM_BIG_WINDOW	(IIO_NUM_ITTES - 1)

#define		IIO_WID			0x00400000	/* Crosstalk Widget Identification */
							/* This register is also accessible from
							 * Crosstalk at address 0x0.  */
#define		IIO_WSTAT		0x00400008	/* Crosstalk Widget Status */
#define		IIO_WCR			0x00400020	/* Crosstalk Widget Control Register */
#define		IIO_ILAPR		0x00400100	/* IO Local Access Protection Register */
#define		IIO_ILAPO		0x00400108	/* IO Local Access Protection Override */
#define		IIO_IOWA		0x00400110	/* IO Outbound Widget Access */
#define		IIO_IIWA		0x00400118	/* IO Inbound Widget Access */
#define		IIO_IIDEM		0x00400120	/* IO Inbound Device Error Mask */
#define		IIO_ILCSR		0x00400128	/* IO LLP Control and Status Register */
#define		IIO_ILLR		0x00400130	/* IO LLP Log Register    */
#define		IIO_IIDSR		0x00400138	/* IO Interrupt Destination */

#define		IIO_IGFX0		0x00400140	/* IO Graphics Node-Widget Map 0 */
#define		IIO_IGFX1		0x00400148	/* IO Graphics Node-Widget Map 1 */

#define		IIO_ISCR0		0x00400150	/* IO Scratch Register 0 */
#define		IIO_ISCR1		0x00400158	/* IO Scratch Register 1 */

#define		IIO_ITTE1		0x00400160	/* IO Translation Table Entry 1 */
#define		IIO_ITTE2		0x00400168	/* IO Translation Table Entry 2 */
#define		IIO_ITTE3		0x00400170	/* IO Translation Table Entry 3 */
#define		IIO_ITTE4		0x00400178	/* IO Translation Table Entry 4 */
#define		IIO_ITTE5		0x00400180	/* IO Translation Table Entry 5 */
#define		IIO_ITTE6		0x00400188	/* IO Translation Table Entry 6 */
#define		IIO_ITTE7		0x00400190	/* IO Translation Table Entry 7 */

#define		IIO_IPRB0		0x00400198	/* IO PRB Entry 0   */
#define		IIO_IPRB8		0x004001A0	/* IO PRB Entry 8   */
#define		IIO_IPRB9		0x004001A8	/* IO PRB Entry 9   */
#define		IIO_IPRBA		0x004001B0	/* IO PRB Entry A   */
#define		IIO_IPRBB		0x004001B8	/* IO PRB Entry B   */
#define		IIO_IPRBC		0x004001C0	/* IO PRB Entry C   */
#define		IIO_IPRBD		0x004001C8	/* IO PRB Entry D   */
#define		IIO_IPRBE		0x004001D0	/* IO PRB Entry E   */
#define		IIO_IPRBF		0x004001D8	/* IO PRB Entry F   */

#define		IIO_IXCC		0x004001E0	/* IO Crosstalk Credit Count Timeout */
#define		IIO_IMEM		0x004001E8	/* IO Miscellaneous Error Mask */
#define		IIO_IXTT		0x004001F0	/* IO Crosstalk Timeout Threshold */
#define		IIO_IECLR		0x004001F8	/* IO Error Clear Register */
#define		IIO_IBCR		0x00400200	/* IO BTE Control Register */

#define		IIO_IXSM		0x00400208	/* IO Crosstalk Spurious Message */
#define		IIO_IXSS		0x00400210	/* IO Crosstalk Spurious Sideband */

#define		IIO_ILCT		0x00400218	/* IO LLP Channel Test    */

#define		IIO_IIEPH1 		0x00400220	/* IO Incoming Error Packet Header, Part 1 */
#define		IIO_IIEPH2 		0x00400228	/* IO Incoming Error Packet Header, Part 2 */

#define		IIO_ISLAPR 		0x00400230	/* IO SXB Local Access Protection Regster */
#define		IIO_ISLAPO 		0x00400238	/* IO SXB Local Access Protection Override */

#define		IIO_IWI			0x00400240	/* IO Wrapper Interrupt Register */
#define		IIO_IWEL		0x00400248	/* IO Wrapper Error Log Register */
#define		IIO_IWC			0x00400250	/* IO Wrapper Control Register */
#define		IIO_IWS			0x00400258	/* IO Wrapper Status Register */
#define		IIO_IWEIM		0x00400260	/* IO Wrapper Error Interrupt Masking Register */

#define		IIO_IPCA		0x00400300	/* IO PRB Counter Adjust */

#define		IIO_IPRTE0_A		0x00400308	/* IO PIO Read Address Table Entry 0, Part A */
#define		IIO_IPRTE1_A		0x00400310	/* IO PIO Read Address Table Entry 1, Part A */
#define		IIO_IPRTE2_A		0x00400318	/* IO PIO Read Address Table Entry 2, Part A */
#define		IIO_IPRTE3_A		0x00400320	/* IO PIO Read Address Table Entry 3, Part A */
#define		IIO_IPRTE4_A		0x00400328	/* IO PIO Read Address Table Entry 4, Part A */
#define		IIO_IPRTE5_A		0x00400330	/* IO PIO Read Address Table Entry 5, Part A */
#define		IIO_IPRTE6_A		0x00400338	/* IO PIO Read Address Table Entry 6, Part A */
#define		IIO_IPRTE7_A		0x00400340	/* IO PIO Read Address Table Entry 7, Part A */

#define		IIO_IPRTE0_B		0x00400348	/* IO PIO Read Address Table Entry 0, Part B */
#define		IIO_IPRTE1_B		0x00400350	/* IO PIO Read Address Table Entry 1, Part B */
#define		IIO_IPRTE2_B		0x00400358	/* IO PIO Read Address Table Entry 2, Part B */
#define		IIO_IPRTE3_B		0x00400360	/* IO PIO Read Address Table Entry 3, Part B */
#define		IIO_IPRTE4_B		0x00400368	/* IO PIO Read Address Table Entry 4, Part B */
#define		IIO_IPRTE5_B		0x00400370	/* IO PIO Read Address Table Entry 5, Part B */
#define		IIO_IPRTE6_B		0x00400378	/* IO PIO Read Address Table Entry 6, Part B */
#define		IIO_IPRTE7_B		0x00400380	/* IO PIO Read Address Table Entry 7, Part B */

#define		IIO_IPDR		0x00400388	/* IO PIO Deallocation Register */
#define		IIO_ICDR		0x00400390	/* IO CRB Entry Deallocation Register */
#define		IIO_IFDR		0x00400398	/* IO IOQ FIFO Depth Register */
#define		IIO_IIAP		0x004003A0	/* IO IIQ Arbitration Parameters */
#define		IIO_ICMR		0x004003A8	/* IO CRB Management Register */
#define		IIO_ICCR		0x004003B0	/* IO CRB Control Register */
#define		IIO_ICTO		0x004003B8	/* IO CRB Timeout   */
#define		IIO_ICTP		0x004003C0	/* IO CRB Timeout Prescalar */

#define		IIO_ICRB0_A		0x00400400	/* IO CRB Entry 0_A */
#define		IIO_ICRB0_B		0x00400408	/* IO CRB Entry 0_B */
#define		IIO_ICRB0_C		0x00400410	/* IO CRB Entry 0_C */
#define		IIO_ICRB0_D		0x00400418	/* IO CRB Entry 0_D */
#define		IIO_ICRB0_E		0x00400420	/* IO CRB Entry 0_E */

#define		IIO_ICRB1_A		0x00400430	/* IO CRB Entry 1_A */
#define		IIO_ICRB1_B		0x00400438	/* IO CRB Entry 1_B */
#define		IIO_ICRB1_C		0x00400440	/* IO CRB Entry 1_C */
#define		IIO_ICRB1_D		0x00400448	/* IO CRB Entry 1_D */
#define		IIO_ICRB1_E		0x00400450	/* IO CRB Entry 1_E */

#define		IIO_ICRB2_A		0x00400460	/* IO CRB Entry 2_A */
#define		IIO_ICRB2_B		0x00400468	/* IO CRB Entry 2_B */
#define		IIO_ICRB2_C		0x00400470	/* IO CRB Entry 2_C */
#define		IIO_ICRB2_D		0x00400478	/* IO CRB Entry 2_D */
#define		IIO_ICRB2_E		0x00400480	/* IO CRB Entry 2_E */

#define		IIO_ICRB3_A		0x00400490	/* IO CRB Entry 3_A */
#define		IIO_ICRB3_B		0x00400498	/* IO CRB Entry 3_B */
#define		IIO_ICRB3_C		0x004004a0	/* IO CRB Entry 3_C */
#define		IIO_ICRB3_D		0x004004a8	/* IO CRB Entry 3_D */
#define		IIO_ICRB3_E		0x004004b0	/* IO CRB Entry 3_E */

#define		IIO_ICRB4_A		0x004004c0	/* IO CRB Entry 4_A */
#define		IIO_ICRB4_B		0x004004c8	/* IO CRB Entry 4_B */
#define		IIO_ICRB4_C		0x004004d0	/* IO CRB Entry 4_C */
#define		IIO_ICRB4_D		0x004004d8	/* IO CRB Entry 4_D */
#define		IIO_ICRB4_E		0x004004e0	/* IO CRB Entry 4_E */

#define		IIO_ICRB5_A		0x004004f0	/* IO CRB Entry 5_A */
#define		IIO_ICRB5_B		0x004004f8	/* IO CRB Entry 5_B */
#define		IIO_ICRB5_C		0x00400500	/* IO CRB Entry 5_C */
#define		IIO_ICRB5_D		0x00400508	/* IO CRB Entry 5_D */
#define		IIO_ICRB5_E		0x00400510	/* IO CRB Entry 5_E */

#define		IIO_ICRB6_A		0x00400520	/* IO CRB Entry 6_A */
#define		IIO_ICRB6_B		0x00400528	/* IO CRB Entry 6_B */
#define		IIO_ICRB6_C		0x00400530	/* IO CRB Entry 6_C */
#define		IIO_ICRB6_D		0x00400538	/* IO CRB Entry 6_D */
#define		IIO_ICRB6_E		0x00400540	/* IO CRB Entry 6_E */

#define		IIO_ICRB7_A		0x00400550	/* IO CRB Entry 7_A */
#define		IIO_ICRB7_B		0x00400558	/* IO CRB Entry 7_B */
#define		IIO_ICRB7_C		0x00400560	/* IO CRB Entry 7_C */
#define		IIO_ICRB7_D		0x00400568	/* IO CRB Entry 7_D */
#define		IIO_ICRB7_E		0x00400570	/* IO CRB Entry 7_E */

#define		IIO_ICRB8_A		0x00400580	/* IO CRB Entry 8_A */
#define		IIO_ICRB8_B		0x00400588	/* IO CRB Entry 8_B */
#define		IIO_ICRB8_C		0x00400590	/* IO CRB Entry 8_C */
#define		IIO_ICRB8_D		0x00400598	/* IO CRB Entry 8_D */
#define		IIO_ICRB8_E		0x004005a0	/* IO CRB Entry 8_E */

#define		IIO_ICRB9_A		0x004005b0	/* IO CRB Entry 9_A */
#define		IIO_ICRB9_B		0x004005b8	/* IO CRB Entry 9_B */
#define		IIO_ICRB9_C		0x004005c0	/* IO CRB Entry 9_C */
#define		IIO_ICRB9_D		0x004005c8	/* IO CRB Entry 9_D */
#define		IIO_ICRB9_E		0x004005d0	/* IO CRB Entry 9_E */

#define		IIO_ICRBA_A		0x004005e0	/* IO CRB Entry A_A */
#define		IIO_ICRBA_B		0x004005e8	/* IO CRB Entry A_B */
#define		IIO_ICRBA_C		0x004005f0	/* IO CRB Entry A_C */
#define		IIO_ICRBA_D		0x004005f8	/* IO CRB Entry A_D */
#define		IIO_ICRBA_E		0x00400600	/* IO CRB Entry A_E */

#define		IIO_ICRBB_A		0x00400610	/* IO CRB Entry B_A */
#define		IIO_ICRBB_B		0x00400618	/* IO CRB Entry B_B */
#define		IIO_ICRBB_C		0x00400620	/* IO CRB Entry B_C */
#define		IIO_ICRBB_D		0x00400628	/* IO CRB Entry B_D */
#define		IIO_ICRBB_E		0x00400630	/* IO CRB Entry B_E */

#define		IIO_ICRBC_A		0x00400640	/* IO CRB Entry C_A */
#define		IIO_ICRBC_B		0x00400648	/* IO CRB Entry C_B */
#define		IIO_ICRBC_C		0x00400650	/* IO CRB Entry C_C */
#define		IIO_ICRBC_D		0x00400658	/* IO CRB Entry C_D */
#define		IIO_ICRBC_E		0x00400660	/* IO CRB Entry C_E */

#define		IIO_ICRBD_A		0x00400670	/* IO CRB Entry D_A */
#define		IIO_ICRBD_B		0x00400678	/* IO CRB Entry D_B */
#define		IIO_ICRBD_C		0x00400680	/* IO CRB Entry D_C */
#define		IIO_ICRBD_D		0x00400688	/* IO CRB Entry D_D */
#define		IIO_ICRBD_E		0x00400690	/* IO CRB Entry D_E */

#define		IIO_ICRBE_A		0x004006a0	/* IO CRB Entry E_A */
#define		IIO_ICRBE_B		0x004006a8	/* IO CRB Entry E_B */
#define		IIO_ICRBE_C		0x004006b0	/* IO CRB Entry E_C */
#define		IIO_ICRBE_D		0x004006b8	/* IO CRB Entry E_D */
#define		IIO_ICRBE_E		0x004006c0	/* IO CRB Entry E_E */

#define		IIO_ICSML		0x00400700	/* IO CRB Spurious Message Low */
#define		IIO_ICSMM		0x00400708	/* IO CRB Spurious Message Middle */
#define		IIO_ICSMH		0x00400710	/* IO CRB Spurious Message High */

#define		IIO_IDBSS		0x00400718	/* IO Debug Submenu Select */

#define		IIO_IBLS0		0x00410000	/* IO BTE Length Status 0 */
#define		IIO_IBSA0		0x00410008	/* IO BTE Source Address 0 */
#define		IIO_IBDA0		0x00410010	/* IO BTE Destination Address 0 */
#define		IIO_IBCT0		0x00410018	/* IO BTE Control Terminate 0 */
#define		IIO_IBNA0		0x00410020	/* IO BTE Notification Address 0 */
#define		IIO_IBIA0		0x00410028	/* IO BTE Interrupt Address 0 */
#define		IIO_IBLS1		0x00420000	/* IO BTE Length Status 1 */
#define		IIO_IBSA1		0x00420008	/* IO BTE Source Address 1 */
#define		IIO_IBDA1		0x00420010	/* IO BTE Destination Address 1 */
#define		IIO_IBCT1		0x00420018	/* IO BTE Control Terminate 1 */
#define		IIO_IBNA1		0x00420020	/* IO BTE Notification Address 1 */
#define		IIO_IBIA1		0x00420028	/* IO BTE Interrupt Address 1 */

#define		IIO_IPCR		0x00430000	/* IO Performance Control */
#define		IIO_IPPR		0x00430008	/* IO Performance Profiling */


typedef union ii_wid_u {
	u64 ii_wid_regval;
	struct {
		u64 w_rsvd_1:1;
		u64 w_mfg_num:11;
		u64 w_part_num:16;
		u64 w_rev_num:4;
		u64 w_rsvd:32;
	} ii_wid_fld_s;
} ii_wid_u_t;


typedef union ii_wstat_u {
	u64 ii_wstat_regval;
	struct {
		u64 w_pending:4;
		u64 w_xt_crd_to:1;
		u64 w_xt_tail_to:1;
		u64 w_rsvd_3:3;
		u64 w_tx_mx_rty:1;
		u64 w_rsvd_2:6;
		u64 w_llp_tx_cnt:8;
		u64 w_rsvd_1:8;
		u64 w_crazy:1;
		u64 w_rsvd:31;
	} ii_wstat_fld_s;
} ii_wstat_u_t;


typedef union ii_wcr_u {
	u64 ii_wcr_regval;
	struct {
		u64 w_wid:4;
		u64 w_tag:1;
		u64 w_rsvd_1:8;
		u64 w_dst_crd:3;
		u64 w_f_bad_pkt:1;
		u64 w_dir_con:1;
		u64 w_e_thresh:5;
		u64 w_rsvd:41;
	} ii_wcr_fld_s;
} ii_wcr_u_t;


typedef union ii_ilapr_u {
	u64 ii_ilapr_regval;
	struct {
		u64 i_region:64;
	} ii_ilapr_fld_s;
} ii_ilapr_u_t;


typedef union ii_ilapo_u {
	u64 ii_ilapo_regval;
	struct {
		u64 i_io_ovrride:64;
	} ii_ilapo_fld_s;
} ii_ilapo_u_t;


typedef union ii_iowa_u {
	u64 ii_iowa_regval;
	struct {
		u64 i_w0_oac:1;
		u64 i_rsvd_1:7;
		u64 i_wx_oac:8;
		u64 i_rsvd:48;
	} ii_iowa_fld_s;
} ii_iowa_u_t;


typedef union ii_iiwa_u {
	u64 ii_iiwa_regval;
	struct {
		u64 i_w0_iac:1;
		u64 i_rsvd_1:7;
		u64 i_wx_iac:8;
		u64 i_rsvd:48;
	} ii_iiwa_fld_s;
} ii_iiwa_u_t;


typedef union ii_iidem_u {
	u64 ii_iidem_regval;
	struct {
		u64 i_w8_dxs:8;
		u64 i_w9_dxs:8;
		u64 i_wa_dxs:8;
		u64 i_wb_dxs:8;
		u64 i_wc_dxs:8;
		u64 i_wd_dxs:8;
		u64 i_we_dxs:8;
		u64 i_wf_dxs:8;
	} ii_iidem_fld_s;
} ii_iidem_u_t;


typedef union ii_ilcsr_u {
	u64 ii_ilcsr_regval;
	struct {
		u64 i_nullto:6;
		u64 i_rsvd_4:2;
		u64 i_wrmrst:1;
		u64 i_rsvd_3:1;
		u64 i_llp_en:1;
		u64 i_bm8:1;
		u64 i_llp_stat:2;
		u64 i_remote_power:1;
		u64 i_rsvd_2:1;
		u64 i_maxrtry:10;
		u64 i_d_avail_sel:2;
		u64 i_rsvd_1:4;
		u64 i_maxbrst:10;
		u64 i_rsvd:22;

	} ii_ilcsr_fld_s;
} ii_ilcsr_u_t;


typedef union ii_illr_u {
	u64 ii_illr_regval;
	struct {
		u64 i_sn_cnt:16;
		u64 i_cb_cnt:16;
		u64 i_rsvd:32;
	} ii_illr_fld_s;
} ii_illr_u_t;


typedef union ii_iidsr_u {
	u64 ii_iidsr_regval;
	struct {
		u64 i_level:8;
		u64 i_pi_id:1;
		u64 i_node:11;
		u64 i_rsvd_3:4;
		u64 i_enable:1;
		u64 i_rsvd_2:3;
		u64 i_int_sent:2;
		u64 i_rsvd_1:2;
		u64 i_pi0_forward_int:1;
		u64 i_pi1_forward_int:1;
		u64 i_rsvd:30;
	} ii_iidsr_fld_s;
} ii_iidsr_u_t;


typedef union ii_igfx0_u {
	u64 ii_igfx0_regval;
	struct {
		u64 i_w_num:4;
		u64 i_pi_id:1;
		u64 i_n_num:12;
		u64 i_p_num:1;
		u64 i_rsvd:46;
	} ii_igfx0_fld_s;
} ii_igfx0_u_t;


typedef union ii_igfx1_u {
	u64 ii_igfx1_regval;
	struct {
		u64 i_w_num:4;
		u64 i_pi_id:1;
		u64 i_n_num:12;
		u64 i_p_num:1;
		u64 i_rsvd:46;
	} ii_igfx1_fld_s;
} ii_igfx1_u_t;


typedef union ii_iscr0_u {
	u64 ii_iscr0_regval;
	struct {
		u64 i_scratch:64;
	} ii_iscr0_fld_s;
} ii_iscr0_u_t;


typedef union ii_iscr1_u {
	u64 ii_iscr1_regval;
	struct {
		u64 i_scratch:64;
	} ii_iscr1_fld_s;
} ii_iscr1_u_t;


typedef union ii_itte1_u {
	u64 ii_itte1_regval;
	struct {
		u64 i_offset:5;
		u64 i_rsvd_1:3;
		u64 i_w_num:4;
		u64 i_iosp:1;
		u64 i_rsvd:51;
	} ii_itte1_fld_s;
} ii_itte1_u_t;


typedef union ii_itte2_u {
	u64 ii_itte2_regval;
	struct {
		u64 i_offset:5;
		u64 i_rsvd_1:3;
		u64 i_w_num:4;
		u64 i_iosp:1;
		u64 i_rsvd:51;
	} ii_itte2_fld_s;
} ii_itte2_u_t;


typedef union ii_itte3_u {
	u64 ii_itte3_regval;
	struct {
		u64 i_offset:5;
		u64 i_rsvd_1:3;
		u64 i_w_num:4;
		u64 i_iosp:1;
		u64 i_rsvd:51;
	} ii_itte3_fld_s;
} ii_itte3_u_t;


typedef union ii_itte4_u {
	u64 ii_itte4_regval;
	struct {
		u64 i_offset:5;
		u64 i_rsvd_1:3;
		u64 i_w_num:4;
		u64 i_iosp:1;
		u64 i_rsvd:51;
	} ii_itte4_fld_s;
} ii_itte4_u_t;


typedef union ii_itte5_u {
	u64 ii_itte5_regval;
	struct {
		u64 i_offset:5;
		u64 i_rsvd_1:3;
		u64 i_w_num:4;
		u64 i_iosp:1;
		u64 i_rsvd:51;
	} ii_itte5_fld_s;
} ii_itte5_u_t;


typedef union ii_itte6_u {
	u64 ii_itte6_regval;
	struct {
		u64 i_offset:5;
		u64 i_rsvd_1:3;
		u64 i_w_num:4;
		u64 i_iosp:1;
		u64 i_rsvd:51;
	} ii_itte6_fld_s;
} ii_itte6_u_t;


typedef union ii_itte7_u {
	u64 ii_itte7_regval;
	struct {
		u64 i_offset:5;
		u64 i_rsvd_1:3;
		u64 i_w_num:4;
		u64 i_iosp:1;
		u64 i_rsvd:51;
	} ii_itte7_fld_s;
} ii_itte7_u_t;


typedef union ii_iprb0_u {
	u64 ii_iprb0_regval;
	struct {
		u64 i_c:8;
		u64 i_na:14;
		u64 i_rsvd_2:2;
		u64 i_nb:14;
		u64 i_rsvd_1:2;
		u64 i_m:2;
		u64 i_f:1;
		u64 i_of_cnt:5;
		u64 i_error:1;
		u64 i_rd_to:1;
		u64 i_spur_wr:1;
		u64 i_spur_rd:1;
		u64 i_rsvd:11;
		u64 i_mult_err:1;
	} ii_iprb0_fld_s;
} ii_iprb0_u_t;


typedef union ii_iprb8_u {
	u64 ii_iprb8_regval;
	struct {
		u64 i_c:8;
		u64 i_na:14;
		u64 i_rsvd_2:2;
		u64 i_nb:14;
		u64 i_rsvd_1:2;
		u64 i_m:2;
		u64 i_f:1;
		u64 i_of_cnt:5;
		u64 i_error:1;
		u64 i_rd_to:1;
		u64 i_spur_wr:1;
		u64 i_spur_rd:1;
		u64 i_rsvd:11;
		u64 i_mult_err:1;
	} ii_iprb8_fld_s;
} ii_iprb8_u_t;


typedef union ii_iprb9_u {
	u64 ii_iprb9_regval;
	struct {
		u64 i_c:8;
		u64 i_na:14;
		u64 i_rsvd_2:2;
		u64 i_nb:14;
		u64 i_rsvd_1:2;
		u64 i_m:2;
		u64 i_f:1;
		u64 i_of_cnt:5;
		u64 i_error:1;
		u64 i_rd_to:1;
		u64 i_spur_wr:1;
		u64 i_spur_rd:1;
		u64 i_rsvd:11;
		u64 i_mult_err:1;
	} ii_iprb9_fld_s;
} ii_iprb9_u_t;


typedef union ii_iprba_u {
	u64 ii_iprba_regval;
	struct {
		u64 i_c:8;
		u64 i_na:14;
		u64 i_rsvd_2:2;
		u64 i_nb:14;
		u64 i_rsvd_1:2;
		u64 i_m:2;
		u64 i_f:1;
		u64 i_of_cnt:5;
		u64 i_error:1;
		u64 i_rd_to:1;
		u64 i_spur_wr:1;
		u64 i_spur_rd:1;
		u64 i_rsvd:11;
		u64 i_mult_err:1;
	} ii_iprba_fld_s;
} ii_iprba_u_t;


typedef union ii_iprbb_u {
	u64 ii_iprbb_regval;
	struct {
		u64 i_c:8;
		u64 i_na:14;
		u64 i_rsvd_2:2;
		u64 i_nb:14;
		u64 i_rsvd_1:2;
		u64 i_m:2;
		u64 i_f:1;
		u64 i_of_cnt:5;
		u64 i_error:1;
		u64 i_rd_to:1;
		u64 i_spur_wr:1;
		u64 i_spur_rd:1;
		u64 i_rsvd:11;
		u64 i_mult_err:1;
	} ii_iprbb_fld_s;
} ii_iprbb_u_t;


typedef union ii_iprbc_u {
	u64 ii_iprbc_regval;
	struct {
		u64 i_c:8;
		u64 i_na:14;
		u64 i_rsvd_2:2;
		u64 i_nb:14;
		u64 i_rsvd_1:2;
		u64 i_m:2;
		u64 i_f:1;
		u64 i_of_cnt:5;
		u64 i_error:1;
		u64 i_rd_to:1;
		u64 i_spur_wr:1;
		u64 i_spur_rd:1;
		u64 i_rsvd:11;
		u64 i_mult_err:1;
	} ii_iprbc_fld_s;
} ii_iprbc_u_t;


typedef union ii_iprbd_u {
	u64 ii_iprbd_regval;
	struct {
		u64 i_c:8;
		u64 i_na:14;
		u64 i_rsvd_2:2;
		u64 i_nb:14;
		u64 i_rsvd_1:2;
		u64 i_m:2;
		u64 i_f:1;
		u64 i_of_cnt:5;
		u64 i_error:1;
		u64 i_rd_to:1;
		u64 i_spur_wr:1;
		u64 i_spur_rd:1;
		u64 i_rsvd:11;
		u64 i_mult_err:1;
	} ii_iprbd_fld_s;
} ii_iprbd_u_t;


typedef union ii_iprbe_u {
	u64 ii_iprbe_regval;
	struct {
		u64 i_c:8;
		u64 i_na:14;
		u64 i_rsvd_2:2;
		u64 i_nb:14;
		u64 i_rsvd_1:2;
		u64 i_m:2;
		u64 i_f:1;
		u64 i_of_cnt:5;
		u64 i_error:1;
		u64 i_rd_to:1;
		u64 i_spur_wr:1;
		u64 i_spur_rd:1;
		u64 i_rsvd:11;
		u64 i_mult_err:1;
	} ii_iprbe_fld_s;
} ii_iprbe_u_t;


typedef union ii_iprbf_u {
	u64 ii_iprbf_regval;
	struct {
		u64 i_c:8;
		u64 i_na:14;
		u64 i_rsvd_2:2;
		u64 i_nb:14;
		u64 i_rsvd_1:2;
		u64 i_m:2;
		u64 i_f:1;
		u64 i_of_cnt:5;
		u64 i_error:1;
		u64 i_rd_to:1;
		u64 i_spur_wr:1;
		u64 i_spur_rd:1;
		u64 i_rsvd:11;
		u64 i_mult_err:1;
	} ii_iprbe_fld_s;
} ii_iprbf_u_t;


typedef union ii_ixcc_u {
	u64 ii_ixcc_regval;
	struct {
		u64 i_time_out:26;
		u64 i_rsvd:38;
	} ii_ixcc_fld_s;
} ii_ixcc_u_t;


typedef union ii_imem_u {
	u64 ii_imem_regval;
	struct {
		u64 i_w0_esd:1;
		u64 i_rsvd_3:3;
		u64 i_b0_esd:1;
		u64 i_rsvd_2:3;
		u64 i_b1_esd:1;
		u64 i_rsvd_1:3;
		u64 i_clr_precise:1;
		u64 i_rsvd:51;
	} ii_imem_fld_s;
} ii_imem_u_t;


typedef union ii_ixtt_u {
	u64 ii_ixtt_regval;
	struct {
		u64 i_tail_to:26;
		u64 i_rsvd_1:6;
		u64 i_rrsp_ps:23;
		u64 i_rrsp_to:5;
		u64 i_rsvd:4;
	} ii_ixtt_fld_s;
} ii_ixtt_u_t;


typedef union ii_ieclr_u {
	u64 ii_ieclr_regval;
	struct {
		u64 i_e_prb_0:1;
		u64 i_rsvd:7;
		u64 i_e_prb_8:1;
		u64 i_e_prb_9:1;
		u64 i_e_prb_a:1;
		u64 i_e_prb_b:1;
		u64 i_e_prb_c:1;
		u64 i_e_prb_d:1;
		u64 i_e_prb_e:1;
		u64 i_e_prb_f:1;
		u64 i_e_crazy:1;
		u64 i_e_bte_0:1;
		u64 i_e_bte_1:1;
		u64 i_reserved_1:10;
		u64 i_spur_rd_hdr:1;
		u64 i_cam_intr_to:1;
		u64 i_cam_overflow:1;
		u64 i_cam_read_miss:1;
		u64 i_ioq_rep_underflow:1;
		u64 i_ioq_req_underflow:1;
		u64 i_ioq_rep_overflow:1;
		u64 i_ioq_req_overflow:1;
		u64 i_iiq_rep_overflow:1;
		u64 i_iiq_req_overflow:1;
		u64 i_ii_xn_rep_cred_overflow:1;
		u64 i_ii_xn_req_cred_overflow:1;
		u64 i_ii_xn_invalid_cmd:1;
		u64 i_xn_ii_invalid_cmd:1;
		u64 i_reserved_2:21;
	} ii_ieclr_fld_s;
} ii_ieclr_u_t;


typedef union ii_ibcr_u {
	u64 ii_ibcr_regval;
	struct {
		u64 i_count:4;
		u64 i_rsvd_1:4;
		u64 i_soft_reset:1;
		u64 i_rsvd:55;
	} ii_ibcr_fld_s;
} ii_ibcr_u_t;


typedef union ii_ixsm_u {
	u64 ii_ixsm_regval;
	struct {
		u64 i_byte_en:32;
		u64 i_reserved:1;
		u64 i_tag:3;
		u64 i_alt_pactyp:4;
		u64 i_bo:1;
		u64 i_error:1;
		u64 i_vbpm:1;
		u64 i_gbr:1;
		u64 i_ds:2;
		u64 i_ct:1;
		u64 i_tnum:5;
		u64 i_pactyp:4;
		u64 i_sidn:4;
		u64 i_didn:4;
	} ii_ixsm_fld_s;
} ii_ixsm_u_t;


typedef union ii_ixss_u {
	u64 ii_ixss_regval;
	struct {
		u64 i_sideband:8;
		u64 i_rsvd:55;
		u64 i_valid:1;
	} ii_ixss_fld_s;
} ii_ixss_u_t;


typedef union ii_ilct_u {
	u64 ii_ilct_regval;
	struct {
		u64 i_test_seed:20;
		u64 i_test_mask:8;
		u64 i_test_data:20;
		u64 i_test_valid:1;
		u64 i_test_cberr:1;
		u64 i_test_flit:3;
		u64 i_test_clear:1;
		u64 i_test_err_capture:1;
		u64 i_rsvd:9;
	} ii_ilct_fld_s;
} ii_ilct_u_t;


typedef union ii_iieph1_u {
	u64 ii_iieph1_regval;
	struct {
		u64 i_command:7;
		u64 i_rsvd_5:1;
		u64 i_suppl:14;
		u64 i_rsvd_4:1;
		u64 i_source:14;
		u64 i_rsvd_3:1;
		u64 i_err_type:4;
		u64 i_rsvd_2:4;
		u64 i_overrun:1;
		u64 i_rsvd_1:3;
		u64 i_valid:1;
		u64 i_rsvd:13;
	} ii_iieph1_fld_s;
} ii_iieph1_u_t;


typedef union ii_iieph2_u {
	u64 ii_iieph2_regval;
	struct {
		u64 i_rsvd_0:3;
		u64 i_address:47;
		u64 i_rsvd_1:10;
		u64 i_tail:1;
		u64 i_rsvd:3;
	} ii_iieph2_fld_s;
} ii_iieph2_u_t;

/******************************/


typedef union ii_islapr_u {
	u64 ii_islapr_regval;
	struct {
		u64 i_region:64;
	} ii_islapr_fld_s;
} ii_islapr_u_t;


typedef union ii_islapo_u {
	u64 ii_islapo_regval;
	struct {
		u64 i_io_sbx_ovrride:56;
		u64 i_rsvd:8;
	} ii_islapo_fld_s;
} ii_islapo_u_t;


typedef union ii_iwi_u {
	u64 ii_iwi_regval;
	struct {
		u64 i_prescale:24;
		u64 i_rsvd:8;
		u64 i_timeout:8;
		u64 i_rsvd1:8;
		u64 i_intrpt_retry_period:8;
		u64 i_rsvd2:8;
	} ii_iwi_fld_s;
} ii_iwi_u_t;


typedef union ii_iwel_u {
	u64 ii_iwel_regval;
	struct {
		u64 i_intr_timed_out:1;
		u64 i_rsvd:7;
		u64 i_cam_overflow:1;
		u64 i_cam_read_miss:1;
		u64 i_rsvd1:2;
		u64 i_ioq_rep_underflow:1;
		u64 i_ioq_req_underflow:1;
		u64 i_ioq_rep_overflow:1;
		u64 i_ioq_req_overflow:1;
		u64 i_iiq_rep_overflow:1;
		u64 i_iiq_req_overflow:1;
		u64 i_rsvd2:6;
		u64 i_ii_xn_rep_cred_over_under:1;
		u64 i_ii_xn_req_cred_over_under:1;
		u64 i_rsvd3:6;
		u64 i_ii_xn_invalid_cmd:1;
		u64 i_xn_ii_invalid_cmd:1;
		u64 i_rsvd4:30;
	} ii_iwel_fld_s;
} ii_iwel_u_t;


typedef union ii_iwc_u {
	u64 ii_iwc_regval;
	struct {
		u64 i_dma_byte_swap:1;
		u64 i_rsvd:3;
		u64 i_cam_read_lines_reset:1;
		u64 i_rsvd1:3;
		u64 i_ii_xn_cred_over_under_log:1;
		u64 i_rsvd2:19;
		u64 i_xn_rep_iq_depth:5;
		u64 i_rsvd3:3;
		u64 i_xn_req_iq_depth:5;
		u64 i_rsvd4:3;
		u64 i_iiq_depth:6;
		u64 i_rsvd5:12;
		u64 i_force_rep_cred:1;
		u64 i_force_req_cred:1;
	} ii_iwc_fld_s;
} ii_iwc_u_t;


typedef union ii_iws_u {
	u64 ii_iws_regval;
	struct {
		u64 i_xn_rep_iq_credits:5;
		u64 i_rsvd:3;
		u64 i_xn_req_iq_credits:5;
		u64 i_rsvd1:51;
	} ii_iws_fld_s;
} ii_iws_u_t;


typedef union ii_iweim_u {
	u64 ii_iweim_regval;
	struct {
		u64 i_intr_timed_out:1;
		u64 i_rsvd:7;
		u64 i_cam_overflow:1;
		u64 i_cam_read_miss:1;
		u64 i_rsvd1:2;
		u64 i_ioq_rep_underflow:1;
		u64 i_ioq_req_underflow:1;
		u64 i_ioq_rep_overflow:1;
		u64 i_ioq_req_overflow:1;
		u64 i_iiq_rep_overflow:1;
		u64 i_iiq_req_overflow:1;
		u64 i_rsvd2:6;
		u64 i_ii_xn_rep_cred_overflow:1;
		u64 i_ii_xn_req_cred_overflow:1;
		u64 i_rsvd3:6;
		u64 i_ii_xn_invalid_cmd:1;
		u64 i_xn_ii_invalid_cmd:1;
		u64 i_rsvd4:30;
	} ii_iweim_fld_s;
} ii_iweim_u_t;


typedef union ii_ipca_u {
	u64 ii_ipca_regval;
	struct {
		u64 i_wid:4;
		u64 i_adjust:1;
		u64 i_rsvd_1:3;
		u64 i_field:2;
		u64 i_rsvd:54;
	} ii_ipca_fld_s;
} ii_ipca_u_t;


typedef union ii_iprte0a_u {
	u64 ii_iprte0a_regval;
	struct {
		u64 i_rsvd_1:54;
		u64 i_widget:4;
		u64 i_to_cnt:5;
		u64 i_vld:1;
	} ii_iprte0a_fld_s;
} ii_iprte0a_u_t;


typedef union ii_iprte1a_u {
	u64 ii_iprte1a_regval;
	struct {
		u64 i_rsvd_1:54;
		u64 i_widget:4;
		u64 i_to_cnt:5;
		u64 i_vld:1;
	} ii_iprte1a_fld_s;
} ii_iprte1a_u_t;


typedef union ii_iprte2a_u {
	u64 ii_iprte2a_regval;
	struct {
		u64 i_rsvd_1:54;
		u64 i_widget:4;
		u64 i_to_cnt:5;
		u64 i_vld:1;
	} ii_iprte2a_fld_s;
} ii_iprte2a_u_t;


typedef union ii_iprte3a_u {
	u64 ii_iprte3a_regval;
	struct {
		u64 i_rsvd_1:54;
		u64 i_widget:4;
		u64 i_to_cnt:5;
		u64 i_vld:1;
	} ii_iprte3a_fld_s;
} ii_iprte3a_u_t;


typedef union ii_iprte4a_u {
	u64 ii_iprte4a_regval;
	struct {
		u64 i_rsvd_1:54;
		u64 i_widget:4;
		u64 i_to_cnt:5;
		u64 i_vld:1;
	} ii_iprte4a_fld_s;
} ii_iprte4a_u_t;


typedef union ii_iprte5a_u {
	u64 ii_iprte5a_regval;
	struct {
		u64 i_rsvd_1:54;
		u64 i_widget:4;
		u64 i_to_cnt:5;
		u64 i_vld:1;
	} ii_iprte5a_fld_s;
} ii_iprte5a_u_t;


typedef union ii_iprte6a_u {
	u64 ii_iprte6a_regval;
	struct {
		u64 i_rsvd_1:54;
		u64 i_widget:4;
		u64 i_to_cnt:5;
		u64 i_vld:1;
	} ii_iprte6a_fld_s;
} ii_iprte6a_u_t;


typedef union ii_iprte7a_u {
	u64 ii_iprte7a_regval;
	struct {
		u64 i_rsvd_1:54;
		u64 i_widget:4;
		u64 i_to_cnt:5;
		u64 i_vld:1;
	} ii_iprtea7_fld_s;
} ii_iprte7a_u_t;


typedef union ii_iprte0b_u {
	u64 ii_iprte0b_regval;
	struct {
		u64 i_rsvd_1:3;
		u64 i_address:47;
		u64 i_init:3;
		u64 i_source:11;
	} ii_iprte0b_fld_s;
} ii_iprte0b_u_t;


typedef union ii_iprte1b_u {
	u64 ii_iprte1b_regval;
	struct {
		u64 i_rsvd_1:3;
		u64 i_address:47;
		u64 i_init:3;
		u64 i_source:11;
	} ii_iprte1b_fld_s;
} ii_iprte1b_u_t;


typedef union ii_iprte2b_u {
	u64 ii_iprte2b_regval;
	struct {
		u64 i_rsvd_1:3;
		u64 i_address:47;
		u64 i_init:3;
		u64 i_source:11;
	} ii_iprte2b_fld_s;
} ii_iprte2b_u_t;


typedef union ii_iprte3b_u {
	u64 ii_iprte3b_regval;
	struct {
		u64 i_rsvd_1:3;
		u64 i_address:47;
		u64 i_init:3;
		u64 i_source:11;
	} ii_iprte3b_fld_s;
} ii_iprte3b_u_t;


typedef union ii_iprte4b_u {
	u64 ii_iprte4b_regval;
	struct {
		u64 i_rsvd_1:3;
		u64 i_address:47;
		u64 i_init:3;
		u64 i_source:11;
	} ii_iprte4b_fld_s;
} ii_iprte4b_u_t;


typedef union ii_iprte5b_u {
	u64 ii_iprte5b_regval;
	struct {
		u64 i_rsvd_1:3;
		u64 i_address:47;
		u64 i_init:3;
		u64 i_source:11;
	} ii_iprte5b_fld_s;
} ii_iprte5b_u_t;


typedef union ii_iprte6b_u {
	u64 ii_iprte6b_regval;
	struct {
		u64 i_rsvd_1:3;
		u64 i_address:47;
		u64 i_init:3;
		u64 i_source:11;

	} ii_iprte6b_fld_s;
} ii_iprte6b_u_t;


typedef union ii_iprte7b_u {
	u64 ii_iprte7b_regval;
	struct {
		u64 i_rsvd_1:3;
		u64 i_address:47;
		u64 i_init:3;
		u64 i_source:11;
	} ii_iprte7b_fld_s;
} ii_iprte7b_u_t;


typedef union ii_ipdr_u {
	u64 ii_ipdr_regval;
	struct {
		u64 i_te:3;
		u64 i_rsvd_1:1;
		u64 i_pnd:1;
		u64 i_init_rpcnt:1;
		u64 i_rsvd:58;
	} ii_ipdr_fld_s;
} ii_ipdr_u_t;


typedef union ii_icdr_u {
	u64 ii_icdr_regval;
	struct {
		u64 i_crb_num:4;
		u64 i_pnd:1;
		u64 i_rsvd:59;
	} ii_icdr_fld_s;
} ii_icdr_u_t;


typedef union ii_ifdr_u {
	u64 ii_ifdr_regval;
	struct {
		u64 i_ioq_max_rq:7;
		u64 i_set_ioq_rq:1;
		u64 i_ioq_max_rp:7;
		u64 i_set_ioq_rp:1;
		u64 i_rsvd:48;
	} ii_ifdr_fld_s;
} ii_ifdr_u_t;


typedef union ii_iiap_u {
	u64 ii_iiap_regval;
	struct {
		u64 i_rq_mls:6;
		u64 i_rsvd_1:2;
		u64 i_rp_mls:6;
		u64 i_rsvd:50;
	} ii_iiap_fld_s;
} ii_iiap_u_t;


typedef union ii_icmr_u {
	u64 ii_icmr_regval;
	struct {
		u64 i_sp_msg:1;
		u64 i_rd_hdr:1;
		u64 i_rsvd_4:2;
		u64 i_c_cnt:4;
		u64 i_rsvd_3:4;
		u64 i_clr_rqpd:1;
		u64 i_clr_rppd:1;
		u64 i_rsvd_2:2;
		u64 i_fc_cnt:4;
		u64 i_crb_vld:15;
		u64 i_crb_mark:15;
		u64 i_rsvd_1:2;
		u64 i_precise:1;
		u64 i_rsvd:11;
	} ii_icmr_fld_s;
} ii_icmr_u_t;


typedef union ii_iccr_u {
	u64 ii_iccr_regval;
	struct {
		u64 i_crb_num:4;
		u64 i_rsvd_1:4;
		u64 i_cmd:8;
		u64 i_pending:1;
		u64 i_rsvd:47;
	} ii_iccr_fld_s;
} ii_iccr_u_t;


typedef union ii_icto_u {
	u64 ii_icto_regval;
	struct {
		u64 i_timeout:8;
		u64 i_rsvd:56;
	} ii_icto_fld_s;
} ii_icto_u_t;


typedef union ii_ictp_u {
	u64 ii_ictp_regval;
	struct {
		u64 i_prescale:24;
		u64 i_rsvd:40;
	} ii_ictp_fld_s;
} ii_ictp_u_t;


typedef union ii_icrb0_a_u {
	u64 ii_icrb0_a_regval;
	struct {
		u64 ia_iow:1;
		u64 ia_vld:1;
		u64 ia_addr:47;
		u64 ia_tnum:5;
		u64 ia_sidn:4;
		u64 ia_rsvd:6;
	} ii_icrb0_a_fld_s;
} ii_icrb0_a_u_t;


typedef union ii_icrb0_b_u {
	u64 ii_icrb0_b_regval;
	struct {
		u64 ib_xt_err:1;
		u64 ib_mark:1;
		u64 ib_ln_uce:1;
		u64 ib_errcode:3;
		u64 ib_error:1;
		u64 ib_stall__bte_1:1;
		u64 ib_stall__bte_0:1;
		u64 ib_stall__intr:1;
		u64 ib_stall_ib:1;
		u64 ib_intvn:1;
		u64 ib_wb:1;
		u64 ib_hold:1;
		u64 ib_ack:1;
		u64 ib_resp:1;
		u64 ib_ack_cnt:11;
		u64 ib_rsvd:7;
		u64 ib_exc:5;
		u64 ib_init:3;
		u64 ib_imsg:8;
		u64 ib_imsgtype:2;
		u64 ib_use_old:1;
		u64 ib_rsvd_1:11;
	} ii_icrb0_b_fld_s;
} ii_icrb0_b_u_t;


typedef union ii_icrb0_c_u {
	u64 ii_icrb0_c_regval;
	struct {
		u64 ic_source:15;
		u64 ic_size:2;
		u64 ic_ct:1;
		u64 ic_bte_num:1;
		u64 ic_gbr:1;
		u64 ic_resprqd:1;
		u64 ic_bo:1;
		u64 ic_suppl:15;
		u64 ic_rsvd:27;
	} ii_icrb0_c_fld_s;
} ii_icrb0_c_u_t;


typedef union ii_icrb0_d_u {
	u64 ii_icrb0_d_regval;
	struct {
		u64 id_pa_be:43;
		u64 id_bte_op:1;
		u64 id_pr_psc:4;
		u64 id_pr_cnt:4;
		u64 id_sleep:1;
		u64 id_rsvd:11;
	} ii_icrb0_d_fld_s;
} ii_icrb0_d_u_t;


typedef union ii_icrb0_e_u {
	u64 ii_icrb0_e_regval;
	struct {
		u64 ie_timeout:8;
		u64 ie_context:15;
		u64 ie_rsvd:1;
		u64 ie_tvld:1;
		u64 ie_cvld:1;
		u64 ie_rsvd_0:38;
	} ii_icrb0_e_fld_s;
} ii_icrb0_e_u_t;


typedef union ii_icsml_u {
	u64 ii_icsml_regval;
	struct {
		u64 i_tt_addr:47;
		u64 i_newsuppl_ex:14;
		u64 i_reserved:2;
		u64 i_overflow:1;
	} ii_icsml_fld_s;
} ii_icsml_u_t;


typedef union ii_icsmm_u {
	u64 ii_icsmm_regval;
	struct {
		u64 i_tt_ack_cnt:11;
		u64 i_reserved:53;
	} ii_icsmm_fld_s;
} ii_icsmm_u_t;


typedef union ii_icsmh_u {
	u64 ii_icsmh_regval;
	struct {
		u64 i_tt_vld:1;
		u64 i_xerr:1;
		u64 i_ft_cwact_o:1;
		u64 i_ft_wact_o:1;
		u64 i_ft_active_o:1;
		u64 i_sync:1;
		u64 i_mnusg:1;
		u64 i_mnusz:1;
		u64 i_plusz:1;
		u64 i_plusg:1;
		u64 i_tt_exc:5;
		u64 i_tt_wb:1;
		u64 i_tt_hold:1;
		u64 i_tt_ack:1;
		u64 i_tt_resp:1;
		u64 i_tt_intvn:1;
		u64 i_g_stall_bte1:1;
		u64 i_g_stall_bte0:1;
		u64 i_g_stall_il:1;
		u64 i_g_stall_ib:1;
		u64 i_tt_imsg:8;
		u64 i_tt_imsgtype:2;
		u64 i_tt_use_old:1;
		u64 i_tt_respreqd:1;
		u64 i_tt_bte_num:1;
		u64 i_cbn:1;
		u64 i_match:1;
		u64 i_rpcnt_lt_34:1;
		u64 i_rpcnt_ge_34:1;
		u64 i_rpcnt_lt_18:1;
		u64 i_rpcnt_ge_18:1;
		u64 i_rpcnt_lt_2:1;
		u64 i_rpcnt_ge_2:1;
		u64 i_rqcnt_lt_18:1;
		u64 i_rqcnt_ge_18:1;
		u64 i_rqcnt_lt_2:1;
		u64 i_rqcnt_ge_2:1;
		u64 i_tt_device:7;
		u64 i_tt_init:3;
		u64 i_reserved:5;
	} ii_icsmh_fld_s;
} ii_icsmh_u_t;


typedef union ii_idbss_u {
	u64 ii_idbss_regval;
	struct {
		u64 i_iioclk_core_submenu:3;
		u64 i_rsvd:5;
		u64 i_fsbclk_wrapper_submenu:3;
		u64 i_rsvd_1:5;
		u64 i_iioclk_menu:5;
		u64 i_rsvd_2:43;
	} ii_idbss_fld_s;
} ii_idbss_u_t;


typedef union ii_ibls0_u {
	u64 ii_ibls0_regval;
	struct {
		u64 i_length:16;
		u64 i_error:1;
		u64 i_rsvd_1:3;
		u64 i_busy:1;
		u64 i_rsvd:43;
	} ii_ibls0_fld_s;
} ii_ibls0_u_t;


typedef union ii_ibsa0_u {
	u64 ii_ibsa0_regval;
	struct {
		u64 i_rsvd_1:7;
		u64 i_addr:42;
		u64 i_rsvd:15;
	} ii_ibsa0_fld_s;
} ii_ibsa0_u_t;


typedef union ii_ibda0_u {
	u64 ii_ibda0_regval;
	struct {
		u64 i_rsvd_1:7;
		u64 i_addr:42;
		u64 i_rsvd:15;
	} ii_ibda0_fld_s;
} ii_ibda0_u_t;


typedef union ii_ibct0_u {
	u64 ii_ibct0_regval;
	struct {
		u64 i_zerofill:1;
		u64 i_rsvd_2:3;
		u64 i_notify:1;
		u64 i_rsvd_1:3;
		u64 i_poison:1;
		u64 i_rsvd:55;
	} ii_ibct0_fld_s;
} ii_ibct0_u_t;


typedef union ii_ibna0_u {
	u64 ii_ibna0_regval;
	struct {
		u64 i_rsvd_1:7;
		u64 i_addr:42;
		u64 i_rsvd:15;
	} ii_ibna0_fld_s;
} ii_ibna0_u_t;


typedef union ii_ibia0_u {
	u64 ii_ibia0_regval;
	struct {
		u64 i_rsvd_2:1;
		u64 i_node_id:11;
		u64 i_rsvd_1:4;
		u64 i_level:7;
		u64 i_rsvd:41;
	} ii_ibia0_fld_s;
} ii_ibia0_u_t;


typedef union ii_ibls1_u {
	u64 ii_ibls1_regval;
	struct {
		u64 i_length:16;
		u64 i_error:1;
		u64 i_rsvd_1:3;
		u64 i_busy:1;
		u64 i_rsvd:43;
	} ii_ibls1_fld_s;
} ii_ibls1_u_t;


typedef union ii_ibsa1_u {
	u64 ii_ibsa1_regval;
	struct {
		u64 i_rsvd_1:7;
		u64 i_addr:33;
		u64 i_rsvd:24;
	} ii_ibsa1_fld_s;
} ii_ibsa1_u_t;


typedef union ii_ibda1_u {
	u64 ii_ibda1_regval;
	struct {
		u64 i_rsvd_1:7;
		u64 i_addr:33;
		u64 i_rsvd:24;
	} ii_ibda1_fld_s;
} ii_ibda1_u_t;


typedef union ii_ibct1_u {
	u64 ii_ibct1_regval;
	struct {
		u64 i_zerofill:1;
		u64 i_rsvd_2:3;
		u64 i_notify:1;
		u64 i_rsvd_1:3;
		u64 i_poison:1;
		u64 i_rsvd:55;
	} ii_ibct1_fld_s;
} ii_ibct1_u_t;


typedef union ii_ibna1_u {
	u64 ii_ibna1_regval;
	struct {
		u64 i_rsvd_1:7;
		u64 i_addr:33;
		u64 i_rsvd:24;
	} ii_ibna1_fld_s;
} ii_ibna1_u_t;


typedef union ii_ibia1_u {
	u64 ii_ibia1_regval;
	struct {
		u64 i_pi_id:1;
		u64 i_node_id:8;
		u64 i_rsvd_1:7;
		u64 i_level:7;
		u64 i_rsvd:41;
	} ii_ibia1_fld_s;
} ii_ibia1_u_t;


typedef union ii_ipcr_u {
	u64 ii_ipcr_regval;
	struct {
		u64 i_ippr0_c:4;
		u64 i_ippr1_c:4;
		u64 i_icct:8;
		u64 i_rsvd:48;
	} ii_ipcr_fld_s;
} ii_ipcr_u_t;


typedef union ii_ippr_u {
	u64 ii_ippr_regval;
	struct {
		u64 i_ippr0:32;
		u64 i_ippr1:32;
	} ii_ippr_fld_s;
} ii_ippr_u_t;


#define IIO_WIDGET              IIO_WID		/* Widget identification */
#define IIO_WIDGET_STAT         IIO_WSTAT	/* Widget status register */
#define IIO_WIDGET_CTRL         IIO_WCR		/* Widget control register */
#define IIO_PROTECT             IIO_ILAPR	/* IO interface protection */
#define IIO_PROTECT_OVRRD       IIO_ILAPO	/* IO protect override */
#define IIO_OUTWIDGET_ACCESS    IIO_IOWA	/* Outbound widget access */
#define IIO_INWIDGET_ACCESS     IIO_IIWA	/* Inbound widget access */
#define IIO_INDEV_ERR_MASK      IIO_IIDEM	/* Inbound device error mask */
#define IIO_LLP_CSR             IIO_ILCSR	/* LLP control and status */
#define IIO_LLP_LOG             IIO_ILLR	/* LLP log */
#define IIO_XTALKCC_TOUT        IIO_IXCC	/* Xtalk credit count timeout */
#define IIO_XTALKTT_TOUT        IIO_IXTT	/* Xtalk tail timeout */
#define IIO_IO_ERR_CLR          IIO_IECLR	/* IO error clear */
#define IIO_IGFX_0 		IIO_IGFX0
#define IIO_IGFX_1 		IIO_IGFX1
#define IIO_IBCT_0		IIO_IBCT0
#define IIO_IBCT_1		IIO_IBCT1
#define IIO_IBLS_0		IIO_IBLS0
#define IIO_IBLS_1		IIO_IBLS1
#define IIO_IBSA_0		IIO_IBSA0
#define IIO_IBSA_1		IIO_IBSA1
#define IIO_IBDA_0		IIO_IBDA0
#define IIO_IBDA_1		IIO_IBDA1
#define IIO_IBNA_0		IIO_IBNA0
#define IIO_IBNA_1		IIO_IBNA1
#define IIO_IBIA_0		IIO_IBIA0
#define IIO_IBIA_1		IIO_IBIA1
#define IIO_IOPRB_0		IIO_IPRB0

#define IIO_PRTE_A(_x)		(IIO_IPRTE0_A + (8 * (_x)))
#define IIO_PRTE_B(_x)		(IIO_IPRTE0_B + (8 * (_x)))
#define IIO_NUM_PRTES		8	/* Total number of PRB table entries */
#define IIO_WIDPRTE_A(x)	IIO_PRTE_A(((x) - 8))	/* widget ID to its PRTE num */
#define IIO_WIDPRTE_B(x)	IIO_PRTE_B(((x) - 8))	/* widget ID to its PRTE num */

#define IIO_NUM_IPRBS 		9

#define IIO_LLP_CSR_IS_UP		0x00002000
#define IIO_LLP_CSR_LLP_STAT_MASK       0x00003000
#define IIO_LLP_CSR_LLP_STAT_SHFT       12

#define IIO_LLP_CB_MAX  0xffff	/* in ILLR CB_CNT, Max Check Bit errors */
#define IIO_LLP_SN_MAX  0xffff	/* in ILLR SN_CNT, Max Sequence Number errors */

/* key to IIO_PROTECT_OVRRD */
#define IIO_PROTECT_OVRRD_KEY   0x53474972756c6573ull	/* "SGIrules" */

/* BTE register names */
#define IIO_BTE_STAT_0          IIO_IBLS_0	/* Also BTE length/status 0 */
#define IIO_BTE_SRC_0           IIO_IBSA_0	/* Also BTE source address  0 */
#define IIO_BTE_DEST_0          IIO_IBDA_0	/* Also BTE dest. address 0 */
#define IIO_BTE_CTRL_0          IIO_IBCT_0	/* Also BTE control/terminate 0 */
#define IIO_BTE_NOTIFY_0        IIO_IBNA_0	/* Also BTE notification 0 */
#define IIO_BTE_INT_0           IIO_IBIA_0	/* Also BTE interrupt 0 */
#define IIO_BTE_OFF_0           0	/* Base offset from BTE 0 regs. */
#define IIO_BTE_OFF_1   	(IIO_IBLS_1 - IIO_IBLS_0)	/* Offset from base to BTE 1 */

/* BTE register offsets from base */
#define BTEOFF_STAT             0
#define BTEOFF_SRC      	(IIO_BTE_SRC_0 - IIO_BTE_STAT_0)
#define BTEOFF_DEST     	(IIO_BTE_DEST_0 - IIO_BTE_STAT_0)
#define BTEOFF_CTRL     	(IIO_BTE_CTRL_0 - IIO_BTE_STAT_0)
#define BTEOFF_NOTIFY   	(IIO_BTE_NOTIFY_0 - IIO_BTE_STAT_0)
#define BTEOFF_INT      	(IIO_BTE_INT_0 - IIO_BTE_STAT_0)

/* names used in shub diags */
#define IIO_BASE_BTE0   IIO_IBLS_0
#define IIO_BASE_BTE1   IIO_IBLS_1

#define IIO_IOPRB(_x)	(IIO_IOPRB_0 + ( ( (_x) < HUB_WIDGET_ID_MIN ? \
                	(_x) : \
                	(_x) - (HUB_WIDGET_ID_MIN-1)) << 3) )

/* GFX Flow Control Node/Widget Register */
#define IIO_IGFX_W_NUM_BITS	4	/* size of widget num field */
#define IIO_IGFX_W_NUM_MASK	((1<<IIO_IGFX_W_NUM_BITS)-1)
#define IIO_IGFX_W_NUM_SHIFT	0
#define IIO_IGFX_PI_NUM_BITS	1	/* size of PI num field */
#define IIO_IGFX_PI_NUM_MASK	((1<<IIO_IGFX_PI_NUM_BITS)-1)
#define IIO_IGFX_PI_NUM_SHIFT	4
#define IIO_IGFX_N_NUM_BITS	8	/* size of node num field */
#define IIO_IGFX_N_NUM_MASK	((1<<IIO_IGFX_N_NUM_BITS)-1)
#define IIO_IGFX_N_NUM_SHIFT	5
#define IIO_IGFX_P_NUM_BITS	1	/* size of processor num field */
#define IIO_IGFX_P_NUM_MASK	((1<<IIO_IGFX_P_NUM_BITS)-1)
#define IIO_IGFX_P_NUM_SHIFT	16
#define IIO_IGFX_INIT(widget, pi, node, cpu)				(\
	(((widget) & IIO_IGFX_W_NUM_MASK) << IIO_IGFX_W_NUM_SHIFT) |	 \
	(((pi)     & IIO_IGFX_PI_NUM_MASK)<< IIO_IGFX_PI_NUM_SHIFT)|	 \
	(((node)   & IIO_IGFX_N_NUM_MASK) << IIO_IGFX_N_NUM_SHIFT) |	 \
	(((cpu)    & IIO_IGFX_P_NUM_MASK) << IIO_IGFX_P_NUM_SHIFT))

/* Scratch registers (all bits available) */
#define IIO_SCRATCH_REG0        IIO_ISCR0
#define IIO_SCRATCH_REG1        IIO_ISCR1
#define IIO_SCRATCH_MASK        0xffffffffffffffffUL

#define IIO_SCRATCH_BIT0_0      0x0000000000000001UL
#define IIO_SCRATCH_BIT0_1      0x0000000000000002UL
#define IIO_SCRATCH_BIT0_2      0x0000000000000004UL
#define IIO_SCRATCH_BIT0_3      0x0000000000000008UL
#define IIO_SCRATCH_BIT0_4      0x0000000000000010UL
#define IIO_SCRATCH_BIT0_5      0x0000000000000020UL
#define IIO_SCRATCH_BIT0_6      0x0000000000000040UL
#define IIO_SCRATCH_BIT0_7      0x0000000000000080UL
#define IIO_SCRATCH_BIT0_8      0x0000000000000100UL
#define IIO_SCRATCH_BIT0_9      0x0000000000000200UL
#define IIO_SCRATCH_BIT0_A      0x0000000000000400UL

#define IIO_SCRATCH_BIT1_0      0x0000000000000001UL
#define IIO_SCRATCH_BIT1_1      0x0000000000000002UL
/* IO Translation Table Entries */
#define IIO_NUM_ITTES   7	/* ITTEs numbered 0..6 */
					/* Hw manuals number them 1..7! */
#define IIO_IMEM_W0ESD  0x1UL	/* Widget 0 shut down due to error */
#define IIO_IMEM_B0ESD	(1UL << 4)	/* BTE 0 shut down due to error */
#define IIO_IMEM_B1ESD	(1UL << 8)	/* BTE 1 Shut down due to error */

#define HUB_NUM_BIG_WINDOW	(IIO_NUM_ITTES - 1)

#define SWIN0_BIGWIN            HUB_NUM_BIG_WINDOW

#define ILCSR_WARM_RESET        0x100

#define IIO_NUM_CRBS            15	/* Number of CRBs */
#define IIO_NUM_PC_CRBS         4	/* Number of partial cache CRBs */
#define IIO_ICRB_OFFSET         8
#define IIO_ICRB_0              IIO_ICRB0_A
#define IIO_ICRB_ADDR_SHFT	2	/* Shift to get proper address */

#define IIO_ICRB_A(_x)	((u64)(IIO_ICRB_0 + (6 * IIO_ICRB_OFFSET * (_x))))
#define IIO_ICRB_B(_x)	((u64)((char *)IIO_ICRB_A(_x) + 1*IIO_ICRB_OFFSET))
#define IIO_ICRB_C(_x)	((u64)((char *)IIO_ICRB_A(_x) + 2*IIO_ICRB_OFFSET))
#define IIO_ICRB_D(_x)	((u64)((char *)IIO_ICRB_A(_x) + 3*IIO_ICRB_OFFSET))
#define IIO_ICRB_E(_x)	((u64)((char *)IIO_ICRB_A(_x) + 4*IIO_ICRB_OFFSET))

#define TNUM_TO_WIDGET_DEV(_tnum)	(_tnum & 0x7)

#define IIO_ICRB_ECODE_DERR     0	/* Directory error due to IIO access */
#define IIO_ICRB_ECODE_PERR     1	/* Poison error on IO access */
#define IIO_ICRB_ECODE_WERR     2	/* Write error by IIO access
					 * e.g. WINV to a Read only line. */
#define IIO_ICRB_ECODE_AERR     3	/* Access error caused by IIO access */
#define IIO_ICRB_ECODE_PWERR    4	/* Error on partial write */
#define IIO_ICRB_ECODE_PRERR    5	/* Error on partial read  */
#define IIO_ICRB_ECODE_TOUT     6	/* CRB timeout before deallocating */
#define IIO_ICRB_ECODE_XTERR    7	/* Incoming xtalk pkt had error bit */

#define IIO_ICRB_IMSGT_XTALK    0	/* Incoming Meessage from Xtalk */
#define IIO_ICRB_IMSGT_BTE      1	/* Incoming message from BTE    */
#define IIO_ICRB_IMSGT_SN1NET   2	/* Incoming message from SN1 net */
#define IIO_ICRB_IMSGT_CRB      3	/* Incoming message from CRB ???  */

#define IIO_ICRB_INIT_XTALK     0	/* Message originated in xtalk  */
#define IIO_ICRB_INIT_BTE0      0x1	/* Message originated in BTE 0  */
#define IIO_ICRB_INIT_SN1NET    0x2	/* Message originated in SN1net */
#define IIO_ICRB_INIT_CRB       0x3	/* Message originated in CRB ?  */
#define IIO_ICRB_INIT_BTE1      0x5	/* MEssage originated in BTE 1  */

#define		   HUBII_XBOW_CREDIT       3
#define		   HUBII_XBOW_REV2_CREDIT  4

#define HUB_CREDIT 4

#define IIO_PRB_MULTI_ERR	(1LL << 63)
#define IIO_PRB_SPUR_RD		(1LL << 51)
#define IIO_PRB_SPUR_WR		(1LL << 50)
#define IIO_PRB_RD_TO		(1LL << 49)
#define IIO_PRB_ERROR		(1LL << 48)



#define IIO_ICMR_CRB_VLD_SHFT   20
#define IIO_ICMR_CRB_VLD_MASK	(0x7fffUL << IIO_ICMR_CRB_VLD_SHFT)

#define IIO_ICMR_FC_CNT_SHFT    16
#define IIO_ICMR_FC_CNT_MASK	(0xf << IIO_ICMR_FC_CNT_SHFT)

#define IIO_ICMR_C_CNT_SHFT     4
#define IIO_ICMR_C_CNT_MASK	(0xf << IIO_ICMR_C_CNT_SHFT)

#define IIO_ICMR_PRECISE	(1UL << 52)
#define IIO_ICMR_CLR_RPPD	(1UL << 13)
#define IIO_ICMR_CLR_RQPD	(1UL << 12)

#define IIO_IPDR_PND    	(1 << 4)

#define IIO_ICDR_PND    	(1 << 4)

#define IBLS_BUSY		(0x1UL << 20)
#define IBLS_ERROR_SHFT		16
#define IBLS_ERROR		(0x1UL << IBLS_ERROR_SHFT)
#define IBLS_LENGTH_MASK	0xffff

#define IBCT_POISON		(0x1UL << 8)
#define IBCT_NOTIFY		(0x1UL << 4)
#define IBCT_ZFIL_MODE		(0x1UL << 0)

#define IIEPH1_VALID		(1UL << 44)
#define IIEPH1_OVERRUN		(1UL << 40)
#define IIEPH1_ERR_TYPE_SHFT	32
#define IIEPH1_ERR_TYPE_MASK	0xf
#define IIEPH1_SOURCE_SHFT	20
#define IIEPH1_SOURCE_MASK	11
#define IIEPH1_SUPPL_SHFT	8
#define IIEPH1_SUPPL_MASK	11
#define IIEPH1_CMD_SHFT		0
#define IIEPH1_CMD_MASK		7

#define IIEPH2_TAIL		(1UL << 40)
#define IIEPH2_ADDRESS_SHFT	0
#define IIEPH2_ADDRESS_MASK	38

#define IIEPH1_ERR_SHORT_REQ	2
#define IIEPH1_ERR_SHORT_REPLY	3
#define IIEPH1_ERR_LONG_REQ	4
#define IIEPH1_ERR_LONG_REPLY	5

#define IECLR_PI1_FWD_INT	(1UL << 31)	/* clear PI1_FORWARD_INT in iidsr */
#define IECLR_PI0_FWD_INT	(1UL << 30)	/* clear PI0_FORWARD_INT in iidsr */
#define IECLR_SPUR_RD_HDR	(1UL << 29)	/* clear valid bit in ixss reg */
#define IECLR_BTE1		(1UL << 18)	/* clear bte error 1 */
#define IECLR_BTE0		(1UL << 17)	/* clear bte error 0 */
#define IECLR_CRAZY		(1UL << 16)	/* clear crazy bit in wstat reg */
#define IECLR_PRB_F		(1UL << 15)	/* clear err bit in PRB_F reg */
#define IECLR_PRB_E		(1UL << 14)	/* clear err bit in PRB_E reg */
#define IECLR_PRB_D		(1UL << 13)	/* clear err bit in PRB_D reg */
#define IECLR_PRB_C		(1UL << 12)	/* clear err bit in PRB_C reg */
#define IECLR_PRB_B		(1UL << 11)	/* clear err bit in PRB_B reg */
#define IECLR_PRB_A		(1UL << 10)	/* clear err bit in PRB_A reg */
#define IECLR_PRB_9		(1UL << 9)	/* clear err bit in PRB_9 reg */
#define IECLR_PRB_8		(1UL << 8)	/* clear err bit in PRB_8 reg */
#define IECLR_PRB_0		(1UL << 0)	/* clear err bit in PRB_0 reg */

#define	IIO_ICCR_PENDING	0x10000
#define	IIO_ICCR_CMD_MASK	0xFF
#define	IIO_ICCR_CMD_SHFT	7
#define	IIO_ICCR_CMD_NOP	0x0	/* No Op */
#define	IIO_ICCR_CMD_WAKE	0x100	/* Reactivate CRB entry and process */
#define	IIO_ICCR_CMD_TIMEOUT	0x200	/* Make CRB timeout & mark invalid */
#define	IIO_ICCR_CMD_EJECT	0x400	/* Contents of entry written to memory
					 * via a WB
					 */
#define	IIO_ICCR_CMD_FLUSH	0x800


typedef ii_icrb0_a_u_t icrba_t;
#define a_sidn		ii_icrb0_a_fld_s.ia_sidn
#define a_tnum		ii_icrb0_a_fld_s.ia_tnum
#define a_addr          ii_icrb0_a_fld_s.ia_addr
#define a_valid         ii_icrb0_a_fld_s.ia_vld
#define a_iow           ii_icrb0_a_fld_s.ia_iow
#define a_regvalue	ii_icrb0_a_regval

typedef ii_icrb0_b_u_t icrbb_t;
#define b_use_old       ii_icrb0_b_fld_s.ib_use_old
#define b_imsgtype      ii_icrb0_b_fld_s.ib_imsgtype
#define b_imsg          ii_icrb0_b_fld_s.ib_imsg
#define b_initiator     ii_icrb0_b_fld_s.ib_init
#define b_exc           ii_icrb0_b_fld_s.ib_exc
#define b_ackcnt        ii_icrb0_b_fld_s.ib_ack_cnt
#define b_resp          ii_icrb0_b_fld_s.ib_resp
#define b_ack           ii_icrb0_b_fld_s.ib_ack
#define b_hold          ii_icrb0_b_fld_s.ib_hold
#define b_wb            ii_icrb0_b_fld_s.ib_wb
#define b_intvn         ii_icrb0_b_fld_s.ib_intvn
#define b_stall_ib      ii_icrb0_b_fld_s.ib_stall_ib
#define b_stall_int     ii_icrb0_b_fld_s.ib_stall__intr
#define b_stall_bte_0   ii_icrb0_b_fld_s.ib_stall__bte_0
#define b_stall_bte_1   ii_icrb0_b_fld_s.ib_stall__bte_1
#define b_error         ii_icrb0_b_fld_s.ib_error
#define b_ecode         ii_icrb0_b_fld_s.ib_errcode
#define b_lnetuce       ii_icrb0_b_fld_s.ib_ln_uce
#define b_mark          ii_icrb0_b_fld_s.ib_mark
#define b_xerr          ii_icrb0_b_fld_s.ib_xt_err
#define b_regvalue	ii_icrb0_b_regval

typedef ii_icrb0_c_u_t icrbc_t;
#define c_suppl         ii_icrb0_c_fld_s.ic_suppl
#define c_barrop        ii_icrb0_c_fld_s.ic_bo
#define c_doresp        ii_icrb0_c_fld_s.ic_resprqd
#define c_gbr           ii_icrb0_c_fld_s.ic_gbr
#define c_btenum        ii_icrb0_c_fld_s.ic_bte_num
#define c_cohtrans      ii_icrb0_c_fld_s.ic_ct
#define c_xtsize        ii_icrb0_c_fld_s.ic_size
#define c_source        ii_icrb0_c_fld_s.ic_source
#define c_regvalue	ii_icrb0_c_regval

typedef ii_icrb0_d_u_t icrbd_t;
#define d_sleep         ii_icrb0_d_fld_s.id_sleep
#define d_pricnt        ii_icrb0_d_fld_s.id_pr_cnt
#define d_pripsc        ii_icrb0_d_fld_s.id_pr_psc
#define d_bteop         ii_icrb0_d_fld_s.id_bte_op
#define d_bteaddr       ii_icrb0_d_fld_s.id_pa_be	/* ic_pa_be fld has 2 names */
#define d_benable       ii_icrb0_d_fld_s.id_pa_be	/* ic_pa_be fld has 2 names */
#define d_regvalue	ii_icrb0_d_regval

typedef ii_icrb0_e_u_t icrbe_t;
#define icrbe_ctxtvld   ii_icrb0_e_fld_s.ie_cvld
#define icrbe_toutvld   ii_icrb0_e_fld_s.ie_tvld
#define icrbe_context   ii_icrb0_e_fld_s.ie_context
#define icrbe_timeout   ii_icrb0_e_fld_s.ie_timeout
#define e_regvalue	ii_icrb0_e_regval

/* Number of widgets supported by shub */
#define HUB_NUM_WIDGET          9
#define HUB_WIDGET_ID_MIN       0x8
#define HUB_WIDGET_ID_MAX       0xf

#define HUB_WIDGET_PART_NUM     0xc120
#define MAX_HUBS_PER_XBOW       2

/* A few more #defines for backwards compatibility */
#define iprb_t          ii_iprb0_u_t
#define iprb_regval     ii_iprb0_regval
#define iprb_mult_err	ii_iprb0_fld_s.i_mult_err
#define iprb_spur_rd	ii_iprb0_fld_s.i_spur_rd
#define iprb_spur_wr	ii_iprb0_fld_s.i_spur_wr
#define iprb_rd_to	ii_iprb0_fld_s.i_rd_to
#define iprb_ovflow     ii_iprb0_fld_s.i_of_cnt
#define iprb_error      ii_iprb0_fld_s.i_error
#define iprb_ff         ii_iprb0_fld_s.i_f
#define iprb_mode       ii_iprb0_fld_s.i_m
#define iprb_bnakctr    ii_iprb0_fld_s.i_nb
#define iprb_anakctr    ii_iprb0_fld_s.i_na
#define iprb_xtalkctr   ii_iprb0_fld_s.i_c

#define LNK_STAT_WORKING        0x2		/* LLP is working */

#define IIO_WSTAT_ECRAZY	(1ULL << 32)	/* Hub gone crazy */
#define IIO_WSTAT_TXRETRY	(1ULL << 9)	/* Hub Tx Retry timeout */
#define IIO_WSTAT_TXRETRY_MASK  0x7F		/* should be 0xFF?? */
#define IIO_WSTAT_TXRETRY_SHFT  16
#define IIO_WSTAT_TXRETRY_CNT(w)	(((w) >> IIO_WSTAT_TXRETRY_SHFT) & \
                          		IIO_WSTAT_TXRETRY_MASK)

/* Number of II perf. counters we can multiplex at once */

#define IO_PERF_SETS	32

/* Bit for the widget in inbound access register */
#define IIO_IIWA_WIDGET(_w)	((u64)(1ULL << _w))
/* Bit for the widget in outbound access register */
#define IIO_IOWA_WIDGET(_w)	((u64)(1ULL << _w))

#define IIO_IIDEM_WIDGETDEV_MASK(w, d)	((u64)(1ULL << (8 * ((w) - 8) + (d))))

/* IO Interrupt Destination Register */
#define IIO_IIDSR_SENT_SHIFT    28
#define IIO_IIDSR_SENT_MASK     0x30000000
#define IIO_IIDSR_ENB_SHIFT     24
#define IIO_IIDSR_ENB_MASK      0x01000000
#define IIO_IIDSR_NODE_SHIFT    9
#define IIO_IIDSR_NODE_MASK     0x000ff700
#define IIO_IIDSR_PI_ID_SHIFT   8
#define IIO_IIDSR_PI_ID_MASK    0x00000100
#define IIO_IIDSR_LVL_SHIFT     0
#define IIO_IIDSR_LVL_MASK      0x000000ff

/* Xtalk timeout threshold register (IIO_IXTT) */
#define IXTT_RRSP_TO_SHFT	55	/* read response timeout */
#define IXTT_RRSP_TO_MASK	(0x1FULL << IXTT_RRSP_TO_SHFT)
#define IXTT_RRSP_PS_SHFT	32	/* read responsed TO prescalar */
#define IXTT_RRSP_PS_MASK	(0x7FFFFFULL << IXTT_RRSP_PS_SHFT)
#define IXTT_TAIL_TO_SHFT	0	/* tail timeout counter threshold */
#define IXTT_TAIL_TO_MASK	(0x3FFFFFFULL << IXTT_TAIL_TO_SHFT)


typedef union hubii_wcr_u {
	u64 wcr_reg_value;
	struct {
		u64 wcr_widget_id:4,	/* LLP crossbar credit */
		 wcr_tag_mode:1,	/* Tag mode */
		 wcr_rsvd1:8,	/* Reserved */
		 wcr_xbar_crd:3,	/* LLP crossbar credit */
		 wcr_f_bad_pkt:1,	/* Force bad llp pkt enable */
		 wcr_dir_con:1,	/* widget direct connect */
		 wcr_e_thresh:5,	/* elasticity threshold */
		 wcr_rsvd:41;	/* unused */
	} wcr_fields_s;
} hubii_wcr_t;

#define iwcr_dir_con    wcr_fields_s.wcr_dir_con



typedef union io_perf_sel {
	u64 perf_sel_reg;
	struct {
		u64 perf_ippr0:4, perf_ippr1:4, perf_icct:8, perf_rsvd:48;
	} perf_sel_bits;
} io_perf_sel_t;


typedef union io_perf_cnt {
	u64 perf_cnt;
	struct {
		u64 perf_cnt:20, perf_rsvd2:12, perf_rsvd1:32;
	} perf_cnt_bits;

} io_perf_cnt_t;

typedef union iprte_a {
	u64 entry;
	struct {
		u64 i_rsvd_1:3;
		u64 i_addr:38;
		u64 i_init:3;
		u64 i_source:8;
		u64 i_rsvd:2;
		u64 i_widget:4;
		u64 i_to_cnt:5;
		u64 i_vld:1;
	} iprte_fields;
} iprte_a_t;

#endif				/* _ASM_IA64_SN_SHUBIO_H */
