

#ifndef _ZCRYPT_PCICC_H_
#define _ZCRYPT_PCICC_H_

struct type6_hdr {
	unsigned char reserved1;	/* 0x00				*/
	unsigned char type;		/* 0x06				*/
	unsigned char reserved2[2];	/* 0x0000			*/
	unsigned char right[4];		/* 0x00000000			*/
	unsigned char reserved3[2];	/* 0x0000			*/
	unsigned char reserved4[2];	/* 0x0000			*/
	unsigned char apfs[4];		/* 0x00000000			*/
	unsigned int  offset1;		/* 0x00000058 (offset to CPRB)	*/
	unsigned int  offset2;		/* 0x00000000			*/
	unsigned int  offset3;		/* 0x00000000			*/
	unsigned int  offset4;		/* 0x00000000			*/
	unsigned char agent_id[16];	/* PCICC:			*/
					/*    0x0100			*/
					/*    0x4343412d4150504c202020	*/
					/*    0x010101			*/
					/* PCIXCC:			*/
					/*    0x4341000000000000	*/
					/*    0x0000000000000000	*/
	unsigned char rqid[2];		/* rqid.  internal to 603	*/
	unsigned char reserved5[2];	/* 0x0000			*/
	unsigned char function_code[2];	/* for PKD, 0x5044 (ascii 'PD')	*/
	unsigned char reserved6[2];	/* 0x0000			*/
	unsigned int  ToCardLen1;	/* (request CPRB len + 3) & -4	*/
	unsigned int  ToCardLen2;	/* db len 0x00000000 for PKD	*/
	unsigned int  ToCardLen3;	/* 0x00000000			*/
	unsigned int  ToCardLen4;	/* 0x00000000			*/
	unsigned int  FromCardLen1;	/* response buffer length	*/
	unsigned int  FromCardLen2;	/* db len 0x00000000 for PKD	*/
	unsigned int  FromCardLen3;	/* 0x00000000			*/
	unsigned int  FromCardLen4;	/* 0x00000000			*/
} __attribute__((packed));

struct CPRB {
	unsigned short cprb_len;	/* CPRB length			 */
	unsigned char cprb_ver_id;	/* CPRB version id.		 */
	unsigned char pad_000;		/* Alignment pad byte.		 */
	unsigned char srpi_rtcode[4];	/* SRPI return code LELONG	 */
	unsigned char srpi_verb;	/* SRPI verb type		 */
	unsigned char flags;		/* flags			 */
	unsigned char func_id[2];	/* function id			 */
	unsigned char checkpoint_flag;	/*				 */
	unsigned char resv2;		/* reserved			 */
	unsigned short req_parml;	/* request parameter buffer	 */
					/* length 16-bit little endian	 */
	unsigned char req_parmp[4];	/* request parameter buffer	 *
					 * pointer (means nothing: the	 *
					 * parameter buffer follows	 *
					 * the CPRB).			 */
	unsigned char req_datal[4];	/* request data buffer		 */
					/* length	  ULELONG	 */
	unsigned char req_datap[4];	/* request data buffer		 */
					/* pointer			 */
	unsigned short rpl_parml;	/* reply  parameter buffer	 */
					/* length 16-bit little endian	 */
	unsigned char pad_001[2];	/* Alignment pad bytes. ULESHORT */
	unsigned char rpl_parmp[4];	/* reply parameter buffer	 *
					 * pointer (means nothing: the	 *
					 * parameter buffer follows	 *
					 * the CPRB).			 */
	unsigned char rpl_datal[4];	/* reply data buffer len ULELONG */
	unsigned char rpl_datap[4];	/* reply data buffer		 */
					/* pointer			 */
	unsigned short ccp_rscode;	/* server reason code	ULESHORT */
	unsigned short ccp_rtcode;	/* server return code	ULESHORT */
	unsigned char repd_parml[2];	/* replied parameter len ULESHORT*/
	unsigned char mac_data_len[2];	/* Mac Data Length	ULESHORT */
	unsigned char repd_datal[4];	/* replied data length	ULELONG	 */
	unsigned char req_pc[2];	/* PC identifier		 */
	unsigned char res_origin[8];	/* resource origin		 */
	unsigned char mac_value[8];	/* Mac Value			 */
	unsigned char logon_id[8];	/* Logon Identifier		 */
	unsigned char usage_domain[2];	/* cdx				 */
	unsigned char resv3[18];	/* reserved for requestor	 */
	unsigned short svr_namel;	/* server name length  ULESHORT	 */
	unsigned char svr_name[8];	/* server name			 */
} __attribute__((packed));

struct type86_hdr {
	unsigned char reserved1;	/* 0x00				*/
	unsigned char type;		/* 0x86				*/
	unsigned char format;		/* 0x01 (error) or 0x02 (ok)	*/
	unsigned char reserved2;	/* 0x00				*/
	unsigned char reply_code;	/* reply code (see above)	*/
	unsigned char reserved3[3];	/* 0x000000			*/
} __attribute__((packed));

#define TYPE86_RSP_CODE 0x86
#define TYPE86_FMT2	0x02

struct type86_fmt2_ext {
	unsigned char	  reserved[4];	/* 0x00000000			*/
	unsigned char	  apfs[4];	/* final status			*/
	unsigned int	  count1;	/* length of CPRB + parameters	*/
	unsigned int	  offset1;	/* offset to CPRB		*/
	unsigned int	  count2;	/* 0x00000000			*/
	unsigned int	  offset2;	/* db offset 0x00000000 for PKD	*/
	unsigned int	  count3;	/* 0x00000000			*/
	unsigned int	  offset3;	/* 0x00000000			*/
	unsigned int	  count4;	/* 0x00000000			*/
	unsigned int	  offset4;	/* 0x00000000			*/
} __attribute__((packed));

struct function_and_rules_block {
	unsigned char function_code[2];
	unsigned short ulen;
	unsigned char only_rule[8];
} __attribute__((packed));

int zcrypt_pcicc_init(void);
void zcrypt_pcicc_exit(void);

#endif /* _ZCRYPT_PCICC_H_ */
