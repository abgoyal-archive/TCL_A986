


#ifndef BPF_MAJOR_VERSION

#ifdef __cplusplus
extern "C" {
#endif

/* BSD style release date */
#define BPF_RELEASE 199606

#ifdef MSDOS /* must be 32-bit */
typedef long          bpf_int32;
typedef unsigned long bpf_u_int32;
#else
typedef	int bpf_int32;
typedef	u_int bpf_u_int32;
#endif

#ifndef __NetBSD__
#define BPF_ALIGNMENT sizeof(bpf_int32)
#else
#define BPF_ALIGNMENT sizeof(long)
#endif
#define BPF_WORDALIGN(x) (((x)+(BPF_ALIGNMENT-1))&~(BPF_ALIGNMENT-1))

#define BPF_MAXINSNS 512
#define BPF_MAXBUFSIZE 0x8000
#define BPF_MINBUFSIZE 32

struct bpf_program {
	u_int bf_len;
	struct bpf_insn *bf_insns;
};
 
struct bpf_version {
	u_short bv_major;
	u_short bv_minor;
};
/* Current version number of filter architecture. */
#define BPF_MAJOR_VERSION 1
#define BPF_MINOR_VERSION 1


#define DLT_NULL	0	/* BSD loopback encapsulation */
#define DLT_EN10MB	1	/* Ethernet (10Mb) */
#define DLT_EN3MB	2	/* Experimental Ethernet (3Mb) */
#define DLT_AX25	3	/* Amateur Radio AX.25 */
#define DLT_PRONET	4	/* Proteon ProNET Token Ring */
#define DLT_CHAOS	5	/* Chaos */
#define DLT_IEEE802	6	/* 802.5 Token Ring */
#define DLT_ARCNET	7	/* ARCNET, with BSD-style header */
#define DLT_SLIP	8	/* Serial Line IP */
#define DLT_PPP		9	/* Point-to-point Protocol */
#define DLT_FDDI	10	/* FDDI */

#define DLT_ATM_RFC1483	11	/* LLC-encapsulated ATM */

#ifdef __OpenBSD__
#define DLT_RAW		14	/* raw IP */
#else
#define DLT_RAW		12	/* raw IP */
#endif

#if defined(__NetBSD__) || defined(__FreeBSD__)
#ifndef DLT_SLIP_BSDOS
#define DLT_SLIP_BSDOS	13	/* BSD/OS Serial Line IP */
#define DLT_PPP_BSDOS	14	/* BSD/OS Point-to-point Protocol */
#endif
#else
#define DLT_SLIP_BSDOS	15	/* BSD/OS Serial Line IP */
#define DLT_PPP_BSDOS	16	/* BSD/OS Point-to-point Protocol */
#endif


#define DLT_ATM_CLIP	19	/* Linux Classical-IP over ATM */

#define DLT_REDBACK_SMARTEDGE	32

#define DLT_PPP_SERIAL	50	/* PPP over serial with HDLC encapsulation */
#define DLT_PPP_ETHER	51	/* PPP over Ethernet */

#define DLT_SYMANTEC_FIREWALL	99


#define DLT_C_HDLC	104	/* Cisco HDLC */
#define DLT_CHDLC	DLT_C_HDLC

#define DLT_IEEE802_11	105	/* IEEE 802.11 wireless */


#define DLT_FRELAY	107

#ifdef __OpenBSD__
#define DLT_LOOP	12
#else
#define DLT_LOOP	108
#endif

#ifdef __OpenBSD__
#define DLT_ENC		13
#else
#define DLT_ENC		109
#endif


#define DLT_LINUX_SLL	113

#define DLT_LTALK	114

#define DLT_ECONET	115

#define DLT_IPFILTER	116

#ifdef __OpenBSD__
#define DLT_OLD_PFLOG	17
#define DLT_PFSYNC	18
#endif
#define DLT_PFLOG	117

#define DLT_CISCO_IOS	118

#define DLT_PRISM_HEADER	119

#define DLT_AIRONET_HEADER	120

#define DLT_HHDLC		121

#define DLT_IP_OVER_FC		122

#define DLT_SUNATM		123	/* Solaris+SunATM */

#define DLT_RIO                 124     /* RapidIO */
#define DLT_PCI_EXP             125     /* PCI Express */
#define DLT_AURORA              126     /* Xilinx Aurora link layer */

#define DLT_IEEE802_11_RADIO	127	/* 802.11 plus radiotap radio header */

#define DLT_TZSP                128     /* Tazmen Sniffer Protocol */

#define DLT_ARCNET_LINUX	129	/* ARCNET */

#define DLT_JUNIPER_MLPPP       130
#define DLT_JUNIPER_MLFR        131
#define DLT_JUNIPER_ES          132
#define DLT_JUNIPER_GGSN        133
#define DLT_JUNIPER_MFR         134
#define DLT_JUNIPER_ATM2        135
#define DLT_JUNIPER_SERVICES    136
#define DLT_JUNIPER_ATM1        137

#define DLT_APPLE_IP_OVER_IEEE1394	138

#define DLT_MTP2_WITH_PHDR	139	/* pseudo-header with various info, followed by MTP2 */
#define DLT_MTP2		140	/* MTP2, without pseudo-header */
#define DLT_MTP3		141	/* MTP3, without pseudo-header or MTP2 */
#define DLT_SCCP		142	/* SCCP, without pseudo-header or MTP2 or MTP3 */

#define DLT_DOCSIS		143

#define DLT_LINUX_IRDA		144

#define DLT_IBM_SP		145
#define DLT_IBM_SN		146

#define DLT_USER0		147
#define DLT_USER1		148
#define DLT_USER2		149
#define DLT_USER3		150
#define DLT_USER4		151
#define DLT_USER5		152
#define DLT_USER6		153
#define DLT_USER7		154
#define DLT_USER8		155
#define DLT_USER9		156
#define DLT_USER10		157
#define DLT_USER11		158
#define DLT_USER12		159
#define DLT_USER13		160
#define DLT_USER14		161
#define DLT_USER15		162

#define DLT_IEEE802_11_RADIO_AVS 163	/* 802.11 plus AVS radio header */

#define DLT_JUNIPER_MONITOR     164

#define DLT_BACNET_MS_TP	165

#define DLT_PPP_PPPD		166

#define DLT_PPP_WITH_DIRECTION	DLT_PPP_PPPD
#define DLT_LINUX_PPP_WITHDIRECTION	DLT_PPP_PPPD

#define DLT_JUNIPER_PPPOE       167
#define DLT_JUNIPER_PPPOE_ATM   168

#define DLT_GPRS_LLC		169	/* GPRS LLC */
#define DLT_GPF_T		170	/* GPF-T (ITU-T G.7041/Y.1303) */
#define DLT_GPF_F		171	/* GPF-F (ITU-T G.7041/Y.1303) */

#define DLT_GCOM_T1E1		172
#define DLT_GCOM_SERIAL		173

#define DLT_JUNIPER_PIC_PEER    174

#define DLT_ERF_ETH		175	/* Ethernet */
#define DLT_ERF_POS		176	/* Packet-over-SONET */

#define DLT_LINUX_LAPD		177

#define DLT_JUNIPER_ETHER       178
#define DLT_JUNIPER_PPP         179
#define DLT_JUNIPER_FRELAY      180
#define DLT_JUNIPER_CHDLC       181

#define DLT_MFR                 182

#define DLT_JUNIPER_VP          183

#define DLT_A429                184

#define DLT_A653_ICM            185

#define DLT_USB			186

#define DLT_BLUETOOTH_HCI_H4	187

#define DLT_IEEE802_16_MAC_CPS	188

#define DLT_USB_LINUX		189

#define DLT_CAN20B              190

#define DLT_IEEE802_15_4_LINUX	191

#define DLT_PPI			192

#define DLT_IEEE802_16_MAC_CPS_RADIO	193

#define DLT_JUNIPER_ISM         194

#define DLT_IEEE802_15_4	195

#define DLT_SITA		196

#define DLT_ERF			197

#define DLT_RAIF1		198

#define DLT_IPMB		199

#define DLT_JUNIPER_ST          200

#define DLT_BLUETOOTH_HCI_H4_WITH_PHDR	201


/* instruction classes */
#define BPF_CLASS(code) ((code) & 0x07)
#define		BPF_LD		0x00
#define		BPF_LDX		0x01
#define		BPF_ST		0x02
#define		BPF_STX		0x03
#define		BPF_ALU		0x04
#define		BPF_JMP		0x05
#define		BPF_RET		0x06
#define		BPF_MISC	0x07

/* ld/ldx fields */
#define BPF_SIZE(code)	((code) & 0x18)
#define		BPF_W		0x00
#define		BPF_H		0x08
#define		BPF_B		0x10
#define BPF_MODE(code)	((code) & 0xe0)
#define		BPF_IMM 	0x00
#define		BPF_ABS		0x20
#define		BPF_IND		0x40
#define		BPF_MEM		0x60
#define		BPF_LEN		0x80
#define		BPF_MSH		0xa0

/* alu/jmp fields */
#define BPF_OP(code)	((code) & 0xf0)
#define		BPF_ADD		0x00
#define		BPF_SUB		0x10
#define		BPF_MUL		0x20
#define		BPF_DIV		0x30
#define		BPF_OR		0x40
#define		BPF_AND		0x50
#define		BPF_LSH		0x60
#define		BPF_RSH		0x70
#define		BPF_NEG		0x80
#define		BPF_JA		0x00
#define		BPF_JEQ		0x10
#define		BPF_JGT		0x20
#define		BPF_JGE		0x30
#define		BPF_JSET	0x40
#define BPF_SRC(code)	((code) & 0x08)
#define		BPF_K		0x00
#define		BPF_X		0x08

/* ret - BPF_K and BPF_X also apply */
#define BPF_RVAL(code)	((code) & 0x18)
#define		BPF_A		0x10

/* misc */
#define BPF_MISCOP(code) ((code) & 0xf8)
#define		BPF_TAX		0x00
#define		BPF_TXA		0x80

struct bpf_insn {
	u_short	code;
	u_char 	jt;
	u_char 	jf;
	bpf_int32 k;
};

#define BPF_STMT(code, k) { (u_short)(code), 0, 0, k }
#define BPF_JUMP(code, k, jt, jf) { (u_short)(code), jt, jf, k }

#if __STDC__ || defined(__cplusplus)
extern int bpf_validate(struct bpf_insn *, int);
extern u_int bpf_filter(struct bpf_insn *, u_char *, u_int, u_int);
#else
extern int bpf_validate();
extern u_int bpf_filter();
#endif

#define BPF_MEMWORDS 16

#ifdef __cplusplus
}
#endif

#endif
