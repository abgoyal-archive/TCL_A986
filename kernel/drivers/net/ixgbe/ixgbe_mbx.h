

#ifndef _IXGBE_MBX_H_
#define _IXGBE_MBX_H_

#include "ixgbe_type.h"

#define IXGBE_VFMAILBOX_SIZE        16 /* 16 32 bit words - 64 bytes */
#define IXGBE_ERR_MBX               -100

#define IXGBE_VFMAILBOX             0x002FC
#define IXGBE_VFMBMEM               0x00200

#define IXGBE_PFMAILBOX(x)          (0x04B00 + (4 * x))
#define IXGBE_PFMBMEM(vfn)          (0x13000 + (64 * vfn))

#define IXGBE_PFMAILBOX_STS   0x00000001 /* Initiate message send to VF */
#define IXGBE_PFMAILBOX_ACK   0x00000002 /* Ack message recv'd from VF */
#define IXGBE_PFMAILBOX_VFU   0x00000004 /* VF owns the mailbox buffer */
#define IXGBE_PFMAILBOX_PFU   0x00000008 /* PF owns the mailbox buffer */
#define IXGBE_PFMAILBOX_RVFU  0x00000010 /* Reset VFU - used when VF stuck */

#define IXGBE_MBVFICR_VFREQ_MASK 0x0000FFFF /* bits for VF messages */
#define IXGBE_MBVFICR_VFREQ_VF1  0x00000001 /* bit for VF 1 message */
#define IXGBE_MBVFICR_VFACK_MASK 0xFFFF0000 /* bits for VF acks */
#define IXGBE_MBVFICR_VFACK_VF1  0x00010000 /* bit for VF 1 ack */


#define IXGBE_VT_MSGTYPE_ACK      0x80000000  /* Messages below or'd with
                                               * this are the ACK */
#define IXGBE_VT_MSGTYPE_NACK     0x40000000  /* Messages below or'd with
                                               * this are the NACK */
#define IXGBE_VT_MSGTYPE_CTS      0x20000000  /* Indicates that VF is still
                                                 clear to send requests */
#define IXGBE_VT_MSGINFO_SHIFT    16
/* bits 23:16 are used for exra info for certain messages */
#define IXGBE_VT_MSGINFO_MASK     (0xFF << IXGBE_VT_MSGINFO_SHIFT)

#define IXGBE_VF_RESET            0x01 /* VF requests reset */
#define IXGBE_VF_SET_MAC_ADDR     0x02 /* VF requests PF to set MAC addr */
#define IXGBE_VF_SET_MULTICAST    0x03 /* VF requests PF to set MC addr */
#define IXGBE_VF_SET_VLAN         0x04 /* VF requests PF to set VLAN */
#define IXGBE_VF_SET_LPE          0x05 /* VF requests PF to set VMOLR.LPE */

/* length of permanent address message returned from PF */
#define IXGBE_VF_PERMADDR_MSG_LEN 4
/* word in permanent address message with the current multicast type */
#define IXGBE_VF_MC_TYPE_WORD     3

#define IXGBE_PF_CONTROL_MSG      0x0100 /* PF control message */

#define IXGBE_VF_MBX_INIT_TIMEOUT 2000 /* number of retries on mailbox */
#define IXGBE_VF_MBX_INIT_DELAY   500  /* microseconds between retries */

s32 ixgbe_read_mbx(struct ixgbe_hw *, u32 *, u16, u16);
s32 ixgbe_write_mbx(struct ixgbe_hw *, u32 *, u16, u16);
s32 ixgbe_read_posted_mbx(struct ixgbe_hw *, u32 *, u16, u16);
s32 ixgbe_write_posted_mbx(struct ixgbe_hw *, u32 *, u16, u16);
s32 ixgbe_check_for_msg(struct ixgbe_hw *, u16);
s32 ixgbe_check_for_ack(struct ixgbe_hw *, u16);
s32 ixgbe_check_for_rst(struct ixgbe_hw *, u16);
void ixgbe_init_mbx_ops_generic(struct ixgbe_hw *hw);
void ixgbe_init_mbx_params_pf(struct ixgbe_hw *);

extern struct ixgbe_mbx_operations mbx_ops_82599;

#endif /* _IXGBE_MBX_H_ */
