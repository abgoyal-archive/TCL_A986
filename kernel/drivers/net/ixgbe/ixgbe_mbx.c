

#include <linux/pci.h>
#include <linux/delay.h>
#include "ixgbe_type.h"
#include "ixgbe_common.h"
#include "ixgbe_mbx.h"

s32 ixgbe_read_mbx(struct ixgbe_hw *hw, u32 *msg, u16 size, u16 mbx_id)
{
	struct ixgbe_mbx_info *mbx = &hw->mbx;
	s32 ret_val = IXGBE_ERR_MBX;

	/* limit read to size of mailbox */
	if (size > mbx->size)
		size = mbx->size;

	if (mbx->ops.read)
		ret_val = mbx->ops.read(hw, msg, size, mbx_id);

	return ret_val;
}

s32 ixgbe_write_mbx(struct ixgbe_hw *hw, u32 *msg, u16 size, u16 mbx_id)
{
	struct ixgbe_mbx_info *mbx = &hw->mbx;
	s32 ret_val = 0;

	if (size > mbx->size)
		ret_val = IXGBE_ERR_MBX;

	else if (mbx->ops.write)
		ret_val = mbx->ops.write(hw, msg, size, mbx_id);

	return ret_val;
}

s32 ixgbe_check_for_msg(struct ixgbe_hw *hw, u16 mbx_id)
{
	struct ixgbe_mbx_info *mbx = &hw->mbx;
	s32 ret_val = IXGBE_ERR_MBX;

	if (mbx->ops.check_for_msg)
		ret_val = mbx->ops.check_for_msg(hw, mbx_id);

	return ret_val;
}

s32 ixgbe_check_for_ack(struct ixgbe_hw *hw, u16 mbx_id)
{
	struct ixgbe_mbx_info *mbx = &hw->mbx;
	s32 ret_val = IXGBE_ERR_MBX;

	if (mbx->ops.check_for_ack)
		ret_val = mbx->ops.check_for_ack(hw, mbx_id);

	return ret_val;
}

s32 ixgbe_check_for_rst(struct ixgbe_hw *hw, u16 mbx_id)
{
	struct ixgbe_mbx_info *mbx = &hw->mbx;
	s32 ret_val = IXGBE_ERR_MBX;

	if (mbx->ops.check_for_rst)
		ret_val = mbx->ops.check_for_rst(hw, mbx_id);

	return ret_val;
}

static s32 ixgbe_poll_for_msg(struct ixgbe_hw *hw, u16 mbx_id)
{
	struct ixgbe_mbx_info *mbx = &hw->mbx;
	int countdown = mbx->timeout;

	if (!countdown || !mbx->ops.check_for_msg)
		goto out;

	while (countdown && mbx->ops.check_for_msg(hw, mbx_id)) {
		countdown--;
		if (!countdown)
			break;
		udelay(mbx->usec_delay);
	}

	/* if we failed, all future posted messages fail until reset */
	if (!countdown)
		mbx->timeout = 0;
out:
	return countdown ? 0 : IXGBE_ERR_MBX;
}

static s32 ixgbe_poll_for_ack(struct ixgbe_hw *hw, u16 mbx_id)
{
	struct ixgbe_mbx_info *mbx = &hw->mbx;
	int countdown = mbx->timeout;

	if (!countdown || !mbx->ops.check_for_ack)
		goto out;

	while (countdown && mbx->ops.check_for_ack(hw, mbx_id)) {
		countdown--;
		if (!countdown)
			break;
		udelay(mbx->usec_delay);
	}

	/* if we failed, all future posted messages fail until reset */
	if (!countdown)
		mbx->timeout = 0;
out:
	return countdown ? 0 : IXGBE_ERR_MBX;
}

s32 ixgbe_read_posted_mbx(struct ixgbe_hw *hw, u32 *msg, u16 size, u16 mbx_id)
{
	struct ixgbe_mbx_info *mbx = &hw->mbx;
	s32 ret_val = IXGBE_ERR_MBX;

	if (!mbx->ops.read)
		goto out;

	ret_val = ixgbe_poll_for_msg(hw, mbx_id);

	/* if ack received read message, otherwise we timed out */
	if (!ret_val)
		ret_val = mbx->ops.read(hw, msg, size, mbx_id);
out:
	return ret_val;
}

s32 ixgbe_write_posted_mbx(struct ixgbe_hw *hw, u32 *msg, u16 size,
                           u16 mbx_id)
{
	struct ixgbe_mbx_info *mbx = &hw->mbx;
	s32 ret_val = IXGBE_ERR_MBX;

	/* exit if either we can't write or there isn't a defined timeout */
	if (!mbx->ops.write || !mbx->timeout)
		goto out;

	/* send msg */
	ret_val = mbx->ops.write(hw, msg, size, mbx_id);

	/* if msg sent wait until we receive an ack */
	if (!ret_val)
		ret_val = ixgbe_poll_for_ack(hw, mbx_id);
out:
	return ret_val;
}

void ixgbe_init_mbx_ops_generic(struct ixgbe_hw *hw)
{
	struct ixgbe_mbx_info *mbx = &hw->mbx;

	mbx->ops.read_posted = ixgbe_read_posted_mbx;
	mbx->ops.write_posted = ixgbe_write_posted_mbx;
}

static s32 ixgbe_check_for_bit_pf(struct ixgbe_hw *hw, u32 mask, s32 index)
{
	u32 mbvficr = IXGBE_READ_REG(hw, IXGBE_MBVFICR(index));
	s32 ret_val = IXGBE_ERR_MBX;

	if (mbvficr & mask) {
		ret_val = 0;
		IXGBE_WRITE_REG(hw, IXGBE_MBVFICR(index), mask);
	}

	return ret_val;
}

static s32 ixgbe_check_for_msg_pf(struct ixgbe_hw *hw, u16 vf_number)
{
	s32 ret_val = IXGBE_ERR_MBX;
	s32 index = IXGBE_MBVFICR_INDEX(vf_number);
	u32 vf_bit = vf_number % 16;

	if (!ixgbe_check_for_bit_pf(hw, IXGBE_MBVFICR_VFREQ_VF1 << vf_bit,
	                            index)) {
		ret_val = 0;
		hw->mbx.stats.reqs++;
	}

	return ret_val;
}

static s32 ixgbe_check_for_ack_pf(struct ixgbe_hw *hw, u16 vf_number)
{
	s32 ret_val = IXGBE_ERR_MBX;
	s32 index = IXGBE_MBVFICR_INDEX(vf_number);
	u32 vf_bit = vf_number % 16;

	if (!ixgbe_check_for_bit_pf(hw, IXGBE_MBVFICR_VFACK_VF1 << vf_bit,
	                            index)) {
		ret_val = 0;
		hw->mbx.stats.acks++;
	}

	return ret_val;
}

static s32 ixgbe_check_for_rst_pf(struct ixgbe_hw *hw, u16 vf_number)
{
	u32 reg_offset = (vf_number < 32) ? 0 : 1;
	u32 vf_shift = vf_number % 32;
	u32 vflre = 0;
	s32 ret_val = IXGBE_ERR_MBX;

	if (hw->mac.type == ixgbe_mac_82599EB)
		vflre = IXGBE_READ_REG(hw, IXGBE_VFLRE(reg_offset));

	if (vflre & (1 << vf_shift)) {
		ret_val = 0;
		IXGBE_WRITE_REG(hw, IXGBE_VFLREC(reg_offset), (1 << vf_shift));
		hw->mbx.stats.rsts++;
	}

	return ret_val;
}

static s32 ixgbe_obtain_mbx_lock_pf(struct ixgbe_hw *hw, u16 vf_number)
{
	s32 ret_val = IXGBE_ERR_MBX;
	u32 p2v_mailbox;

	/* Take ownership of the buffer */
	IXGBE_WRITE_REG(hw, IXGBE_PFMAILBOX(vf_number), IXGBE_PFMAILBOX_PFU);

	/* reserve mailbox for vf use */
	p2v_mailbox = IXGBE_READ_REG(hw, IXGBE_PFMAILBOX(vf_number));
	if (p2v_mailbox & IXGBE_PFMAILBOX_PFU)
		ret_val = 0;

	return ret_val;
}

static s32 ixgbe_write_mbx_pf(struct ixgbe_hw *hw, u32 *msg, u16 size,
                              u16 vf_number)
{
	s32 ret_val;
	u16 i;

	/* lock the mailbox to prevent pf/vf race condition */
	ret_val = ixgbe_obtain_mbx_lock_pf(hw, vf_number);
	if (ret_val)
		goto out_no_write;

	/* flush msg and acks as we are overwriting the message buffer */
	ixgbe_check_for_msg_pf(hw, vf_number);
	ixgbe_check_for_ack_pf(hw, vf_number);

	/* copy the caller specified message to the mailbox memory buffer */
	for (i = 0; i < size; i++)
		IXGBE_WRITE_REG_ARRAY(hw, IXGBE_PFMBMEM(vf_number), i, msg[i]);

	/* Interrupt VF to tell it a message has been sent and release buffer*/
	IXGBE_WRITE_REG(hw, IXGBE_PFMAILBOX(vf_number), IXGBE_PFMAILBOX_STS);

	/* update stats */
	hw->mbx.stats.msgs_tx++;

out_no_write:
	return ret_val;

}

static s32 ixgbe_read_mbx_pf(struct ixgbe_hw *hw, u32 *msg, u16 size,
                             u16 vf_number)
{
	s32 ret_val;
	u16 i;

	/* lock the mailbox to prevent pf/vf race condition */
	ret_val = ixgbe_obtain_mbx_lock_pf(hw, vf_number);
	if (ret_val)
		goto out_no_read;

	/* copy the message to the mailbox memory buffer */
	for (i = 0; i < size; i++)
		msg[i] = IXGBE_READ_REG_ARRAY(hw, IXGBE_PFMBMEM(vf_number), i);

	/* Acknowledge the message and release buffer */
	IXGBE_WRITE_REG(hw, IXGBE_PFMAILBOX(vf_number), IXGBE_PFMAILBOX_ACK);

	/* update stats */
	hw->mbx.stats.msgs_rx++;

out_no_read:
	return ret_val;
}

void ixgbe_init_mbx_params_pf(struct ixgbe_hw *hw)
{
	struct ixgbe_mbx_info *mbx = &hw->mbx;

	if (hw->mac.type != ixgbe_mac_82599EB)
		return;

	mbx->timeout = 0;
	mbx->usec_delay = 0;

	mbx->size = IXGBE_VFMAILBOX_SIZE;

	mbx->stats.msgs_tx = 0;
	mbx->stats.msgs_rx = 0;
	mbx->stats.reqs = 0;
	mbx->stats.acks = 0;
	mbx->stats.rsts = 0;
}

struct ixgbe_mbx_operations mbx_ops_82599 = {
	.read                   = ixgbe_read_mbx_pf,
	.write                  = ixgbe_write_mbx_pf,
	.read_posted            = ixgbe_read_posted_mbx,
	.write_posted           = ixgbe_write_posted_mbx,
	.check_for_msg          = ixgbe_check_for_msg_pf,
	.check_for_ack          = ixgbe_check_for_ack_pf,
	.check_for_rst          = ixgbe_check_for_rst_pf,
};

