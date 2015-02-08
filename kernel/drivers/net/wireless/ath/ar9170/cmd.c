

#include "ar9170.h"
#include "cmd.h"

int ar9170_write_mem(struct ar9170 *ar, const __le32 *data, size_t len)
{
	int err;

	if (unlikely(!IS_ACCEPTING_CMD(ar)))
		return 0;

	err = ar->exec_cmd(ar, AR9170_CMD_WMEM, len, (u8 *) data, 0, NULL);
	if (err)
		printk(KERN_DEBUG "%s: writing memory failed\n",
		       wiphy_name(ar->hw->wiphy));
	return err;
}

int ar9170_write_reg(struct ar9170 *ar, const u32 reg, const u32 val)
{
	__le32 buf[2] = {
		cpu_to_le32(reg),
		cpu_to_le32(val),
	};
	int err;

	if (unlikely(!IS_ACCEPTING_CMD(ar)))
		return 0;

	err = ar->exec_cmd(ar, AR9170_CMD_WREG, sizeof(buf),
			   (u8 *) buf, 0, NULL);
	if (err)
		printk(KERN_DEBUG "%s: writing reg %#x (val %#x) failed\n",
		       wiphy_name(ar->hw->wiphy), reg, val);
	return err;
}

int ar9170_read_mreg(struct ar9170 *ar, int nregs, const u32 *regs, u32 *out)
{
	int i, err;
	__le32 *offs, *res;

	if (unlikely(!IS_ACCEPTING_CMD(ar)))
		return 0;

	/* abuse "out" for the register offsets, must be same length */
	offs = (__le32 *)out;
	for (i = 0; i < nregs; i++)
		offs[i] = cpu_to_le32(regs[i]);

	/* also use the same buffer for the input */
	res = (__le32 *)out;

	err = ar->exec_cmd(ar, AR9170_CMD_RREG,
			   4 * nregs, (u8 *)offs,
			   4 * nregs, (u8 *)res);
	if (err)
		return err;

	/* convert result to cpu endian */
	for (i = 0; i < nregs; i++)
		out[i] = le32_to_cpu(res[i]);

	return 0;
}

int ar9170_read_reg(struct ar9170 *ar, u32 reg, u32 *val)
{
	return ar9170_read_mreg(ar, 1, &reg, val);
}

int ar9170_echo_test(struct ar9170 *ar, u32 v)
{
	__le32 echobuf = cpu_to_le32(v);
	__le32 echores;
	int err;

	if (unlikely(!IS_ACCEPTING_CMD(ar)))
		return -ENODEV;

	err = ar->exec_cmd(ar, AR9170_CMD_ECHO,
			   4, (u8 *)&echobuf,
			   4, (u8 *)&echores);
	if (err)
		return err;

	if (echobuf != echores)
		return -EINVAL;

	return 0;
}
