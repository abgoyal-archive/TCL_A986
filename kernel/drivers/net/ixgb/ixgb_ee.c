

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "ixgb_hw.h"
#include "ixgb_ee.h"
/* Local prototypes */
static u16 ixgb_shift_in_bits(struct ixgb_hw *hw);

static void ixgb_shift_out_bits(struct ixgb_hw *hw,
				u16 data,
				u16 count);
static void ixgb_standby_eeprom(struct ixgb_hw *hw);

static bool ixgb_wait_eeprom_command(struct ixgb_hw *hw);

static void ixgb_cleanup_eeprom(struct ixgb_hw *hw);

static void
ixgb_raise_clock(struct ixgb_hw *hw,
		  u32 *eecd_reg)
{
	/* Raise the clock input to the EEPROM (by setting the SK bit), and then
	 *  wait 50 microseconds.
	 */
	*eecd_reg = *eecd_reg | IXGB_EECD_SK;
	IXGB_WRITE_REG(hw, EECD, *eecd_reg);
	udelay(50);
}

static void
ixgb_lower_clock(struct ixgb_hw *hw,
		  u32 *eecd_reg)
{
	/* Lower the clock input to the EEPROM (by clearing the SK bit), and then
	 * wait 50 microseconds.
	 */
	*eecd_reg = *eecd_reg & ~IXGB_EECD_SK;
	IXGB_WRITE_REG(hw, EECD, *eecd_reg);
	udelay(50);
}

static void
ixgb_shift_out_bits(struct ixgb_hw *hw,
					 u16 data,
					 u16 count)
{
	u32 eecd_reg;
	u32 mask;

	/* We need to shift "count" bits out to the EEPROM. So, value in the
	 * "data" parameter will be shifted out to the EEPROM one bit at a time.
	 * In order to do this, "data" must be broken down into bits.
	 */
	mask = 0x01 << (count - 1);
	eecd_reg = IXGB_READ_REG(hw, EECD);
	eecd_reg &= ~(IXGB_EECD_DO | IXGB_EECD_DI);
	do {
		/* A "1" is shifted out to the EEPROM by setting bit "DI" to a "1",
		 * and then raising and then lowering the clock (the SK bit controls
		 * the clock input to the EEPROM).  A "0" is shifted out to the EEPROM
		 * by setting "DI" to "0" and then raising and then lowering the clock.
		 */
		eecd_reg &= ~IXGB_EECD_DI;

		if (data & mask)
			eecd_reg |= IXGB_EECD_DI;

		IXGB_WRITE_REG(hw, EECD, eecd_reg);

		udelay(50);

		ixgb_raise_clock(hw, &eecd_reg);
		ixgb_lower_clock(hw, &eecd_reg);

		mask = mask >> 1;

	} while (mask);

	/* We leave the "DI" bit set to "0" when we leave this routine. */
	eecd_reg &= ~IXGB_EECD_DI;
	IXGB_WRITE_REG(hw, EECD, eecd_reg);
}

static u16
ixgb_shift_in_bits(struct ixgb_hw *hw)
{
	u32 eecd_reg;
	u32 i;
	u16 data;

	/* In order to read a register from the EEPROM, we need to shift 16 bits
	 * in from the EEPROM. Bits are "shifted in" by raising the clock input to
	 * the EEPROM (setting the SK bit), and then reading the value of the "DO"
	 * bit.  During this "shifting in" process the "DI" bit should always be
	 * clear..
	 */

	eecd_reg = IXGB_READ_REG(hw, EECD);

	eecd_reg &= ~(IXGB_EECD_DO | IXGB_EECD_DI);
	data = 0;

	for (i = 0; i < 16; i++) {
		data = data << 1;
		ixgb_raise_clock(hw, &eecd_reg);

		eecd_reg = IXGB_READ_REG(hw, EECD);

		eecd_reg &= ~(IXGB_EECD_DI);
		if (eecd_reg & IXGB_EECD_DO)
			data |= 1;

		ixgb_lower_clock(hw, &eecd_reg);
	}

	return data;
}

static void
ixgb_setup_eeprom(struct ixgb_hw *hw)
{
	u32 eecd_reg;

	eecd_reg = IXGB_READ_REG(hw, EECD);

	/*  Clear SK and DI  */
	eecd_reg &= ~(IXGB_EECD_SK | IXGB_EECD_DI);
	IXGB_WRITE_REG(hw, EECD, eecd_reg);

	/*  Set CS  */
	eecd_reg |= IXGB_EECD_CS;
	IXGB_WRITE_REG(hw, EECD, eecd_reg);
}

static void
ixgb_standby_eeprom(struct ixgb_hw *hw)
{
	u32 eecd_reg;

	eecd_reg = IXGB_READ_REG(hw, EECD);

	/*  Deselect EEPROM  */
	eecd_reg &= ~(IXGB_EECD_CS | IXGB_EECD_SK);
	IXGB_WRITE_REG(hw, EECD, eecd_reg);
	udelay(50);

	/*  Clock high  */
	eecd_reg |= IXGB_EECD_SK;
	IXGB_WRITE_REG(hw, EECD, eecd_reg);
	udelay(50);

	/*  Select EEPROM  */
	eecd_reg |= IXGB_EECD_CS;
	IXGB_WRITE_REG(hw, EECD, eecd_reg);
	udelay(50);

	/*  Clock low  */
	eecd_reg &= ~IXGB_EECD_SK;
	IXGB_WRITE_REG(hw, EECD, eecd_reg);
	udelay(50);
}

static void
ixgb_clock_eeprom(struct ixgb_hw *hw)
{
	u32 eecd_reg;

	eecd_reg = IXGB_READ_REG(hw, EECD);

	/*  Rising edge of clock  */
	eecd_reg |= IXGB_EECD_SK;
	IXGB_WRITE_REG(hw, EECD, eecd_reg);
	udelay(50);

	/*  Falling edge of clock  */
	eecd_reg &= ~IXGB_EECD_SK;
	IXGB_WRITE_REG(hw, EECD, eecd_reg);
	udelay(50);
}

static void
ixgb_cleanup_eeprom(struct ixgb_hw *hw)
{
	u32 eecd_reg;

	eecd_reg = IXGB_READ_REG(hw, EECD);

	eecd_reg &= ~(IXGB_EECD_CS | IXGB_EECD_DI);

	IXGB_WRITE_REG(hw, EECD, eecd_reg);

	ixgb_clock_eeprom(hw);
}

static bool
ixgb_wait_eeprom_command(struct ixgb_hw *hw)
{
	u32 eecd_reg;
	u32 i;

	/* Toggle the CS line.  This in effect tells to EEPROM to actually execute
	 * the command in question.
	 */
	ixgb_standby_eeprom(hw);

	/* Now read DO repeatedly until is high (equal to '1').  The EEPROM will
	 * signal that the command has been completed by raising the DO signal.
	 * If DO does not go high in 10 milliseconds, then error out.
	 */
	for (i = 0; i < 200; i++) {
		eecd_reg = IXGB_READ_REG(hw, EECD);

		if (eecd_reg & IXGB_EECD_DO)
			return (true);

		udelay(50);
	}
	ASSERT(0);
	return (false);
}

bool
ixgb_validate_eeprom_checksum(struct ixgb_hw *hw)
{
	u16 checksum = 0;
	u16 i;

	for (i = 0; i < (EEPROM_CHECKSUM_REG + 1); i++)
		checksum += ixgb_read_eeprom(hw, i);

	if (checksum == (u16) EEPROM_SUM)
		return (true);
	else
		return (false);
}

void
ixgb_update_eeprom_checksum(struct ixgb_hw *hw)
{
	u16 checksum = 0;
	u16 i;

	for (i = 0; i < EEPROM_CHECKSUM_REG; i++)
		checksum += ixgb_read_eeprom(hw, i);

	checksum = (u16) EEPROM_SUM - checksum;

	ixgb_write_eeprom(hw, EEPROM_CHECKSUM_REG, checksum);
}

void
ixgb_write_eeprom(struct ixgb_hw *hw, u16 offset, u16 data)
{
	struct ixgb_ee_map_type *ee_map = (struct ixgb_ee_map_type *)hw->eeprom;

	/* Prepare the EEPROM for writing */
	ixgb_setup_eeprom(hw);

	/*  Send the 9-bit EWEN (write enable) command to the EEPROM (5-bit opcode
	 *  plus 4-bit dummy).  This puts the EEPROM into write/erase mode.
	 */
	ixgb_shift_out_bits(hw, EEPROM_EWEN_OPCODE, 5);
	ixgb_shift_out_bits(hw, 0, 4);

	/*  Prepare the EEPROM  */
	ixgb_standby_eeprom(hw);

	/*  Send the Write command (3-bit opcode + 6-bit addr)  */
	ixgb_shift_out_bits(hw, EEPROM_WRITE_OPCODE, 3);
	ixgb_shift_out_bits(hw, offset, 6);

	/*  Send the data  */
	ixgb_shift_out_bits(hw, data, 16);

	ixgb_wait_eeprom_command(hw);

	/*  Recover from write  */
	ixgb_standby_eeprom(hw);

	/* Send the 9-bit EWDS (write disable) command to the EEPROM (5-bit
	 * opcode plus 4-bit dummy).  This takes the EEPROM out of write/erase
	 * mode.
	 */
	ixgb_shift_out_bits(hw, EEPROM_EWDS_OPCODE, 5);
	ixgb_shift_out_bits(hw, 0, 4);

	/*  Done with writing  */
	ixgb_cleanup_eeprom(hw);

	/* clear the init_ctrl_reg_1 to signify that the cache is invalidated */
	ee_map->init_ctrl_reg_1 = cpu_to_le16(EEPROM_ICW1_SIGNATURE_CLEAR);
}

u16
ixgb_read_eeprom(struct ixgb_hw *hw,
		  u16 offset)
{
	u16 data;

	/*  Prepare the EEPROM for reading  */
	ixgb_setup_eeprom(hw);

	/*  Send the READ command (opcode + addr)  */
	ixgb_shift_out_bits(hw, EEPROM_READ_OPCODE, 3);
	/*
	 * We have a 64 word EEPROM, there are 6 address bits
	 */
	ixgb_shift_out_bits(hw, offset, 6);

	/*  Read the data  */
	data = ixgb_shift_in_bits(hw);

	/*  End this read operation  */
	ixgb_standby_eeprom(hw);

	return (data);
}

bool
ixgb_get_eeprom_data(struct ixgb_hw *hw)
{
	u16 i;
	u16 checksum = 0;
	struct ixgb_ee_map_type *ee_map;

	ENTER();

	ee_map = (struct ixgb_ee_map_type *)hw->eeprom;

	pr_debug("Reading eeprom data\n");
	for (i = 0; i < IXGB_EEPROM_SIZE ; i++) {
		u16 ee_data;
		ee_data = ixgb_read_eeprom(hw, i);
		checksum += ee_data;
		hw->eeprom[i] = cpu_to_le16(ee_data);
	}

	if (checksum != (u16) EEPROM_SUM) {
		pr_debug("Checksum invalid\n");
		/* clear the init_ctrl_reg_1 to signify that the cache is
		 * invalidated */
		ee_map->init_ctrl_reg_1 = cpu_to_le16(EEPROM_ICW1_SIGNATURE_CLEAR);
		return (false);
	}

	if ((ee_map->init_ctrl_reg_1 & cpu_to_le16(EEPROM_ICW1_SIGNATURE_MASK))
		 != cpu_to_le16(EEPROM_ICW1_SIGNATURE_VALID)) {
		pr_debug("Signature invalid\n");
		return(false);
	}

	return(true);
}

static bool
ixgb_check_and_get_eeprom_data (struct ixgb_hw* hw)
{
	struct ixgb_ee_map_type *ee_map = (struct ixgb_ee_map_type *)hw->eeprom;

	if ((ee_map->init_ctrl_reg_1 & cpu_to_le16(EEPROM_ICW1_SIGNATURE_MASK))
	    == cpu_to_le16(EEPROM_ICW1_SIGNATURE_VALID)) {
		return (true);
	} else {
		return ixgb_get_eeprom_data(hw);
	}
}

__le16
ixgb_get_eeprom_word(struct ixgb_hw *hw, u16 index)
{

	if ((index < IXGB_EEPROM_SIZE) &&
		(ixgb_check_and_get_eeprom_data(hw) == true)) {
	   return(hw->eeprom[index]);
	}

	return(0);
}

void
ixgb_get_ee_mac_addr(struct ixgb_hw *hw,
			u8 *mac_addr)
{
	int i;
	struct ixgb_ee_map_type *ee_map = (struct ixgb_ee_map_type *)hw->eeprom;

	ENTER();

	if (ixgb_check_and_get_eeprom_data(hw) == true) {
		for (i = 0; i < IXGB_ETH_LENGTH_OF_ADDRESS; i++) {
			mac_addr[i] = ee_map->mac_addr[i];
		}
		pr_debug("eeprom mac address = %pM\n", mac_addr);
	}
}


u32
ixgb_get_ee_pba_number(struct ixgb_hw *hw)
{
	if (ixgb_check_and_get_eeprom_data(hw) == true)
		return (le16_to_cpu(hw->eeprom[EEPROM_PBA_1_2_REG])
			| (le16_to_cpu(hw->eeprom[EEPROM_PBA_3_4_REG])<<16));

	return(0);
}


u16
ixgb_get_ee_device_id(struct ixgb_hw *hw)
{
	struct ixgb_ee_map_type *ee_map = (struct ixgb_ee_map_type *)hw->eeprom;

	if (ixgb_check_and_get_eeprom_data(hw) == true)
		return (le16_to_cpu(ee_map->device_id));

	return (0);
}

