

#include <plat/i2c.h>
#include <plat/mux.h>
#include <plat/cpu.h>

void __init omap1_i2c_mux_pins(int bus_id)
{
	if (cpu_is_omap7xx()) {
		omap_cfg_reg(I2C_7XX_SDA);
		omap_cfg_reg(I2C_7XX_SCL);
	} else {
		omap_cfg_reg(I2C_SDA);
		omap_cfg_reg(I2C_SCL);
	}
}
