

#ifndef __CVMX_GPIO_DEFS_H__
#define __CVMX_GPIO_DEFS_H__

#define CVMX_GPIO_BIT_CFGX(offset) \
	 CVMX_ADD_IO_SEG(0x0001070000000800ull + (((offset) & 15) * 8))
#define CVMX_GPIO_BOOT_ENA \
	 CVMX_ADD_IO_SEG(0x00010700000008A8ull)
#define CVMX_GPIO_CLK_GENX(offset) \
	 CVMX_ADD_IO_SEG(0x00010700000008C0ull + (((offset) & 3) * 8))
#define CVMX_GPIO_DBG_ENA \
	 CVMX_ADD_IO_SEG(0x00010700000008A0ull)
#define CVMX_GPIO_INT_CLR \
	 CVMX_ADD_IO_SEG(0x0001070000000898ull)
#define CVMX_GPIO_RX_DAT \
	 CVMX_ADD_IO_SEG(0x0001070000000880ull)
#define CVMX_GPIO_TX_CLR \
	 CVMX_ADD_IO_SEG(0x0001070000000890ull)
#define CVMX_GPIO_TX_SET \
	 CVMX_ADD_IO_SEG(0x0001070000000888ull)
#define CVMX_GPIO_XBIT_CFGX(offset) \
	 CVMX_ADD_IO_SEG(0x0001070000000900ull + (((offset) & 31) * 8) - 8 * 16)

union cvmx_gpio_bit_cfgx {
	uint64_t u64;
	struct cvmx_gpio_bit_cfgx_s {
		uint64_t reserved_15_63:49;
		uint64_t clk_gen:1;
		uint64_t clk_sel:2;
		uint64_t fil_sel:4;
		uint64_t fil_cnt:4;
		uint64_t int_type:1;
		uint64_t int_en:1;
		uint64_t rx_xor:1;
		uint64_t tx_oe:1;
	} s;
	struct cvmx_gpio_bit_cfgx_cn30xx {
		uint64_t reserved_12_63:52;
		uint64_t fil_sel:4;
		uint64_t fil_cnt:4;
		uint64_t int_type:1;
		uint64_t int_en:1;
		uint64_t rx_xor:1;
		uint64_t tx_oe:1;
	} cn30xx;
	struct cvmx_gpio_bit_cfgx_cn30xx cn31xx;
	struct cvmx_gpio_bit_cfgx_cn30xx cn38xx;
	struct cvmx_gpio_bit_cfgx_cn30xx cn38xxp2;
	struct cvmx_gpio_bit_cfgx_cn30xx cn50xx;
	struct cvmx_gpio_bit_cfgx_s cn52xx;
	struct cvmx_gpio_bit_cfgx_s cn52xxp1;
	struct cvmx_gpio_bit_cfgx_s cn56xx;
	struct cvmx_gpio_bit_cfgx_s cn56xxp1;
	struct cvmx_gpio_bit_cfgx_cn30xx cn58xx;
	struct cvmx_gpio_bit_cfgx_cn30xx cn58xxp1;
};

union cvmx_gpio_boot_ena {
	uint64_t u64;
	struct cvmx_gpio_boot_ena_s {
		uint64_t reserved_12_63:52;
		uint64_t boot_ena:4;
		uint64_t reserved_0_7:8;
	} s;
	struct cvmx_gpio_boot_ena_s cn30xx;
	struct cvmx_gpio_boot_ena_s cn31xx;
	struct cvmx_gpio_boot_ena_s cn50xx;
};

union cvmx_gpio_clk_genx {
	uint64_t u64;
	struct cvmx_gpio_clk_genx_s {
		uint64_t reserved_32_63:32;
		uint64_t n:32;
	} s;
	struct cvmx_gpio_clk_genx_s cn52xx;
	struct cvmx_gpio_clk_genx_s cn52xxp1;
	struct cvmx_gpio_clk_genx_s cn56xx;
	struct cvmx_gpio_clk_genx_s cn56xxp1;
};

union cvmx_gpio_dbg_ena {
	uint64_t u64;
	struct cvmx_gpio_dbg_ena_s {
		uint64_t reserved_21_63:43;
		uint64_t dbg_ena:21;
	} s;
	struct cvmx_gpio_dbg_ena_s cn30xx;
	struct cvmx_gpio_dbg_ena_s cn31xx;
	struct cvmx_gpio_dbg_ena_s cn50xx;
};

union cvmx_gpio_int_clr {
	uint64_t u64;
	struct cvmx_gpio_int_clr_s {
		uint64_t reserved_16_63:48;
		uint64_t type:16;
	} s;
	struct cvmx_gpio_int_clr_s cn30xx;
	struct cvmx_gpio_int_clr_s cn31xx;
	struct cvmx_gpio_int_clr_s cn38xx;
	struct cvmx_gpio_int_clr_s cn38xxp2;
	struct cvmx_gpio_int_clr_s cn50xx;
	struct cvmx_gpio_int_clr_s cn52xx;
	struct cvmx_gpio_int_clr_s cn52xxp1;
	struct cvmx_gpio_int_clr_s cn56xx;
	struct cvmx_gpio_int_clr_s cn56xxp1;
	struct cvmx_gpio_int_clr_s cn58xx;
	struct cvmx_gpio_int_clr_s cn58xxp1;
};

union cvmx_gpio_rx_dat {
	uint64_t u64;
	struct cvmx_gpio_rx_dat_s {
		uint64_t reserved_24_63:40;
		uint64_t dat:24;
	} s;
	struct cvmx_gpio_rx_dat_s cn30xx;
	struct cvmx_gpio_rx_dat_s cn31xx;
	struct cvmx_gpio_rx_dat_cn38xx {
		uint64_t reserved_16_63:48;
		uint64_t dat:16;
	} cn38xx;
	struct cvmx_gpio_rx_dat_cn38xx cn38xxp2;
	struct cvmx_gpio_rx_dat_s cn50xx;
	struct cvmx_gpio_rx_dat_cn38xx cn52xx;
	struct cvmx_gpio_rx_dat_cn38xx cn52xxp1;
	struct cvmx_gpio_rx_dat_cn38xx cn56xx;
	struct cvmx_gpio_rx_dat_cn38xx cn56xxp1;
	struct cvmx_gpio_rx_dat_cn38xx cn58xx;
	struct cvmx_gpio_rx_dat_cn38xx cn58xxp1;
};

union cvmx_gpio_tx_clr {
	uint64_t u64;
	struct cvmx_gpio_tx_clr_s {
		uint64_t reserved_24_63:40;
		uint64_t clr:24;
	} s;
	struct cvmx_gpio_tx_clr_s cn30xx;
	struct cvmx_gpio_tx_clr_s cn31xx;
	struct cvmx_gpio_tx_clr_cn38xx {
		uint64_t reserved_16_63:48;
		uint64_t clr:16;
	} cn38xx;
	struct cvmx_gpio_tx_clr_cn38xx cn38xxp2;
	struct cvmx_gpio_tx_clr_s cn50xx;
	struct cvmx_gpio_tx_clr_cn38xx cn52xx;
	struct cvmx_gpio_tx_clr_cn38xx cn52xxp1;
	struct cvmx_gpio_tx_clr_cn38xx cn56xx;
	struct cvmx_gpio_tx_clr_cn38xx cn56xxp1;
	struct cvmx_gpio_tx_clr_cn38xx cn58xx;
	struct cvmx_gpio_tx_clr_cn38xx cn58xxp1;
};

union cvmx_gpio_tx_set {
	uint64_t u64;
	struct cvmx_gpio_tx_set_s {
		uint64_t reserved_24_63:40;
		uint64_t set:24;
	} s;
	struct cvmx_gpio_tx_set_s cn30xx;
	struct cvmx_gpio_tx_set_s cn31xx;
	struct cvmx_gpio_tx_set_cn38xx {
		uint64_t reserved_16_63:48;
		uint64_t set:16;
	} cn38xx;
	struct cvmx_gpio_tx_set_cn38xx cn38xxp2;
	struct cvmx_gpio_tx_set_s cn50xx;
	struct cvmx_gpio_tx_set_cn38xx cn52xx;
	struct cvmx_gpio_tx_set_cn38xx cn52xxp1;
	struct cvmx_gpio_tx_set_cn38xx cn56xx;
	struct cvmx_gpio_tx_set_cn38xx cn56xxp1;
	struct cvmx_gpio_tx_set_cn38xx cn58xx;
	struct cvmx_gpio_tx_set_cn38xx cn58xxp1;
};

union cvmx_gpio_xbit_cfgx {
	uint64_t u64;
	struct cvmx_gpio_xbit_cfgx_s {
		uint64_t reserved_12_63:52;
		uint64_t fil_sel:4;
		uint64_t fil_cnt:4;
		uint64_t reserved_2_3:2;
		uint64_t rx_xor:1;
		uint64_t tx_oe:1;
	} s;
	struct cvmx_gpio_xbit_cfgx_s cn30xx;
	struct cvmx_gpio_xbit_cfgx_s cn31xx;
	struct cvmx_gpio_xbit_cfgx_s cn50xx;
};

#endif
