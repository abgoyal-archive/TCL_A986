

static inline int pwm_cfg_src_is_tclk(unsigned long tcfg)
{
	return tcfg == S3C2410_TCFG1_MUX_TCLK;
}

static inline unsigned long tcfg_to_divisor(unsigned long tcfg1)
{
	return 1 << (1 + tcfg1);
}

static inline unsigned int pwm_tdiv_has_div1(void)
{
	return 0;
}

static inline unsigned long pwm_tdiv_div_bits(unsigned int div)
{
	return ilog2(div) - 1;
}

#define S3C_TCFG1_MUX_TCLK S3C2410_TCFG1_MUX_TCLK
