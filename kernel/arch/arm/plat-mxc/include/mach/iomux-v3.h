

#ifndef __MACH_IOMUX_V3_H__
#define __MACH_IOMUX_V3_H__


struct pad_desc {
	unsigned mux_ctrl_ofs:12; /* IOMUXC_SW_MUX_CTL_PAD offset */
	unsigned mux_mode:8;
	unsigned pad_ctrl_ofs:12; /* IOMUXC_SW_PAD_CTRL offset */
#define	NO_PAD_CTRL	(1 << 16)
	unsigned pad_ctrl:17;
	unsigned select_input_ofs:12; /* IOMUXC_SELECT_INPUT offset */
	unsigned select_input:3;
};

#define IOMUX_PAD(_pad_ctrl_ofs, _mux_ctrl_ofs, _mux_mode, _select_input_ofs, \
		_select_input, _pad_ctrl)				\
		{							\
			.mux_ctrl_ofs     = _mux_ctrl_ofs,		\
			.mux_mode         = _mux_mode,			\
			.pad_ctrl_ofs     = _pad_ctrl_ofs,		\
			.pad_ctrl         = _pad_ctrl,			\
			.select_input_ofs = _select_input_ofs,		\
			.select_input     = _select_input,		\
		}


#define PAD_CTL_DVS			(1 << 13)
#define PAD_CTL_HYS			(1 << 8)

#define PAD_CTL_PKE			(1 << 7)
#define PAD_CTL_PUE			(1 << 6)
#define PAD_CTL_PUS_100K_DOWN		(0 << 4)
#define PAD_CTL_PUS_47K_UP		(1 << 4)
#define PAD_CTL_PUS_100K_UP		(2 << 4)
#define PAD_CTL_PUS_22K_UP		(3 << 4)

#define PAD_CTL_ODE			(1 << 3)

#define PAD_CTL_DSE_LOW			(0 << 1)
#define PAD_CTL_DSE_MED			(1 << 1)
#define PAD_CTL_DSE_HIGH		(2 << 1)
#define PAD_CTL_DSE_MAX			(3 << 1)

#define PAD_CTL_SRE_FAST		(1 << 0)
#define PAD_CTL_SRE_SLOW		(0 << 0)

int mxc_iomux_v3_setup_pad(struct pad_desc *pad);

int mxc_iomux_v3_setup_multiple_pads(struct pad_desc *pad_list, unsigned count);

void mxc_iomux_v3_init(void __iomem *iomux_v3_base);

#endif /* __MACH_IOMUX_V3_H__*/

