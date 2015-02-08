

#ifndef mcfqspi_h
#define mcfqspi_h

#if defined(CONFIG_M523x) || defined(CONFIG_M527x) || defined(CONFIG_M528x)
#define	MCFQSPI_IOBASE		(MCF_IPSBAR + 0x340)
#elif defined(CONFIG_M5249)
#define MCFQSPI_IOBASE		(MCF_MBAR + 0x300)
#elif defined(CONFIG_M520x) || defined(CONFIG_M532x)
#define MCFQSPI_IOBASE		0xFC058000
#endif
#define MCFQSPI_IOSIZE		0x40

struct mcfqspi_cs_control {
	int 	(*setup)(struct mcfqspi_cs_control *);
	void	(*teardown)(struct mcfqspi_cs_control *);
	void	(*select)(struct mcfqspi_cs_control *, u8, bool);
	void	(*deselect)(struct mcfqspi_cs_control *, u8, bool);
};

struct mcfqspi_platform_data {
	s16	bus_num;
	u16	num_chipselect;
	struct mcfqspi_cs_control *cs_control;
};

#endif /* mcfqspi_h */
