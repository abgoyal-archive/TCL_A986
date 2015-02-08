
/*****************************************************************************/


/*****************************************************************************/
#ifndef	_ISTALLION_H
#define	_ISTALLION_H
/*****************************************************************************/

#define	STL_MAXBRDS		4
#define	STL_MAXPANELS		4
#define	STL_MAXPORTS		64
#define	STL_MAXCHANS		(STL_MAXPORTS + 1)
#define	STL_MAXDEVS		(STL_MAXBRDS * STL_MAXPORTS)



struct stliport {
	unsigned long		magic;
	struct tty_port		port;
	unsigned int		portnr;
	unsigned int		panelnr;
	unsigned int		brdnr;
	unsigned long		state;
	unsigned int		devnr;
	int			baud_base;
	int			custom_divisor;
	int			closing_wait;
	int			rc;
	int			argsize;
	void			*argp;
	unsigned int		rxmarkmsk;
	wait_queue_head_t	raw_wait;
	struct asysigs		asig;
	unsigned long		addr;
	unsigned long		rxoffset;
	unsigned long		txoffset;
	unsigned long		sigs;
	unsigned long		pflag;
	unsigned int		rxsize;
	unsigned int		txsize;
	unsigned char		reqbit;
	unsigned char		portidx;
	unsigned char		portbit;
};

struct stlibrd {
	unsigned long	magic;
	unsigned int	brdnr;
	unsigned int	brdtype;
	unsigned int	state;
	unsigned int	nrpanels;
	unsigned int	nrports;
	unsigned int	nrdevs;
	unsigned int	iobase;
	int		iosize;
	unsigned long	memaddr;
	void		__iomem *membase;
	unsigned long	memsize;
	int		pagesize;
	int		hostoffset;
	int		slaveoffset;
	int		bitsize;
	int		enabval;
	unsigned int	panels[STL_MAXPANELS];
	int		panelids[STL_MAXPANELS];
	void		(*init)(struct stlibrd *brdp);
	void		(*enable)(struct stlibrd *brdp);
	void		(*reenable)(struct stlibrd *brdp);
	void		(*disable)(struct stlibrd *brdp);
	void		__iomem *(*getmemptr)(struct stlibrd *brdp, unsigned long offset, int line);
	void		(*intr)(struct stlibrd *brdp);
	void		(*reset)(struct stlibrd *brdp);
	struct stliport	*ports[STL_MAXPORTS];
};


#define	STLI_PORTMAGIC	0xe671c7a1
#define	STLI_BOARDMAGIC	0x4bc6c825

/*****************************************************************************/
#endif
