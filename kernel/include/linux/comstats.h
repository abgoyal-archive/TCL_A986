
/*****************************************************************************/


/*****************************************************************************/
#ifndef	_COMSTATS_H
#define	_COMSTATS_H
/*****************************************************************************/


typedef struct {
	unsigned long	brd;
	unsigned long	panel;
	unsigned long	port;
	unsigned long	hwid;
	unsigned long	type;
	unsigned long	txtotal;
	unsigned long	rxtotal;
	unsigned long	txbuffered;
	unsigned long	rxbuffered;
	unsigned long	rxoverrun;
	unsigned long	rxparity;
	unsigned long	rxframing;
	unsigned long	rxlost;
	unsigned long	txbreaks;
	unsigned long	rxbreaks;
	unsigned long	txxon;
	unsigned long	txxoff;
	unsigned long	rxxon;
	unsigned long	rxxoff;
	unsigned long	txctson;
	unsigned long	txctsoff;
	unsigned long	rxrtson;
	unsigned long	rxrtsoff;
	unsigned long	modem;
	unsigned long	state;
	unsigned long	flags;
	unsigned long	ttystate;
	unsigned long	cflags;
	unsigned long	iflags;
	unsigned long	oflags;
	unsigned long	lflags;
	unsigned long	signals;
} comstats_t;



#define	COM_MAXPANELS	8

typedef struct {
	unsigned long	panel;
	unsigned long	type;
	unsigned long	hwid;
	unsigned long	nrports;
} companel_t;

typedef struct {
	unsigned long	brd;
	unsigned long	type;
	unsigned long	hwid;
	unsigned long	state;
	unsigned long	ioaddr;
	unsigned long	ioaddr2;
	unsigned long	memaddr;
	unsigned long	irq;
	unsigned long	nrpanels;
	unsigned long	nrports;
	companel_t	panels[COM_MAXPANELS];
} combrd_t;


#include <linux/ioctl.h>

#define	COM_GETPORTSTATS	_IO('c',30)
#define	COM_CLRPORTSTATS	_IO('c',31)
#define	COM_GETBRDSTATS		_IO('c',32)


#define	COM_READPORT		_IO('c',40)
#define	COM_READBOARD		_IO('c',41)
#define	COM_READPANEL		_IO('c',42)

/*****************************************************************************/
#endif
