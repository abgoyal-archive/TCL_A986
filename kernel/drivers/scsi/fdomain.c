

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/proc_fs.h>
#include <linux/pci.h>
#include <linux/stat.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <scsi/scsicam.h>

#include <asm/system.h>

#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi_ioctl.h>
#include "fdomain.h"

#ifndef PCMCIA
MODULE_AUTHOR("Rickard E. Faith");
MODULE_DESCRIPTION("Future domain SCSI driver");
MODULE_LICENSE("GPL");
#endif

  
#define VERSION          "$Revision: 5.51 $"

/* START OF USER DEFINABLE OPTIONS */

#define DEBUG            0	/* Enable debugging output */
#define ENABLE_PARITY    1	/* Enable SCSI Parity */
#define FIFO_COUNT       2	/* Number of 512 byte blocks before INTR */

/* END OF USER DEFINABLE OPTIONS */

#if DEBUG
#define EVERY_ACCESS     0	/* Write a line on every scsi access */
#define ERRORS_ONLY      1	/* Only write a line if there is an error */
#define DEBUG_DETECT     0	/* Debug fdomain_16x0_detect() */
#define DEBUG_MESSAGES   1	/* Debug MESSAGE IN phase */
#define DEBUG_ABORT      1	/* Debug abort() routine */
#define DEBUG_RESET      1	/* Debug reset() routine */
#define DEBUG_RACE       1      /* Debug interrupt-driven race condition */
#else
#define EVERY_ACCESS     0	/* LEAVE THESE ALONE--CHANGE THE ONES ABOVE */
#define ERRORS_ONLY      0
#define DEBUG_DETECT     0
#define DEBUG_MESSAGES   0
#define DEBUG_ABORT      0
#define DEBUG_RESET      0
#define DEBUG_RACE       0
#endif

/* Errors are reported on the line, so we don't need to report them again */
#if EVERY_ACCESS
#undef ERRORS_ONLY
#define ERRORS_ONLY      0
#endif

#if ENABLE_PARITY
#define PARITY_MASK      0x08
#else
#define PARITY_MASK      0x00
#endif

enum chip_type {
   unknown          = 0x00,
   tmc1800          = 0x01,
   tmc18c50         = 0x02,
   tmc18c30         = 0x03,
};

enum {
   in_arbitration   = 0x02,
   in_selection     = 0x04,
   in_other         = 0x08,
   disconnect       = 0x10,
   aborted          = 0x20,
   sent_ident       = 0x40,
};

enum in_port_type {
   Read_SCSI_Data   =  0,
   SCSI_Status      =  1,
   TMC_Status       =  2,
   FIFO_Status      =  3,	/* tmc18c50/tmc18c30 only */
   Interrupt_Cond   =  4,	/* tmc18c50/tmc18c30 only */
   LSB_ID_Code      =  5,
   MSB_ID_Code      =  6,
   Read_Loopback    =  7,
   SCSI_Data_NoACK  =  8,
   Interrupt_Status =  9,
   Configuration1   = 10,
   Configuration2   = 11,	/* tmc18c50/tmc18c30 only */
   Read_FIFO        = 12,
   FIFO_Data_Count  = 14
};

enum out_port_type {
   Write_SCSI_Data  =  0,
   SCSI_Cntl        =  1,
   Interrupt_Cntl   =  2,
   SCSI_Mode_Cntl   =  3,
   TMC_Cntl         =  4,
   Memory_Cntl      =  5,	/* tmc18c50/tmc18c30 only */
   Write_Loopback   =  7,
   IO_Control       = 11,	/* tmc18c30 only */
   Write_FIFO       = 12
};

/* .bss will zero all the static variables below */
static int               port_base;
static unsigned long     bios_base;
static void __iomem *    bios_mem;
static int               bios_major;
static int               bios_minor;
static int               PCI_bus;
#ifdef CONFIG_PCI
static struct pci_dev	*PCI_dev;
#endif
static int               Quantum;	/* Quantum board variant */
static int               interrupt_level;
static volatile int      in_command;
static struct scsi_cmnd  *current_SC;
static enum chip_type    chip              = unknown;
static int               adapter_mask;
static int               this_id;
static int               setup_called;

#if DEBUG_RACE
static volatile int      in_interrupt_flag;
#endif

static int               FIFO_Size = 0x2000; /* 8k FIFO for
						pre-tmc18c30 chips */

static irqreturn_t       do_fdomain_16x0_intr( int irq, void *dev_id );
static char * fdomain = NULL;
module_param(fdomain, charp, 0);

#ifndef PCMCIA

static unsigned long addresses[] = {
   0xc8000,
   0xca000,
   0xce000,
   0xde000,
   0xcc000,		/* Extra addresses for PCI boards */
   0xd0000,
   0xe0000,
};
#define ADDRESS_COUNT ARRAY_SIZE(addresses)

static unsigned short ports[] = { 0x140, 0x150, 0x160, 0x170 };
#define PORT_COUNT ARRAY_SIZE(ports)

static unsigned short ints[] = { 3, 5, 10, 11, 12, 14, 15, 0 };

#endif /* !PCMCIA */


#ifndef PCMCIA

static struct signature {
   const char *signature;
   int  sig_offset;
   int  sig_length;
   int  major_bios_version;
   int  minor_bios_version;
   int  flag; /* 1 == PCI_bus, 2 == ISA_200S, 3 == ISA_250MG, 4 == ISA_200S */
} signatures[] = {
   /*          1         2         3         4         5         6 */
   /* 123456789012345678901234567890123456789012345678901234567890 */
   { "FUTURE DOMAIN CORP. (C) 1986-1990 1800-V2.07/28/89",  5, 50,  2,  0, 0 },
   { "FUTURE DOMAIN CORP. (C) 1986-1990 1800-V1.07/28/89",  5, 50,  2,  0, 0 },
   { "FUTURE DOMAIN CORP. (C) 1986-1990 1800-V2.07/28/89", 72, 50,  2,  0, 2 },
   { "FUTURE DOMAIN CORP. (C) 1986-1990 1800-V2.0",        73, 43,  2,  0, 3 },
   { "FUTURE DOMAIN CORP. (C) 1991 1800-V2.0.",            72, 39,  2,  0, 4 },
   { "FUTURE DOMAIN CORP. (C) 1992 V3.00.004/02/92",        5, 44,  3,  0, 0 },
   { "FUTURE DOMAIN TMC-18XX (C) 1993 V3.203/12/93",        5, 44,  3,  2, 0 },
   { "IBM F1 P2 BIOS v1.0104/29/93",                        5, 28,  3, -1, 0 },
   { "Future Domain Corp. V1.0008/18/93",                   5, 33,  3,  4, 0 },
   { "Future Domain Corp. V1.0008/18/93",                  26, 33,  3,  4, 1 },
   { "Adaptec AHA-2920 PCI-SCSI Card",                     42, 31,  3, -1, 1 },
   { "IBM F1 P264/32",                                      5, 14,  3, -1, 1 },
				/* This next signature may not be a 3.5 bios */
   { "Future Domain Corp. V2.0108/18/93",                   5, 33,  3,  5, 0 },
   { "FUTURE DOMAIN CORP.  V3.5008/18/93",                  5, 34,  3,  5, 0 },
   { "FUTURE DOMAIN 18c30/18c50/1800 (C) 1994 V3.5",        5, 44,  3,  5, 0 },
   { "FUTURE DOMAIN CORP.  V3.6008/18/93",                  5, 34,  3,  6, 0 },
   { "FUTURE DOMAIN CORP.  V3.6108/18/93",                  5, 34,  3,  6, 0 },
   { "FUTURE DOMAIN TMC-18XX",                              5, 22, -1, -1, 0 },

   /* READ NOTICE ABOVE *BEFORE* YOU WASTE YOUR TIME ADDING A SIGNATURE
    Also, fix the disk geometry code for your signature and send your
    changes for faith@cs.unc.edu.  Above all, do *NOT* change any old
    signatures!

    Note that the last line will match a "generic" 18XX bios.  Because
    Future Domain has changed the host SCSI ID and/or the location of the
    geometry information in the on-board RAM area for each of the first
    three BIOS's, it is still important to enter a fully qualified
    signature in the table for any new BIOS's (after the host SCSI ID and
    geometry location are verified). */
};

#define SIGNATURE_COUNT ARRAY_SIZE(signatures)

#endif /* !PCMCIA */

static void print_banner( struct Scsi_Host *shpnt )
{
   if (!shpnt) return;		/* This won't ever happen */

   if (bios_major < 0 && bios_minor < 0) {
      printk(KERN_INFO "scsi%d: <fdomain> No BIOS; using scsi id %d\n",
	      shpnt->host_no, shpnt->this_id);
   } else {
      printk(KERN_INFO "scsi%d: <fdomain> BIOS version ", shpnt->host_no);

      if (bios_major >= 0) printk("%d.", bios_major);
      else                 printk("?.");

      if (bios_minor >= 0) printk("%d", bios_minor);
      else                 printk("?.");

      printk( " at 0x%lx using scsi id %d\n",
	      bios_base, shpnt->this_id );
   }

				/* If this driver works for later FD PCI
				   boards, we will have to modify banner
				   for additional PCI cards, but for now if
				   it's PCI it's a TMC-3260 - JTM */
   printk(KERN_INFO "scsi%d: <fdomain> %s chip at 0x%x irq ",
	   shpnt->host_no,
	   chip == tmc1800 ? "TMC-1800" : (chip == tmc18c50 ? "TMC-18C50" : (chip == tmc18c30 ? (PCI_bus ? "TMC-36C70 (PCI bus)" : "TMC-18C30") : "Unknown")),
	   port_base);

   if (interrupt_level)
   	printk("%d", interrupt_level);
   else
        printk("<none>");

   printk( "\n" );
}

int fdomain_setup(char *str)
{
	int ints[4];

	(void)get_options(str, ARRAY_SIZE(ints), ints);

	if (setup_called++ || ints[0] < 2 || ints[0] > 3) {
		printk(KERN_INFO "scsi: <fdomain> Usage: fdomain=<PORT_BASE>,<IRQ>[,<ADAPTER_ID>]\n");
		printk(KERN_ERR "scsi: <fdomain> Bad LILO/INSMOD parameters?\n");
		return 0;
	}

	port_base       = ints[0] >= 1 ? ints[1] : 0;
	interrupt_level = ints[0] >= 2 ? ints[2] : 0;
	this_id         = ints[0] >= 3 ? ints[3] : 0;
   
	bios_major = bios_minor = -1; /* Use geometry for BIOS version >= 3.4 */
	++setup_called;
	return 1;
}

__setup("fdomain=", fdomain_setup);


static void do_pause(unsigned amount)	/* Pause for amount*10 milliseconds */
{
	mdelay(10*amount);
}

static inline void fdomain_make_bus_idle( void )
{
   outb(0, port_base + SCSI_Cntl);
   outb(0, port_base + SCSI_Mode_Cntl);
   if (chip == tmc18c50 || chip == tmc18c30)
	 outb(0x21 | PARITY_MASK, port_base + TMC_Cntl); /* Clear forced intr. */
   else
	 outb(0x01 | PARITY_MASK, port_base + TMC_Cntl);
}

static int fdomain_is_valid_port( int port )
{
#if DEBUG_DETECT 
   printk( " (%x%x),",
	   inb( port + MSB_ID_Code ), inb( port + LSB_ID_Code ) );
#endif

   /* The MCA ID is a unique id for each MCA compatible board.  We
      are using ISA boards, but Future Domain provides the MCA ID
      anyway.  We can use this ID to ensure that this is a Future
      Domain TMC-1660/TMC-1680.
    */

   if (inb( port + LSB_ID_Code ) != 0xe9) { /* test for 0x6127 id */
      if (inb( port + LSB_ID_Code ) != 0x27) return 0;
      if (inb( port + MSB_ID_Code ) != 0x61) return 0;
      chip = tmc1800;
   } else {				    /* test for 0xe960 id */
      if (inb( port + MSB_ID_Code ) != 0x60) return 0;
      chip = tmc18c50;

				/* Try to toggle 32-bit mode.  This only
				   works on an 18c30 chip.  (User reports
				   say this works, so we should switch to
				   it in the near future.) */

      outb( 0x80, port + IO_Control );
      if ((inb( port + Configuration2 ) & 0x80) == 0x80) {
	 outb( 0x00, port + IO_Control );
	 if ((inb( port + Configuration2 ) & 0x80) == 0x00) {
	    chip = tmc18c30;
	    FIFO_Size = 0x800;	/* 2k FIFO */
	 }
      }
				/* If that failed, we are an 18c50. */
   }

   return 1;
}

static int fdomain_test_loopback( void )
{
   int i;
   int result;

   for (i = 0; i < 255; i++) {
      outb( i, port_base + Write_Loopback );
      result = inb( port_base + Read_Loopback );
      if (i != result)
	    return 1;
   }
   return 0;
}

#ifndef PCMCIA


static int fdomain_get_irq( int base )
{
   int options = inb(base + Configuration1);

#if DEBUG_DETECT
   printk("scsi: <fdomain> Options = %x\n", options);
#endif
 
   /* Check for board with lowest bios_base --
      this isn't valid for the 18c30 or for
      boards on the PCI bus, so just assume we
      have the right board. */

   if (chip != tmc18c30 && !PCI_bus && addresses[(options & 0xc0) >> 6 ] != bios_base)
   	return 0;
   return ints[(options & 0x0e) >> 1];
}

static int fdomain_isa_detect( int *irq, int *iobase )
{
   int i, j;
   int base = 0xdeadbeef;
   int flag = 0;

#if DEBUG_DETECT
   printk( "scsi: <fdomain> fdomain_isa_detect:" );
#endif

   for (i = 0; i < ADDRESS_COUNT; i++) {
      void __iomem *p = ioremap(addresses[i], 0x2000);
      if (!p)
	continue;
#if DEBUG_DETECT
      printk( " %lx(%lx),", addresses[i], bios_base );
#endif
      for (j = 0; j < SIGNATURE_COUNT; j++) {
	 if (check_signature(p + signatures[j].sig_offset,
			     signatures[j].signature,
			     signatures[j].sig_length )) {
	    bios_major = signatures[j].major_bios_version;
	    bios_minor = signatures[j].minor_bios_version;
	    PCI_bus    = (signatures[j].flag == 1);
	    Quantum    = (signatures[j].flag > 1) ? signatures[j].flag : 0;
	    bios_base  = addresses[i];
	    bios_mem   = p;
	    goto found;
	 }
      }
      iounmap(p);
   }
 
found:
   if (bios_major == 2) {
      /* The TMC-1660/TMC-1680 has a RAM area just after the BIOS ROM.
	 Assuming the ROM is enabled (otherwise we wouldn't have been
	 able to read the ROM signature :-), then the ROM sets up the
	 RAM area with some magic numbers, such as a list of port
	 base addresses and a list of the disk "geometry" reported to
	 DOS (this geometry has nothing to do with physical geometry).
       */

      switch (Quantum) {
      case 2:			/* ISA_200S */
      case 3:			/* ISA_250MG */
	 base = readb(bios_mem + 0x1fa2) + (readb(bios_mem + 0x1fa3) << 8);
	 break;
      case 4:			/* ISA_200S (another one) */
	 base = readb(bios_mem + 0x1fa3) + (readb(bios_mem + 0x1fa4) << 8);
	 break;
      default:
	 base = readb(bios_mem + 0x1fcc) + (readb(bios_mem + 0x1fcd) << 8);
	 break;
      }
   
#if DEBUG_DETECT
      printk( " %x,", base );
#endif

      for (i = 0; i < PORT_COUNT; i++) {
	if (base == ports[i]) {
		if (!request_region(base, 0x10, "fdomain"))
			break;
		if (!fdomain_is_valid_port(base)) {
			release_region(base, 0x10);
			break;
		}
		*irq    = fdomain_get_irq( base );
		*iobase = base;
		return 1;
	}
      }

      /* This is a bad sign.  It usually means that someone patched the
	 BIOS signature list (the signatures variable) to contain a BIOS
	 signature for a board *OTHER THAN* the TMC-1660/TMC-1680. */
      
#if DEBUG_DETECT
      printk( " RAM FAILED, " );
#endif
   }

   /* Anyway, the alternative to finding the address in the RAM is to just
      search through every possible port address for one that is attached
      to the Future Domain card.  Don't panic, though, about reading all
      these random port addresses -- there are rumors that the Future
      Domain BIOS does something very similar.

      Do not, however, check ports which the kernel knows are being used by
      another driver. */

   for (i = 0; i < PORT_COUNT; i++) {
      base = ports[i];
      if (!request_region(base, 0x10, "fdomain")) {
#if DEBUG_DETECT
	 printk( " (%x inuse),", base );
#endif
	 continue;
      }
#if DEBUG_DETECT
      printk( " %x,", base );
#endif
      flag = fdomain_is_valid_port(base);
      if (flag)
	break;
      release_region(base, 0x10);
   }

#if DEBUG_DETECT
   if (flag) printk( " SUCCESS\n" );
   else      printk( " FAILURE\n" );
#endif

   if (!flag) return 0;		/* iobase not found */

   *irq    = fdomain_get_irq( base );
   *iobase = base;

   return 1;			/* success */
}

#else /* PCMCIA */

static int fdomain_isa_detect( int *irq, int *iobase )
{
	if (irq)
		*irq = 0;
	if (iobase)
		*iobase = 0;
	return 0;
}

#endif /* !PCMCIA */



#ifdef CONFIG_PCI
static int fdomain_pci_bios_detect( int *irq, int *iobase, struct pci_dev **ret_pdev )
{
   unsigned int     pci_irq;                /* PCI interrupt line */
   unsigned long    pci_base;               /* PCI I/O base address */
   struct pci_dev   *pdev = NULL;

#if DEBUG_DETECT
   /* Tell how to print a list of the known PCI devices from bios32 and
      list vendor and device IDs being used if in debug mode.  */
      
   printk( "scsi: <fdomain> INFO: use lspci -v to see list of PCI devices\n" );
   printk( "scsi: <fdomain> TMC-3260 detect:"
	   " Using Vendor ID: 0x%x and Device ID: 0x%x\n",
	   PCI_VENDOR_ID_FD, 
	   PCI_DEVICE_ID_FD_36C70 );
#endif 

   if ((pdev = pci_get_device(PCI_VENDOR_ID_FD, PCI_DEVICE_ID_FD_36C70, pdev)) == NULL)
		return 0;
   if (pci_enable_device(pdev))
   	goto fail;
       
#if DEBUG_DETECT
   printk( "scsi: <fdomain> TMC-3260 detect:"
	   " PCI bus %u, device %u, function %u\n",
	   pdev->bus->number,
	   PCI_SLOT(pdev->devfn),
	   PCI_FUNC(pdev->devfn));
#endif

   /* We now have the appropriate device function for the FD board so we
      just read the PCI config info from the registers.  */

   pci_base = pci_resource_start(pdev, 0);
   pci_irq = pdev->irq;

   if (!request_region( pci_base, 0x10, "fdomain" ))
   	goto fail;

   /* Now we have the I/O base address and interrupt from the PCI
      configuration registers. */

   *irq    = pci_irq;
   *iobase = pci_base;
   *ret_pdev = pdev;

#if DEBUG_DETECT
   printk( "scsi: <fdomain> TMC-3260 detect:"
	   " IRQ = %d, I/O base = 0x%x [0x%lx]\n", *irq, *iobase, pci_base );
#endif

   if (!fdomain_is_valid_port(pci_base)) {
      printk(KERN_ERR "scsi: <fdomain> PCI card detected, but driver not loaded (invalid port)\n" );
      release_region(pci_base, 0x10);
      goto fail;
   }

				/* Fill in a few global variables.  Ugh. */
   bios_major = bios_minor = -1;
   PCI_bus    = 1;
   PCI_dev    = pdev;
   Quantum    = 0;
   bios_base  = 0;
   
   return 1;
fail:
   pci_dev_put(pdev);
   return 0;
}

#endif

struct Scsi_Host *__fdomain_16x0_detect(struct scsi_host_template *tpnt )
{
   int              retcode;
   struct Scsi_Host *shpnt;
   struct pci_dev *pdev = NULL;

   if (setup_called) {
#if DEBUG_DETECT
      printk( "scsi: <fdomain> No BIOS, using port_base = 0x%x, irq = %d\n",
	      port_base, interrupt_level );
#endif
      if (!request_region(port_base, 0x10, "fdomain")) {
	 printk( "scsi: <fdomain> port 0x%x is busy\n", port_base );
	 printk( "scsi: <fdomain> Bad LILO/INSMOD parameters?\n" );
	 return NULL;
      }
      if (!fdomain_is_valid_port( port_base )) {
	 printk( "scsi: <fdomain> Cannot locate chip at port base 0x%x\n",
		 port_base );
	 printk( "scsi: <fdomain> Bad LILO/INSMOD parameters?\n" );
	 release_region(port_base, 0x10);
	 return NULL;
      }
   } else {
      int flag = 0;

#ifdef CONFIG_PCI
				/* Try PCI detection first */
      flag = fdomain_pci_bios_detect( &interrupt_level, &port_base, &pdev );
#endif
      if (!flag) {
				/* Then try ISA bus detection */
	 flag = fdomain_isa_detect( &interrupt_level, &port_base );

	 if (!flag) {
	    printk( "scsi: <fdomain> Detection failed (no card)\n" );
	    return NULL;
	 }
      }
   }

   fdomain_16x0_bus_reset(NULL);

   if (fdomain_test_loopback()) {
      printk(KERN_ERR  "scsi: <fdomain> Detection failed (loopback test failed at port base 0x%x)\n", port_base);
      if (setup_called) {
	 printk(KERN_ERR "scsi: <fdomain> Bad LILO/INSMOD parameters?\n");
      }
      goto fail;
   }

   if (this_id) {
      tpnt->this_id = (this_id & 0x07);
      adapter_mask  = (1 << tpnt->this_id);
   } else {
      if (PCI_bus || (bios_major == 3 && bios_minor >= 2) || bios_major < 0) {
	 tpnt->this_id = 7;
	 adapter_mask  = 0x80;
      } else {
	 tpnt->this_id = 6;
	 adapter_mask  = 0x40;
      }
   }


   shpnt = scsi_register( tpnt, 0 );
   if(shpnt == NULL) {
	release_region(port_base, 0x10);
   	return NULL;
   }
   shpnt->irq = interrupt_level;
   shpnt->io_port = port_base;
   shpnt->n_io_port = 0x10;
   print_banner( shpnt );

   /* Log IRQ with kernel */   
   if (!interrupt_level) {
      printk(KERN_ERR "scsi: <fdomain> Card Detected, but driver not loaded (no IRQ)\n" );
      goto fail;
   } else {
      /* Register the IRQ with the kernel */

      retcode = request_irq( interrupt_level,
			     do_fdomain_16x0_intr, pdev?IRQF_SHARED:0, "fdomain", shpnt);

      if (retcode < 0) {
	 if (retcode == -EINVAL) {
	    printk(KERN_ERR "scsi: <fdomain> IRQ %d is bad!\n", interrupt_level );
	    printk(KERN_ERR "                This shouldn't happen!\n" );
	    printk(KERN_ERR "                Send mail to faith@acm.org\n" );
	 } else if (retcode == -EBUSY) {
	    printk(KERN_ERR "scsi: <fdomain> IRQ %d is already in use!\n", interrupt_level );
	    printk(KERN_ERR "                Please use another IRQ!\n" );
	 } else {
	    printk(KERN_ERR "scsi: <fdomain> Error getting IRQ %d\n", interrupt_level );
	    printk(KERN_ERR "                This shouldn't happen!\n" );
	    printk(KERN_ERR "                Send mail to faith@acm.org\n" );
	 }
	 printk(KERN_ERR "scsi: <fdomain> Detected, but driver not loaded (IRQ)\n" );
	 goto fail;
      }
   }
   return shpnt;
fail:
   pci_dev_put(pdev);
   release_region(port_base, 0x10);
   return NULL;
}

static int fdomain_16x0_detect(struct scsi_host_template *tpnt)
{
	if (fdomain)
		fdomain_setup(fdomain);
	return (__fdomain_16x0_detect(tpnt) != NULL);
}

static const char *fdomain_16x0_info( struct Scsi_Host *ignore )
{
   static char buffer[128];
   char        *pt;
   
   strcpy( buffer, "Future Domain 16-bit SCSI Driver Version" );
   if (strchr( VERSION, ':')) { /* Assume VERSION is an RCS Revision string */
      strcat( buffer, strchr( VERSION, ':' ) + 1 );
      pt = strrchr( buffer, '$') - 1;
      if (!pt)  		/* Stripped RCS Revision string? */
	    pt = buffer + strlen( buffer ) - 1;
      if (*pt != ' ')
	    ++pt;
      *pt = '\0';
   } else {			/* Assume VERSION is a number */
      strcat( buffer, " " VERSION );
   }
      
   return buffer;
}

#if 0
static int fdomain_arbitrate( void )
{
   int           status = 0;
   unsigned long timeout;

#if EVERY_ACCESS
   printk( "fdomain_arbitrate()\n" );
#endif
   
   outb(0x00, port_base + SCSI_Cntl);              /* Disable data drivers */
   outb(adapter_mask, port_base + SCSI_Data_NoACK); /* Set our id bit */
   outb(0x04 | PARITY_MASK, port_base + TMC_Cntl); /* Start arbitration */

   timeout = 500;
   do {
      status = inb(port_base + TMC_Status);        /* Read adapter status */
      if (status & 0x02)		      /* Arbitration complete */
	    return 0;
      mdelay(1);			/* Wait one millisecond */
   } while (--timeout);

   /* Make bus idle */
   fdomain_make_bus_idle();

#if EVERY_ACCESS
   printk( "Arbitration failed, status = %x\n", status );
#endif
#if ERRORS_ONLY
   printk( "scsi: <fdomain> Arbitration failed, status = %x\n", status );
#endif
   return 1;
}
#endif

static int fdomain_select( int target )
{
   int           status;
   unsigned long timeout;
#if ERRORS_ONLY
   static int    flag = 0;
#endif

   outb(0x82, port_base + SCSI_Cntl); /* Bus Enable + Select */
   outb(adapter_mask | (1 << target), port_base + SCSI_Data_NoACK);

   /* Stop arbitration and enable parity */
   outb(PARITY_MASK, port_base + TMC_Cntl); 

   timeout = 350;			/* 350 msec */

   do {
      status = inb(port_base + SCSI_Status); /* Read adapter status */
      if (status & 1) {			/* Busy asserted */
	 /* Enable SCSI Bus (on error, should make bus idle with 0) */
	 outb(0x80, port_base + SCSI_Cntl);
	 return 0;
      }
      mdelay(1);			/* wait one msec */
   } while (--timeout);
   /* Make bus idle */
   fdomain_make_bus_idle();
#if EVERY_ACCESS
   if (!target) printk( "Selection failed\n" );
#endif
#if ERRORS_ONLY
   if (!target) {
      if (!flag) /* Skip first failure for all chips. */
	    ++flag;
      else
	    printk( "scsi: <fdomain> Selection failed\n" );
   }
#endif
   return 1;
}

static void my_done(int error)
{
   if (in_command) {
      in_command = 0;
      outb(0x00, port_base + Interrupt_Cntl);
      fdomain_make_bus_idle();
      current_SC->result = error;
      if (current_SC->scsi_done)
	    current_SC->scsi_done( current_SC );
      else panic( "scsi: <fdomain> current_SC->scsi_done() == NULL" );
   } else {
      panic( "scsi: <fdomain> my_done() called outside of command\n" );
   }
#if DEBUG_RACE
   in_interrupt_flag = 0;
#endif
}

static irqreturn_t do_fdomain_16x0_intr(int irq, void *dev_id)
{
   unsigned long flags;
   int      status;
   int      done = 0;
   unsigned data_count;

				/* The fdomain_16x0_intr is only called via
				   the interrupt handler.  The goal of the
				   sti() here is to allow other
				   interruptions while this routine is
				   running. */

   /* Check for other IRQ sources */
   if ((inb(port_base + TMC_Status) & 0x01) == 0)
   	return IRQ_NONE;

   /* It is our IRQ */   	
   outb(0x00, port_base + Interrupt_Cntl);

   /* We usually have one spurious interrupt after each command.  Ignore it. */
   if (!in_command || !current_SC) {	/* Spurious interrupt */
#if EVERY_ACCESS
      printk( "Spurious interrupt, in_command = %d, current_SC = %x\n",
	      in_command, current_SC );
#endif
      return IRQ_NONE;
   }

   /* Abort calls my_done, so we do nothing here. */
   if (current_SC->SCp.phase & aborted) {
#if DEBUG_ABORT
      printk( "scsi: <fdomain> Interrupt after abort, ignoring\n" );
#endif
      /*
      return IRQ_HANDLED; */
   }

#if DEBUG_RACE
   ++in_interrupt_flag;
#endif

   if (current_SC->SCp.phase & in_arbitration) {
      status = inb(port_base + TMC_Status);        /* Read adapter status */
      if (!(status & 0x02)) {
#if EVERY_ACCESS
	 printk( " AFAIL " );
#endif
         spin_lock_irqsave(current_SC->device->host->host_lock, flags);
	 my_done( DID_BUS_BUSY << 16 );
         spin_unlock_irqrestore(current_SC->device->host->host_lock, flags);
	 return IRQ_HANDLED;
      }
      current_SC->SCp.phase = in_selection;
      
      outb(0x40 | FIFO_COUNT, port_base + Interrupt_Cntl);

      outb(0x82, port_base + SCSI_Cntl); /* Bus Enable + Select */
      outb(adapter_mask | (1 << scmd_id(current_SC)), port_base + SCSI_Data_NoACK);
      
      /* Stop arbitration and enable parity */
      outb(0x10 | PARITY_MASK, port_base + TMC_Cntl);
#if DEBUG_RACE
      in_interrupt_flag = 0;
#endif
      return IRQ_HANDLED;
   } else if (current_SC->SCp.phase & in_selection) {
      status = inb(port_base + SCSI_Status);
      if (!(status & 0x01)) {
	 /* Try again, for slow devices */
	 if (fdomain_select( scmd_id(current_SC) )) {
#if EVERY_ACCESS
	    printk( " SFAIL " );
#endif
            spin_lock_irqsave(current_SC->device->host->host_lock, flags);
	    my_done( DID_NO_CONNECT << 16 );
            spin_unlock_irqrestore(current_SC->device->host->host_lock, flags);
	    return IRQ_HANDLED;
	 } else {
#if EVERY_ACCESS
	    printk( " AltSel " );
#endif
	    /* Stop arbitration and enable parity */
	    outb(0x10 | PARITY_MASK, port_base + TMC_Cntl);
	 }
      }
      current_SC->SCp.phase = in_other;
      outb(0x90 | FIFO_COUNT, port_base + Interrupt_Cntl);
      outb(0x80, port_base + SCSI_Cntl);
#if DEBUG_RACE
      in_interrupt_flag = 0;
#endif
      return IRQ_HANDLED;
   }
   
   /* current_SC->SCp.phase == in_other: this is the body of the routine */
   
   status = inb(port_base + SCSI_Status);
   
   if (status & 0x10) {	/* REQ */
      
      switch (status & 0x0e) {
       
      case 0x08:		/* COMMAND OUT */
	 outb(current_SC->cmnd[current_SC->SCp.sent_command++],
	      port_base + Write_SCSI_Data);
#if EVERY_ACCESS
	 printk( "CMD = %x,",
		 current_SC->cmnd[ current_SC->SCp.sent_command - 1] );
#endif
	 break;
      case 0x00:		/* DATA OUT -- tmc18c50/tmc18c30 only */
	 if (chip != tmc1800 && !current_SC->SCp.have_data_in) {
	    current_SC->SCp.have_data_in = -1;
	    outb(0xd0 | PARITY_MASK, port_base + TMC_Cntl);
	 }
	 break;
      case 0x04:		/* DATA IN -- tmc18c50/tmc18c30 only */
	 if (chip != tmc1800 && !current_SC->SCp.have_data_in) {
	    current_SC->SCp.have_data_in = 1;
	    outb(0x90 | PARITY_MASK, port_base + TMC_Cntl);
	 }
	 break;
      case 0x0c:		/* STATUS IN */
	 current_SC->SCp.Status = inb(port_base + Read_SCSI_Data);
#if EVERY_ACCESS
	 printk( "Status = %x, ", current_SC->SCp.Status );
#endif
#if ERRORS_ONLY
	 if (current_SC->SCp.Status
	     && current_SC->SCp.Status != 2
	     && current_SC->SCp.Status != 8) {
	    printk( "scsi: <fdomain> target = %d, command = %x, status = %x\n",
		    current_SC->device->id,
		    current_SC->cmnd[0],
		    current_SC->SCp.Status );
	 }
#endif
	       break;
      case 0x0a:		/* MESSAGE OUT */
	 outb(MESSAGE_REJECT, port_base + Write_SCSI_Data); /* Reject */
	 break;
      case 0x0e:		/* MESSAGE IN */
	 current_SC->SCp.Message = inb(port_base + Read_SCSI_Data);
#if EVERY_ACCESS
	 printk( "Message = %x, ", current_SC->SCp.Message );
#endif
	 if (!current_SC->SCp.Message) ++done;
#if DEBUG_MESSAGES || EVERY_ACCESS
	 if (current_SC->SCp.Message) {
	    printk( "scsi: <fdomain> message = %x\n",
		    current_SC->SCp.Message );
	 }
#endif
	 break;
      }
   }

   if (chip == tmc1800 && !current_SC->SCp.have_data_in
       && (current_SC->SCp.sent_command >= current_SC->cmd_len)) {
      
      if(current_SC->sc_data_direction == DMA_TO_DEVICE)
      {
	 current_SC->SCp.have_data_in = -1;
	 outb(0xd0 | PARITY_MASK, port_base + TMC_Cntl);
      }
      else
      {
	 current_SC->SCp.have_data_in = 1;
	 outb(0x90 | PARITY_MASK, port_base + TMC_Cntl);
      }
   }

   if (current_SC->SCp.have_data_in == -1) { /* DATA OUT */
      while ((data_count = FIFO_Size - inw(port_base + FIFO_Data_Count)) > 512) {
#if EVERY_ACCESS
	 printk( "DC=%d, ", data_count ) ;
#endif
	 if (data_count > current_SC->SCp.this_residual)
	       data_count = current_SC->SCp.this_residual;
	 if (data_count > 0) {
#if EVERY_ACCESS
	    printk( "%d OUT, ", data_count );
#endif
	    if (data_count == 1) {
	       outb(*current_SC->SCp.ptr++, port_base + Write_FIFO);
	       --current_SC->SCp.this_residual;
	    } else {
	       data_count >>= 1;
	       outsw(port_base + Write_FIFO, current_SC->SCp.ptr, data_count);
	       current_SC->SCp.ptr += 2 * data_count;
	       current_SC->SCp.this_residual -= 2 * data_count;
	    }
	 }
	 if (!current_SC->SCp.this_residual) {
	    if (current_SC->SCp.buffers_residual) {
	       --current_SC->SCp.buffers_residual;
	       ++current_SC->SCp.buffer;
	       current_SC->SCp.ptr = sg_virt(current_SC->SCp.buffer);
	       current_SC->SCp.this_residual = current_SC->SCp.buffer->length;
	    } else
		  break;
	 }
      }
   }
   
   if (current_SC->SCp.have_data_in == 1) { /* DATA IN */
      while ((data_count = inw(port_base + FIFO_Data_Count)) > 0) {
#if EVERY_ACCESS
	 printk( "DC=%d, ", data_count );
#endif
	 if (data_count > current_SC->SCp.this_residual)
	       data_count = current_SC->SCp.this_residual;
	 if (data_count) {
#if EVERY_ACCESS
	    printk( "%d IN, ", data_count );
#endif
	    if (data_count == 1) {
	       *current_SC->SCp.ptr++ = inb(port_base + Read_FIFO);
	       --current_SC->SCp.this_residual;
	    } else {
	       data_count >>= 1; /* Number of words */
	       insw(port_base + Read_FIFO, current_SC->SCp.ptr, data_count);
	       current_SC->SCp.ptr += 2 * data_count;
	       current_SC->SCp.this_residual -= 2 * data_count;
	    }
	 }
	 if (!current_SC->SCp.this_residual
	     && current_SC->SCp.buffers_residual) {
	    --current_SC->SCp.buffers_residual;
	    ++current_SC->SCp.buffer;
	    current_SC->SCp.ptr = sg_virt(current_SC->SCp.buffer);
	    current_SC->SCp.this_residual = current_SC->SCp.buffer->length;
	 }
      }
   }
   
   if (done) {
#if EVERY_ACCESS
      printk( " ** IN DONE %d ** ", current_SC->SCp.have_data_in );
#endif

#if ERRORS_ONLY
      if (current_SC->cmnd[0] == REQUEST_SENSE && !current_SC->SCp.Status) {
	      char *buf = scsi_sglist(current_SC);
	 if ((unsigned char)(*(buf + 2)) & 0x0f) {
	    unsigned char key;
	    unsigned char code;
	    unsigned char qualifier;

	    key = (unsigned char)(*(buf + 2)) & 0x0f;
	    code = (unsigned char)(*(buf + 12));
	    qualifier = (unsigned char)(*(buf + 13));

	    if (key != UNIT_ATTENTION
		&& !(key == NOT_READY
		     && code == 0x04
		     && (!qualifier || qualifier == 0x02 || qualifier == 0x01))
		&& !(key == ILLEGAL_REQUEST && (code == 0x25
						|| code == 0x24
						|| !code)))
		  
		  printk( "scsi: <fdomain> REQUEST SENSE"
			  " Key = %x, Code = %x, Qualifier = %x\n",
			  key, code, qualifier );
	 }
      }
#endif
#if EVERY_ACCESS
      printk( "BEFORE MY_DONE. . ." );
#endif
      spin_lock_irqsave(current_SC->device->host->host_lock, flags);
      my_done( (current_SC->SCp.Status & 0xff)
	       | ((current_SC->SCp.Message & 0xff) << 8) | (DID_OK << 16) );
      spin_unlock_irqrestore(current_SC->device->host->host_lock, flags);
#if EVERY_ACCESS
      printk( "RETURNING.\n" );
#endif
      
   } else {
      if (current_SC->SCp.phase & disconnect) {
	 outb(0xd0 | FIFO_COUNT, port_base + Interrupt_Cntl);
	 outb(0x00, port_base + SCSI_Cntl);
      } else {
	 outb(0x90 | FIFO_COUNT, port_base + Interrupt_Cntl);
      }
   }
#if DEBUG_RACE
   in_interrupt_flag = 0;
#endif
   return IRQ_HANDLED;
}

static int fdomain_16x0_queue(struct scsi_cmnd *SCpnt,
		void (*done)(struct scsi_cmnd *))
{
   if (in_command) {
      panic( "scsi: <fdomain> fdomain_16x0_queue() NOT REENTRANT!\n" );
   }
#if EVERY_ACCESS
   printk( "queue: target = %d cmnd = 0x%02x pieces = %d size = %u\n",
	   SCpnt->target,
	   *(unsigned char *)SCpnt->cmnd,
	   scsi_sg_count(SCpnt),
	   scsi_bufflen(SCpnt));
#endif

   fdomain_make_bus_idle();

   current_SC            = SCpnt; /* Save this for the done function */
   current_SC->scsi_done = done;

   /* Initialize static data */

   if (scsi_sg_count(current_SC)) {
	   current_SC->SCp.buffer = scsi_sglist(current_SC);
	   current_SC->SCp.ptr = sg_virt(current_SC->SCp.buffer);
	   current_SC->SCp.this_residual    = current_SC->SCp.buffer->length;
	   current_SC->SCp.buffers_residual = scsi_sg_count(current_SC) - 1;
   } else {
	   current_SC->SCp.ptr              = NULL;
	   current_SC->SCp.this_residual    = 0;
	   current_SC->SCp.buffer           = NULL;
	   current_SC->SCp.buffers_residual = 0;
   }

   current_SC->SCp.Status              = 0;
   current_SC->SCp.Message             = 0;
   current_SC->SCp.have_data_in        = 0;
   current_SC->SCp.sent_command        = 0;
   current_SC->SCp.phase               = in_arbitration;

   /* Start arbitration */
   outb(0x00, port_base + Interrupt_Cntl);
   outb(0x00, port_base + SCSI_Cntl);              /* Disable data drivers */
   outb(adapter_mask, port_base + SCSI_Data_NoACK); /* Set our id bit */
   ++in_command;
   outb(0x20, port_base + Interrupt_Cntl);
   outb(0x14 | PARITY_MASK, port_base + TMC_Cntl); /* Start arbitration */

   return 0;
}

#if DEBUG_ABORT
static void print_info(struct scsi_cmnd *SCpnt)
{
   unsigned int imr;
   unsigned int irr;
   unsigned int isr;

   if (!SCpnt || !SCpnt->device || !SCpnt->device->host) {
      printk(KERN_WARNING "scsi: <fdomain> Cannot provide detailed information\n");
      return;
   }
   
   printk(KERN_INFO "%s\n", fdomain_16x0_info( SCpnt->device->host ) );
   print_banner(SCpnt->device->host);
   switch (SCpnt->SCp.phase) {
   case in_arbitration: printk("arbitration"); break;
   case in_selection:   printk("selection");   break;
   case in_other:       printk("other");       break;
   default:             printk("unknown");     break;
   }

   printk( " (%d), target = %d cmnd = 0x%02x pieces = %d size = %u\n",
	   SCpnt->SCp.phase,
	   SCpnt->device->id,
	   *(unsigned char *)SCpnt->cmnd,
	   scsi_sg_count(SCpnt),
	   scsi_bufflen(SCpnt));
   printk( "sent_command = %d, have_data_in = %d, timeout = %d\n",
	   SCpnt->SCp.sent_command,
	   SCpnt->SCp.have_data_in,
	   SCpnt->timeout );
#if DEBUG_RACE
   printk( "in_interrupt_flag = %d\n", in_interrupt_flag );
#endif

   imr = (inb( 0x0a1 ) << 8) + inb( 0x21 );
   outb( 0x0a, 0xa0 );
   irr = inb( 0xa0 ) << 8;
   outb( 0x0a, 0x20 );
   irr += inb( 0x20 );
   outb( 0x0b, 0xa0 );
   isr = inb( 0xa0 ) << 8;
   outb( 0x0b, 0x20 );
   isr += inb( 0x20 );

				/* Print out interesting information */
   printk( "IMR = 0x%04x", imr );
   if (imr & (1 << interrupt_level))
	 printk( " (masked)" );
   printk( ", IRR = 0x%04x, ISR = 0x%04x\n", irr, isr );

   printk( "SCSI Status      = 0x%02x\n", inb(port_base + SCSI_Status));
   printk( "TMC Status       = 0x%02x", inb(port_base + TMC_Status));
   if (inb((port_base + TMC_Status) & 1))
	 printk( " (interrupt)" );
   printk( "\n" );
   printk("Interrupt Status = 0x%02x", inb(port_base + Interrupt_Status));
   if (inb(port_base + Interrupt_Status) & 0x08)
	 printk( " (enabled)" );
   printk( "\n" );
   if (chip == tmc18c50 || chip == tmc18c30) {
      printk("FIFO Status      = 0x%02x\n", inb(port_base + FIFO_Status));
      printk( "Int. Condition   = 0x%02x\n",
	      inb( port_base + Interrupt_Cond ) );
   }
   printk( "Configuration 1  = 0x%02x\n", inb( port_base + Configuration1 ) );
   if (chip == tmc18c50 || chip == tmc18c30)
	 printk( "Configuration 2  = 0x%02x\n",
		 inb( port_base + Configuration2 ) );
}
#endif

static int fdomain_16x0_abort(struct scsi_cmnd *SCpnt)
{
#if EVERY_ACCESS || ERRORS_ONLY || DEBUG_ABORT
   printk( "scsi: <fdomain> abort " );
#endif

   if (!in_command) {
#if EVERY_ACCESS || ERRORS_ONLY
      printk( " (not in command)\n" );
#endif
      return FAILED;
   } else printk( "\n" );

#if DEBUG_ABORT
   print_info( SCpnt );
#endif

   fdomain_make_bus_idle();
   current_SC->SCp.phase |= aborted;
   current_SC->result = DID_ABORT << 16;
   
   /* Aborts are not done well. . . */
   my_done(DID_ABORT << 16);
   return SUCCESS;
}

int fdomain_16x0_bus_reset(struct scsi_cmnd *SCpnt)
{
   unsigned long flags;

   local_irq_save(flags);

   outb(1, port_base + SCSI_Cntl);
   do_pause( 2 );
   outb(0, port_base + SCSI_Cntl);
   do_pause( 115 );
   outb(0, port_base + SCSI_Mode_Cntl);
   outb(PARITY_MASK, port_base + TMC_Cntl);

   local_irq_restore(flags);
   return SUCCESS;
}

static int fdomain_16x0_biosparam(struct scsi_device *sdev,
		struct block_device *bdev,
		sector_t capacity, int *info_array)
{
   int              drive;
   int		    size      = capacity;
   unsigned long    offset;
   struct drive_info {
      unsigned short cylinders;
      unsigned char  heads;
      unsigned char  sectors;
   } i;
   
   /* NOTES:
      The RAM area starts at 0x1f00 from the bios_base address.

      For BIOS Version 2.0:
      
      The drive parameter table seems to start at 0x1f30.
      The first byte's purpose is not known.
      Next is the cylinder, head, and sector information.
      The last 4 bytes appear to be the drive's size in sectors.
      The other bytes in the drive parameter table are unknown.
      If anyone figures them out, please send me mail, and I will
      update these notes.

      Tape drives do not get placed in this table.

      There is another table at 0x1fea:
      If the byte is 0x01, then the SCSI ID is not in use.
      If the byte is 0x18 or 0x48, then the SCSI ID is in use,
      although tapes don't seem to be in this table.  I haven't
      seen any other numbers (in a limited sample).

      0x1f2d is a drive count (i.e., not including tapes)

      The table at 0x1fcc are I/O ports addresses for the various
      operations.  I calculate these by hand in this driver code.

      
      
      For the ISA-200S version of BIOS Version 2.0:

      The drive parameter table starts at 0x1f33.

      WARNING: Assume that the table entry is 25 bytes long.  Someone needs
      to check this for the Quantum ISA-200S card.

      
      
      For BIOS Version 3.2:

      The drive parameter table starts at 0x1f70.  Each entry is
      0x0a bytes long.  Heads are one less than we need to report.
    */

   if (MAJOR(bdev->bd_dev) != SCSI_DISK0_MAJOR) {
      printk("scsi: <fdomain> fdomain_16x0_biosparam: too many disks");
      return 0;
   }
   drive = MINOR(bdev->bd_dev) >> 4;

   if (bios_major == 2) {
      switch (Quantum) {
      case 2:			/* ISA_200S */
				/* The value of 25 has never been verified.
				   It should probably be 15. */
	 offset = 0x1f33 + drive * 25;
	 break;
      case 3:			/* ISA_250MG */
	 offset = 0x1f36 + drive * 15;
	 break;
      case 4:			/* ISA_200S (another one) */
	 offset = 0x1f34 + drive * 15;
	 break;
      default:
	 offset = 0x1f31 + drive * 25;
	 break;
      }
      memcpy_fromio( &i, bios_mem + offset, sizeof( struct drive_info ) );
      info_array[0] = i.heads;
      info_array[1] = i.sectors;
      info_array[2] = i.cylinders;
   } else if (bios_major == 3
	      && bios_minor >= 0
	      && bios_minor < 4) { /* 3.0 and 3.2 BIOS */
      memcpy_fromio( &i, bios_mem + 0x1f71 + drive * 10,
		     sizeof( struct drive_info ) );
      info_array[0] = i.heads + 1;
      info_array[1] = i.sectors;
      info_array[2] = i.cylinders;
   } else {			/* 3.4 BIOS (and up?) */
      /* This algorithm was provided by Future Domain (much thanks!). */
      unsigned char *p = scsi_bios_ptable(bdev);

      if (p && p[65] == 0xaa && p[64] == 0x55 /* Partition table valid */
	  && p[4]) {			    /* Partition type */

	 /* The partition table layout is as follows:

	    Start: 0x1b3h
	    Offset: 0 = partition status
		    1 = starting head
		    2 = starting sector and cylinder (word, encoded)
		    4 = partition type
		    5 = ending head
		    6 = ending sector and cylinder (word, encoded)
		    8 = starting absolute sector (double word)
		    c = number of sectors (double word)
	    Signature: 0x1fe = 0x55aa

	    So, this algorithm assumes:
	    1) the first partition table is in use,
	    2) the data in the first entry is correct, and
	    3) partitions never divide cylinders

	    Note that (1) may be FALSE for NetBSD (and other BSD flavors),
	    as well as for Linux.  Note also, that Linux doesn't pay any
	    attention to the fields that are used by this algorithm -- it
	    only uses the absolute sector data.  Recent versions of Linux's
	    fdisk(1) will fill this data in correctly, and forthcoming
	    versions will check for consistency.

	    Checking for a non-zero partition type is not part of the
	    Future Domain algorithm, but it seemed to be a reasonable thing
	    to do, especially in the Linux and BSD worlds. */

	 info_array[0] = p[5] + 1;	    /* heads */
	 info_array[1] = p[6] & 0x3f;	    /* sectors */
      } else {

 	 /* Note that this new method guarantees that there will always be
	    less than 1024 cylinders on a platter.  This is good for drives
	    up to approximately 7.85GB (where 1GB = 1024 * 1024 kB). */

	 if ((unsigned int)size >= 0x7e0000U) {
	    info_array[0] = 0xff; /* heads   = 255 */
	    info_array[1] = 0x3f; /* sectors =  63 */
	 } else if ((unsigned int)size >= 0x200000U) {
	    info_array[0] = 0x80; /* heads   = 128 */
	    info_array[1] = 0x3f; /* sectors =  63 */
	 } else {
	    info_array[0] = 0x40; /* heads   =  64 */
	    info_array[1] = 0x20; /* sectors =  32 */
	 }
      }
				/* For both methods, compute the cylinders */
      info_array[2] = (unsigned int)size / (info_array[0] * info_array[1] );
      kfree(p);
   }
   
   return 0;
}

static int fdomain_16x0_release(struct Scsi_Host *shpnt)
{
	if (shpnt->irq)
		free_irq(shpnt->irq, shpnt);
	if (shpnt->io_port && shpnt->n_io_port)
		release_region(shpnt->io_port, shpnt->n_io_port);
	if (PCI_bus)
		pci_dev_put(PCI_dev);
	return 0;
}

struct scsi_host_template fdomain_driver_template = {
	.module			= THIS_MODULE,
	.name			= "fdomain",
	.proc_name		= "fdomain",
	.detect			= fdomain_16x0_detect,
	.info			= fdomain_16x0_info,
	.queuecommand		= fdomain_16x0_queue,
	.eh_abort_handler	= fdomain_16x0_abort,
	.eh_bus_reset_handler	= fdomain_16x0_bus_reset,
	.bios_param		= fdomain_16x0_biosparam,
	.release		= fdomain_16x0_release,
	.can_queue		= 1,
	.this_id		= 6,
	.sg_tablesize		= 64,
	.cmd_per_lun		= 1,
	.use_clustering		= DISABLE_CLUSTERING,
};

#ifndef PCMCIA
#ifdef CONFIG_PCI

static struct pci_device_id fdomain_pci_tbl[] __devinitdata = {
	{ PCI_VENDOR_ID_FD, PCI_DEVICE_ID_FD_36C70,
	  PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0UL },
	{ }
};
MODULE_DEVICE_TABLE(pci, fdomain_pci_tbl);
#endif
#define driver_template fdomain_driver_template
#include "scsi_module.c"

#endif
