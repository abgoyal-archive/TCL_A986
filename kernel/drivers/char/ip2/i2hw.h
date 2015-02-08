
#ifndef I2HW_H
#define I2HW_H 1
//------------------------------------------------------------------------------
// Revision History:
//
// 23 September 1991 MAG   First Draft Started...through...
// 11 October 1991   ...   Continuing development...
//  6 August 1993          Added support for ISA-4 (asic) which is architected
//                         as an ISA-CEX with a single 4-port box.
//
// 20 December 1996  AKM   Version for Linux
//
//------------------------------------------------------------------------------

//------------------------------------------------
// Mandatory Includes:
//------------------------------------------------
//
#include "ip2types.h"

//-------------------------------------------------------------------------
// Manifests for the I/O map:
//-------------------------------------------------------------------------
// R/W: Data port (byte) for IntelliPort-II,
// R/W: Data port (byte or word) for IntelliPort-IIEX
// Incoming or outgoing data passes through a FIFO, the status of which is
// available in some of the bits in FIFO_STATUS. This (bidirectional) FIFO is
// the primary means of transferring data, commands, flow-control, and status
// information between the host and board.
//
#define FIFO_DATA 0

// Another way of passing information between the board and the host is
// through "mailboxes". Unlike a FIFO, a mailbox holds only a single byte of
// data.  Writing data to the mailbox causes a status bit to be set, and
// potentially interrupting the intended receiver. The sender has some way to
// determine whether the data has been read yet; as soon as it has, it may send
// more. The mailboxes are handled differently on -II and -IIEX products, as
// suggested below.
//------------------------------------------------------------------------------
// Read: Status Register for IntelliPort-II or -IIEX
// The presence of any bit set here will cause an interrupt to the host,
// provided the corresponding bit has been unmasked in the interrupt mask
// register. Furthermore, interrupts to the host are disabled globally until the
// loadware selects the irq line to use. With the exception of STN_MR, the bits
// remain set so long as the associated condition is true.
//
#define FIFO_STATUS 2

// Bit map of status bits which are identical for -II and -IIEX
//
#define ST_OUT_FULL  0x40  // Outbound FIFO full
#define ST_IN_EMPTY  0x20  // Inbound FIFO empty
#define ST_IN_MAIL   0x04  // Inbound Mailbox full

// The following exists only on the Intelliport-IIEX, and indicates that the
// board has not read the last outgoing mailbox data yet. In the IntelliPort-II,
// the outgoing mailbox may be read back: a zero indicates the board has read
// the data.
//
#define STE_OUT_MAIL 0x80  // Outbound mailbox full (!)

// The following bits are defined differently for -II and -IIEX boards. Code
// which relies on these bits will need to be functionally different for the two
// types of boards and should be generally avoided because of the additional
// complexity this creates:

// Bit map of status bits only on -II

// Fifo has been RESET (cleared when the status register is read). Note that
// this condition cannot be masked and would always interrupt the host, except
// that the hardware reset also disables interrupts globally from the board
// until re-enabled by loadware. This could also arise from the
// Am4701-supported command to reset the chip, but this command is generally not
// used here.
//
#define STN_MR       0x80

// See the AMD Am4701 data sheet for details on the following four bits. They
// are not presently used by Computone drivers.
//
#define STN_OUT_AF  0x10  // Outbound FIFO almost full (programmable)
#define STN_IN_AE   0x08  // Inbound FIFO almost empty (programmable)
#define STN_BD      0x02  // Inbound byte detected
#define STN_PE      0x01  // Parity/Framing condition detected

// Bit-map of status bits only on -IIEX
//
#define STE_OUT_HF  0x10  // Outbound FIFO half full
#define STE_IN_HF   0x08  // Inbound FIFO half full
#define STE_IN_FULL 0x02  // Inbound FIFO full
#define STE_OUT_MT  0x01  // Outbound FIFO empty

//------------------------------------------------------------------------------

// Intelliport-II -- Write Only: the pointer register.
// Values are written to this register to select the Am4701 internal register to
// be accessed on the next operation.
//
#define FIFO_PTR    0x02

// Values for the pointer register
//
#define SEL_COMMAND 0x1    // Selects the Am4701 command register

// Some possible commands:
//
#define SEL_CMD_MR  0x80	// Am4701 command to reset the chip
#define SEL_CMD_SH  0x40	// Am4701 command to map the "other" port into the
							// status register.
#define SEL_CMD_UNSH   0	// Am4701 command to "unshift": port maps into its
							// own status register.
#define SEL_MASK     0x2	// Selects the Am4701 interrupt mask register. The
							// interrupt mask register is bit-mapped to match 
							// the status register (FIFO_STATUS) except for
							// STN_MR. (See above.)
#define SEL_BYTE_DET 0x3	// Selects the Am4701 byte-detect register. (Not
							// normally used except in diagnostics.)
#define SEL_OUTMAIL  0x4	// Selects the outbound mailbox (R/W). Reading back
							// a value of zero indicates that the mailbox has
							// been read by the board and is available for more
							// data./ Writing to the mailbox optionally
							// interrupts the board, depending on the loadware's
							// setting of its interrupt mask register.
#define SEL_AEAF     0x5	// Selects AE/AF threshold register.
#define SEL_INMAIL   0x6	// Selects the inbound mailbox (Read)

//------------------------------------------------------------------------------
// IntelliPort-IIEX --  Write Only: interrupt mask (and misc flags) register:
// Unlike IntelliPort-II, bit assignments do NOT match those of the status
// register.
//
#define FIFO_MASK    0x2

// Mailbox readback select:
// If set, reads to FIFO_MAIL will read the OUTBOUND mailbox (host to board). If
// clear (default on reset) reads to FIFO_MAIL will read the INBOUND mailbox.
// This is the normal situation. The clearing of a mailbox is determined on
// -IIEX boards by waiting for the STE_OUT_MAIL bit to clear. Readback
// capability is provided for diagnostic purposes only.
//
#define  MX_OUTMAIL_RSEL   0x80

#define  MX_IN_MAIL  0x40	// Enables interrupts when incoming mailbox goes
							// full (ST_IN_MAIL set).
#define  MX_IN_FULL  0x20	// Enables interrupts when incoming FIFO goes full
							// (STE_IN_FULL).
#define  MX_IN_MT    0x08	// Enables interrupts when incoming FIFO goes empty
							// (ST_IN_MT).
#define  MX_OUT_FULL 0x04	// Enables interrupts when outgoing FIFO goes full
							// (ST_OUT_FULL).
#define  MX_OUT_MT   0x01	// Enables interrupts when outgoing FIFO goes empty
							// (STE_OUT_MT).

// Any remaining bits are reserved, and should be written to ZERO for
// compatibility with future Computone products.

//------------------------------------------------------------------------------
// IntelliPort-IIEX: -- These are only 6-bit mailboxes !!! -- 11111100 (low two
// bits always read back 0).
// Read:  One of the mailboxes, usually Inbound.
//        Inbound Mailbox (MX_OUTMAIL_RSEL = 0)
//        Outbound Mailbox (MX_OUTMAIL_RSEL = 1)
// Write: Outbound Mailbox
// For the IntelliPort-II boards, the outbound mailbox is read back to determine
// whether the board has read the data (0 --> data has been read). For the
// IntelliPort-IIEX, this is done by reading a status register. To determine
// whether mailbox is available for more outbound data, use the STE_OUT_MAIL bit
// in FIFO_STATUS. Moreover, although the Outbound Mailbox can be read back by
// setting MX_OUTMAIL_RSEL, it is NOT cleared when the board reads it, as is the
// case with the -II boards. For this reason, FIFO_MAIL is normally used to read
// the inbound FIFO, and MX_OUTMAIL_RSEL kept clear. (See above for
// MX_OUTMAIL_RSEL description.)
//
#define  FIFO_MAIL   0x3

//------------------------------------------------------------------------------
// WRITE ONLY:  Resets the board. (Data doesn't matter).
//
#define  FIFO_RESET  0x7

//------------------------------------------------------------------------------
// READ ONLY:  Will have no effect. (Data is undefined.)
// Actually, there will be an effect, in that the operation is sure to generate
// a bus cycle: viz., an I/O byte Read. This fact can be used to enforce short
// delays when no comparable time constant is available.
//
#define  FIFO_NOP    0x7

//------------------------------------------------------------------------------
// RESET & POWER-ON RESET MESSAGE

//----------------------------------------
// Format of Power-on Reset Message
//----------------------------------------

typedef union _porStr		// "por" stands for Power On Reset
{
	unsigned char  c[16];	// array used when considering the message as a
							// string of undifferentiated characters

	struct					// Elements used when considering values
	{
		// The first two bytes out of the FIFO are two magic numbers. These are
		// intended to establish that there is indeed a member of the
		// IntelliPort-II(EX) family present. The remaining bytes may be 
		// expected // to be valid. When reading the Power-on Reset message, 
		// if the magic numbers do not match it is probably best to stop
		// reading immediately. You are certainly not reading our board (unless
		// hardware is faulty), and may in fact be reading some other piece of
		// hardware.

		unsigned char porMagic1;   // magic number: first byte == POR_MAGIC_1 
		unsigned char porMagic2;   // magic number: second byte == POR_MAGIC_2 

		// The Version, Revision, and Subrevision are stored as absolute numbers
		// and would normally be displayed in the format V.R.S (e.g. 1.0.2)

		unsigned char porVersion;  // Bootstrap firmware version number
		unsigned char porRevision; // Bootstrap firmware revision number
		unsigned char porSubRev;   // Bootstrap firmware sub-revision number

		unsigned char porID;	// Product ID:  Bit-mapped according to
								// conventions described below. Among other
								// things, this allows us to distinguish
								// IntelliPort-II boards from IntelliPort-IIEX
								// boards.

		unsigned char porBus;	// IntelliPort-II: Unused
								// IntelliPort-IIEX: Bus Information:
								// Bit-mapped below

		unsigned char porMemory;	// On-board DRAM size: in 32k blocks

		// porPorts1 (and porPorts2) are used to determine the ports which are
		// available to the board. For non-expandable product, a single number 
		// is sufficient. For expandable product, the board may be connected
		// to as many as four boxes. Each box may be (so far) either a 16-port
		// or an 8-port size. Whenever an 8-port box is used, the remaining 8
		// ports leave gaps between existing channels. For that reason,
		// expandable products must report a MAP of available channels. Since 
		// each UART supports four ports, we represent each UART found by a
		// single bit. Using two bytes to supply the mapping information we
		// report the presense or absense of up to 16 UARTS, or 64 ports in
		// steps of 4 ports. For -IIEX products, the ports are numbered
		// starting at the box closest to the controller in the "chain".

		// Interpreted Differently for IntelliPort-II and -IIEX.
		// -II:   Number of ports (Derived actually from product ID). See
		// Diag1&2 to indicate if uart was actually detected.
		// -IIEX: Bit-map of UARTS found, LSB (see below for MSB of this). This
		//        bitmap is based on detecting the uarts themselves; 
		//        see porFlags for information from the box i.d's.
		unsigned char  porPorts1;

		unsigned char  porDiag1;	// Results of on-board P.O.S.T, 1st byte
		unsigned char  porDiag2;	// Results of on-board P.O.S.T, 2nd byte
		unsigned char  porSpeed;	// Speed of local CPU: given as MHz x10
									// e.g., 16.0 MHz CPU is reported as 160
		unsigned char  porFlags;	// Misc information (see manifests below)
									// Bit-mapped: CPU type, UART's present

		unsigned char  porPorts2;	// -II:  Undefined
									// -IIEX: Bit-map of UARTS found, MSB (see
									//        above for LSB)

		// IntelliPort-II: undefined
		// IntelliPort-IIEX: 1 << porFifoSize gives the size, in bytes, of the
		// host interface FIFO, in each direction. When running the -IIEX in
		// 8-bit mode, fifo capacity is halved. The bootstrap firmware will
		// have already accounted for this fact in generating this number.
		unsigned char  porFifoSize;

		// IntelliPort-II: undefined
		// IntelliPort-IIEX: The number of boxes connected. (Presently 1-4)
		unsigned char  porNumBoxes;
	} e;
} porStr, *porStrPtr;

//--------------------------
// Values for porStr fields
//--------------------------

//---------------------
// porMagic1, porMagic2
//----------------------
//
#define  POR_MAGIC_1    0x96  // The only valid value for porMagic1
#define  POR_MAGIC_2    0x35  // The only valid value for porMagic2
#define  POR_1_INDEX    0     // Byte position of POR_MAGIC_1
#define  POR_2_INDEX    1     // Ditto for POR_MAGIC_2

//----------------------
// porID
//----------------------
//
#define  POR_ID_FAMILY  0xc0	// These bits indicate the general family of
								// product.
#define  POR_ID_FII     0x00	// Family is "IntelliPort-II"
#define  POR_ID_FIIEX   0x40	// Family is "IntelliPort-IIEX"

// These bits are reserved, presently zero. May be used at a later date to
// convey other product information.
//
#define POR_ID_RESERVED 0x3c

#define POR_ID_SIZE     0x03	// Remaining bits indicate number of ports &
								// Connector information.
#define POR_ID_II_8     0x00	// For IntelliPort-II, indicates 8-port using
								// standard brick.
#define POR_ID_II_8R    0x01	// For IntelliPort-II, indicates 8-port using
								// RJ11's (no CTS)
#define POR_ID_II_6     0x02	// For IntelliPort-II, indicates 6-port using
								// RJ45's
#define POR_ID_II_4     0x03	// For IntelliPort-II, indicates 4-port using
								// 4xRJ45 connectors
#define POR_ID_EX       0x00	// For IntelliPort-IIEX, indicates standard
								// expandable controller (other values reserved)

//----------------------
// porBus
//----------------------

// IntelliPort-IIEX only: Board is installed in a 16-bit slot
//
#define POR_BUS_SLOT16  0x20

// IntelliPort-IIEX only: DIP switch #8 is on, selecting 16-bit host interface
// operation.
// 
#define POR_BUS_DIP16   0x10

// Bits 0-2 indicate type of bus: This information is stored in the bootstrap
// loadware, different loadware being used on different products for different
// buses. For most situations, the drivers do not need this information; but it
// is handy in a diagnostic environment. For example, on microchannel boards,
// you would not want to try to test several interrupts, only the one for which
// you were configured.
//
#define  POR_BUS_TYPE   0x07

// Unknown:  this product doesn't know what bus it is running in. (e.g. if same
// bootstrap firmware were wanted for two different buses.)
//
#define  POR_BUS_T_UNK  0

// Note: existing firmware for ISA-8 and MC-8 currently report the POR_BUS_T_UNK
// state, since the same bootstrap firmware is used for each.

#define  POR_BUS_T_MCA  1  // MCA BUS */
#define  POR_BUS_T_EISA 2  // EISA BUS */
#define  POR_BUS_T_ISA  3  // ISA BUS */

// Values 4-7 Reserved

// Remaining bits are reserved

//----------------------
// porDiag1
//----------------------

#define  POR_BAD_MAPPER 0x80	// HW failure on P.O.S.T: Chip mapper failed

// These two bits valid only for the IntelliPort-II
//
#define  POR_BAD_UART1  0x01	// First  1400 bad
#define  POR_BAD_UART2  0x02	// Second 1400 bad

//----------------------
// porDiag2
//----------------------

#define  POR_DEBUG_PORT 0x80	// debug port was detected by the P.O.S.T
#define  POR_DIAG_OK    0x00	// Indicates passage: Failure codes not yet
								// available.
								// Other bits undefined.
//----------------------
// porFlags
//----------------------

#define  POR_CPU     0x03	// These bits indicate supposed CPU type
#define  POR_CPU_8   0x01	// Board uses an 80188 (no such thing yet)
#define  POR_CPU_6   0x02	// Board uses an 80186 (all existing products)
#define  POR_CEX4    0x04	// If set, this is an ISA-CEX/4: An ISA-4 (asic)
							// which is architected like an ISA-CEX connected
							// to a (hitherto impossible) 4-port box.
#define POR_BOXES    0xf0	// Valid for IntelliPort-IIEX only: Map of Box
							// sizes based on box I.D.
#define POR_BOX_16   0x10	// Set indicates 16-port, clear 8-port

//-------------------------------------
// LOADWARE and DOWNLOADING CODE
//-------------------------------------



//-----------------------------------------------------------------------------
// Format of loadware block
//
// This is defined as a union so we can pass a pointer to one of these items
// and (if it is the first block) pick out the version information, etc.
//
// Otherwise, to deal with this as a simple character array
//------------------------------------------------------------------------------

#define LOADWARE_BLOCK_SIZE   512   // Number of bytes in each block of loadware

typedef union _loadHdrStr
{
	unsigned char c[LOADWARE_BLOCK_SIZE];  // Valid for every block

	struct	// These fields are valid for only the first block of loadware.
	{
		unsigned char loadMagic;		// Magic number: see below
		unsigned char loadBlocksMore;	// How many more blocks?
		unsigned char loadCRC[2];		// Two CRC bytes: used by loader
		unsigned char loadVersion;		// Version number
		unsigned char loadRevision;		// Revision number
		unsigned char loadSubRevision;	// Sub-revision number
		unsigned char loadSpares[9];	// Presently unused
		unsigned char loadDates[32];	// Null-terminated string which can give
										// date and time of compilation
	} e;
} loadHdrStr, *loadHdrStrPtr;

//------------------------------------
// Defines for downloading code:
//------------------------------------

// The loadMagic field in the first block of the loadfile must be this, else the
// file is not valid.
//
#define  MAGIC_LOADFILE 0x3c

// How do we know the load was successful? On completion of the load, the
// bootstrap firmware returns a code to indicate whether it thought the download
// was valid and intends to execute it. These are the only possible valid codes:
//
#define  LOADWARE_OK    0xc3        // Download was ok
#define  LOADWARE_BAD   0x5a        // Download was bad (CRC error)

// Constants applicable to writing blocks of loadware:
// The first block of loadware might take 600 mS to load, in extreme cases.
// (Expandable board: worst case for sending startup messages to the LCD's).
// The 600mS figure is not really a calculation, but a conservative
// guess/guarantee. Usually this will be within 100 mS, like subsequent blocks.
//
#define  MAX_DLOAD_START_TIME 1000  // 1000 mS
#define  MAX_DLOAD_READ_TIME  100   // 100 mS

// Firmware should respond with status (see above) within this long of host
// having sent the final block.
//
#define  MAX_DLOAD_ACK_TIME   100   // 100 mS, again!

//------------------------------------------------------
// MAXIMUM NUMBER OF PORTS PER BOARD:
// This is fixed for now (with the expandable), but may
// be expanding according to even newer products.
//------------------------------------------------------
//
#define ABS_MAX_BOXES   4     // Absolute most boxes per board
#define ABS_BIGGEST_BOX 16    // Absolute the most ports per box
#define ABS_MOST_PORTS  (ABS_MAX_BOXES * ABS_BIGGEST_BOX)

#define I2_OUTSW(port, addr, count)	outsw((port), (addr), (((count)+1)/2))
#define I2_OUTSB(port, addr, count)	outsb((port), (addr), (((count)+1))&-2)
#define I2_INSW(port, addr, count)	insw((port), (addr), (((count)+1)/2))
#define I2_INSB(port, addr, count)	insb((port), (addr), (((count)+1))&-2)

#endif   // I2HW_H

