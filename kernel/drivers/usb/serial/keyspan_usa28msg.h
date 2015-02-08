

#ifndef	__USA28MSG__
#define	__USA28MSG__


struct keyspan_usa28_portControlMessage
{
	/*
		there are four types of "commands" sent in the control message:

		1.	configuration changes which must be requested by setting
			the corresponding "set" flag (and should only be requested
			when necessary, to reduce overhead on the USA28):
	*/
	u8	setBaudRate,	// 0=don't set, 1=baudLo/Hi, 2=115.2K, 3=230.4K
		baudLo,			// host does baud divisor calculation
		baudHi;			// baudHi is only used for first port (gives lower rates)

	/*
		2.	configuration changes which are done every time (because it's
			hardly more trouble to do them than to check whether to do them):
	*/
	u8	parity,			// 1=use parity, 0=don't
		ctsFlowControl,	        // all except 19Q: 1=use CTS flow control, 0=don't
					// 19Q: 0x08:CTSflowControl 0x10:DSRflowControl
		xonFlowControl,	// 1=use XON/XOFF flow control, 0=don't
		rts,			// 1=on, 0=off
		dtr;			// 1=on, 0=off

	/*
		3.	configuration data which is simply used as is (no overhead,
			but must be correct in every host message).
	*/
	u8	forwardingLength,  // forward when this number of chars available
		forwardMs,		// forward this many ms after last rx data
		breakThreshold,	// specified in ms, 1-255 (see note below)
		xonChar,		// specified in current character format
		xoffChar;		// specified in current character format

	/*
		4.	commands which are flags only; these are processed in order
			(so that, e.g., if both _txOn and _txOff flags are set, the
			port ends in a TX_OFF state); any non-zero value is respected
	*/
	u8	_txOn,			// enable transmitting (and continue if there's data)
		_txOff,			// stop transmitting
		txFlush,		// toss outbound data
		txForceXoff,	// pretend we've received XOFF
		txBreak,		// turn on break (leave on until txOn clears it)
		rxOn,			// turn on receiver
		rxOff,			// turn off receiver
		rxFlush,		// toss inbound data
		rxForward,		// forward all inbound data, NOW
		returnStatus,	// return current status n times (1 or 2)
		resetDataToggle;// reset data toggle state to DATA0
	
};

struct keyspan_usa28_portStatusMessage
{
	u8	port,			// 0=first, 1=second, 2=global (see below)
		cts,
		dsr,			// (not used in all products)
		dcd,

		ri,				// (not used in all products)
		_txOff,			// port has been disabled (by host)
		_txXoff,		// port is in XOFF state (either host or RX XOFF)
		dataLost,		// count of lost chars; wraps; not guaranteed exact

		rxEnabled,		// as configured by rxOn/rxOff 1=on, 0=off
		rxBreak,		// 1=we're in break state
		rs232invalid,	// 1=no valid signals on rs-232 inputs
		controlResponse;// 1=a control messages has been processed
};

// bit defines in txState
#define	TX_OFF			0x01	// requested by host txOff command
#define	TX_XOFF			0x02	// either real, or simulated by host

struct keyspan_usa28_globalControlMessage
{
	u8	sendGlobalStatus,	// 2=request for two status responses
		resetStatusToggle,	// 1=reset global status toggle
		resetStatusCount;	// a cycling value
};

struct keyspan_usa28_globalStatusMessage
{
	u8	port,				// 3
		sendGlobalStatus,	// from request, decremented
		resetStatusCount;	// as in request
};

struct keyspan_usa28_globalDebugMessage
{
	u8	port,				// 2
		n,					// typically a count/status byte
		b;					// typically a data byte
};

// ie: the maximum length of an EZUSB endpoint buffer
#define	MAX_DATA_LEN			64

// the parity bytes have only one significant bit
#define	RX_PARITY_BIT			0x04
#define	TX_PARITY_BIT			0x01

// update status approx. 60 times a second (16.6666 ms)
#define	STATUS_UPDATE_INTERVAL	16

#endif

