

#ifndef	__USA49MSG__
#define	__USA49MSG__



struct keyspan_usa49_portControlMessage
{
	/*
		0.	0/1/2/3 	port control message follows
			0x80 set	non-port control message follows
	*/
	u8	portNumber,

	/*
		there are three types of "commands" sent in the control message:

		1.	configuration changes which must be requested by setting
			the corresponding "set" flag (and should only be requested
			when necessary, to reduce overhead on the USA26):
	*/
		setClocking,	// host requests baud rate be set
		baudLo,			// host does baud divisor calculation
		baudHi,			// baudHi is only used for first port (gives lower rates)
		prescaler,		// specified as N/8; values 8-ff are valid
						// must be set any time internal baud rate is set;
		txClocking,		// 0=internal, 1=external/DSR
		rxClocking,		// 0=internal, 1=external/DSR

		setLcr,			// host requests lcr be set
		lcr,			// use PARITY, STOPBITS, DATABITS below

		setFlowControl,	// host requests flow control be set
		ctsFlowControl,	// 1=use CTS flow control, 0=don't
		xonFlowControl,	// 1=use XON/XOFF flow control, 0=don't
		xonChar,		// specified in current character format
		xoffChar,		// specified in current character format

		setRts,			// host requests RTS output be set
		rts,			// 1=active, 0=inactive

		setDtr,			// host requests DTR output be set
		dtr;			// 1=on, 0=off


	/*
		3.	configuration data which is simply used as is (no overhead,
			but must be specified correctly in every host message).
	*/
	u8	forwardingLength,  // forward when this number of chars available
		dsrFlowControl,	// 1=use DSR flow control, 0=don't
		txAckThreshold,	// 0=not allowed, 1=normal, 2-255 deliver ACK faster
		loopbackMode;	// 0=no loopback, 1=loopback enabled

	/*
		4.	commands which are flags only; these are processed in order
			(so that, e.g., if both _txOn and _txOff flags are set, the
			port ends in a TX_OFF state); any non-zero value is respected
	*/
	u8	_txOn,			// enable transmitting (and continue if there's data)
		_txOff,			// stop transmitting
		txFlush,		// toss outbound data
		txBreak,		// turn on break (cleared by _txOn)
		rxOn,			// turn on receiver
		rxOff,			// turn off receiver
		rxFlush,		// toss inbound data
		rxForward,		// forward all inbound data, NOW (as if fwdLen==1)
		returnStatus,	// return current status (even if it hasn't changed)
		resetDataToggle,// reset data toggle state to DATA0
		enablePort,		// start servicing port (move data, check status)
		disablePort;	// stop servicing port (does implicit tx/rx flush/off)
	
};

// defines for bits in lcr
#define	USA_DATABITS_5		0x00
#define	USA_DATABITS_6		0x01
#define	USA_DATABITS_7		0x02
#define	USA_DATABITS_8		0x03
#define	STOPBITS_5678_1		0x00	// 1 stop bit for all byte sizes
#define	STOPBITS_5_1p5		0x04	// 1.5 stop bits for 5-bit byte
#define	STOPBITS_678_2		0x04	// 2 stop bits for 6/7/8-bit byte
#define	USA_PARITY_NONE		0x00
#define	USA_PARITY_ODD		0x08
#define	USA_PARITY_EVEN		0x18
#define	PARITY_1			0x28
#define	PARITY_0			0x38


struct keyspan_usa49_globalControlMessage
{
	u8	portNumber,			// 0x80
		sendGlobalStatus,	// 1/2=number of status responses requested
		resetStatusToggle,	// 1=reset global status toggle
		resetStatusCount,	// a cycling value
		remoteWakeupEnable,		// 0x10=P1, 0x20=P2, 0x40=P3, 0x80=P4
		disableStatusMessages;	// 1=send no status until host talks
};


struct keyspan_usa49_portStatusMessage	// one for each port
{
	u8	portNumber,		// 0,1,2,3
		cts,			// reports CTS pin
		dcd,			// reports DCD pin
		dsr,			// reports DSR pin
		ri,				// reports RI pin
		_txOff,			// transmit has been disabled (by host)
		_txXoff,		// transmit is in XOFF state (either host or RX XOFF)
		rxEnabled,		// as configured by rxOn/rxOff 1=on, 0=off
		controlResponse,// 1=a control message has been processed
		txAck,			// ACK (data TX complete)
		rs232valid;		// RS-232 signal valid
};

// bits in RX data message when STAT byte is included
#define	RXERROR_OVERRUN	0x02
#define	RXERROR_PARITY	0x04
#define	RXERROR_FRAMING	0x08
#define	RXERROR_BREAK	0x10

struct keyspan_usa49_globalStatusMessage
{
	u8	portNumber,			// 0x80=globalStatusMessage
		sendGlobalStatus,	// from request, decremented
		resetStatusCount;	// as in request
};

struct keyspan_usa49_globalDebugMessage
{
	u8	portNumber,			// 0x81=globalDebugMessage
		n,					// typically a count/status byte
		b;					// typically a data byte
};

// ie: the maximum length of an EZUSB endpoint buffer
#define	MAX_DATA_LEN			64

// update status approx. 60 times a second (16.6666 ms)
#define	STATUS_UPDATE_INTERVAL	16

// status rationing tuning value (each port gets checked each n ms)
#define	STATUS_RATION	10

#endif
