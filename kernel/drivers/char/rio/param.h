

#ifndef __rio_param_h__
#define __rio_param_h__


struct phb_param {
	u8 Cmd;			/* It is very important that these line up */
	u8 Cor1;		/* with what is expected at the other end. */
	u8 Cor2;		/* to confirm that you've got it right,    */
	u8 Cor4;		/* check with cirrus/cirrus.h              */
	u8 Cor5;
	u8 TxXon;		/* Transmit X-On character */
	u8 TxXoff;		/* Transmit X-Off character */
	u8 RxXon;		/* Receive X-On character */
	u8 RxXoff;		/* Receive X-Off character */
	u8 LNext;		/* Literal-next character */
	u8 TxBaud;		/* Transmit baudrate */
	u8 RxBaud;		/* Receive baudrate */
};

#endif
