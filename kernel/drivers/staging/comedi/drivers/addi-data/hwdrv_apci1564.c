


#include <linux/delay.h>
#include "hwdrv_apci1564.h"

/* Global variables */
static unsigned int ui_InterruptStatus_1564 = 0;
static unsigned int ui_InterruptData, ui_Type;

int i_APCI1564_ConfigDigitalInput(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	devpriv->tsk_Current = current;
   /*******************************/
	/* Set the digital input logic */
   /*******************************/
	if (data[0] == ADDIDATA_ENABLE) {
		data[2] = data[2] << 4;
		data[3] = data[3] << 4;
		outl(data[2],
			devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP +
			APCI1564_DIGITAL_IP_INTERRUPT_MODE1);
		outl(data[3],
			devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP +
			APCI1564_DIGITAL_IP_INTERRUPT_MODE2);
		if (data[1] == ADDIDATA_OR) {
			outl(0x4,
				devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP +
				APCI1564_DIGITAL_IP_IRQ);
		}		/*  if  (data[1] == ADDIDATA_OR) */
		else {
			outl(0x6,
				devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP +
				APCI1564_DIGITAL_IP_IRQ);
		}		/*  else if  (data[1] == ADDIDATA_OR) */
	}			/*  if  (data[0] == ADDIDATA_ENABLE) */
	else {
		outl(0x0,
			devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP +
			APCI1564_DIGITAL_IP_INTERRUPT_MODE1);
		outl(0x0,
			devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP +
			APCI1564_DIGITAL_IP_INTERRUPT_MODE2);
		outl(0x0,
			devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP +
			APCI1564_DIGITAL_IP_IRQ);
	}			/*  else if  (data[0] == ADDIDATA_ENABLE) */

	return insn->n;
}

int i_APCI1564_Read1DigitalInput(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ui_TmpValue = 0;
	unsigned int ui_Channel;

	ui_Channel = CR_CHAN(insn->chanspec);
	if (ui_Channel <= 31) {
		ui_TmpValue =
			(unsigned int) inl(devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP);
		*data = (ui_TmpValue >> ui_Channel) & 0x1;
	}			/*  if  (ui_Channel >= 0 && ui_Channel <=31) */
	else {
		comedi_error(dev, "Not a valid channel number !!! \n");
		return -EINVAL;	/*  "sorry channel spec wrong " */
	}			/* else if  (ui_Channel >= 0 && ui_Channel <=31) */
	return insn->n;
}

int i_APCI1564_ReadMoreDigitalInput(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ui_PortValue = data[0];
	unsigned int ui_Mask = 0;
	unsigned int ui_NoOfChannels;

	ui_NoOfChannels = CR_CHAN(insn->chanspec);
	if (data[1] == 0) {
		*data = (unsigned int) inl(devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP);
		switch (ui_NoOfChannels) {
		case 2:
			ui_Mask = 3;
			*data = (*data >> (2 * ui_PortValue)) & ui_Mask;
			break;
		case 4:
			ui_Mask = 15;
			*data = (*data >> (4 * ui_PortValue)) & ui_Mask;
			break;
		case 8:
			ui_Mask = 255;
			*data = (*data >> (8 * ui_PortValue)) & ui_Mask;
			break;
		case 16:
			ui_Mask = 65535;
			*data = (*data >> (16 * ui_PortValue)) & ui_Mask;
			break;
		case 31:
			break;
		default:
			comedi_error(dev, "Not a valid Channel number !!!\n");
			return -EINVAL;	/*  "sorry channel spec wrong " */
			break;
		}		/*  switch  (ui_NoOfChannels) */
	}			/*  if  (data[1]==0) */
	else {
		if (data[1] == 1) {
			*data = ui_InterruptStatus_1564;
		}		/*  if  (data[1]==1) */
	}			/*  else if  (data[1]==0) */
	return insn->n;
}

int i_APCI1564_ConfigDigitalOutput(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ul_Command = 0;

	if ((data[0] != 0) && (data[0] != 1)) {
		comedi_error(dev,
			"Not a valid Data !!! ,Data should be 1 or 0\n");
		return -EINVAL;
	}			/*  if  ((data[0]!=0) && (data[0]!=1)) */
	if (data[0]) {
		devpriv->b_OutputMemoryStatus = ADDIDATA_ENABLE;
	}			/*  if  (data[0]) */
	else {
		devpriv->b_OutputMemoryStatus = ADDIDATA_DISABLE;
	}			/*  else if  (data[0]) */
	if (data[1] == ADDIDATA_ENABLE) {
		ul_Command = ul_Command | 0x1;
	}			/*  if  (data[1] == ADDIDATA_ENABLE) */
	else {
		ul_Command = ul_Command & 0xFFFFFFFE;
	}			/*  else if  (data[1] == ADDIDATA_ENABLE) */
	if (data[2] == ADDIDATA_ENABLE) {
		ul_Command = ul_Command | 0x2;
	}			/*  if  (data[2] == ADDIDATA_ENABLE) */
	else {
		ul_Command = ul_Command & 0xFFFFFFFD;
	}			/*  else if  (data[2] == ADDIDATA_ENABLE) */
	outl(ul_Command,
		devpriv->i_IobaseAmcc + APCI1564_DIGITAL_OP +
		APCI1564_DIGITAL_OP_INTERRUPT);
	ui_InterruptData =
		inl(devpriv->i_IobaseAmcc + APCI1564_DIGITAL_OP +
		APCI1564_DIGITAL_OP_INTERRUPT);
	devpriv->tsk_Current = current;
	return insn->n;
}

int i_APCI1564_WriteDigitalOutput(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ui_Temp, ui_Temp1;
	unsigned int ui_NoOfChannel;

	ui_NoOfChannel = CR_CHAN(insn->chanspec);
	if (devpriv->b_OutputMemoryStatus) {
		ui_Temp =
			inl(devpriv->i_IobaseAmcc + APCI1564_DIGITAL_OP +
			APCI1564_DIGITAL_OP_RW);
	}			/*  if  (devpriv->b_OutputMemoryStatus ) */
	else {
		ui_Temp = 0;
	}			/*  else if  (devpriv->b_OutputMemoryStatus ) */
	if (data[3] == 0) {
		if (data[1] == 0) {
			data[0] = (data[0] << ui_NoOfChannel) | ui_Temp;
			outl(data[0],
				devpriv->i_IobaseAmcc + APCI1564_DIGITAL_OP +
				APCI1564_DIGITAL_OP_RW);
		}		/*  if  (data[1]==0) */
		else {
			if (data[1] == 1) {
				switch (ui_NoOfChannel) {
				case 2:
					data[0] =
						(data[0] << (2 *
							data[2])) | ui_Temp;
					break;
				case 4:
					data[0] =
						(data[0] << (4 *
							data[2])) | ui_Temp;
					break;
				case 8:
					data[0] =
						(data[0] << (8 *
							data[2])) | ui_Temp;
					break;
				case 16:
					data[0] =
						(data[0] << (16 *
							data[2])) | ui_Temp;
					break;
				case 31:
					data[0] = data[0] | ui_Temp;
					break;
				default:
					comedi_error(dev, " chan spec wrong");
					return -EINVAL;	/*  "sorry channel spec wrong " */
				}	/*  switch (ui_NoOfChannels) */
				outl(data[0],
					devpriv->i_IobaseAmcc +
					APCI1564_DIGITAL_OP +
					APCI1564_DIGITAL_OP_RW);
			}	/*  if  (data[1]==1) */
			else {
				printk("\nSpecified channel not supported\n");
			}	/*  else if  (data[1]==1) */
		}		/*  else if (data[1]==0) */
	}			/* if(data[3]==0) */
	else {
		if (data[3] == 1) {
			if (data[1] == 0) {
				data[0] = ~data[0] & 0x1;
				ui_Temp1 = 1;
				ui_Temp1 = ui_Temp1 << ui_NoOfChannel;
				ui_Temp = ui_Temp | ui_Temp1;
				data[0] =
					(data[0] << ui_NoOfChannel) ^
					0xffffffff;
				data[0] = data[0] & ui_Temp;
				outl(data[0],
					devpriv->i_IobaseAmcc +
					APCI1564_DIGITAL_OP +
					APCI1564_DIGITAL_OP_RW);
			}	/*  if  (data[1]==0) */
			else {
				if (data[1] == 1) {
					switch (ui_NoOfChannel) {
					case 2:
						data[0] = ~data[0] & 0x3;
						ui_Temp1 = 3;
						ui_Temp1 =
							ui_Temp1 << 2 * data[2];
						ui_Temp = ui_Temp | ui_Temp1;
						data[0] =
							((data[0] << (2 *
									data
									[2])) ^
							0xffffffff) & ui_Temp;
						break;
					case 4:
						data[0] = ~data[0] & 0xf;
						ui_Temp1 = 15;
						ui_Temp1 =
							ui_Temp1 << 4 * data[2];
						ui_Temp = ui_Temp | ui_Temp1;
						data[0] =
							((data[0] << (4 *
									data
									[2])) ^
							0xffffffff) & ui_Temp;
						break;
					case 8:
						data[0] = ~data[0] & 0xff;
						ui_Temp1 = 255;
						ui_Temp1 =
							ui_Temp1 << 8 * data[2];
						ui_Temp = ui_Temp | ui_Temp1;
						data[0] =
							((data[0] << (8 *
									data
									[2])) ^
							0xffffffff) & ui_Temp;
						break;
					case 16:
						data[0] = ~data[0] & 0xffff;
						ui_Temp1 = 65535;
						ui_Temp1 =
							ui_Temp1 << 16 *
							data[2];
						ui_Temp = ui_Temp | ui_Temp1;
						data[0] =
							((data[0] << (16 *
									data
									[2])) ^
							0xffffffff) & ui_Temp;
						break;
					case 31:
						break;
					default:
						comedi_error(dev,
							" chan spec wrong");
						return -EINVAL;	/*  "sorry channel spec wrong " */
					}	/* switch(ui_NoOfChannels) */
					outl(data[0],
						devpriv->i_IobaseAmcc +
						APCI1564_DIGITAL_OP +
						APCI1564_DIGITAL_OP_RW);
				}	/*  if  (data[1]==1) */
				else {
					printk("\nSpecified channel not supported\n");
				}	/*  else if  (data[1]==1) */
			}	/*  else if  (data[1]==0) */
		}		/*  if  (data[3]==1); */
		else {
			printk("\nSpecified functionality does not exist\n");
			return -EINVAL;
		}		/*  else if (data[3]==1) */
	}			/*  else if (data[3]==0) */
	return insn->n;
}

int i_APCI1564_ReadDigitalOutput(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ui_Temp;
	unsigned int ui_NoOfChannel;

	ui_NoOfChannel = CR_CHAN(insn->chanspec);
	ui_Temp = data[0];
	*data = inl(devpriv->i_IobaseAmcc + APCI1564_DIGITAL_OP +
		APCI1564_DIGITAL_OP_RW);
	if (ui_Temp == 0) {
		*data = (*data >> ui_NoOfChannel) & 0x1;
	}			/*  if  (ui_Temp==0) */
	else {
		if (ui_Temp == 1) {
			switch (ui_NoOfChannel) {
			case 2:
				*data = (*data >> (2 * data[1])) & 3;
				break;

			case 4:
				*data = (*data >> (4 * data[1])) & 15;
				break;

			case 8:
				*data = (*data >> (8 * data[1])) & 255;
				break;

			case 16:
				*data = (*data >> (16 * data[1])) & 65535;
				break;

			case 31:
				break;

			default:
				comedi_error(dev, " chan spec wrong");
				return -EINVAL;	/*  "sorry channel spec wrong " */
				break;
			}	/*  switch(ui_NoOfChannels) */
		}		/*  if  (ui_Temp==1) */
		else {
			printk("\nSpecified channel not supported \n");
		}		/*  else if (ui_Temp==1) */
	}			/*  else if  (ui_Temp==0) */
	return insn->n;
}

int i_APCI1564_ConfigTimerCounterWatchdog(struct comedi_device *dev,
	struct comedi_subdevice *s, struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ul_Command1 = 0;
	devpriv->tsk_Current = current;
	if (data[0] == ADDIDATA_WATCHDOG) {
		devpriv->b_TimerSelectMode = ADDIDATA_WATCHDOG;

		/* Disable the watchdog */
		outl(0x0,
			devpriv->i_IobaseAmcc + APCI1564_DIGITAL_OP_WATCHDOG +
			APCI1564_TCW_PROG);
		/* Loading the Reload value */
		outl(data[3],
			devpriv->i_IobaseAmcc + APCI1564_DIGITAL_OP_WATCHDOG +
			APCI1564_TCW_RELOAD_VALUE);
	}			/*  if  (data[0]==ADDIDATA_WATCHDOG) */
	else if (data[0] == ADDIDATA_TIMER) {
		/* First Stop The Timer */
		ul_Command1 =
			inl(devpriv->i_IobaseAmcc + APCI1564_TIMER +
			APCI1564_TCW_PROG);
		ul_Command1 = ul_Command1 & 0xFFFFF9FEUL;
		outl(ul_Command1, devpriv->i_IobaseAmcc + APCI1564_TIMER + APCI1564_TCW_PROG);	/* Stop The Timer */

		devpriv->b_TimerSelectMode = ADDIDATA_TIMER;
		if (data[1] == 1) {
			outl(0x02, devpriv->i_IobaseAmcc + APCI1564_TIMER + APCI1564_TCW_PROG);	/* Enable TIMER int & DISABLE ALL THE OTHER int SOURCES */
			outl(0x0,
				devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP +
				APCI1564_DIGITAL_IP_IRQ);
			outl(0x0,
				devpriv->i_IobaseAmcc + APCI1564_DIGITAL_OP +
				APCI1564_DIGITAL_OP_IRQ);
			outl(0x0,
				devpriv->i_IobaseAmcc +
				APCI1564_DIGITAL_OP_WATCHDOG +
				APCI1564_TCW_IRQ);
			outl(0x0,
				devpriv->iobase + APCI1564_COUNTER1 +
				APCI1564_TCW_IRQ);
			outl(0x0,
				devpriv->iobase + APCI1564_COUNTER2 +
				APCI1564_TCW_IRQ);
			outl(0x0,
				devpriv->iobase + APCI1564_COUNTER3 +
				APCI1564_TCW_IRQ);
			outl(0x0,
				devpriv->iobase + APCI1564_COUNTER4 +
				APCI1564_TCW_IRQ);
		}		/*  if  (data[1]==1) */
		else {
			outl(0x0, devpriv->i_IobaseAmcc + APCI1564_TIMER + APCI1564_TCW_PROG);	/* disable Timer interrupt */
		}		/*  else if  (data[1]==1) */

		/*  Loading Timebase */

		outl(data[2],
			devpriv->i_IobaseAmcc + APCI1564_TIMER +
			APCI1564_TCW_TIMEBASE);

		/* Loading the Reload value */
		outl(data[3],
			devpriv->i_IobaseAmcc + APCI1564_TIMER +
			APCI1564_TCW_RELOAD_VALUE);

		ul_Command1 =
			inl(devpriv->i_IobaseAmcc + APCI1564_TIMER +
			APCI1564_TCW_PROG);
		ul_Command1 =
			(ul_Command1 & 0xFFF719E2UL) | 2UL << 13UL | 0x10UL;
		outl(ul_Command1, devpriv->i_IobaseAmcc + APCI1564_TIMER + APCI1564_TCW_PROG);	/* mode 2 */
	}			/*  else if  (data[0]==ADDIDATA_TIMER) */
	else if (data[0] == ADDIDATA_COUNTER) {
		devpriv->b_TimerSelectMode = ADDIDATA_COUNTER;
		devpriv->b_ModeSelectRegister = data[5];

		/* First Stop The Counter */
		ul_Command1 =
			inl(devpriv->iobase + ((data[5] - 1) * 0x20) +
			APCI1564_TCW_PROG);
		ul_Command1 = ul_Command1 & 0xFFFFF9FEUL;
		outl(ul_Command1, devpriv->iobase + ((data[5] - 1) * 0x20) + APCI1564_TCW_PROG);	/* Stop The Timer */

      /************************/
		/* Set the reload value */
      /************************/
		outl(data[3],
			devpriv->iobase + ((data[5] - 1) * 0x20) +
			APCI1564_TCW_RELOAD_VALUE);

      /******************************/
		/* Set the mode :             */
		/* - Disable the hardware     */
		/* - Disable the counter mode */
		/* - Disable the warning      */
		/* - Disable the reset        */
		/* - Disable the timer mode   */
		/* - Enable the counter mode  */
      /******************************/
		ul_Command1 =
			(ul_Command1 & 0xFFFC19E2UL) | 0x80000UL |
			(unsigned int) ((unsigned int) data[4] << 16UL);
		outl(ul_Command1,
			devpriv->iobase + ((data[5] - 1) * 0x20) +
			APCI1564_TCW_PROG);

		/*  Enable or Disable Interrupt */
		ul_Command1 = (ul_Command1 & 0xFFFFF9FD) | (data[1] << 1);
		outl(ul_Command1,
			devpriv->iobase + ((data[5] - 1) * 0x20) +
			APCI1564_TCW_PROG);

      /*****************************/
		/* Set the Up/Down selection */
      /*****************************/
		ul_Command1 = (ul_Command1 & 0xFFFBF9FFUL) | (data[6] << 18);
		outl(ul_Command1,
			devpriv->iobase + ((data[5] - 1) * 0x20) +
			APCI1564_TCW_PROG);
	}			/*  else if  (data[0]==ADDIDATA_COUNTER) */
	else {
		printk(" Invalid subdevice.");
	}			/*  else if  (data[0]==ADDIDATA_WATCHDOG) */

	return insn->n;
}

int i_APCI1564_StartStopWriteTimerCounterWatchdog(struct comedi_device *dev,
	struct comedi_subdevice *s, struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ul_Command1 = 0;
	if (devpriv->b_TimerSelectMode == ADDIDATA_WATCHDOG) {
		switch (data[1]) {
		case 0:	/* stop the watchdog */
			outl(0x0, devpriv->i_IobaseAmcc + APCI1564_DIGITAL_OP_WATCHDOG + APCI1564_TCW_PROG);	/* disable the watchdog */
			break;
		case 1:	/* start the watchdog */
			outl(0x0001,
				devpriv->i_IobaseAmcc +
				APCI1564_DIGITAL_OP_WATCHDOG +
				APCI1564_TCW_PROG);
			break;
		case 2:	/* Software trigger */
			outl(0x0201,
				devpriv->i_IobaseAmcc +
				APCI1564_DIGITAL_OP_WATCHDOG +
				APCI1564_TCW_PROG);
			break;
		default:
			printk("\nSpecified functionality does not exist\n");
			return -EINVAL;
		}		/*  switch (data[1]) */
	}			/*  if  (devpriv->b_TimerSelectMode==ADDIDATA_WATCHDOG) */
	if (devpriv->b_TimerSelectMode == ADDIDATA_TIMER) {
		if (data[1] == 1) {
			ul_Command1 =
				inl(devpriv->i_IobaseAmcc + APCI1564_TIMER +
				APCI1564_TCW_PROG);
			ul_Command1 = (ul_Command1 & 0xFFFFF9FFUL) | 0x1UL;

			/* Enable the Timer */
			outl(ul_Command1,
				devpriv->i_IobaseAmcc + APCI1564_TIMER +
				APCI1564_TCW_PROG);
		}		/*  if  (data[1]==1) */
		else if (data[1] == 0) {
			/* Stop The Timer */

			ul_Command1 =
				inl(devpriv->i_IobaseAmcc + APCI1564_TIMER +
				APCI1564_TCW_PROG);
			ul_Command1 = ul_Command1 & 0xFFFFF9FEUL;
			outl(ul_Command1,
				devpriv->i_IobaseAmcc + APCI1564_TIMER +
				APCI1564_TCW_PROG);
		}		/*  else if(data[1]==0) */
	}			/*  if  (devpriv->b_TimerSelectMode==ADDIDATA_TIMER) */
	if (devpriv->b_TimerSelectMode == ADDIDATA_COUNTER) {
		ul_Command1 =
			inl(devpriv->iobase + ((devpriv->b_ModeSelectRegister -
					1) * 0x20) + APCI1564_TCW_PROG);
		if (data[1] == 1) {
			/* Start the Counter subdevice */
			ul_Command1 = (ul_Command1 & 0xFFFFF9FFUL) | 0x1UL;
		}		/*  if  (data[1] == 1) */
		else if (data[1] == 0) {
			/*  Stops the Counter subdevice */
			ul_Command1 = 0;

		}		/*  else if  (data[1] == 0) */
		else if (data[1] == 2) {
			/*  Clears the Counter subdevice */
			ul_Command1 = (ul_Command1 & 0xFFFFF9FFUL) | 0x400;
		}		/*  else if  (data[1] == 3) */
		outl(ul_Command1,
			devpriv->iobase + ((devpriv->b_ModeSelectRegister -
					1) * 0x20) + APCI1564_TCW_PROG);
	}			/*  if (devpriv->b_TimerSelectMode==ADDIDATA_COUNTER) */
	return insn->n;
}

int i_APCI1564_ReadTimerCounterWatchdog(struct comedi_device *dev,
	struct comedi_subdevice *s, struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ul_Command1 = 0;

	if (devpriv->b_TimerSelectMode == ADDIDATA_WATCHDOG) {
		/*  Stores the status of the Watchdog */
		data[0] =
			inl(devpriv->i_IobaseAmcc +
			APCI1564_DIGITAL_OP_WATCHDOG +
			APCI1564_TCW_TRIG_STATUS) & 0x1;
		data[1] =
			inl(devpriv->i_IobaseAmcc +
			APCI1564_DIGITAL_OP_WATCHDOG);
	}			/*  if  (devpriv->b_TimerSelectMode==ADDIDATA_WATCHDOG) */
	else if (devpriv->b_TimerSelectMode == ADDIDATA_TIMER) {
		/*  Stores the status of the Timer */
		data[0] =
			inl(devpriv->i_IobaseAmcc + APCI1564_TIMER +
			APCI1564_TCW_TRIG_STATUS) & 0x1;

		/*  Stores the Actual value of the Timer */
		data[1] = inl(devpriv->i_IobaseAmcc + APCI1564_TIMER);
	}			/*  else if  (devpriv->b_TimerSelectMode==ADDIDATA_TIMER) */
	else if (devpriv->b_TimerSelectMode == ADDIDATA_COUNTER) {
		/*  Read the Counter Actual Value. */
		data[0] =
			inl(devpriv->iobase + ((devpriv->b_ModeSelectRegister -
					1) * 0x20) +
			APCI1564_TCW_SYNC_ENABLEDISABLE);
		ul_Command1 =
			inl(devpriv->iobase + ((devpriv->b_ModeSelectRegister -
					1) * 0x20) + APCI1564_TCW_TRIG_STATUS);

      /***********************************/
		/* Get the software trigger status */
      /***********************************/
		data[1] = (unsigned char) ((ul_Command1 >> 1) & 1);

      /***********************************/
		/* Get the hardware trigger status */
      /***********************************/
		data[2] = (unsigned char) ((ul_Command1 >> 2) & 1);

      /*********************************/
		/* Get the software clear status */
      /*********************************/
		data[3] = (unsigned char) ((ul_Command1 >> 3) & 1);

      /***************************/
		/* Get the overflow status */
      /***************************/
		data[4] = (unsigned char) ((ul_Command1 >> 0) & 1);
	}			/*  else  if  (devpriv->b_TimerSelectMode==ADDIDATA_COUNTER) */
	else if ((devpriv->b_TimerSelectMode != ADDIDATA_TIMER)
		&& (devpriv->b_TimerSelectMode != ADDIDATA_WATCHDOG)
		&& (devpriv->b_TimerSelectMode != ADDIDATA_COUNTER)) {
		printk("\n Invalid Subdevice !!!\n");
	}			/*  else if ((devpriv->b_TimerSelectMode!=ADDIDATA_TIMER) && (devpriv->b_TimerSelectMode!=ADDIDATA_WATCHDOG)&& (devpriv->b_TimerSelectMode!=ADDIDATA_COUNTER)) */
	return insn->n;
}


int i_APCI1564_ReadInterruptStatus(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	*data = ui_Type;
	return insn->n;
}

static void v_APCI1564_Interrupt(int irq, void *d)
{
	struct comedi_device *dev = d;
	unsigned int ui_DO, ui_DI;
	unsigned int ui_Timer;
	unsigned int ui_C1, ui_C2, ui_C3, ui_C4;
	unsigned int ul_Command2 = 0;
	ui_DI = inl(devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP +
		APCI1564_DIGITAL_IP_IRQ) & 0x01;
	ui_DO = inl(devpriv->i_IobaseAmcc + APCI1564_DIGITAL_OP +
		APCI1564_DIGITAL_OP_IRQ) & 0x01;
	ui_Timer =
		inl(devpriv->i_IobaseAmcc + APCI1564_TIMER +
		APCI1564_TCW_IRQ) & 0x01;
	ui_C1 = inl(devpriv->iobase + APCI1564_COUNTER1 +
		APCI1564_TCW_IRQ) & 0x1;
	ui_C2 = inl(devpriv->iobase + APCI1564_COUNTER2 +
		APCI1564_TCW_IRQ) & 0x1;
	ui_C3 = inl(devpriv->iobase + APCI1564_COUNTER3 +
		APCI1564_TCW_IRQ) & 0x1;
	ui_C4 = inl(devpriv->iobase + APCI1564_COUNTER4 +
		APCI1564_TCW_IRQ) & 0x1;
	if (ui_DI == 0 && ui_DO == 0 && ui_Timer == 0 && ui_C1 == 0
		&& ui_C2 == 0 && ui_C3 == 0 && ui_C4 == 0) {
		printk("\nInterrupt from unknown source\n");
	}			/*  if(ui_DI==0 && ui_DO==0 && ui_Timer==0 && ui_C1==0 && ui_C2==0 && ui_C3==0 && ui_C4==0) */

	if (ui_DI == 1) {
		ui_DI = inl(devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP +
			APCI1564_DIGITAL_IP_IRQ);
		outl(0x0,
			devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP +
			APCI1564_DIGITAL_IP_IRQ);
		ui_InterruptStatus_1564 =
			inl(devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP +
			APCI1564_DIGITAL_IP_INTERRUPT_STATUS);
		ui_InterruptStatus_1564 = ui_InterruptStatus_1564 & 0X000FFFF0;
		send_sig(SIGIO, devpriv->tsk_Current, 0);	/*  send signal to the sample */
		outl(ui_DI, devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP + APCI1564_DIGITAL_IP_IRQ);	/* enable the interrupt */
		return;
	}

	if (ui_DO == 1) {
		/*  Check for Digital Output interrupt Type - 1: Vcc interrupt 2: CC interrupt. */
		ui_Type =
			inl(devpriv->i_IobaseAmcc + APCI1564_DIGITAL_OP +
			APCI1564_DIGITAL_OP_INTERRUPT_STATUS) & 0x3;
		/* Disable the  Interrupt */
		outl(0x0,
			devpriv->i_IobaseAmcc + APCI1564_DIGITAL_OP +
			APCI1564_DIGITAL_OP_INTERRUPT);

		/* Sends signal to user space */
		send_sig(SIGIO, devpriv->tsk_Current, 0);

	}			/*  if  (ui_DO) */

	if (ui_Timer == 1) {
		devpriv->b_TimerSelectMode = ADDIDATA_TIMER;
		if (devpriv->b_TimerSelectMode) {

			/*  Disable Timer Interrupt */
			ul_Command2 =
				inl(devpriv->i_IobaseAmcc + APCI1564_TIMER +
				    APCI1564_TCW_PROG);
			outl(0x0,
			     devpriv->i_IobaseAmcc + APCI1564_TIMER +
			     APCI1564_TCW_PROG);

			/* Send a signal to from kernel to user space */
			send_sig(SIGIO, devpriv->tsk_Current, 0);

			/*  Enable Timer Interrupt */

			outl(ul_Command2,
			     devpriv->i_IobaseAmcc + APCI1564_TIMER +
			     APCI1564_TCW_PROG);
		}
	}/* if  (ui_Timer == 1) */


	if (ui_C1 == 1) {
		devpriv->b_TimerSelectMode = ADDIDATA_COUNTER;
		if (devpriv->b_TimerSelectMode) {

			/*  Disable Counter Interrupt */
			ul_Command2 =
				inl(devpriv->iobase + APCI1564_COUNTER1 +
				    APCI1564_TCW_PROG);
			outl(0x0,
			     devpriv->iobase + APCI1564_COUNTER1 +
			     APCI1564_TCW_PROG);

			/* Send a signal to from kernel to user space */
			send_sig(SIGIO, devpriv->tsk_Current, 0);

			/*  Enable Counter Interrupt */
			outl(ul_Command2,
			     devpriv->iobase + APCI1564_COUNTER1 +
			     APCI1564_TCW_PROG);
		}
	} /* if  (ui_C1 == 1) */

	if (ui_C2 == 1) {
		devpriv->b_TimerSelectMode = ADDIDATA_COUNTER;
		if (devpriv->b_TimerSelectMode) {

			/*  Disable Counter Interrupt */
			ul_Command2 =
				inl(devpriv->iobase + APCI1564_COUNTER2 +
				    APCI1564_TCW_PROG);
			outl(0x0,
			     devpriv->iobase + APCI1564_COUNTER2 +
			     APCI1564_TCW_PROG);

			/* Send a signal to from kernel to user space */
			send_sig(SIGIO, devpriv->tsk_Current, 0);

			/*  Enable Counter Interrupt */
			outl(ul_Command2,
			     devpriv->iobase + APCI1564_COUNTER2 +
			     APCI1564_TCW_PROG);
		}
	} /*  if  ((ui_C2 == 1) */

	if (ui_C3 == 1) {
		devpriv->b_TimerSelectMode = ADDIDATA_COUNTER;
		if (devpriv->b_TimerSelectMode) {

			/*  Disable Counter Interrupt */
			ul_Command2 =
				inl(devpriv->iobase + APCI1564_COUNTER3 +
				    APCI1564_TCW_PROG);
			outl(0x0,
			     devpriv->iobase + APCI1564_COUNTER3 +
			     APCI1564_TCW_PROG);

			/* Send a signal to from kernel to user space */
			send_sig(SIGIO, devpriv->tsk_Current, 0);

			/*  Enable Counter Interrupt */
			outl(ul_Command2,
			     devpriv->iobase + APCI1564_COUNTER3 +
			     APCI1564_TCW_PROG);
		}
	}	/*  if ((ui_C3 == 1) */

	if (ui_C4 == 1) {
		devpriv->b_TimerSelectMode = ADDIDATA_COUNTER;
		if (devpriv->b_TimerSelectMode) {

			/*  Disable Counter Interrupt */
			ul_Command2 =
				inl(devpriv->iobase + APCI1564_COUNTER4 +
				    APCI1564_TCW_PROG);
			outl(0x0,
			     devpriv->iobase + APCI1564_COUNTER4 +
			     APCI1564_TCW_PROG);

			/* Send a signal to from kernel to user space */
			send_sig(SIGIO, devpriv->tsk_Current, 0);

			/*  Enable Counter Interrupt */
			outl(ul_Command2,
			     devpriv->iobase + APCI1564_COUNTER4 +
			     APCI1564_TCW_PROG);
		}
	}	/*  if (ui_C4 == 1) */
	return;
}


int i_APCI1564_Reset(struct comedi_device *dev)
{
	outl(0x0, devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP_IRQ);	/* disable the interrupts */
	inl(devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP_INTERRUPT_STATUS);	/* Reset the interrupt status register */
	outl(0x0, devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP_INTERRUPT_MODE1);	/* Disable the and/or interrupt */
	outl(0x0, devpriv->i_IobaseAmcc + APCI1564_DIGITAL_IP_INTERRUPT_MODE2);
	devpriv->b_DigitalOutputRegister = 0;
	ui_Type = 0;
	outl(0x0, devpriv->i_IobaseAmcc + APCI1564_DIGITAL_OP);	/* Resets the output channels */
	outl(0x0, devpriv->i_IobaseAmcc + APCI1564_DIGITAL_OP_INTERRUPT);	/* Disables the interrupt. */
	outl(0x0,
		devpriv->i_IobaseAmcc + APCI1564_DIGITAL_OP_WATCHDOG +
		APCI1564_TCW_RELOAD_VALUE);
	outl(0x0, devpriv->i_IobaseAmcc + APCI1564_TIMER);
	outl(0x0, devpriv->i_IobaseAmcc + APCI1564_TIMER + APCI1564_TCW_PROG);

	outl(0x0, devpriv->iobase + APCI1564_COUNTER1 + APCI1564_TCW_PROG);
	outl(0x0, devpriv->iobase + APCI1564_COUNTER2 + APCI1564_TCW_PROG);
	outl(0x0, devpriv->iobase + APCI1564_COUNTER3 + APCI1564_TCW_PROG);
	outl(0x0, devpriv->iobase + APCI1564_COUNTER4 + APCI1564_TCW_PROG);
	return 0;
}
