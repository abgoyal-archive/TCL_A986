

#include "hwdrv_apci3501.h"


int i_APCI3501_ReadDigitalInput(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ui_Temp;
	unsigned int ui_NoOfChannel;
	ui_NoOfChannel = CR_CHAN(insn->chanspec);
	ui_Temp = data[0];
	*data = inl(devpriv->iobase + APCI3501_DIGITAL_IP);
	if (ui_Temp == 0) {
		*data = (*data >> ui_NoOfChannel) & 0x1;
	}			/* if  (ui_Temp==0) */
	else {
		if (ui_Temp == 1) {

			*data = *data & 0x3;
		}		/* if  (ui_Temp==1) */
		else {
			printk("\nSpecified channel not supported \n");
		}		/* elseif  (ui_Temp==1) */
	}			/* elseif  (ui_Temp==0) */
	return insn->n;
}

int i_APCI3501_ConfigDigitalOutput(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{

	if ((data[0] != 0) && (data[0] != 1)) {
		comedi_error(dev,
			"Not a valid Data !!! ,Data should be 1 or 0\n");
		return -EINVAL;
	}			/* if  ( (data[0]!=0) && (data[0]!=1) ) */
	if (data[0]) {
		devpriv->b_OutputMemoryStatus = ADDIDATA_ENABLE;
	}			/*  if  (data[0]) */
	else {
		devpriv->b_OutputMemoryStatus = ADDIDATA_DISABLE;
	}			/* else if  (data[0]) */
	return insn->n;
}

int i_APCI3501_WriteDigitalOutput(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ui_Temp, ui_Temp1;
	unsigned int ui_NoOfChannel = CR_CHAN(insn->chanspec);	/*  get the channel */
	if (devpriv->b_OutputMemoryStatus) {
		ui_Temp = inl(devpriv->iobase + APCI3501_DIGITAL_OP);
	}			/* if(devpriv->b_OutputMemoryStatus ) */
	else {
		ui_Temp = 0;
	}			/* if(devpriv->b_OutputMemoryStatus ) */
	if (data[3] == 0) {
		if (data[1] == 0) {
			data[0] = (data[0] << ui_NoOfChannel) | ui_Temp;
			outl(data[0], devpriv->iobase + APCI3501_DIGITAL_OP);
		}		/* if(data[1]==0) */
		else {
			if (data[1] == 1) {
				data[0] = (data[0] << (2 * data[2])) | ui_Temp;
				outl(data[0],
					devpriv->iobase + APCI3501_DIGITAL_OP);
			}	/*  if(data[1]==1) */
			else {
				printk("\nSpecified channel not supported\n");
			}	/* else if(data[1]==1) */
		}		/* elseif(data[1]==0) */
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
					devpriv->iobase + APCI3501_DIGITAL_OP);
			}	/* if(data[1]==0) */
			else {
				if (data[1] == 1) {
					data[0] = ~data[0] & 0x3;
					ui_Temp1 = 3;
					ui_Temp1 = ui_Temp1 << 2 * data[2];
					ui_Temp = ui_Temp | ui_Temp1;
					data[0] =
						((data[0] << (2 *
								data[2])) ^
						0xffffffff) & ui_Temp;
					outl(data[0],
						devpriv->iobase +
						APCI3501_DIGITAL_OP);
				}	/*  if(data[1]==1) */
				else {
					printk("\nSpecified channel not supported\n");
				}	/* else if(data[1]==1) */
			}	/* elseif(data[1]==0) */
		}		/* if(data[3]==1); */
		else {
			printk("\nSpecified functionality does not exist\n");
			return -EINVAL;
		}		/* if else data[3]==1) */
	}			/* if else data[3]==0) */
	return insn->n;
}

int i_APCI3501_ReadDigitalOutput(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ui_Temp;
	unsigned int ui_NoOfChannel;

	ui_NoOfChannel = CR_CHAN(insn->chanspec);
	ui_Temp = data[0];
	*data = inl(devpriv->iobase + APCI3501_DIGITAL_OP);
	if (ui_Temp == 0) {
		*data = (*data >> ui_NoOfChannel) & 0x1;
	}			/*  if  (ui_Temp==0) */
	else {
		if (ui_Temp == 1) {
			*data = *data & 0x3;

		}		/*  if  (ui_Temp==1) */
		else {
			printk("\nSpecified channel not supported \n");
		}		/*  else if (ui_Temp==1) */
	}			/*  else if  (ui_Temp==0) */
	return insn->n;
}

int i_APCI3501_ConfigAnalogOutput(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	outl(data[0],
		devpriv->iobase + APCI3501_ANALOG_OUTPUT +
		APCI3501_AO_VOLT_MODE);

	if (data[0]) {
		devpriv->b_InterruptMode = MODE1;
	} else {
		devpriv->b_InterruptMode = MODE0;
	}
	return insn->n;
}

int i_APCI3501_WriteAnalogOutput(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ul_Command1 = 0, ul_Channel_no, ul_Polarity, ul_DAC_Ready = 0;;

	ul_Channel_no = CR_CHAN(insn->chanspec);

	if (devpriv->b_InterruptMode == MODE1) {
		ul_Polarity = 0x80000000;
		if ((*data < 0) || (*data > 16384)) {
			printk("\nIn WriteAnalogOutput :: Not Valid Data\n");
		}

	}			/*  end if(devpriv->b_InterruptMode==MODE1) */
	else {
		ul_Polarity = 0;
		if ((*data < 0) || (*data > 8192)) {
			printk("\nIn WriteAnalogOutput :: Not Valid Data\n");
		}

	}			/*  end else */

	if ((ul_Channel_no < 0) || (ul_Channel_no > 7)) {
		printk("\nIn WriteAnalogOutput :: Not Valid Channel\n");
	}			/*  end if((ul_Channel_no<0)||(ul_Channel_no>7)) */

	ul_DAC_Ready = inl(devpriv->iobase + APCI3501_ANALOG_OUTPUT);

	while (ul_DAC_Ready == 0) {
		ul_DAC_Ready = inl(devpriv->iobase + APCI3501_ANALOG_OUTPUT);
		ul_DAC_Ready = (ul_DAC_Ready >> 8) & 1;
	}

	if (ul_DAC_Ready) {
/* Output the Value on the output channels. */
		ul_Command1 =
			(unsigned int) ((unsigned int) (ul_Channel_no & 0xFF) |
			(unsigned int) ((*data << 0x8) & 0x7FFFFF00L) |
			(unsigned int) (ul_Polarity));
		outl(ul_Command1,
			devpriv->iobase + APCI3501_ANALOG_OUTPUT +
			APCI3501_AO_PROG);
	}

	return insn->n;
}

int i_APCI3501_ConfigTimerCounterWatchdog(struct comedi_device *dev,
	struct comedi_subdevice *s, struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ul_Command1 = 0;
	devpriv->tsk_Current = current;
	if (data[0] == ADDIDATA_WATCHDOG) {

		devpriv->b_TimerSelectMode = ADDIDATA_WATCHDOG;
		/* Disable the watchdog */
		outl(0x0, devpriv->iobase + APCI3501_WATCHDOG + APCI3501_TCW_PROG);	/* disable Wa */

		if (data[1] == 1) {
			/* Enable TIMER int & DISABLE ALL THE OTHER int SOURCES */
			outl(0x02,
				devpriv->iobase + APCI3501_WATCHDOG +
				APCI3501_TCW_PROG);
		} else {
			outl(0x0, devpriv->iobase + APCI3501_WATCHDOG + APCI3501_TCW_PROG);	/* disable Timer interrupt */
		}

		/* Loading the Timebase value */
		outl(data[2],
			devpriv->iobase + APCI3501_WATCHDOG +
			APCI3501_TCW_TIMEBASE);

		/* Loading the Reload value */
		outl(data[3],
			devpriv->iobase + APCI3501_WATCHDOG +
			APCI3501_TCW_RELOAD_VALUE);
		/* Set the mode */
		ul_Command1 = inl(devpriv->iobase + APCI3501_WATCHDOG + APCI3501_TCW_PROG) | 0xFFF819E0UL;	/* e2->e0 */
		outl(ul_Command1,
			devpriv->iobase + APCI3501_WATCHDOG +
			APCI3501_TCW_PROG);
	}			/* end if(data[0]==ADDIDATA_WATCHDOG) */

	else if (data[0] == ADDIDATA_TIMER) {
		/* First Stop The Timer */
		ul_Command1 =
			inl(devpriv->iobase + APCI3501_WATCHDOG +
			APCI3501_TCW_PROG);
		ul_Command1 = ul_Command1 & 0xFFFFF9FEUL;
		outl(ul_Command1, devpriv->iobase + APCI3501_WATCHDOG + APCI3501_TCW_PROG);	/* Stop The Timer */
		devpriv->b_TimerSelectMode = ADDIDATA_TIMER;
		if (data[1] == 1) {
			/* Enable TIMER int & DISABLE ALL THE OTHER int SOURCES */
			outl(0x02,
				devpriv->iobase + APCI3501_WATCHDOG +
				APCI3501_TCW_PROG);
		} else {
			outl(0x0, devpriv->iobase + APCI3501_WATCHDOG + APCI3501_TCW_PROG);	/* disable Timer interrupt */
		}

		/*  Loading Timebase */
		outl(data[2],
			devpriv->iobase + APCI3501_WATCHDOG +
			APCI3501_TCW_TIMEBASE);

		/* Loading the Reload value */
		outl(data[3],
			devpriv->iobase + APCI3501_WATCHDOG +
			APCI3501_TCW_RELOAD_VALUE);

		/*  printk ("\nTimer Address :: %x\n", (devpriv->iobase+APCI3501_WATCHDOG)); */
		ul_Command1 =
			inl(devpriv->iobase + APCI3501_WATCHDOG +
			APCI3501_TCW_PROG);
		ul_Command1 =
			(ul_Command1 & 0xFFF719E2UL) | 2UL << 13UL | 0x10UL;
		outl(ul_Command1, devpriv->iobase + APCI3501_WATCHDOG + APCI3501_TCW_PROG);	/* mode 2 */

	}			/* end if(data[0]==ADDIDATA_TIMER) */

	return insn->n;
}


int i_APCI3501_StartStopWriteTimerCounterWatchdog(struct comedi_device *dev,
	struct comedi_subdevice *s, struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ul_Command1 = 0;
	int i_Temp;
	if (devpriv->b_TimerSelectMode == ADDIDATA_WATCHDOG) {

		if (data[1] == 1) {
			ul_Command1 =
				inl(devpriv->iobase + APCI3501_WATCHDOG +
				APCI3501_TCW_PROG);
			ul_Command1 = (ul_Command1 & 0xFFFFF9FFUL) | 0x1UL;
			/* Enable the Watchdog */
			outl(ul_Command1,
				devpriv->iobase + APCI3501_WATCHDOG +
				APCI3501_TCW_PROG);
		}

		else if (data[1] == 0)	/* Stop The Watchdog */
		{
			/* Stop The Watchdog */
			ul_Command1 =
				inl(devpriv->iobase + APCI3501_WATCHDOG +
				APCI3501_TCW_PROG);
			ul_Command1 = ul_Command1 & 0xFFFFF9FEUL;
			outl(0x0,
				devpriv->iobase + APCI3501_WATCHDOG +
				APCI3501_TCW_PROG);
		} else if (data[1] == 2) {
			ul_Command1 =
				inl(devpriv->iobase + APCI3501_WATCHDOG +
				APCI3501_TCW_PROG);
			ul_Command1 = (ul_Command1 & 0xFFFFF9FFUL) | 0x200UL;
			outl(ul_Command1,
				devpriv->iobase + APCI3501_WATCHDOG +
				APCI3501_TCW_PROG);
		}		/* if(data[1]==2) */
	}			/*  end if (devpriv->b_TimerSelectMode==ADDIDATA_WATCHDOG) */

	if (devpriv->b_TimerSelectMode == ADDIDATA_TIMER) {
		if (data[1] == 1) {

			ul_Command1 =
				inl(devpriv->iobase + APCI3501_WATCHDOG +
				APCI3501_TCW_PROG);
			ul_Command1 = (ul_Command1 & 0xFFFFF9FFUL) | 0x1UL;
			/* Enable the Timer */
			outl(ul_Command1,
				devpriv->iobase + APCI3501_WATCHDOG +
				APCI3501_TCW_PROG);
		} else if (data[1] == 0) {
			/* Stop The Timer */
			ul_Command1 =
				inl(devpriv->iobase + APCI3501_WATCHDOG +
				APCI3501_TCW_PROG);
			ul_Command1 = ul_Command1 & 0xFFFFF9FEUL;
			outl(ul_Command1,
				devpriv->iobase + APCI3501_WATCHDOG +
				APCI3501_TCW_PROG);
		}

		else if (data[1] == 2) {
			/* Trigger the Timer */
			ul_Command1 =
				inl(devpriv->iobase + APCI3501_WATCHDOG +
				APCI3501_TCW_PROG);
			ul_Command1 = (ul_Command1 & 0xFFFFF9FFUL) | 0x200UL;
			outl(ul_Command1,
				devpriv->iobase + APCI3501_WATCHDOG +
				APCI3501_TCW_PROG);
		}

	}			/*  end if (devpriv->b_TimerSelectMode==ADDIDATA_TIMER) */
	i_Temp = inl(devpriv->iobase + APCI3501_WATCHDOG +
		APCI3501_TCW_TRIG_STATUS) & 0x1;
	return insn->n;
}


int i_APCI3501_ReadTimerCounterWatchdog(struct comedi_device *dev,
	struct comedi_subdevice *s, struct comedi_insn *insn, unsigned int *data)
{

	if (devpriv->b_TimerSelectMode == ADDIDATA_WATCHDOG) {
		data[0] =
			inl(devpriv->iobase + APCI3501_WATCHDOG +
			APCI3501_TCW_TRIG_STATUS) & 0x1;
		data[1] = inl(devpriv->iobase + APCI3501_WATCHDOG);
	}			/*  end if  (devpriv->b_TimerSelectMode==ADDIDATA_WATCHDOG) */

	else if (devpriv->b_TimerSelectMode == ADDIDATA_TIMER) {
		data[0] =
			inl(devpriv->iobase + APCI3501_WATCHDOG +
			APCI3501_TCW_TRIG_STATUS) & 0x1;
		data[1] = inl(devpriv->iobase + APCI3501_WATCHDOG);
	}			/*  end if  (devpriv->b_TimerSelectMode==ADDIDATA_TIMER) */

	else if ((devpriv->b_TimerSelectMode != ADDIDATA_TIMER)
		&& (devpriv->b_TimerSelectMode != ADDIDATA_WATCHDOG)) {
		printk("\nIn ReadTimerCounterWatchdog :: Invalid Subdevice \n");
	}
	return insn->n;
}


int i_APCI3501_Reset(struct comedi_device *dev)
{
	int i_Count = 0, i_temp = 0;
	unsigned int ul_Command1 = 0, ul_Polarity, ul_DAC_Ready = 0;
	outl(0x0, devpriv->iobase + APCI3501_DIGITAL_OP);
	outl(1, devpriv->iobase + APCI3501_ANALOG_OUTPUT +
		APCI3501_AO_VOLT_MODE);

	ul_Polarity = 0x80000000;

	for (i_Count = 0; i_Count <= 7; i_Count++) {
		ul_DAC_Ready = inl(devpriv->iobase + APCI3501_ANALOG_OUTPUT);

		while (ul_DAC_Ready == 0) {
			ul_DAC_Ready =
				inl(devpriv->iobase + APCI3501_ANALOG_OUTPUT);
			ul_DAC_Ready = (ul_DAC_Ready >> 8) & 1;
		}

		if (ul_DAC_Ready) {
			/*  Output the Value on the output channels. */
			ul_Command1 =
				(unsigned int) ((unsigned int) (i_Count & 0xFF) |
				(unsigned int) ((i_temp << 0x8) & 0x7FFFFF00L) |
				(unsigned int) (ul_Polarity));
			outl(ul_Command1,
				devpriv->iobase + APCI3501_ANALOG_OUTPUT +
				APCI3501_AO_PROG);
		}
	}

	return 0;
}

void v_APCI3501_Interrupt(int irq, void *d)
{
	int i_temp;
	struct comedi_device *dev = d;
	unsigned int ui_Timer_AOWatchdog;
	unsigned long ul_Command1;
	/*  Disable Interrupt */
	ul_Command1 =
		inl(devpriv->iobase + APCI3501_WATCHDOG + APCI3501_TCW_PROG);

	ul_Command1 = (ul_Command1 & 0xFFFFF9FDul);
	outl(ul_Command1,
		devpriv->iobase + APCI3501_WATCHDOG + APCI3501_TCW_PROG);

	ui_Timer_AOWatchdog =
		inl(devpriv->iobase + APCI3501_WATCHDOG +
		APCI3501_TCW_IRQ) & 0x1;

	if ((!ui_Timer_AOWatchdog)) {
		comedi_error(dev, "IRQ from unknown source");
		return;
	}

	send_sig(SIGIO, devpriv->tsk_Current, 0);
	ul_Command1 =
		inl(devpriv->iobase + APCI3501_WATCHDOG + APCI3501_TCW_PROG);
	ul_Command1 = ((ul_Command1 & 0xFFFFF9FDul) | 1 << 1);
	outl(ul_Command1,
		devpriv->iobase + APCI3501_WATCHDOG + APCI3501_TCW_PROG);
	i_temp = inl(devpriv->iobase + APCI3501_WATCHDOG +
		APCI3501_TCW_TRIG_STATUS) & 0x1;
	return;
}
