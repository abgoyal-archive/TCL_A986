

#include "hwdrv_apci035.h"
static int i_WatchdogNbr = 0;
static int i_Temp = 0;
static int i_Flag = 1;
int i_APCI035_ConfigTimerWatchdog(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ui_Status = 0;
	unsigned int ui_Command = 0;
	unsigned int ui_Mode = 0;
	i_Temp = 0;
	devpriv->tsk_Current = current;
	devpriv->b_TimerSelectMode = data[0];
	i_WatchdogNbr = data[1];
	if (data[0] == 0) {
		ui_Mode = 2;
	} else {
		ui_Mode = 0;
	}
/* ui_Command = inl(devpriv->iobase+((i_WatchdogNbr-1)*32)+12); */
	ui_Command = 0;
/* ui_Command = ui_Command & 0xFFFFF9FEUL; */
	outl(ui_Command, devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);
	ui_Command = 0;
	ui_Command = inl(devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);
/************************/
/* Set the reload value */
/************************/
	outl(data[3], devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 4);
/*********************/
/* Set the time unit */
/*********************/
	outl(data[2], devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 8);
	if (data[0] == ADDIDATA_TIMER) {

		 /******************************/
		/* Set the mode :             */
		/* - Disable the hardware     */
		/* - Disable the counter mode */
		/* - Disable the warning      */
		/* - Disable the reset        */
		/* - Enable the timer mode    */
		/* - Set the timer mode       */
		 /******************************/

		ui_Command =
			(ui_Command & 0xFFF719E2UL) | ui_Mode << 13UL | 0x10UL;

	}			/* if (data[0] == ADDIDATA_TIMER) */
	else {
		if (data[0] == ADDIDATA_WATCHDOG) {

		 /******************************/
			/* Set the mode :             */
			/* - Disable the hardware     */
			/* - Disable the counter mode */
			/* - Disable the warning      */
			/* - Disable the reset        */
			/* - Disable the timer mode   */
		 /******************************/

			ui_Command = ui_Command & 0xFFF819E2UL;

		} else {
			printk("\n The parameter for Timer/watchdog selection is in error\n");
			return -EINVAL;
		}
	}
	outl(ui_Command, devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);
	ui_Command = 0;
	ui_Command = inl(devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);
/********************************/
/* Disable the hardware trigger */
/********************************/
	ui_Command = ui_Command & 0xFFFFF89FUL;
	if (data[4] == ADDIDATA_ENABLE) {
    /**********************************/
		/* Set the hardware trigger level */
    /**********************************/
		ui_Command = ui_Command | (data[5] << 5);
	}
	outl(ui_Command, devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);
	ui_Command = 0;
	ui_Command = inl(devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);
/*****************************/
/* Disable the hardware gate */
/*****************************/
	ui_Command = ui_Command & 0xFFFFF87FUL;
	if (data[6] == ADDIDATA_ENABLE) {
/*******************************/
/* Set the hardware gate level */
/*******************************/
		ui_Command = ui_Command | (data[7] << 7);
	}
	outl(ui_Command, devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);
	ui_Command = 0;
	ui_Command = inl(devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);
/*******************************/
/* Disable the hardware output */
/*******************************/
	ui_Command = ui_Command & 0xFFFFF9FBUL;
/*********************************/
/* Set the hardware output level */
/*********************************/
	ui_Command = ui_Command | (data[8] << 2);
	outl(ui_Command, devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);
	if (data[9] == ADDIDATA_ENABLE) {
   /************************/
		/* Set the reload value */
   /************************/
		outl(data[11],
			devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 24);
   /**********************/
		/* Set the time unite */
   /**********************/
		outl(data[10],
			devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 28);
	}

	ui_Command = 0;
	ui_Command = inl(devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);
 /*******************************/
	/* Disable the hardware output */
 /*******************************/
	ui_Command = ui_Command & 0xFFFFF9F7UL;
   /*********************************/
	/* Set the hardware output level */
   /*********************************/
	ui_Command = ui_Command | (data[12] << 3);
	outl(ui_Command, devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);
 /*************************************/
 /**  Enable the watchdog interrupt  **/
 /*************************************/
	ui_Command = 0;
	ui_Command = inl(devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);
/*******************************/
/* Set the interrupt selection */
/*******************************/
	ui_Status = inl(devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 16);

	ui_Command = (ui_Command & 0xFFFFF9FDUL) | (data[13] << 1);
	outl(ui_Command, devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);

	return insn->n;
}

int i_APCI035_StartStopWriteTimerWatchdog(struct comedi_device *dev,
	struct comedi_subdevice *s, struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ui_Command = 0;
	int i_Count = 0;
	if (data[0] == 1) {
		ui_Command =
			inl(devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);
	 /**********************/
		/* Start the hardware */
	 /**********************/
		ui_Command = (ui_Command & 0xFFFFF9FFUL) | 0x1UL;
		outl(ui_Command,
			devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);
	}			/*  if  (data[0]==1) */
	if (data[0] == 2) {
		ui_Command =
			inl(devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);
	 /***************************/
		/* Set the trigger command */
	 /***************************/
		ui_Command = (ui_Command & 0xFFFFF9FFUL) | 0x200UL;
		outl(ui_Command,
			devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);
	}

	if (data[0] == 0)	/* Stop The Watchdog */
	{
		/* Stop The Watchdog */
		ui_Command = 0;
		outl(ui_Command,
			devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 12);
	}			/*   if (data[1]==0) */
	if (data[0] == 3)	/* stop all Watchdogs */
	{
		ui_Command = 0;
		for (i_Count = 1; i_Count <= 4; i_Count++) {
			if (devpriv->b_TimerSelectMode == ADDIDATA_WATCHDOG) {
				ui_Command = 0x2UL;
			} else {
				ui_Command = 0x10UL;
			}
			i_WatchdogNbr = i_Count;
			outl(ui_Command,
				devpriv->iobase + ((i_WatchdogNbr - 1) * 32) +
				0);
		}

	}
	if (data[0] == 4)	/* start all Watchdogs */
	{
		ui_Command = 0;
		for (i_Count = 1; i_Count <= 4; i_Count++) {
			if (devpriv->b_TimerSelectMode == ADDIDATA_WATCHDOG) {
				ui_Command = 0x1UL;
			} else {
				ui_Command = 0x8UL;
			}
			i_WatchdogNbr = i_Count;
			outl(ui_Command,
				devpriv->iobase + ((i_WatchdogNbr - 1) * 32) +
				0);
		}
	}
	if (data[0] == 5)	/* trigger all Watchdogs */
	{
		ui_Command = 0;
		for (i_Count = 1; i_Count <= 4; i_Count++) {
			if (devpriv->b_TimerSelectMode == ADDIDATA_WATCHDOG) {
				ui_Command = 0x4UL;
			} else {
				ui_Command = 0x20UL;
			}

			i_WatchdogNbr = i_Count;
			outl(ui_Command,
				devpriv->iobase + ((i_WatchdogNbr - 1) * 32) +
				0);
		}
		i_Temp = 1;
	}
	return insn->n;
}

int i_APCI035_ReadTimerWatchdog(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ui_Status = 0;	/*  Status register */
	i_WatchdogNbr = insn->unused[0];

	/******************/
	/* Get the status */
	/******************/

	ui_Status = inl(devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 16);

	/***********************************/
	/* Get the software trigger status */
	/***********************************/

	data[0] = ((ui_Status >> 1) & 1);
	/***********************************/
	/* Get the hardware trigger status */
	/***********************************/
	data[1] = ((ui_Status >> 2) & 1);
	/*********************************/
	/* Get the software clear status */
	/*********************************/
	data[2] = ((ui_Status >> 3) & 1);
	/***************************/
	/* Get the overflow status */
	/***************************/
	data[3] = ((ui_Status >> 0) & 1);
	if (devpriv->b_TimerSelectMode == ADDIDATA_TIMER) {
		data[4] = inl(devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 0);

	}			/*   if  (devpriv->b_TimerSelectMode==ADDIDATA_TIMER) */

	return insn->n;
}

int i_APCI035_ConfigAnalogInput(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	devpriv->tsk_Current = current;
	outl(0x200 | 0, devpriv->iobase + 128 + 0x4);
	outl(0, devpriv->iobase + 128 + 0);
/********************************/
/* Initialise the warning value */
/********************************/
	outl(0x300 | 0, devpriv->iobase + 128 + 0x4);
	outl((data[0] << 8), devpriv->iobase + 128 + 0);
	outl(0x200000UL, devpriv->iobase + 128 + 12);

	return insn->n;
}

int i_APCI035_ReadAnalogInput(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ui_CommandRegister = 0;
/******************/
/*  Set the start */
/******************/
	ui_CommandRegister = 0x80000;
 /******************************/
	/* Write the command register */
 /******************************/
	outl(ui_CommandRegister, devpriv->iobase + 128 + 8);

/***************************************/
/* Read the digital value of the input */
/***************************************/
	data[0] = inl(devpriv->iobase + 128 + 28);
	return insn->n;
}

int i_APCI035_Reset(struct comedi_device *dev)
{
	int i_Count = 0;
	for (i_Count = 1; i_Count <= 4; i_Count++) {
		i_WatchdogNbr = i_Count;
		outl(0x0, devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 0);	/* stop all timers */
	}
	outl(0x0, devpriv->iobase + 128 + 12);	/* Disable the warning delay */

	return 0;
}

static void v_APCI035_Interrupt(int irq, void *d)
{
	struct comedi_device *dev = d;
	unsigned int ui_StatusRegister1 = 0;
	unsigned int ui_StatusRegister2 = 0;
	unsigned int ui_ReadCommand = 0;
	unsigned int ui_ChannelNumber = 0;
	unsigned int ui_DigitalTemperature = 0;
	if (i_Temp == 1) {
		i_WatchdogNbr = i_Flag;
		i_Flag = i_Flag + 1;
	}
  /**************************************/
	/* Read the interrupt status register of temperature Warning */
  /**************************************/
	ui_StatusRegister1 = inl(devpriv->iobase + 128 + 16);
  /**************************************/
	/* Read the interrupt status register for Watchdog/timer */
   /**************************************/

	ui_StatusRegister2 =
		inl(devpriv->iobase + ((i_WatchdogNbr - 1) * 32) + 20);

	if ((((ui_StatusRegister1) & 0x8) == 0x8))	/* Test if warning relay interrupt */
	{
	/**********************************/
		/* Disable the temperature warning */
	/**********************************/
		ui_ReadCommand = inl(devpriv->iobase + 128 + 12);
		ui_ReadCommand = ui_ReadCommand & 0xFFDF0000UL;
		outl(ui_ReadCommand, devpriv->iobase + 128 + 12);
      /***************************/
		/* Read the channel number */
      /***************************/
		ui_ChannelNumber = inl(devpriv->iobase + 128 + 60);
	/**************************************/
		/* Read the digital temperature value */
	/**************************************/
		ui_DigitalTemperature = inl(devpriv->iobase + 128 + 60);
		send_sig(SIGIO, devpriv->tsk_Current, 0);	/*  send signal to the sample */
	}			/* if (((ui_StatusRegister1 & 0x8) == 0x8)) */

	else {
		if ((ui_StatusRegister2 & 0x1) == 0x1) {
			send_sig(SIGIO, devpriv->tsk_Current, 0);	/*  send signal to the sample */
		}
	}			/* else if (((ui_StatusRegister1 & 0x8) == 0x8)) */

	return;
}
