

#include "hwdrv_apci1032.h"
#include <linux/delay.h>

static unsigned int ui_InterruptStatus;


int i_APCI1032_ConfigDigitalInput(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ui_TmpValue;

	unsigned int ul_Command1 = 0;
	unsigned int ul_Command2 = 0;
	devpriv->tsk_Current = current;

  /*******************************/
	/* Set the digital input logic */
  /*******************************/
	if (data[0] == ADDIDATA_ENABLE) {
		ul_Command1 = ul_Command1 | data[2];
		ul_Command2 = ul_Command2 | data[3];
		outl(ul_Command1,
			devpriv->iobase + APCI1032_DIGITAL_IP_INTERRUPT_MODE1);
		outl(ul_Command2,
			devpriv->iobase + APCI1032_DIGITAL_IP_INTERRUPT_MODE2);
		if (data[1] == ADDIDATA_OR) {
			outl(0x4, devpriv->iobase + APCI1032_DIGITAL_IP_IRQ);
			ui_TmpValue =
				inl(devpriv->iobase + APCI1032_DIGITAL_IP_IRQ);
		}		/* if (data[1] == ADDIDATA_OR) */
		else
			outl(0x6, devpriv->iobase + APCI1032_DIGITAL_IP_IRQ);
				/* else if(data[1] == ADDIDATA_OR) */
	}			/*  if( data[0] == ADDIDATA_ENABLE) */
	else {
		ul_Command1 = ul_Command1 & 0xFFFF0000;
		ul_Command2 = ul_Command2 & 0xFFFF0000;
		outl(ul_Command1,
			devpriv->iobase + APCI1032_DIGITAL_IP_INTERRUPT_MODE1);
		outl(ul_Command2,
			devpriv->iobase + APCI1032_DIGITAL_IP_INTERRUPT_MODE2);
		outl(0x0, devpriv->iobase + APCI1032_DIGITAL_IP_IRQ);
	}			/* else if  ( data[0] == ADDIDATA_ENABLE) */

	return insn->n;
}

int i_APCI1032_Read1DigitalInput(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ui_TmpValue = 0;
	unsigned int ui_Channel;
	ui_Channel = CR_CHAN(insn->chanspec);
	if (ui_Channel <= 31) {
		ui_TmpValue = (unsigned int) inl(devpriv->iobase + APCI1032_DIGITAL_IP);
		*data = (ui_TmpValue >> ui_Channel) & 0x1;
	}			/* if(ui_Channel >= 0 && ui_Channel <=31) */
	else {
		/* comedi_error(dev," \n chan spec wrong\n"); */
		return -EINVAL;	/*  "sorry channel spec wrong " */
	}			/* else if(ui_Channel >= 0 && ui_Channel <=31) */
	return insn->n;
}


int i_APCI1032_ReadMoreDigitalInput(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	unsigned int ui_PortValue = data[0];
	unsigned int ui_Mask = 0;
	unsigned int ui_NoOfChannels;

	ui_NoOfChannels = CR_CHAN(insn->chanspec);
	if (data[1] == 0) {
		*data = (unsigned int) inl(devpriv->iobase + APCI1032_DIGITAL_IP);
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
			/* comedi_error(dev," \nchan spec wrong\n"); */
			return -EINVAL;	/*  "sorry channel spec wrong " */
			break;
		}		/* switch(ui_NoOfChannels) */
	}			/* if(data[1]==0) */
	else {
		if (data[1] == 1)
			*data = ui_InterruptStatus;
				/* if(data[1]==1) */
	}			/* else if(data[1]==0) */
	return insn->n;
}

static void v_APCI1032_Interrupt(int irq, void *d)
{
	struct comedi_device *dev = d;

	unsigned int ui_Temp;
	/* disable the interrupt */
	ui_Temp = inl(devpriv->iobase + APCI1032_DIGITAL_IP_IRQ);
	outl(ui_Temp & APCI1032_DIGITAL_IP_INTERRUPT_DISABLE,
		devpriv->iobase + APCI1032_DIGITAL_IP_IRQ);
	ui_InterruptStatus =
		inl(devpriv->iobase + APCI1032_DIGITAL_IP_INTERRUPT_STATUS);
	ui_InterruptStatus = ui_InterruptStatus & 0X0000FFFF;
	send_sig(SIGIO, devpriv->tsk_Current, 0);	/*  send signal to the sample */
	outl(ui_Temp, devpriv->iobase + APCI1032_DIGITAL_IP_IRQ);	/* enable the interrupt */
	return;
}


int i_APCI1032_Reset(struct comedi_device *dev)
{
	outl(0x0, devpriv->iobase + APCI1032_DIGITAL_IP_IRQ);	/* disable the interrupts */
	inl(devpriv->iobase + APCI1032_DIGITAL_IP_INTERRUPT_STATUS);	/* Reset the interrupt status register */
	outl(0x0, devpriv->iobase + APCI1032_DIGITAL_IP_INTERRUPT_MODE1);	/* Disable the and/or interrupt */
	outl(0x0, devpriv->iobase + APCI1032_DIGITAL_IP_INTERRUPT_MODE2);
	return 0;
}
