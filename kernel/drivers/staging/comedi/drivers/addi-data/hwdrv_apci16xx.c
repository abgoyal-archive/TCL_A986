


#include "hwdrv_apci16xx.h"


int i_APCI16XX_InsnConfigInitTTLIO(struct comedi_device *dev,
	struct comedi_subdevice *s, struct comedi_insn *insn, unsigned int *data)
{
	int i_ReturnValue = insn->n;
	unsigned char b_Command = 0;
	unsigned char b_Cpt = 0;
	unsigned char b_NumberOfPort =
		(unsigned char) (devpriv->ps_BoardInfo->i_NbrTTLChannel / 8);

	/************************/
	/* Test the buffer size */
	/************************/

	if (insn->n >= 1) {
	   /*******************/
		/* Get the command */
		/* **************** */

		b_Command = (unsigned char) data[0];

	   /********************/
		/* Test the command */
	   /********************/

		if ((b_Command == APCI16XX_TTL_INIT) ||
			(b_Command == APCI16XX_TTL_INITDIRECTION) ||
			(b_Command == APCI16XX_TTL_OUTPUTMEMORY)) {
	      /***************************************/
			/* Test the initialisation buffer size */
	      /***************************************/

			if ((b_Command == APCI16XX_TTL_INITDIRECTION)
				&& ((unsigned char) (insn->n - 1) != b_NumberOfPort)) {
		 /*******************/
				/* Data size error */
		 /*******************/

				printk("\nBuffer size error");
				i_ReturnValue = -101;
			}

			if ((b_Command == APCI16XX_TTL_OUTPUTMEMORY)
				&& ((unsigned char) (insn->n) != 2)) {
		 /*******************/
				/* Data size error */
		 /*******************/

				printk("\nBuffer size error");
				i_ReturnValue = -101;
			}
		} else {
	      /************************/
			/* Config command error */
	      /************************/

			printk("\nCommand selection error");
			i_ReturnValue = -100;
		}
	} else {
	   /*******************/
		/* Data size error */
	   /*******************/

		printk("\nBuffer size error");
		i_ReturnValue = -101;
	}

	/**************************************************************************/
	/* Test if no error occur and APCI16XX_TTL_INITDIRECTION command selected */
	/**************************************************************************/

	if ((i_ReturnValue >= 0) && (b_Command == APCI16XX_TTL_INITDIRECTION)) {
		memset(devpriv->ul_TTLPortConfiguration, 0,
			sizeof(devpriv->ul_TTLPortConfiguration));

	   /*************************************/
		/* Test the port direction selection */
	   /*************************************/

		for (b_Cpt = 1;
			(b_Cpt <= b_NumberOfPort) && (i_ReturnValue >= 0);
			b_Cpt++) {
	      /**********************/
			/* Test the direction */
	      /**********************/

			if ((data[b_Cpt] != 0) && (data[b_Cpt] != 0xFF)) {
		 /************************/
				/* Port direction error */
		 /************************/

				printk("\nPort %d direction selection error",
					(int) b_Cpt);
				i_ReturnValue = -(int) b_Cpt;
			}

	      /**************************/
			/* Save the configuration */
	      /**************************/

			devpriv->ul_TTLPortConfiguration[(b_Cpt - 1) / 4] =
				devpriv->ul_TTLPortConfiguration[(b_Cpt -
					1) / 4] | (data[b_Cpt] << (8 * ((b_Cpt -
							1) % 4)));
		}
	}

	/**************************/
	/* Test if no error occur */
	/**************************/

	if (i_ReturnValue >= 0) {
	   /***********************************/
		/* Test if TTL port initilaisation */
	   /***********************************/

		if ((b_Command == APCI16XX_TTL_INIT)
			|| (b_Command == APCI16XX_TTL_INITDIRECTION)) {
	      /******************************/
			/* Set all port configuration */
	      /******************************/

			for (b_Cpt = 0; b_Cpt <= b_NumberOfPort; b_Cpt++) {
				if ((b_Cpt % 4) == 0) {
		    /*************************/
					/* Set the configuration */
		    /*************************/

					outl(devpriv->
						ul_TTLPortConfiguration[b_Cpt /
							4],
						devpriv->iobase + 32 + b_Cpt);
				}
			}
		}
	}

	/************************************************/
	/* Test if output memory initialisation command */
	/************************************************/

	if (b_Command == APCI16XX_TTL_OUTPUTMEMORY) {
		if (data[1]) {
			devpriv->b_OutputMemoryStatus = ADDIDATA_ENABLE;
		} else {
			devpriv->b_OutputMemoryStatus = ADDIDATA_DISABLE;
		}
	}

	return i_ReturnValue;
}



int i_APCI16XX_InsnBitsReadTTLIO(struct comedi_device *dev,
	struct comedi_subdevice *s, struct comedi_insn *insn, unsigned int *data)
{
	int i_ReturnValue = insn->n;
	unsigned char b_Command = 0;
	unsigned char b_NumberOfPort =
		(unsigned char) (devpriv->ps_BoardInfo->i_NbrTTLChannel / 8);
	unsigned char b_SelectedPort = CR_RANGE(insn->chanspec);
	unsigned char b_InputChannel = CR_CHAN(insn->chanspec);
	unsigned char *pb_Status;
	unsigned int dw_Status;

	/************************/
	/* Test the buffer size */
	/************************/

	if (insn->n >= 1) {
	   /*******************/
		/* Get the command */
		/* **************** */

		b_Command = (unsigned char) data[0];

	   /********************/
		/* Test the command */
	   /********************/

		if ((b_Command == APCI16XX_TTL_READCHANNEL)
			|| (b_Command == APCI16XX_TTL_READPORT)) {
	      /**************************/
			/* Test the selected port */
	      /**************************/

			if (b_SelectedPort < b_NumberOfPort) {
		 /**********************/
				/* Test if input port */
		 /**********************/

				if (((devpriv->ul_TTLPortConfiguration
							[b_SelectedPort /
								4] >> (8 *
								(b_SelectedPort
									%
									4))) &
						0xFF) == 0) {
		    /***************************/
					/* Test the channel number */
		    /***************************/

					if ((b_Command ==
							APCI16XX_TTL_READCHANNEL)
						&& (b_InputChannel > 7)) {
		       /*******************************************/
						/* The selected TTL digital input is wrong */
		       /*******************************************/

						printk("\nChannel selection error");
						i_ReturnValue = -103;
					}
				} else {
		    /****************************************/
					/* The selected TTL input port is wrong */
		    /****************************************/

					printk("\nPort selection error");
					i_ReturnValue = -102;
				}
			} else {
		 /****************************************/
				/* The selected TTL input port is wrong */
		 /****************************************/

				printk("\nPort selection error");
				i_ReturnValue = -102;
			}
		} else {
	      /************************/
			/* Config command error */
	      /************************/

			printk("\nCommand selection error");
			i_ReturnValue = -100;
		}
	} else {
	   /*******************/
		/* Data size error */
	   /*******************/

		printk("\nBuffer size error");
		i_ReturnValue = -101;
	}

	/**************************/
	/* Test if no error occur */
	/**************************/

	if (i_ReturnValue >= 0) {
		pb_Status = (unsigned char *) &data[0];

	   /*******************************/
		/* Get the digital inpu status */
	   /*******************************/

		dw_Status =
			inl(devpriv->iobase + 8 + ((b_SelectedPort / 4) * 4));
		dw_Status = (dw_Status >> (8 * (b_SelectedPort % 4))) & 0xFF;

	   /***********************/
		/* Save the port value */
	   /***********************/

		*pb_Status = (unsigned char) dw_Status;

	   /***************************************/
		/* Test if read channel status command */
	   /***************************************/

		if (b_Command == APCI16XX_TTL_READCHANNEL) {
			*pb_Status = (*pb_Status >> b_InputChannel) & 1;
		}
	}

	return i_ReturnValue;
}


int i_APCI16XX_InsnReadTTLIOAllPortValue(struct comedi_device *dev,
	struct comedi_subdevice *s, struct comedi_insn *insn, unsigned int *data)
{
	unsigned char b_Command = (unsigned char) CR_AREF(insn->chanspec);
	int i_ReturnValue = insn->n;
	unsigned char b_Cpt = 0;
	unsigned char b_NumberOfPort = 0;
	unsigned int *pls_ReadData = data;

	/********************/
	/* Test the command */
	/********************/

	if ((b_Command == APCI16XX_TTL_READ_ALL_INPUTS)
		|| (b_Command == APCI16XX_TTL_READ_ALL_OUTPUTS)) {
	   /**********************************/
		/* Get the number of 32-Bit ports */
	   /**********************************/

		b_NumberOfPort =
			(unsigned char) (devpriv->ps_BoardInfo->i_NbrTTLChannel / 32);
		if ((b_NumberOfPort * 32) <
			devpriv->ps_BoardInfo->i_NbrTTLChannel) {
			b_NumberOfPort = b_NumberOfPort + 1;
		}

	   /************************/
		/* Test the buffer size */
	   /************************/

		if (insn->n >= b_NumberOfPort) {
			if (b_Command == APCI16XX_TTL_READ_ALL_INPUTS) {
		 /**************************/
				/* Read all digital input */
		 /**************************/

				for (b_Cpt = 0; b_Cpt < b_NumberOfPort; b_Cpt++) {
		    /************************/
					/* Read the 32-Bit port */
		    /************************/

					pls_ReadData[b_Cpt] =
						inl(devpriv->iobase + 8 +
						(b_Cpt * 4));

		    /**************************************/
					/* Mask all channels used als outputs */
		    /**************************************/

					pls_ReadData[b_Cpt] =
						pls_ReadData[b_Cpt] &
						(~devpriv->
						ul_TTLPortConfiguration[b_Cpt]);
				}
			} else {
		 /****************************/
				/* Read all digital outputs */
		 /****************************/

				for (b_Cpt = 0; b_Cpt < b_NumberOfPort; b_Cpt++) {
		    /************************/
					/* Read the 32-Bit port */
		    /************************/

					pls_ReadData[b_Cpt] =
						inl(devpriv->iobase + 20 +
						(b_Cpt * 4));

		    /**************************************/
					/* Mask all channels used als outputs */
		    /**************************************/

					pls_ReadData[b_Cpt] =
						pls_ReadData[b_Cpt] & devpriv->
						ul_TTLPortConfiguration[b_Cpt];
				}
			}
		} else {
	      /*******************/
			/* Data size error */
	      /*******************/

			printk("\nBuffer size error");
			i_ReturnValue = -101;
		}
	} else {
	   /*****************/
		/* Command error */
	   /*****************/

		printk("\nCommand selection error");
		i_ReturnValue = -100;
	}

	return i_ReturnValue;
}



int i_APCI16XX_InsnBitsWriteTTLIO(struct comedi_device *dev,
	struct comedi_subdevice *s, struct comedi_insn *insn, unsigned int *data)
{
	int i_ReturnValue = insn->n;
	unsigned char b_Command = 0;
	unsigned char b_NumberOfPort =
		(unsigned char) (devpriv->ps_BoardInfo->i_NbrTTLChannel / 8);
	unsigned char b_SelectedPort = CR_RANGE(insn->chanspec);
	unsigned char b_OutputChannel = CR_CHAN(insn->chanspec);
	unsigned int dw_Status = 0;

	/************************/
	/* Test the buffer size */
	/************************/

	if (insn->n >= 1) {
	   /*******************/
		/* Get the command */
		/* **************** */

		b_Command = (unsigned char) data[0];

	   /********************/
		/* Test the command */
	   /********************/

		if ((b_Command == APCI16XX_TTL_WRITECHANNEL_ON) ||
			(b_Command == APCI16XX_TTL_WRITEPORT_ON) ||
			(b_Command == APCI16XX_TTL_WRITECHANNEL_OFF) ||
			(b_Command == APCI16XX_TTL_WRITEPORT_OFF)) {
	      /**************************/
			/* Test the selected port */
	      /**************************/

			if (b_SelectedPort < b_NumberOfPort) {
		 /***********************/
				/* Test if output port */
		 /***********************/

				if (((devpriv->ul_TTLPortConfiguration
							[b_SelectedPort /
								4] >> (8 *
								(b_SelectedPort
									%
									4))) &
						0xFF) == 0xFF) {
		    /***************************/
					/* Test the channel number */
		    /***************************/

					if (((b_Command == APCI16XX_TTL_WRITECHANNEL_ON) || (b_Command == APCI16XX_TTL_WRITECHANNEL_OFF)) && (b_OutputChannel > 7)) {
		       /********************************************/
						/* The selected TTL digital output is wrong */
		       /********************************************/

						printk("\nChannel selection error");
						i_ReturnValue = -103;
					}

					if (((b_Command == APCI16XX_TTL_WRITECHANNEL_OFF) || (b_Command == APCI16XX_TTL_WRITEPORT_OFF)) && (devpriv->b_OutputMemoryStatus == ADDIDATA_DISABLE)) {
		       /********************************************/
						/* The selected TTL digital output is wrong */
		       /********************************************/

						printk("\nOutput memory disabled");
						i_ReturnValue = -104;
					}

		    /************************/
					/* Test the buffer size */
		    /************************/

					if (((b_Command == APCI16XX_TTL_WRITEPORT_ON) || (b_Command == APCI16XX_TTL_WRITEPORT_OFF)) && (insn->n < 2)) {
		       /*******************/
						/* Data size error */
		       /*******************/

						printk("\nBuffer size error");
						i_ReturnValue = -101;
					}
				} else {
		    /*****************************************/
					/* The selected TTL output port is wrong */
		    /*****************************************/

					printk("\nPort selection error %lX",
						(unsigned long)devpriv->
						ul_TTLPortConfiguration[0]);
					i_ReturnValue = -102;
				}
			} else {
		 /****************************************/
				/* The selected TTL output port is wrong */
		 /****************************************/

				printk("\nPort selection error %d %d",
					b_SelectedPort, b_NumberOfPort);
				i_ReturnValue = -102;
			}
		} else {
	      /************************/
			/* Config command error */
	      /************************/

			printk("\nCommand selection error");
			i_ReturnValue = -100;
		}
	} else {
	   /*******************/
		/* Data size error */
	   /*******************/

		printk("\nBuffer size error");
		i_ReturnValue = -101;
	}

	/**************************/
	/* Test if no error occur */
	/**************************/

	if (i_ReturnValue >= 0) {
	   /********************************/
		/* Get the digital output state */
	   /********************************/

		dw_Status =
			inl(devpriv->iobase + 20 + ((b_SelectedPort / 4) * 4));

	   /**********************************/
		/* Test if output memory not used */
	   /**********************************/

		if (devpriv->b_OutputMemoryStatus == ADDIDATA_DISABLE) {
	      /*********************************/
			/* Clear the selected port value */
	      /*********************************/

			dw_Status =
				dw_Status & (0xFFFFFFFFUL -
				(0xFFUL << (8 * (b_SelectedPort % 4))));
		}

	   /******************************/
		/* Test if setting channel ON */
	   /******************************/

		if (b_Command == APCI16XX_TTL_WRITECHANNEL_ON) {
			dw_Status =
				dw_Status | (1UL << ((8 * (b_SelectedPort %
							4)) + b_OutputChannel));
		}

	   /***************************/
		/* Test if setting port ON */
	   /***************************/

		if (b_Command == APCI16XX_TTL_WRITEPORT_ON) {
			dw_Status =
				dw_Status | ((data[1] & 0xFF) << (8 *
					(b_SelectedPort % 4)));
		}

	   /*******************************/
		/* Test if setting channel OFF */
	   /*******************************/

		if (b_Command == APCI16XX_TTL_WRITECHANNEL_OFF) {
			dw_Status =
				dw_Status & (0xFFFFFFFFUL -
				(1UL << ((8 * (b_SelectedPort % 4)) +
						b_OutputChannel)));
		}

	   /****************************/
		/* Test if setting port OFF */
	   /****************************/

		if (b_Command == APCI16XX_TTL_WRITEPORT_OFF) {
			dw_Status =
				dw_Status & (0xFFFFFFFFUL -
				((data[1] & 0xFF) << (8 * (b_SelectedPort %
							4))));
		}

		outl(dw_Status,
			devpriv->iobase + 20 + ((b_SelectedPort / 4) * 4));
	}

	return i_ReturnValue;
}


int i_APCI16XX_Reset(struct comedi_device *dev)
{
	return 0;
}
