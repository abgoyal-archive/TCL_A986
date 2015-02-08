


#include "APCI1710_Inp_cpt.h"


int i_APCI1710_InsnConfigInitPulseEncoder(struct comedi_device *dev,
	struct comedi_subdevice *s, struct comedi_insn *insn, unsigned int *data)
{
	int i_ReturnValue = 0;
	unsigned int dw_IntRegister;

	unsigned char b_ModulNbr;
	unsigned char b_PulseEncoderNbr;
	unsigned char b_InputLevelSelection;
	unsigned char b_TriggerOutputAction;
	unsigned int ul_StartValue;

	b_ModulNbr = (unsigned char) CR_AREF(insn->chanspec);
	b_PulseEncoderNbr = (unsigned char) data[0];
	b_InputLevelSelection = (unsigned char) data[1];
	b_TriggerOutputAction = (unsigned char) data[2];
	ul_StartValue = (unsigned int) data[3];

	i_ReturnValue = insn->n;

	/***********************************/
	/* Test the selected module number */
	/***********************************/

	if (b_ModulNbr <= 3) {
	   /*************************/
		/* Test if pulse encoder */
	   /*************************/

		if ((devpriv->s_BoardInfos.
				dw_MolduleConfiguration[b_ModulNbr] &
				APCI1710_PULSE_ENCODER) ==
			APCI1710_PULSE_ENCODER) {
	      /******************************************/
			/* Test the selected pulse encoder number */
	      /******************************************/

			if (b_PulseEncoderNbr <= 3) {
		 /************************/
				/* Test the input level */
		 /************************/

				if ((b_InputLevelSelection == 0)
					|| (b_InputLevelSelection == 1)) {
		    /*******************************************/
					/* Test the ouput TRIGGER action selection */
		    /*******************************************/

					if ((b_TriggerOutputAction <= 2)
						|| (b_PulseEncoderNbr > 0)) {
						if (ul_StartValue > 1) {

							dw_IntRegister =
								inl(devpriv->
								s_BoardInfos.
								ui_Address +
								20 +
								(64 * b_ModulNbr));

			  /***********************/
							/* Set the start value */
			  /***********************/

							outl(ul_StartValue,
								devpriv->
								s_BoardInfos.
								ui_Address +
								(b_PulseEncoderNbr
									* 4) +
								(64 * b_ModulNbr));

			  /***********************/
							/* Set the input level */
			  /***********************/
							devpriv->
								s_ModuleInfo
								[b_ModulNbr].
								s_PulseEncoderModuleInfo.
								dw_SetRegister =
								(devpriv->
								s_ModuleInfo
								[b_ModulNbr].
								s_PulseEncoderModuleInfo.
								dw_SetRegister &
								(0xFFFFFFFFUL -
									(1UL << (8 + b_PulseEncoderNbr)))) | ((1UL & (~b_InputLevelSelection)) << (8 + b_PulseEncoderNbr));

			  /*******************************/
							/* Test if output trigger used */
			  /*******************************/

							if ((b_TriggerOutputAction > 0) && (b_PulseEncoderNbr > 1)) {
			     /****************************/
								/* Enable the output action */
			     /****************************/

								devpriv->
									s_ModuleInfo
									[b_ModulNbr].
									s_PulseEncoderModuleInfo.
									dw_SetRegister
									=
									devpriv->
									s_ModuleInfo
									[b_ModulNbr].
									s_PulseEncoderModuleInfo.
									dw_SetRegister
									| (1UL
									<< (4 + b_PulseEncoderNbr));

			     /*********************************/
								/* Set the output TRIGGER action */
			     /*********************************/

								devpriv->
									s_ModuleInfo
									[b_ModulNbr].
									s_PulseEncoderModuleInfo.
									dw_SetRegister
									=
									(devpriv->
									s_ModuleInfo
									[b_ModulNbr].
									s_PulseEncoderModuleInfo.
									dw_SetRegister
									&
									(0xFFFFFFFFUL
										-
										(1UL << (12 + b_PulseEncoderNbr)))) | ((1UL & (b_TriggerOutputAction - 1)) << (12 + b_PulseEncoderNbr));
							} else {
			     /*****************************/
								/* Disable the output action */
			     /*****************************/

								devpriv->
									s_ModuleInfo
									[b_ModulNbr].
									s_PulseEncoderModuleInfo.
									dw_SetRegister
									=
									devpriv->
									s_ModuleInfo
									[b_ModulNbr].
									s_PulseEncoderModuleInfo.
									dw_SetRegister
									&
									(0xFFFFFFFFUL
									-
									(1UL << (4 + b_PulseEncoderNbr)));
							}

			  /*************************/
							/* Set the configuration */
			  /*************************/

							outl(devpriv->
								s_ModuleInfo
								[b_ModulNbr].
								s_PulseEncoderModuleInfo.
								dw_SetRegister,
								devpriv->
								s_BoardInfos.
								ui_Address +
								20 +
								(64 * b_ModulNbr));

							devpriv->
								s_ModuleInfo
								[b_ModulNbr].
								s_PulseEncoderModuleInfo.
								s_PulseEncoderInfo
								[b_PulseEncoderNbr].
								b_PulseEncoderInit
								= 1;
						} else {
			  /**************************************/
							/* Pulse encoder start value is wrong */
			  /**************************************/

							DPRINTK("Pulse encoder start value is wrong\n");
							i_ReturnValue = -6;
						}
					} else {
		       /****************************************************/
						/* Digital TRIGGER output action selection is wrong */
		       /****************************************************/

						DPRINTK("Digital TRIGGER output action selection is wrong\n");
						i_ReturnValue = -5;
					}
				} else {
		    /**********************************/
					/* Input level selection is wrong */
		    /**********************************/

					DPRINTK("Input level selection is wrong\n");
					i_ReturnValue = -4;
				}
			} else {
		 /************************************/
				/* Pulse encoder selection is wrong */
		 /************************************/

				DPRINTK("Pulse encoder selection is wrong\n");
				i_ReturnValue = -3;
			}
		} else {
	      /********************************************/
			/* The module is not a pulse encoder module */
	      /********************************************/

			DPRINTK("The module is not a pulse encoder module\n");
			i_ReturnValue = -2;
		}
	} else {
	   /********************************************/
		/* The module is not a pulse encoder module */
	   /********************************************/

		DPRINTK("The module is not a pulse encoder module\n");
		i_ReturnValue = -2;
	}

	return i_ReturnValue;
}


int i_APCI1710_InsnWriteEnableDisablePulseEncoder(struct comedi_device *dev,
	struct comedi_subdevice *s, struct comedi_insn *insn, unsigned int *data)
{
	int i_ReturnValue = 0;
	unsigned char b_ModulNbr;
	unsigned char b_PulseEncoderNbr;
	unsigned char b_CycleSelection;
	unsigned char b_InterruptHandling;
	unsigned char b_Action;

	i_ReturnValue = insn->n;
	b_ModulNbr = (unsigned char) CR_AREF(insn->chanspec);
	b_Action = (unsigned char) data[0];
	b_PulseEncoderNbr = (unsigned char) data[1];
	b_CycleSelection = (unsigned char) data[2];
	b_InterruptHandling = (unsigned char) data[3];

	/***********************************/
	/* Test the selected module number */
	/***********************************/

	if (b_ModulNbr <= 3) {
	   /******************************************/
		/* Test the selected pulse encoder number */
	   /******************************************/

		if (b_PulseEncoderNbr <= 3) {
	      /*************************************/
			/* Test if pulse encoder initialised */
	      /*************************************/

			if (devpriv->s_ModuleInfo[b_ModulNbr].
				s_PulseEncoderModuleInfo.
				s_PulseEncoderInfo[b_PulseEncoderNbr].
				b_PulseEncoderInit == 1) {
				switch (b_Action) {

				case APCI1710_ENABLE:
		 /****************************/
					/* Test the cycle selection */
		 /****************************/

					if (b_CycleSelection ==
						APCI1710_CONTINUOUS
						|| b_CycleSelection ==
						APCI1710_SINGLE) {
		    /*******************************/
						/* Test the interrupt handling */
		    /*******************************/

						if (b_InterruptHandling ==
							APCI1710_ENABLE
							|| b_InterruptHandling
							== APCI1710_DISABLE) {
		       /******************************/
							/* Test if interrupt not used */
		       /******************************/

							if (b_InterruptHandling
								==
								APCI1710_DISABLE)
							{
			  /*************************/
								/* Disable the interrupt */
			  /*************************/

								devpriv->
									s_ModuleInfo
									[b_ModulNbr].
									s_PulseEncoderModuleInfo.
									dw_SetRegister
									=
									devpriv->
									s_ModuleInfo
									[b_ModulNbr].
									s_PulseEncoderModuleInfo.
									dw_SetRegister
									&
									(0xFFFFFFFFUL
									-
									(1UL << b_PulseEncoderNbr));
							} else {

			     /************************/
								/* Enable the interrupt */
			     /************************/

								devpriv->
									s_ModuleInfo
									[b_ModulNbr].
									s_PulseEncoderModuleInfo.
									dw_SetRegister
									=
									devpriv->
									s_ModuleInfo
									[b_ModulNbr].
									s_PulseEncoderModuleInfo.
									dw_SetRegister
									| (1UL
									<<
									b_PulseEncoderNbr);
								devpriv->tsk_Current = current;	/*  Save the current process task structure */

							}

							if (i_ReturnValue >= 0) {
			  /***********************************/
								/* Enable or disable the interrupt */
			  /***********************************/

								outl(devpriv->
									s_ModuleInfo
									[b_ModulNbr].
									s_PulseEncoderModuleInfo.
									dw_SetRegister,
									devpriv->
									s_BoardInfos.
									ui_Address
									+ 20 +
									(64 * b_ModulNbr));

			  /****************************/
								/* Enable the pulse encoder */
			  /****************************/
								devpriv->
									s_ModuleInfo
									[b_ModulNbr].
									s_PulseEncoderModuleInfo.
									dw_ControlRegister
									=
									devpriv->
									s_ModuleInfo
									[b_ModulNbr].
									s_PulseEncoderModuleInfo.
									dw_ControlRegister
									| (1UL
									<<
									b_PulseEncoderNbr);

			  /**********************/
								/* Set the cycle mode */
			  /**********************/

								devpriv->
									s_ModuleInfo
									[b_ModulNbr].
									s_PulseEncoderModuleInfo.
									dw_ControlRegister
									=
									(devpriv->
									s_ModuleInfo
									[b_ModulNbr].
									s_PulseEncoderModuleInfo.
									dw_ControlRegister
									&
									(0xFFFFFFFFUL
										-
										(1 << (b_PulseEncoderNbr + 4)))) | ((b_CycleSelection & 1UL) << (4 + b_PulseEncoderNbr));

			  /****************************/
								/* Enable the pulse encoder */
			  /****************************/

								outl(devpriv->
									s_ModuleInfo
									[b_ModulNbr].
									s_PulseEncoderModuleInfo.
									dw_ControlRegister,
									devpriv->
									s_BoardInfos.
									ui_Address
									+ 16 +
									(64 * b_ModulNbr));
							}
						} else {
		       /************************************/
							/* Interrupt handling mode is wrong */
		       /************************************/

							DPRINTK("Interrupt handling mode is wrong\n");
							i_ReturnValue = -6;
						}
					} else {
		    /*********************************/
						/* Cycle selection mode is wrong */
		    /*********************************/

						DPRINTK("Cycle selection mode is wrong\n");
						i_ReturnValue = -5;
					}
					break;

				case APCI1710_DISABLE:
					devpriv->s_ModuleInfo[b_ModulNbr].
						s_PulseEncoderModuleInfo.
						dw_ControlRegister =
						devpriv->
						s_ModuleInfo[b_ModulNbr].
						s_PulseEncoderModuleInfo.
						dw_ControlRegister &
						(0xFFFFFFFFUL -
						(1UL << b_PulseEncoderNbr));

		 /*****************************/
					/* Disable the pulse encoder */
		 /*****************************/

					outl(devpriv->s_ModuleInfo[b_ModulNbr].
						s_PulseEncoderModuleInfo.
						dw_ControlRegister,
						devpriv->s_BoardInfos.
						ui_Address + 16 +
						(64 * b_ModulNbr));

					break;
				}	/*  switch End */

			} else {
		 /*********************************/
				/* Pulse encoder not initialised */
		 /*********************************/

				DPRINTK("Pulse encoder not initialised\n");
				i_ReturnValue = -4;
			}
		} else {
	      /************************************/
			/* Pulse encoder selection is wrong */
	      /************************************/

			DPRINTK("Pulse encoder selection is wrong\n");
			i_ReturnValue = -3;
		}
	} else {
	   /*****************************/
		/* Module selection is wrong */
	   /*****************************/

		DPRINTK("Module selection is wrong\n");
		i_ReturnValue = -2;
	}

	return i_ReturnValue;
}


int i_APCI1710_InsnBitsReadWritePulseEncoder(struct comedi_device *dev,
	struct comedi_subdevice *s, struct comedi_insn *insn, unsigned int *data)
{
	int i_ReturnValue = 0;
	unsigned int dw_StatusRegister;
	unsigned char b_ModulNbr;
	unsigned char b_PulseEncoderNbr;
	unsigned char *pb_Status;
	unsigned char b_Type;
	unsigned int *pul_ReadValue;
	unsigned int ul_WriteValue;

	i_ReturnValue = insn->n;
	b_ModulNbr = (unsigned char) CR_AREF(insn->chanspec);
	b_Type = (unsigned char) data[0];
	b_PulseEncoderNbr = (unsigned char) data[1];
	pb_Status = (unsigned char *) &data[0];
	pul_ReadValue = (unsigned int *) &data[1];

	/***********************************/
	/* Test the selected module number */
	/***********************************/

	if (b_ModulNbr <= 3) {
	   /******************************************/
		/* Test the selected pulse encoder number */
	   /******************************************/

		if (b_PulseEncoderNbr <= 3) {
	      /*************************************/
			/* Test if pulse encoder initialised */
	      /*************************************/

			if (devpriv->s_ModuleInfo[b_ModulNbr].
				s_PulseEncoderModuleInfo.
				s_PulseEncoderInfo[b_PulseEncoderNbr].
				b_PulseEncoderInit == 1) {

				switch (b_Type) {
				case APCI1710_PULSEENCODER_READ:
		 /****************************/
					/* Read the status register */
		 /****************************/

					dw_StatusRegister =
						inl(devpriv->s_BoardInfos.
						ui_Address + 16 +
						(64 * b_ModulNbr));

					devpriv->s_ModuleInfo[b_ModulNbr].
						s_PulseEncoderModuleInfo.
						dw_StatusRegister = devpriv->
						s_ModuleInfo[b_ModulNbr].
						s_PulseEncoderModuleInfo.
						dw_StatusRegister |
						dw_StatusRegister;

					*pb_Status =
						(unsigned char) (devpriv->
						s_ModuleInfo[b_ModulNbr].
						s_PulseEncoderModuleInfo.
						dw_StatusRegister >> (1 +
							b_PulseEncoderNbr)) & 1;

					devpriv->s_ModuleInfo[b_ModulNbr].
						s_PulseEncoderModuleInfo.
						dw_StatusRegister =
						devpriv->
						s_ModuleInfo[b_ModulNbr].
						s_PulseEncoderModuleInfo.
						dw_StatusRegister &
						(0xFFFFFFFFUL - (1 << (1 +
								b_PulseEncoderNbr)));

		 /******************/
					/* Read the value */
		 /******************/

					*pul_ReadValue =
						inl(devpriv->s_BoardInfos.
						ui_Address +
						(4 * b_PulseEncoderNbr) +
						(64 * b_ModulNbr));
					break;

				case APCI1710_PULSEENCODER_WRITE:
					ul_WriteValue = (unsigned int) data[2];
			/*******************/
					/* Write the value */
			/*******************/

					outl(ul_WriteValue,
						devpriv->s_BoardInfos.
						ui_Address +
						(4 * b_PulseEncoderNbr) +
						(64 * b_ModulNbr));

				}	/* end of switch */
			} else {
		 /*********************************/
				/* Pulse encoder not initialised */
		 /*********************************/

				DPRINTK("Pulse encoder not initialised\n");
				i_ReturnValue = -4;
			}
		} else {
	      /************************************/
			/* Pulse encoder selection is wrong */
	      /************************************/

			DPRINTK("Pulse encoder selection is wrong\n");
			i_ReturnValue = -3;
		}
	} else {
	   /*****************************/
		/* Module selection is wrong */
	   /*****************************/

		DPRINTK("Module selection is wrong\n");
		i_ReturnValue = -2;
	}

	return i_ReturnValue;
}

int i_APCI1710_InsnReadInterruptPulseEncoder(struct comedi_device *dev,
	struct comedi_subdevice *s, struct comedi_insn *insn, unsigned int *data)
{

	data[0] = devpriv->s_InterruptParameters.
		s_FIFOInterruptParameters[devpriv->
		s_InterruptParameters.ui_Read].b_OldModuleMask;
	data[1] = devpriv->s_InterruptParameters.
		s_FIFOInterruptParameters[devpriv->
		s_InterruptParameters.ui_Read].ul_OldInterruptMask;
	data[2] = devpriv->s_InterruptParameters.
		s_FIFOInterruptParameters[devpriv->
		s_InterruptParameters.ui_Read].ul_OldCounterLatchValue;

	/***************************/
	/* Increment the read FIFO */
	/***************************/

	devpriv->s_InterruptParameters.
		ui_Read = (devpriv->
		s_InterruptParameters.ui_Read + 1) % APCI1710_SAVE_INTERRUPT;

	return insn->n;

}
