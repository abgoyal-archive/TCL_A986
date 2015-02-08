


#include "APCI1710_Ttl.h"


int i_APCI1710_InsnConfigInitTTLIO(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	int i_ReturnValue = 0;
	unsigned char b_ModulNbr;
	unsigned char b_InitType;
	unsigned char b_PortAMode;
	unsigned char b_PortBMode;
	unsigned char b_PortCMode;
	unsigned char b_PortDMode;

	b_ModulNbr = (unsigned char) CR_AREF(insn->chanspec);
	b_InitType = (unsigned char) data[0];
	i_ReturnValue = insn->n;

	/**************************/
	/* Test the module number */
	/**************************/

	if (b_ModulNbr < 4) {
	   /**************************/
		/* Test if TTL I/O module */
	   /**************************/

		if ((devpriv->s_BoardInfos.
				dw_MolduleConfiguration[b_ModulNbr] &
				0xFFFF0000UL) == APCI1710_TTL_IO) {
			switch (b_InitType) {
			case APCI1710_TTL_INIT:

				devpriv->s_ModuleInfo[b_ModulNbr].
					s_TTLIOInfo.b_TTLInit = 1;

	      /***************************/
				/* Set TTL port A to input */
	      /***************************/

				devpriv->s_ModuleInfo[b_ModulNbr].
					s_TTLIOInfo.b_PortConfiguration[0] = 0;

	      /***************************/
				/* Set TTL port B to input */
	      /***************************/

				devpriv->s_ModuleInfo[b_ModulNbr].
					s_TTLIOInfo.b_PortConfiguration[1] = 0;

	      /***************************/
				/* Set TTL port C to input */
	      /***************************/

				devpriv->s_ModuleInfo[b_ModulNbr].
					s_TTLIOInfo.b_PortConfiguration[2] = 0;

	      /****************************/
				/* Set TTL port D to output */
	      /****************************/

				devpriv->s_ModuleInfo[b_ModulNbr].
					s_TTLIOInfo.b_PortConfiguration[3] = 1;

	      /*************************/
				/* Set the configuration */
	      /*************************/

				outl(0x8,
					devpriv->s_BoardInfos.ui_Address + 20 +
					(64 * b_ModulNbr));
				break;

			case APCI1710_TTL_INITDIRECTION:

				b_PortAMode = (unsigned char) data[1];
				b_PortBMode = (unsigned char) data[2];
				b_PortCMode = (unsigned char) data[3];
				b_PortDMode = (unsigned char) data[4];

	      /********************/
				/* Test the version */
	      /********************/

				if ((devpriv->s_BoardInfos.
						dw_MolduleConfiguration
						[b_ModulNbr] & 0xFFFF) >=
					0x3230) {
		 /************************/
					/* Test the port A mode */
		 /************************/

					if ((b_PortAMode == 0)
						|| (b_PortAMode == 1)) {
		    /************************/
						/* Test the port B mode */
		    /************************/

						if ((b_PortBMode == 0)
							|| (b_PortBMode == 1)) {
		       /************************/
							/* Test the port C mode */
		       /************************/

							if ((b_PortCMode == 0)
								|| (b_PortCMode
									== 1)) {
			  /************************/
								/* Test the port D mode */
			  /************************/

								if ((b_PortDMode == 0) || (b_PortDMode == 1)) {
									devpriv->
										s_ModuleInfo
										[b_ModulNbr].
										s_TTLIOInfo.
										b_TTLInit
										=
										1;

			     /***********************/
									/* Set TTL port A mode */
			     /***********************/

									devpriv->
										s_ModuleInfo
										[b_ModulNbr].
										s_TTLIOInfo.
										b_PortConfiguration
										[0]
										=
										b_PortAMode;

			     /***********************/
									/* Set TTL port B mode */
			     /***********************/

									devpriv->
										s_ModuleInfo
										[b_ModulNbr].
										s_TTLIOInfo.
										b_PortConfiguration
										[1]
										=
										b_PortBMode;

			     /***********************/
									/* Set TTL port C mode */
			     /***********************/

									devpriv->
										s_ModuleInfo
										[b_ModulNbr].
										s_TTLIOInfo.
										b_PortConfiguration
										[2]
										=
										b_PortCMode;

			     /***********************/
									/* Set TTL port D mode */
			     /***********************/

									devpriv->
										s_ModuleInfo
										[b_ModulNbr].
										s_TTLIOInfo.
										b_PortConfiguration
										[3]
										=
										b_PortDMode;

			     /*************************/
									/* Set the configuration */
			     /*************************/

									outl((b_PortAMode << 0) | (b_PortBMode << 1) | (b_PortCMode << 2) | (b_PortDMode << 3), devpriv->s_BoardInfos.ui_Address + 20 + (64 * b_ModulNbr));
								} else {
			     /**********************************/
									/* Port D mode selection is wrong */
			     /**********************************/

									DPRINTK("Port D mode selection is wrong\n");
									i_ReturnValue
										=
										-8;
								}
							} else {
			  /**********************************/
								/* Port C mode selection is wrong */
			  /**********************************/

								DPRINTK("Port C mode selection is wrong\n");
								i_ReturnValue =
									-7;
							}
						} else {
		       /**********************************/
							/* Port B mode selection is wrong */
		       /**********************************/

							DPRINTK("Port B mode selection is wrong\n");
							i_ReturnValue = -6;
						}
					} else {
		    /**********************************/
						/* Port A mode selection is wrong */
		    /**********************************/

						DPRINTK("Port A mode selection is wrong\n");
						i_ReturnValue = -5;
					}
				} else {
		 /*******************************************/
					/* Function not available for this version */
		 /*******************************************/

					DPRINTK("Function not available for this version\n");
					i_ReturnValue = -4;
				}
				break;

				DPRINTK("\n");
			default:
				printk("Bad Config Type\n");
			}	/*  switch end */
		} else {
	      /**********************************/
			/* The module is not a TTL module */
	      /**********************************/

			DPRINTK("The module is not a TTL module\n");
			i_ReturnValue = -3;
		}
	} else {
	   /***********************/
		/* Module number error */
	   /***********************/

		DPRINTK("Module number error\n");
		i_ReturnValue = -2;
	}

	return i_ReturnValue;
}



int i_APCI1710_InsnBitsReadTTLIO(struct comedi_device *dev, struct comedi_subdevice *s,
	struct comedi_insn *insn, unsigned int *data)
{
	int i_ReturnValue = 0;
	unsigned int dw_StatusReg;
	unsigned char b_ModulNbr;
	unsigned char b_SelectedPort;
	unsigned char b_InputChannel;
	unsigned char b_ReadType;
	unsigned char *pb_ChannelStatus;
	unsigned char *pb_PortValue;

	i_ReturnValue = insn->n;
	b_ReadType = (unsigned char) data[0];
	b_ModulNbr = CR_AREF(insn->chanspec);
	b_SelectedPort = CR_RANGE(insn->chanspec);
	b_InputChannel = CR_CHAN(insn->chanspec);

	/**************************/
	/* Test the module number */
	/**************************/

	if (b_ModulNbr < 4) {
	   /**************************/
		/* Test if TTL I/O module */
	   /**************************/

		if ((devpriv->s_BoardInfos.
				dw_MolduleConfiguration[b_ModulNbr] &
				0xFFFF0000UL) == APCI1710_TTL_IO) {
			switch (b_ReadType) {

			case APCI1710_TTL_READCHANNEL:
				pb_ChannelStatus = (unsigned char *) &data[0];
	      /********************************/
				/* Test the TTL I/O port number */
	      /********************************/

				if (((b_SelectedPort <= 2)
						&& ((devpriv->s_BoardInfos.
								dw_MolduleConfiguration
								[b_ModulNbr] &
								0xFFFF) ==
							0x3130))
					|| ((b_SelectedPort <= 3)
						&& ((devpriv->s_BoardInfos.
								dw_MolduleConfiguration
								[b_ModulNbr] &
								0xFFFF) >=
							0x3230))) {
		 /******************************************/
					/* Test the digital imnput channel number */
		 /******************************************/

					if (((b_InputChannel <= 7)
							&& (b_SelectedPort < 3))
						|| ((b_InputChannel <= 1)
							&& (b_SelectedPort ==
								3))) {
		    /******************************************/
						/* Test if the TTL I/O module initialised */
		    /******************************************/

						if (devpriv->
							s_ModuleInfo
							[b_ModulNbr].
							s_TTLIOInfo.b_TTLInit ==
							1) {
		       /***********************************/
							/* Test if TTL port used for input */
		       /***********************************/

							if (((devpriv->s_BoardInfos.dw_MolduleConfiguration[b_ModulNbr] & 0xFFFF) == 0x3130) || (((devpriv->s_BoardInfos.dw_MolduleConfiguration[b_ModulNbr] & 0xFFFF) >= 0x3230) && (devpriv->s_ModuleInfo[b_ModulNbr].s_TTLIOInfo.b_PortConfiguration[b_SelectedPort] == 0))) {
			  /**************************/
								/* Read all digital input */
			  /**************************/

								dw_StatusReg =
									inl
									(devpriv->
									s_BoardInfos.
									ui_Address
									+
									(64 * b_ModulNbr));

								*pb_ChannelStatus
									=
									(unsigned char) (
									(dw_StatusReg
										>>
										(8 * b_SelectedPort)) >> b_InputChannel) & 1;
							} else {
			  /*******************************/
								/* Selected TTL I/O port error */
			  /*******************************/

								DPRINTK("Selected TTL I/O port error\n");
								i_ReturnValue =
									-4;
							}
						} else {
		       /***************************/
							/* TTL I/O not initialised */
		       /***************************/

							DPRINTK("TTL I/O not initialised\n");
							i_ReturnValue = -6;
						}
					} else {
		    /********************************/
						/* Selected digital input error */
		    /********************************/

						DPRINTK("Selected digital input error\n");
						i_ReturnValue = -5;
					}
				} else {
		 /*******************************/
					/* Selected TTL I/O port error */
		 /*******************************/

					DPRINTK("Selected TTL I/O port error\n");
					i_ReturnValue = -4;
				}
				break;

			case APCI1710_TTL_READPORT:
				pb_PortValue = (unsigned char *) &data[0];
			  /********************************/
				/* Test the TTL I/O port number */
			  /********************************/

				if (((b_SelectedPort <= 2)
						&& ((devpriv->s_BoardInfos.
								dw_MolduleConfiguration
								[b_ModulNbr] &
								0xFFFF) ==
							0x3130))
					|| ((b_SelectedPort <= 3)
						&& ((devpriv->s_BoardInfos.
								dw_MolduleConfiguration
								[b_ModulNbr] &
								0xFFFF) >=
							0x3230))) {
		 /******************************************/
					/* Test if the TTL I/O module initialised */
		 /******************************************/

					if (devpriv->s_ModuleInfo[b_ModulNbr].
						s_TTLIOInfo.b_TTLInit == 1) {
		    /***********************************/
						/* Test if TTL port used for input */
		    /***********************************/

						if (((devpriv->s_BoardInfos.
									dw_MolduleConfiguration
									[b_ModulNbr]
									&
									0xFFFF)
								== 0x3130)
							|| (((devpriv->s_BoardInfos.dw_MolduleConfiguration[b_ModulNbr] & 0xFFFF) >= 0x3230) && (devpriv->s_ModuleInfo[b_ModulNbr].s_TTLIOInfo.b_PortConfiguration[b_SelectedPort] == 0))) {
		       /**************************/
							/* Read all digital input */
		       /**************************/

							dw_StatusReg =
								inl(devpriv->
								s_BoardInfos.
								ui_Address +
								(64 * b_ModulNbr));

							*pb_PortValue =
								(unsigned char) (
								(dw_StatusReg >>
									(8 * b_SelectedPort)) & 0xFF);
						} else {
		       /*******************************/
							/* Selected TTL I/O port error */
		       /*******************************/

							DPRINTK("Selected TTL I/O port error\n");
							i_ReturnValue = -4;
						}
					} else {
		    /***************************/
						/* TTL I/O not initialised */
		    /***************************/

						DPRINTK("TTL I/O not initialised\n");
						i_ReturnValue = -5;
					}
				} else {
		 /*******************************/
					/* Selected TTL I/O port error */
		 /*******************************/

					DPRINTK("Selected TTL I/O port error\n");
					i_ReturnValue = -4;
				}
				break;

			default:
				printk("Bad ReadType\n");

			}	/* End Switch */
		} else {
	      /**********************************/
			/* The module is not a TTL module */
	      /**********************************/

			DPRINTK("The module is not a TTL module\n");
			i_ReturnValue = -3;
		}
	} else {
	   /***********************/
		/* Module number error */
	   /***********************/

		DPRINTK("Module number error\n");
		i_ReturnValue = -2;
	}

	return i_ReturnValue;
}


int i_APCI1710_InsnReadTTLIOAllPortValue(struct comedi_device *dev,
	struct comedi_subdevice *s, struct comedi_insn *insn, unsigned int *data)
{
	int i_ReturnValue = 0;
	unsigned int dw_StatusReg;
	unsigned char b_ModulNbr;
	unsigned int *pul_PortValue;

	b_ModulNbr = (unsigned char) CR_AREF(insn->chanspec);
	i_ReturnValue = insn->n;
	pul_PortValue = (unsigned int *) &data[0];

	/**************************/
	/* Test the module number */
	/**************************/

	if (b_ModulNbr < 4) {
	   /**************************/
		/* Test if TTL I/O module */
	   /**************************/

		if ((devpriv->s_BoardInfos.
				dw_MolduleConfiguration[b_ModulNbr] &
				0xFFFF0000UL) == APCI1710_TTL_IO) {
	      /******************************************/
			/* Test if the TTL I/O module initialised */
	      /******************************************/

			if (devpriv->
				s_ModuleInfo[b_ModulNbr].
				s_TTLIOInfo.b_TTLInit == 1) {
		 /**************************/
				/* Read all digital input */
		 /**************************/

				dw_StatusReg = inl(devpriv->s_BoardInfos.
					ui_Address + (64 * b_ModulNbr));

		 /**********************/
				/* Test if TTL Rev1.0 */
		 /**********************/

				if ((devpriv->s_BoardInfos.
						dw_MolduleConfiguration
						[b_ModulNbr] & 0xFFFF) ==
					0x3130) {
					*pul_PortValue =
						dw_StatusReg & 0xFFFFFFUL;
				} else {
		    /**************************************/
					/* Test if port A not used for output */
		    /**************************************/

					if (devpriv->s_ModuleInfo[b_ModulNbr].
						s_TTLIOInfo.
						b_PortConfiguration[0] == 1) {
						*pul_PortValue =
							dw_StatusReg &
							0x3FFFF00UL;
					}

		    /**************************************/
					/* Test if port B not used for output */
		    /**************************************/

					if (devpriv->
						s_ModuleInfo[b_ModulNbr].
						s_TTLIOInfo.
						b_PortConfiguration[1] == 1) {
						*pul_PortValue =
							dw_StatusReg &
							0x3FF00FFUL;
					}

		    /**************************************/
					/* Test if port C not used for output */
		    /**************************************/

					if (devpriv->
						s_ModuleInfo[b_ModulNbr].
						s_TTLIOInfo.
						b_PortConfiguration[2] == 1) {
						*pul_PortValue =
							dw_StatusReg &
							0x300FFFFUL;
					}

		    /**************************************/
					/* Test if port D not used for output */
		    /**************************************/

					if (devpriv->
						s_ModuleInfo[b_ModulNbr].
						s_TTLIOInfo.
						b_PortConfiguration[3] == 1) {
						*pul_PortValue =
							dw_StatusReg &
							0xFFFFFFUL;
					}
				}
			} else {
		 /***************************/
				/* TTL I/O not initialised */
		 /***************************/
				DPRINTK("TTL I/O not initialised\n");
				i_ReturnValue = -5;
			}
		} else {
	      /**********************************/
			/* The module is not a TTL module */
	      /**********************************/
			DPRINTK("The module is not a TTL module\n");
			i_ReturnValue = -3;
		}
	} else {
	   /***********************/
		/* Module number error */
	   /***********************/
		DPRINTK("Module number error\n");
		i_ReturnValue = -2;
	}

	return i_ReturnValue;
}


+----------------------------------------------------------------------------+
| Output Parameters : -                                                      |
+----------------------------------------------------------------------------+
| Return Value      : 0: No error                                            |
|                    -1: The handle parameter of the board is wrong          |
|                    -2: The module parameter is wrong                       |
|                    -3: The module is not a TTL I/O module                  |
|                    -4: The selected digital output is wrong                |
|                    -5: TTL I/O not initialised see function                |
|                        " i_APCI1710_InitTTLIO"
+----------------------------------------------------------------------------+
*/

int i_APCI1710_InsnWriteSetTTLIOChlOnOff(struct comedi_device *dev,
	struct comedi_subdevice *s, struct comedi_insn *insn, unsigned int *data)
{
	int i_ReturnValue = 0;
	unsigned int dw_StatusReg = 0;
	unsigned char b_ModulNbr;
	unsigned char b_OutputChannel;
	unsigned int ui_State;

	i_ReturnValue = insn->n;
	b_ModulNbr = CR_AREF(insn->chanspec);
	b_OutputChannel = CR_CHAN(insn->chanspec);
	ui_State = data[0];	/*  ON or OFF */

	/**************************/
	/* Test the module number */
	/**************************/

	if (b_ModulNbr < 4) {
	   /**************************/
		/* Test if TTL I/O module */
	   /**************************/

		if ((devpriv->s_BoardInfos.
				dw_MolduleConfiguration[b_ModulNbr] &
				0xFFFF0000UL) == APCI1710_TTL_IO) {
	      /******************************************/
			/* Test if the TTL I/O module initialised */
	      /******************************************/

			if (devpriv->s_ModuleInfo[b_ModulNbr].
				s_TTLIOInfo.b_TTLInit == 1) {
		 /***********************************/
				/* Test the TTL I/O channel number */
		 /***********************************/

				if (((b_OutputChannel <= 1)
						&& ((devpriv->s_BoardInfos.
								dw_MolduleConfiguration
								[b_ModulNbr] &
								0xFFFF) ==
							0x3130))
					|| ((b_OutputChannel <= 25)
						&& ((devpriv->s_BoardInfos.
								dw_MolduleConfiguration
								[b_ModulNbr] &
								0xFFFF) >=
							0x3230))) {
		    /****************************************************/
					/* Test if the selected channel is a output channel */
		    /****************************************************/

					if (((b_OutputChannel <= 1)
							&& (devpriv->
								s_ModuleInfo
								[b_ModulNbr].
								s_TTLIOInfo.
								b_PortConfiguration
								[3] == 1))
						|| ((b_OutputChannel >= 2)
							&& (b_OutputChannel <=
								9)
							&& (devpriv->
								s_ModuleInfo
								[b_ModulNbr].
								s_TTLIOInfo.
								b_PortConfiguration
								[0] == 1))
						|| ((b_OutputChannel >= 10)
							&& (b_OutputChannel <=
								17)
							&& (devpriv->
								s_ModuleInfo
								[b_ModulNbr].
								s_TTLIOInfo.
								b_PortConfiguration
								[1] == 1))
						|| ((b_OutputChannel >= 18)
							&& (b_OutputChannel <=
								25)
							&& (devpriv->
								s_ModuleInfo
								[b_ModulNbr].
								s_TTLIOInfo.
								b_PortConfiguration
								[2] == 1))) {
		       /************************/
						/* Test if PD0 selected */
		       /************************/

						if (b_OutputChannel == 0) {

							outl(ui_State,
								devpriv->
								s_BoardInfos.
								ui_Address +
								(64 * b_ModulNbr));
						} else {
			  /************************/
							/* Test if PD1 selected */
			  /************************/

							if (b_OutputChannel ==
								1) {

								outl(ui_State,
									devpriv->
									s_BoardInfos.
									ui_Address
									+ 4 +
									(64 * b_ModulNbr));
							} else {
								b_OutputChannel
									=
									b_OutputChannel
									- 2;

			     /********************/
								/* Read all channel */
			     /********************/

								dw_StatusReg =
									inl
									(devpriv->
									s_BoardInfos.
									ui_Address
									+
									(64 * b_ModulNbr));
								if (ui_State)	/*  ON */
								{
									dw_StatusReg
										=
										(dw_StatusReg
										>>
										((b_OutputChannel / 8) * 8)) & 0xFF;
									dw_StatusReg
										=
										dw_StatusReg
										|
										(1
										<<
										(b_OutputChannel
											%
											8));
								} else	/*  Off */
								{
									dw_StatusReg
										=
										(dw_StatusReg
										>>
										((b_OutputChannel / 8) * 8)) & 0xFF;
									dw_StatusReg
										=
										dw_StatusReg
										&
										(0xFF
										-
										(1 << (b_OutputChannel % 8)));

								}

			     /****************************/
								/* Set the new output value */
			     /****************************/

								outl(dw_StatusReg, devpriv->s_BoardInfos.ui_Address + 8 + ((b_OutputChannel / 8) * 4) + (64 * b_ModulNbr));
							}
						}
					} else {
		       /************************************/
						/* The selected TTL output is wrong */
		       /************************************/

						DPRINTK(" The selected TTL output is wrong\n");
						i_ReturnValue = -4;
					}
				} else {
		    /************************************/
					/* The selected TTL output is wrong */
		    /************************************/

					DPRINTK("The selected TTL output is wrong\n");
					i_ReturnValue = -4;
				}
			} else {
		 /***************************/
				/* TTL I/O not initialised */
		 /***************************/

				DPRINTK("TTL I/O not initialised\n");
				i_ReturnValue = -5;
			}
		} else {
	      /**************************************/
			/* The module is not a TTL I/O module */
	      /**************************************/

			DPRINTK("The module is not a TTL I/O module\n");
			i_ReturnValue = -3;
		}
	} else {
	   /***********************/
		/* Module number error */
	   /***********************/

		DPRINTK("Module number error\n");
		i_ReturnValue = -2;
	}

	return i_ReturnValue;
}
