

#include "APCI1710_82x54.h"


int i_APCI1710_InsnConfigInitTimer(struct comedi_device *dev, struct comedi_subdevice *s,
				   struct comedi_insn *insn, unsigned int *data)
{

	int i_ReturnValue = 0;
	unsigned char b_ModulNbr;
	unsigned char b_TimerNbr;
	unsigned char b_TimerMode;
	unsigned int ul_ReloadValue;
	unsigned char b_InputClockSelection;
	unsigned char b_InputClockLevel;
	unsigned char b_OutputLevel;
	unsigned char b_HardwareGateLevel;

	/* BEGIN JK 27.10.2003 : Add the possibility to use a 40 Mhz quartz */
	unsigned int dw_Test = 0;
	/* END JK 27.10.2003 : Add the possibility to use a 40 Mhz quartz */

	i_ReturnValue = insn->n;
	b_ModulNbr = (unsigned char) CR_AREF(insn->chanspec);
	b_TimerNbr = (unsigned char) CR_CHAN(insn->chanspec);
	b_TimerMode = (unsigned char) data[0];
	ul_ReloadValue = (unsigned int) data[1];
	b_InputClockSelection = (unsigned char) data[2];
	b_InputClockLevel = (unsigned char) data[3];
	b_OutputLevel = (unsigned char) data[4];
	b_HardwareGateLevel = (unsigned char) data[5];

	/* Test the module number */
	if (b_ModulNbr < 4) {
		/* Test if 82X54 timer */
		if ((devpriv->s_BoardInfos.dw_MolduleConfiguration[b_ModulNbr] & 0xFFFF0000UL) == APCI1710_82X54_TIMER) {
			/* Test the timer number */

			if (b_TimerNbr <= 2) {
				/* Test the timer mode */
				if (b_TimerMode <= 5) {
					/* BEGIN JK 27.10.2003 : Add the possibility to use a 40 Mhz quartz */
					/* Test te imput clock selection */
					/*
					   if (((b_TimerNbr == 0) && (b_InputClockSelection == 0)) ||
					   ((b_TimerNbr != 0) && ((b_InputClockSelection == 0) || (b_InputClockSelection == 1))))
					 */

					if (((b_TimerNbr == 0) &&
					     (b_InputClockSelection == APCI1710_PCI_BUS_CLOCK)) ||
					    ((b_TimerNbr == 0) &&
					     (b_InputClockSelection == APCI1710_10MHZ)) ||
					    ((b_TimerNbr != 0) &&
					     ((b_InputClockSelection == APCI1710_PCI_BUS_CLOCK) ||
					      (b_InputClockSelection == APCI1710_FRONT_CONNECTOR_INPUT) ||
					      (b_InputClockSelection == APCI1710_10MHZ)))) {
						/* BEGIN JK 27.10.2003 : Add the possibility to use a 40 Mhz quartz */
						if (((b_InputClockSelection == APCI1710_10MHZ) &&
						     ((devpriv->s_BoardInfos.dw_MolduleConfiguration[b_ModulNbr] & 0x0000FFFFUL) >= 0x3131)) ||
						     (b_InputClockSelection != APCI1710_10MHZ)) {
							/* END JK 27.10.2003 : Add the possibility to use a 40 Mhz quartz */
							/* Test the input clock level selection */

							if ((b_InputClockLevel == 0) ||
							    (b_InputClockLevel == 1)) {
								/* Test the output clock level selection */
								if ((b_OutputLevel == 0) || (b_OutputLevel == 1)) {
									/* Test the hardware gate level selection */
									if ((b_HardwareGateLevel == 0) || (b_HardwareGateLevel == 1)) {
										/* BEGIN JK 27.10.03 : Add the possibility to use a 40 Mhz quartz */
										/* Test if version > 1.1 and clock selection = 10MHz */
										if ((b_InputClockSelection == APCI1710_10MHZ) && ((devpriv->s_BoardInfos.dw_MolduleConfiguration[b_ModulNbr] & 0x0000FFFFUL) > 0x3131)) {
											/* Test if 40MHz quartz on board */
											dw_Test = inl(devpriv->s_BoardInfos.ui_Address + (16 + (b_TimerNbr * 4) + (64 * b_ModulNbr)));

											dw_Test = (dw_Test >> 16) & 1;
										} else {
											dw_Test = 1;
										}

										/* Test if detection OK */
										if (dw_Test == 1) {
											/* END JK 27.10.03 : Add the possibility to use a 40 Mhz quartz */
											/* Initialisation OK */
											devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].b_82X54Init = 1;

											/* Save the input clock selection */
											devpriv-> s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].b_InputClockSelection = b_InputClockSelection;

											/* Save the input clock level */
											devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].b_InputClockLevel = ~b_InputClockLevel & 1;

											/* Save the output level */
											devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].b_OutputLevel = ~b_OutputLevel & 1;

											/* Save the gate level */
											devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].b_HardwareGateLevel = b_HardwareGateLevel;

											/* Set the configuration word and disable the timer */
											/* BEGIN JK 27.10.03 : Add the possibility to use a 40 Mhz quartz */
											/*
											   devpriv->s_ModuleInfo [b_ModulNbr].
											   s_82X54ModuleInfo.
											   s_82X54TimerInfo  [b_TimerNbr].
											   dw_ConfigurationWord = (unsigned int) (((b_HardwareGateLevel         << 0) & 0x1) |
											   ((b_InputClockLevel           << 1) & 0x2) |
											   (((~b_OutputLevel       & 1)  << 2) & 0x4) |
											   ((b_InputClockSelection       << 4) & 0x10));
											 */
											/* Test if 10MHz selected */
											if (b_InputClockSelection == APCI1710_10MHZ) {
												b_InputClockSelection = 2;
											}

											devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].dw_ConfigurationWord = (unsigned int)(((b_HardwareGateLevel << 0) & 0x1) | ((b_InputClockLevel << 1) & 0x2) | (((~b_OutputLevel & 1) << 2) & 0x4) | ((b_InputClockSelection << 4) & 0x30));
											/* END JK 27.10.03 : Add the possibility to use a 40 Mhz quartz */
											outl(devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].dw_ConfigurationWord, devpriv->s_BoardInfos.ui_Address + 32 + (b_TimerNbr * 4) + (64 * b_ModulNbr));

											/* Initialise the 82X54 Timer */
											outl((unsigned int) b_TimerMode, devpriv->s_BoardInfos.ui_Address + 16 + (b_TimerNbr * 4) + (64 * b_ModulNbr));

											/* Write the reload value */
											outl(ul_ReloadValue, devpriv->s_BoardInfos.ui_Address + 0 + (b_TimerNbr * 4) + (64 * b_ModulNbr));
											/* BEGIN JK 27.10.03 : Add the possibility to use a 40 Mhz quartz */
										}	/*  if (dw_Test == 1) */
										else {
											/* Input timer clock selection is wrong */
											i_ReturnValue = -6;
										}	/*  if (dw_Test == 1) */
										/* END JK 27.10.03 : Add the possibility to use a 40 Mhz quartz */
									}	/*  if ((b_HardwareGateLevel == 0) || (b_HardwareGateLevel == 1)) */
									else {
										/* Selection from hardware gate level is wrong */
										DPRINTK("Selection from hardware gate level is wrong\n");
										i_ReturnValue = -9;
									}	/*  if ((b_HardwareGateLevel == 0) || (b_HardwareGateLevel == 1)) */
								}	/*  if ((b_OutputLevel == 0) || (b_OutputLevel == 1)) */
								else {
									/* Selection from output clock level is wrong */
									DPRINTK("Selection from output clock level is wrong\n");
									i_ReturnValue = -8;
								}	/*  if ((b_OutputLevel == 0) || (b_OutputLevel == 1)) */
							}	/*  if ((b_InputClockLevel == 0) || (b_InputClockLevel == 1)) */
							else {
								/* Selection from input clock level is wrong */
								DPRINTK("Selection from input clock level is wrong\n");
								i_ReturnValue = -7;
							}	/*  if ((b_InputClockLevel == 0) || (b_InputClockLevel == 1)) */
						} else {
							/* Input timer clock selection is wrong */
							DPRINTK("Input timer clock selection is wrong\n");
							i_ReturnValue = -6;
						}
					} else {
						/* Input timer clock selection is wrong */
						DPRINTK("Input timer clock selection is wrong\n");
						i_ReturnValue = -6;
					}
				}	/*  if ((b_TimerMode >= 0) && (b_TimerMode <= 5)) */
				else {
					/* Timer mode selection is wrong */
					DPRINTK("Timer mode selection is wrong\n");
					i_ReturnValue = -5;
				}	/*  if ((b_TimerMode >= 0) && (b_TimerMode <= 5)) */
			}	/*  if ((b_TimerNbr >= 0) && (b_TimerNbr <= 2)) */
			else {
				/* Timer selection wrong */
				DPRINTK("Timer selection wrong\n");
				i_ReturnValue = -3;
			}	/*  if ((b_TimerNbr >= 0) && (b_TimerNbr <= 2)) */
		} else {
			/* The module is not a TIMER module */
			DPRINTK("The module is not a TIMER module\n");
			i_ReturnValue = -4;
		}
	} else {
		/* Module number error */
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
|                    -2: Module selection wrong                              |
|                    -3: Timer selection wrong                               |
|                    -4: The module is not a TIMER module                    |
|                    -5: Timer not initialised see function                  |
|                        "i_APCI1710_InitTimer"                              |
|                    -6: Interrupt parameter is wrong                        |
|                    -7: Interrupt function not initialised.                 |
|                        See function "i_APCI1710_SetBoardIntRoutineX"       |
+----------------------------------------------------------------------------+
*/

int i_APCI1710_InsnWriteEnableDisableTimer(struct comedi_device *dev,
					   struct comedi_subdevice *s,
					   struct comedi_insn *insn, unsigned int *data)
{
	int i_ReturnValue = 0;
	unsigned int dw_DummyRead;
	unsigned char b_ModulNbr;
	unsigned char b_TimerNbr;
	unsigned char b_ActionType;
	unsigned char b_InterruptEnable;

	i_ReturnValue = insn->n;
	b_ModulNbr = (unsigned char) CR_AREF(insn->chanspec);
	b_TimerNbr = (unsigned char) CR_CHAN(insn->chanspec);
	b_ActionType = (unsigned char) data[0];	/*  enable disable */

	/* Test the module number */
	if (b_ModulNbr < 4) {
		/* Test if 82X54 timer */
		if ((devpriv->s_BoardInfos.dw_MolduleConfiguration[b_ModulNbr] & 0xFFFF0000UL) == APCI1710_82X54_TIMER) {
			/* Test the timer number */
			if (b_TimerNbr <= 2) {
				/* Test if timer initialised */
				if (devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].b_82X54Init == 1) {

					switch (b_ActionType) {
					case APCI1710_ENABLE:
						b_InterruptEnable = (unsigned char) data[1];
						/* Test the interrupt selection */
						if ((b_InterruptEnable == APCI1710_ENABLE) ||
						    (b_InterruptEnable == APCI1710_DISABLE)) {
							if (b_InterruptEnable == APCI1710_ENABLE) {

								dw_DummyRead = inl(devpriv->s_BoardInfos.ui_Address + 12 + (b_TimerNbr * 4) + (64 * b_ModulNbr));

								/* Enable the interrupt */
								devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].dw_ConfigurationWord = devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].dw_ConfigurationWord | 0x8;

								outl(devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].dw_ConfigurationWord, devpriv->s_BoardInfos.ui_Address + 32 + (b_TimerNbr * 4) + (64 * b_ModulNbr));
								devpriv->tsk_Current = current;	/*  Save the current process task structure */

							}	/*  if (b_InterruptEnable == APCI1710_ENABLE) */
							else {
								/* Disable the interrupt */
								devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].dw_ConfigurationWord = devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].dw_ConfigurationWord & 0xF7;

								outl(devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].dw_ConfigurationWord, devpriv->s_BoardInfos.ui_Address + 32 + (b_TimerNbr * 4) + (64 * b_ModulNbr));

								/* Save the interrupt flag */
								devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.b_InterruptMask = devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.b_InterruptMask & (0xFF - (1 << b_TimerNbr));
							}	/*  if (b_InterruptEnable == APCI1710_ENABLE) */

							/* Test if error occur */
							if (i_ReturnValue >= 0) {
								/* Save the interrupt flag */
								devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.b_InterruptMask = devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.b_InterruptMask | ((1 & b_InterruptEnable) << b_TimerNbr);

								/* Enable the timer */
								outl(1, devpriv->s_BoardInfos.ui_Address + 44 + (b_TimerNbr * 4) + (64 * b_ModulNbr));
							}
						} else {
							/* Interrupt parameter is wrong */
							DPRINTK("\n");
							i_ReturnValue = -6;
						}
						break;
					case APCI1710_DISABLE:
						/* Test the interrupt flag */
						if (((devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.b_InterruptMask >> b_TimerNbr) & 1) == 1) {
							/* Disable the interrupt */

							devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr]. dw_ConfigurationWord = devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].dw_ConfigurationWord & 0xF7;

							outl(devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].dw_ConfigurationWord, devpriv->s_BoardInfos.ui_Address + 32 + (b_TimerNbr * 4) + (64 * b_ModulNbr));

							/* Save the interrupt flag */
							devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.b_InterruptMask = devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.b_InterruptMask & (0xFF - (1 << b_TimerNbr));
						}

						/* Disable the timer */
						outl(0, devpriv->s_BoardInfos.ui_Address + 44 + (b_TimerNbr * 4) + (64 * b_ModulNbr));
						break;
					}	/*  Switch end */
				} else {
					/* Timer not initialised see function */
					DPRINTK ("Timer not initialised see function\n");
					i_ReturnValue = -5;
				}
			} else {
				/* Timer selection wrong */
				DPRINTK("Timer selection wrong\n");
				i_ReturnValue = -3;
			}	/*  if ((b_TimerNbr >= 0) && (b_TimerNbr <= 2)) */
		} else {
			/* The module is not a TIMER module */
			DPRINTK("The module is not a TIMER module\n");
			i_ReturnValue = -4;
		}
	} else {
		/* Module number error */
		DPRINTK("Module number error\n");
		i_ReturnValue = -2;
	}

	return i_ReturnValue;
}


int i_APCI1710_InsnReadAllTimerValue(struct comedi_device *dev, struct comedi_subdevice *s,
				     struct comedi_insn *insn, unsigned int *data)
{
	int i_ReturnValue = 0;
	unsigned char b_ModulNbr, b_ReadType;
	unsigned int *pul_TimerValueArray;

	b_ModulNbr = CR_AREF(insn->chanspec);
	b_ReadType = CR_CHAN(insn->chanspec);
	pul_TimerValueArray = (unsigned int *) data;
	i_ReturnValue = insn->n;

	switch (b_ReadType) {
	case APCI1710_TIMER_READINTERRUPT:

		data[0] = devpriv->s_InterruptParameters.s_FIFOInterruptParameters[devpriv->s_InterruptParameters.ui_Read].b_OldModuleMask;
		data[1] = devpriv->s_InterruptParameters.s_FIFOInterruptParameters[devpriv->s_InterruptParameters.ui_Read].ul_OldInterruptMask;
		data[2] = devpriv->s_InterruptParameters.s_FIFOInterruptParameters[devpriv->s_InterruptParameters.ui_Read].ul_OldCounterLatchValue;

		/* Increment the read FIFO */
		devpriv->s_InterruptParameters.ui_Read = (devpriv->s_InterruptParameters.ui_Read + 1) % APCI1710_SAVE_INTERRUPT;

		break;

	case APCI1710_TIMER_READALLTIMER:
		/* Test the module number */
		if (b_ModulNbr < 4) {
			/* Test if 82X54 timer */
			if ((devpriv->s_BoardInfos.dw_MolduleConfiguration[b_ModulNbr] & 0xFFFF0000UL) == APCI1710_82X54_TIMER) {
				/* Test if timer 0 iniutialised */
				if (devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[0].b_82X54Init == 1) {
					/* Test if timer 1 iniutialised */
					if (devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[1].b_82X54Init == 1) {
						/* Test if timer 2 iniutialised */
						if (devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[2].b_82X54Init == 1) {
							/* Latch all counter */
							outl(0x17, devpriv->s_BoardInfos.ui_Address + 12 + (64 * b_ModulNbr));

							/* Read the timer 0 value */
							pul_TimerValueArray[0] = inl(devpriv->s_BoardInfos.ui_Address + 0 + (64 * b_ModulNbr));

							/* Read the timer 1 value */
							pul_TimerValueArray[1] = inl(devpriv->s_BoardInfos.ui_Address + 4 + (64 * b_ModulNbr));

							/* Read the timer 2 value */
							pul_TimerValueArray[2] = inl(devpriv->s_BoardInfos.ui_Address + 8 + (64 * b_ModulNbr));
						} else {
							/* Timer 2 not initialised see function */
							DPRINTK("Timer 2 not initialised see function\n");
							i_ReturnValue = -6;
						}
					} else {
						/* Timer 1 not initialised see function */
						DPRINTK("Timer 1 not initialised see function\n");
						i_ReturnValue = -5;
					}
				} else {
					/* Timer 0 not initialised see function */
					DPRINTK("Timer 0 not initialised see function\n");
					i_ReturnValue = -4;
				}
			} else {
				/* The module is not a TIMER module */
				DPRINTK("The module is not a TIMER module\n");
				i_ReturnValue = -3;
			}
		} else {
			/* Module number error */
			DPRINTK("Module number error\n");
			i_ReturnValue = -2;
		}

	}			/*  End of Switch */
	return i_ReturnValue;
}


int i_APCI1710_InsnBitsTimer(struct comedi_device *dev, struct comedi_subdevice *s,
			     struct comedi_insn *insn, unsigned int *data)
{
	unsigned char b_BitsType;
	int i_ReturnValue = 0;
	b_BitsType = data[0];

	printk("\n82X54");

	switch (b_BitsType) {
	case APCI1710_TIMER_READVALUE:
		i_ReturnValue = i_APCI1710_ReadTimerValue(dev,
							  (unsigned char)CR_AREF(insn->chanspec),
							  (unsigned char)CR_CHAN(insn->chanspec),
							  (unsigned int *) &data[0]);
		break;

	case APCI1710_TIMER_GETOUTPUTLEVEL:
		i_ReturnValue = i_APCI1710_GetTimerOutputLevel(dev,
							       (unsigned char)CR_AREF(insn->chanspec),
							       (unsigned char)CR_CHAN(insn->chanspec),
							       (unsigned char *) &data[0]);
		break;

	case APCI1710_TIMER_GETPROGRESSSTATUS:
		i_ReturnValue = i_APCI1710_GetTimerProgressStatus(dev,
								  (unsigned char)CR_AREF(insn->chanspec),
								  (unsigned char)CR_CHAN(insn->chanspec),
								  (unsigned char *)&data[0]);
		break;

	case APCI1710_TIMER_WRITEVALUE:
		i_ReturnValue = i_APCI1710_WriteTimerValue(dev,
							   (unsigned char)CR_AREF(insn->chanspec),
							   (unsigned char)CR_CHAN(insn->chanspec),
							   (unsigned int)data[1]);

		break;

	default:
		printk("Bits Config Parameter Wrong\n");
		i_ReturnValue = -1;
	}

	if (i_ReturnValue >= 0)
		i_ReturnValue = insn->n;
	return i_ReturnValue;
}


int i_APCI1710_ReadTimerValue(struct comedi_device *dev,
			      unsigned char b_ModulNbr, unsigned char b_TimerNbr,
			      unsigned int *pul_TimerValue)
{
	int i_ReturnValue = 0;

	/* Test the module number */
	if (b_ModulNbr < 4) {
		/* Test if 82X54 timer */
		if ((devpriv->s_BoardInfos.
		     dw_MolduleConfiguration[b_ModulNbr] &
		     0xFFFF0000UL) == APCI1710_82X54_TIMER) {
			/* Test the timer number */
			if (b_TimerNbr <= 2) {
				/* Test if timer initialised */
				if (devpriv->
				    s_ModuleInfo[b_ModulNbr].
				    s_82X54ModuleInfo.
				    s_82X54TimerInfo[b_TimerNbr].
				    b_82X54Init == 1) {
					/* Latch the timer value */
					outl((2 << b_TimerNbr) | 0xD0,
					     devpriv->s_BoardInfos.
					     ui_Address + 12 +
					     (64 * b_ModulNbr));

					/* Read the counter value */
					*pul_TimerValue =
					    inl(devpriv->s_BoardInfos.
						ui_Address + (b_TimerNbr * 4) +
						(64 * b_ModulNbr));
				} else {
					/* Timer not initialised see function */
					DPRINTK("Timer not initialised see function\n");
					i_ReturnValue = -5;
				}
			} else {
				/* Timer selection wrong */
				DPRINTK("Timer selection wrong\n");
				i_ReturnValue = -3;
			}	/*  if ((b_TimerNbr >= 0) && (b_TimerNbr <= 2)) */
		} else {
			/* The module is not a TIMER module */
			DPRINTK("The module is not a TIMER module\n");
			i_ReturnValue = -4;
		}
	} else {
		/* Module number error */
		DPRINTK("Module number error\n");
		i_ReturnValue = -2;
	}

	return i_ReturnValue;
}

	/*
	   +----------------------------------------------------------------------------+
	   | Function Name     : _INT_     i_APCI1710_GetTimerOutputLevel               |
	   |                                       (unsigned char_     b_BoardHandle,            |
	   |                                        unsigned char_     b_ModulNbr,               |
	   |                                        unsigned char_     b_TimerNbr,               |
	   |                                        unsigned char *_   pb_OutputLevel)            |
	   +----------------------------------------------------------------------------+
	   | Task              : Return the output signal level (pb_OutputLevel) from   |
	   |                     selected digital timer (b_TimerNbr) from selected timer|
	   |                     module (b_ModulNbr).                                   |
	   +----------------------------------------------------------------------------+
	   | Input Parameters  : unsigned char_   b_BoardHandle     : Handle of board            |
	   |                                                 APCI-1710                  |
	   |                     unsigned char_   b_ModulNbr        : Selected module number     |
	   |                                                 (0 to 3)                   |
	   |                     unsigned char_   b_TimerNbr        : Timer number to test       |
	   |                                                 (0 to 2)                   |
	   +----------------------------------------------------------------------------+
	   | Output Parameters : unsigned char *_ pb_OutputLevel     : Output signal level        |
	   |                                                 0 : The output is low      |
	   |                                                 1 : The output is high     |
	   +----------------------------------------------------------------------------+
	   | Return Value      : 0: No error                                            |
	   |                    -1: The handle parameter of the board is wrong          |
	   |                    -2: Module selection wrong                              |
	   |                    -3: Timer selection wrong                               |
	   |                    -4: The module is not a TIMER module                    |
	   |                    -5: Timer not initialised see function                  |
	   |                        "i_APCI1710_InitTimer"                              |
	   +----------------------------------------------------------------------------+
	 */

int i_APCI1710_GetTimerOutputLevel(struct comedi_device *dev,
				   unsigned char b_ModulNbr, unsigned char b_TimerNbr,
				   unsigned char *pb_OutputLevel)
{
	int i_ReturnValue = 0;
	unsigned int dw_TimerStatus;

	/* Test the module number */
	if (b_ModulNbr < 4) {
		/* Test if 82X54 timer */
		if ((devpriv->s_BoardInfos.dw_MolduleConfiguration[b_ModulNbr] & 0xFFFF0000UL) == APCI1710_82X54_TIMER) {
			/* Test the timer number */
			if (b_TimerNbr <= 2) {
				/* Test if timer initialised */
				if (devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].b_82X54Init == 1) {
					/* Latch the timer value */
					outl((2 << b_TimerNbr) | 0xE0, devpriv->s_BoardInfos.ui_Address + 12 + (64 * b_ModulNbr));

					/* Read the timer status */
					dw_TimerStatus = inl(devpriv->s_BoardInfos.ui_Address + 16 + (b_TimerNbr * 4) + (64 * b_ModulNbr));

					*pb_OutputLevel = (unsigned char) (((dw_TimerStatus >> 7) & 1) ^ devpriv-> s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].b_OutputLevel);
				} else {
					/* Timer not initialised see function */
					DPRINTK("Timer not initialised see function\n");
					i_ReturnValue = -5;
				}
			} else {
				/* Timer selection wrong */
				DPRINTK("Timer selection wrong\n");
				i_ReturnValue = -3;
			}	/*  if ((b_TimerNbr >= 0) && (b_TimerNbr <= 2)) */
		} else {
			/* The module is not a TIMER module */
			DPRINTK("The module is not a TIMER module\n");
			i_ReturnValue = -4;
		}
	} else {
		/* Module number error */
		DPRINTK("Module number error\n");
		i_ReturnValue = -2;
	}

	return i_ReturnValue;
}


int i_APCI1710_GetTimerProgressStatus(struct comedi_device *dev,
				      unsigned char b_ModulNbr, unsigned char b_TimerNbr,
				      unsigned char *pb_TimerStatus)
{
	int i_ReturnValue = 0;
	unsigned int dw_TimerStatus;

	/* Test the module number */
	if (b_ModulNbr < 4) {
		/* Test if 82X54 timer */

		if ((devpriv->s_BoardInfos.dw_MolduleConfiguration[b_ModulNbr] & 0xFFFF0000UL) == APCI1710_82X54_TIMER) {
			/* Test the timer number */
			if (b_TimerNbr <= 2) {
				/* Test if timer initialised */
				if (devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].b_82X54Init == 1) {
					/* Latch the timer value */
					outl((2 << b_TimerNbr) | 0xE0, devpriv->s_BoardInfos.ui_Address + 12 + (64 * b_ModulNbr));

					/* Read the timer status */
					dw_TimerStatus = inl(devpriv->s_BoardInfos.ui_Address + 16 + (b_TimerNbr * 4) + (64 * b_ModulNbr));

					*pb_TimerStatus = (unsigned char) ((dw_TimerStatus) >> 8) & 1;
					printk("ProgressStatus : %d", *pb_TimerStatus);
				} else {
					/* Timer not initialised see function */
					i_ReturnValue = -5;
				}
			} else {
				/* Timer selection wrong */
				i_ReturnValue = -3;
			}	/*  if ((b_TimerNbr >= 0) && (b_TimerNbr <= 2)) */
		} else {
			/* The module is not a TIMER module */

			i_ReturnValue = -4;
		}
	} else {
		/* Module number error */

		i_ReturnValue = -2;
	}

	return i_ReturnValue;
}


int i_APCI1710_WriteTimerValue(struct comedi_device *dev,
			       unsigned char b_ModulNbr, unsigned char b_TimerNbr,
			       unsigned int ul_WriteValue)
{
	int i_ReturnValue = 0;

	/* Test the module number */
	if (b_ModulNbr < 4) {
		/* Test if 82X54 timer */
		if ((devpriv->s_BoardInfos.dw_MolduleConfiguration[b_ModulNbr] & 0xFFFF0000UL) == APCI1710_82X54_TIMER) {
			/* Test the timer number */
			if (b_TimerNbr <= 2) {
				/* Test if timer initialised */
				if (devpriv->s_ModuleInfo[b_ModulNbr].s_82X54ModuleInfo.s_82X54TimerInfo[b_TimerNbr].b_82X54Init == 1) {
					/* Write the value */
					outl(ul_WriteValue, devpriv->s_BoardInfos.ui_Address + (b_TimerNbr * 4) + (64 * b_ModulNbr));
				} else {
					/* Timer not initialised see function */
					DPRINTK("Timer not initialised see function\n");
					i_ReturnValue = -5;
				}
			} else {
				/* Timer selection wrong */
				DPRINTK("Timer selection wrong\n");
				i_ReturnValue = -3;
			}	/*  if ((b_TimerNbr >= 0) && (b_TimerNbr <= 2)) */
		} else {
			/* The module is not a TIMER module */
			DPRINTK("The module is not a TIMER module\n");
			i_ReturnValue = -4;
		}
	} else {
		/* Module number error */
		DPRINTK("Module number error\n");
		i_ReturnValue = -2;
	}

	return i_ReturnValue;
}
