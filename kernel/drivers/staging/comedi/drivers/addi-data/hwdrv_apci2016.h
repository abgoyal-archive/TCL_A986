
/*********      Definitions for APCI-2016 card  *****/

#define APCI2016_BOARD_VENDOR_ID 0x15B8
#define APCI2016_ADDRESS_RANGE   8

/* DIGITAL INPUT-OUTPUT DEFINE */

#define APCI2016_DIGITAL_OP                 	0x04
#define APCI2016_DIGITAL_OP_RW                 	4

/* ADDIDATA Enable Disable */

#define ADDIDATA_ENABLE                            1
#define ADDIDATA_DISABLE                           0

/* TIMER COUNTER WATCHDOG DEFINES */

#define ADDIDATA_WATCHDOG                          2
#define APCI2016_DIGITAL_OP_WATCHDOG               0
#define APCI2016_WATCHDOG_ENABLEDISABLE            12
#define APCI2016_WATCHDOG_RELOAD_VALUE             4
#define APCI2016_WATCHDOG_STATUS                   16

/* Hardware Layer  functions for Apci2016 */

/* DO */
int i_APCI2016_ConfigDigitalOutput(struct comedi_device *dev, struct comedi_subdevice *s,
				   struct comedi_insn *insn, unsigned int *data);

int i_APCI2016_WriteDigitalOutput(struct comedi_device *dev, struct comedi_subdevice *s,
				  struct comedi_insn *insn, unsigned int *data);

int i_APCI2016_BitsDigitalOutput(struct comedi_device *dev, struct comedi_subdevice *s,
				 struct comedi_insn *insn, unsigned int *data);


int i_APCI2016_ConfigWatchdog(struct comedi_device *dev, struct comedi_subdevice *s,
			      struct comedi_insn *insn, unsigned int *data);

int i_APCI2016_StartStopWriteWatchdog(struct comedi_device *dev, struct comedi_subdevice *s,
				      struct comedi_insn *insn, unsigned int *data);

int i_APCI2016_ReadWatchdog(struct comedi_device *dev, struct comedi_subdevice *s,
			    struct comedi_insn *insn, unsigned int *data);

/* Interrupt functions..... */

/* void v_APCI2016_Interrupt(int irq, void *d); */

/* void v_APCI2016_Interrupt(int irq, void *d); */
/* RESET */
int i_APCI2016_Reset(struct comedi_device *dev);
