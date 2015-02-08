

#ifndef COMEDI_SUBD_TTLIO
#define COMEDI_SUBD_TTLIO   11	/* Digital Input Output But TTL */
#endif

#ifndef ADDIDATA_ENABLE
#define ADDIDATA_ENABLE  1
#define ADDIDATA_DISABLE 0
#endif

#define APCI16XX_TTL_INIT           0
#define APCI16XX_TTL_INITDIRECTION  1
#define APCI16XX_TTL_OUTPUTMEMORY   2

#define APCI16XX_TTL_READCHANNEL            0
#define APCI16XX_TTL_READPORT               1

#define APCI16XX_TTL_WRITECHANNEL_ON        0
#define APCI16XX_TTL_WRITECHANNEL_OFF       1
#define APCI16XX_TTL_WRITEPORT_ON           2
#define APCI16XX_TTL_WRITEPORT_OFF          3

#define APCI16XX_TTL_READ_ALL_INPUTS        0
#define APCI16XX_TTL_READ_ALL_OUTPUTS       1

#ifdef __KERNEL__

static const struct comedi_lrange range_apci16xx_ttl = { 12,
	{BIP_RANGE(1),
	 BIP_RANGE(1),
	 BIP_RANGE(1),
	 BIP_RANGE(1),
	 BIP_RANGE(1),
	 BIP_RANGE(1),
	 BIP_RANGE(1),
	 BIP_RANGE(1),
	 BIP_RANGE(1),
	 BIP_RANGE(1),
	 BIP_RANGE(1),
	 BIP_RANGE(1)}
};


int i_APCI16XX_InsnConfigInitTTLIO(struct comedi_device *dev,
				   struct comedi_subdevice *s, struct comedi_insn *insn,
				   unsigned int *data);


int i_APCI16XX_InsnBitsReadTTLIO(struct comedi_device *dev,
				 struct comedi_subdevice *s, struct comedi_insn *insn,
				 unsigned int *data);

int i_APCI16XX_InsnReadTTLIOAllPortValue(struct comedi_device *dev,
					 struct comedi_subdevice *s,
					 struct comedi_insn *insn, unsigned int *data);


int i_APCI16XX_InsnBitsWriteTTLIO(struct comedi_device *dev,
				  struct comedi_subdevice *s, struct comedi_insn *insn,
				  unsigned int *data);

int i_APCI16XX_Reset(struct comedi_device *dev);
#endif
