

#include <linux/kernel.h>
#include "spu.h"

/* This file holds the Spu opcode table */



const struct spu_opcode spu_opcodes[] = {
#define APUOP(TAG,MACFORMAT,OPCODE,MNEMONIC,ASMFORMAT,DEP,PIPE) \
	{ MACFORMAT, OPCODE, MNEMONIC, ASMFORMAT },
#define APUOPFB(TAG,MACFORMAT,OPCODE,FB,MNEMONIC,ASMFORMAT,DEP,PIPE) \
	{ MACFORMAT, OPCODE, MNEMONIC, ASMFORMAT },
#include "spu-insns.h"
#undef APUOP
#undef APUOPFB
};

const int spu_num_opcodes = ARRAY_SIZE(spu_opcodes);
