
/***********************************************************************/
/***********************************************************************/
#ifndef _HPIDSPCD_H_
#define _HPIDSPCD_H_

#include "hpi_internal.h"

#ifndef DISABLE_PRAGMA_PACK1
#pragma pack(push, 1)
#endif

/** Descriptor for dspcode from firmware loader */
struct dsp_code {
	/**  Firmware descriptor */
	const struct firmware *ps_firmware;
	struct pci_dev *ps_dev;
	/** Expected number of words in the whole dsp code,INCL header */
	long int block_length;
	/** Number of words read so far */
	long int word_count;
	/** Version read from dsp code file */
	u32 version;
	/** CRC read from dsp code file */
	u32 crc;
};

#ifndef DISABLE_PRAGMA_PACK1
#pragma pack(pop)
#endif

short hpi_dsp_code_open(
	/** Code identifier, usually adapter family */
	u32 adapter,
	/** Pointer to DSP code control structure */
	struct dsp_code *ps_dsp_code,
	/** Pointer to dword to receive OS specific error code */
	u32 *pos_error_code);

/** Close the DSP code file */
void hpi_dsp_code_close(struct dsp_code *ps_dsp_code);

/** Rewind to the beginning of the DSP code file (for verify) */
void hpi_dsp_code_rewind(struct dsp_code *ps_dsp_code);

short hpi_dsp_code_read_word(struct dsp_code *ps_dsp_code,
				      /**< DSP code descriptor */
	u32 *pword /**< where to store the read word */
	);

short hpi_dsp_code_read_block(size_t words_requested,
	struct dsp_code *ps_dsp_code,
	/* Pointer to store (Pointer to code buffer) */
	u32 **ppblock);

#endif
