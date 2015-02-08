

#ifndef __MICHAEL_H__
#define __MICHAEL_H__

/*---------------------  Export Definitions -------------------------*/

/*---------------------  Export Types  ------------------------------*/

void MIC_vInit(DWORD dwK0, DWORD dwK1);

void MIC_vUnInit(void);

// Append bytes to the message to be MICed
void MIC_vAppend(PBYTE src, unsigned int nBytes);

// Get the MIC result. Destination should accept 8 bytes of result.
// This also resets the message to empty.
void MIC_vGetMIC(PDWORD pdwL, PDWORD pdwR);

/*---------------------  Export Macros ------------------------------*/

// Rotation functions on 32 bit values
#define ROL32( A, n ) \
 ( ((A) << (n)) | ( ((A)>>(32-(n)))  & ( (1UL << (n)) - 1 ) ) )
#define ROR32( A, n ) ROL32( (A), 32-(n) )

#endif /* __MICHAEL_H__ */
