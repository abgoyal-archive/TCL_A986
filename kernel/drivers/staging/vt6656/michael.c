

#include "tmacro.h"
#include "michael.h"

/*---------------------  Static Definitions -------------------------*/

/*---------------------  Static Variables  --------------------------*/

/*---------------------  Static Functions  --------------------------*/
static void s_vClear(void);		/* Clear the internal message,
					 * resets the object to the
					 * state just after construction. */
static void s_vSetKey(DWORD dwK0, DWORD dwK1);
static void s_vAppendByte(BYTE b);	/* Add a single byte to the internal
					 * message */

/*---------------------  Export Variables  --------------------------*/
static DWORD  L, R;		/* Current state */
static DWORD  K0, K1;		/* Key */
static DWORD  M;		/* Message accumulator (single word) */
static unsigned int   nBytesInM;	/* # bytes in M */

/*---------------------  Export Functions  --------------------------*/


static void s_vClear(void)
{
	/* Reset the state to the empty message. */
	L = K0;
	R = K1;
	nBytesInM = 0;
	M = 0;
}

static void s_vSetKey(DWORD dwK0, DWORD dwK1)
{
	/* Set the key */
	K0 = dwK0;
	K1 = dwK1;
	/* and reset the message */
	s_vClear();
}

static void s_vAppendByte(BYTE b)
{
	/* Append the byte to our word-sized buffer */
	M |= b << (8*nBytesInM);
	nBytesInM++;
	/* Process the word if it is full. */
	if (nBytesInM >= 4) {
		L ^= M;
		R ^= ROL32(L, 17);
		L += R;
		R ^= ((L & 0xff00ff00) >> 8) | ((L & 0x00ff00ff) << 8);
		L += R;
		R ^= ROL32(L, 3);
		L += R;
		R ^= ROR32(L, 2);
		L += R;
		/* Clear the buffer */
		M = 0;
		nBytesInM = 0;
	}
}

void MIC_vInit(DWORD dwK0, DWORD dwK1)
{
	/* Set the key */
	s_vSetKey(dwK0, dwK1);
}


void MIC_vUnInit(void)
{
	/* Wipe the key material */
	K0 = 0;
	K1 = 0;

	/* And the other fields as well. */
	/* Note that this sets (L,R) to (K0,K1) which is just fine. */
	s_vClear();
}

void MIC_vAppend(PBYTE src, unsigned int nBytes)
{
    /* This is simple */
	while (nBytes > 0) {
		s_vAppendByte(*src++);
		nBytes--;
	}
}

void MIC_vGetMIC(PDWORD pdwL, PDWORD pdwR)
{
	/* Append the minimum padding */
	s_vAppendByte(0x5a);
	s_vAppendByte(0);
	s_vAppendByte(0);
	s_vAppendByte(0);
	s_vAppendByte(0);
	/* and then zeroes until the length is a multiple of 4 */
	while (nBytesInM != 0)
		s_vAppendByte(0);
	/* The s_vAppendByte function has already computed the result. */
	*pdwL = L;
	*pdwR = R;
	/* Reset to the empty message. */
	s_vClear();
}
