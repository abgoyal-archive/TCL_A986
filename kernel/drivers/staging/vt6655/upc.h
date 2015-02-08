

#ifndef __UPC_H__
#define __UPC_H__

#include "device.h"
#include "ttype.h"

/*---------------------  Export Definitions -------------------------*/


//
//  For IO mapped
//

#ifdef IO_MAP

#define VNSvInPortB(dwIOAddress, pbyData) {                     \
	*(pbyData) = inb(dwIOAddress);                              \
}


#define VNSvInPortW(dwIOAddress, pwData) {                      \
	    *(pwData) = inw(dwIOAddress);                           \
}

#define VNSvInPortD(dwIOAddress, pdwData) {                     \
	    *(pdwData) = inl(dwIOAddress);                          \
}


#define VNSvOutPortB(dwIOAddress, byData) {                     \
        outb(byData, dwIOAddress);                              \
}


#define VNSvOutPortW(dwIOAddress, wData) {                      \
        outw(wData, dwIOAddress);                               \
}

#define VNSvOutPortD(dwIOAddress, dwData) {                     \
        outl(dwData, dwIOAddress);                              \
}

#else

//
//  For memory mapped IO
//


#define VNSvInPortB(dwIOAddress, pbyData) {                     \
	volatile BYTE* pbyAddr = ((PBYTE)(dwIOAddress));            \
	*(pbyData) = readb(pbyAddr);                           \
}


#define VNSvInPortW(dwIOAddress, pwData) {                      \
	volatile WORD* pwAddr = ((PWORD)(dwIOAddress));             \
	*(pwData) = readw(pwAddr);                             \
}

#define VNSvInPortD(dwIOAddress, pdwData) {                     \
	volatile DWORD* pdwAddr = ((PDWORD)(dwIOAddress));          \
	*(pdwData) = readl(pdwAddr);                           \
}


#define VNSvOutPortB(dwIOAddress, byData) {                     \
    volatile BYTE* pbyAddr = ((PBYTE)(dwIOAddress));            \
    writeb((BYTE)byData, pbyAddr);							\
}


#define VNSvOutPortW(dwIOAddress, wData) {                      \
    volatile WORD* pwAddr = ((PWORD)(dwIOAddress));             \
    writew((WORD)wData, pwAddr);							\
}

#define VNSvOutPortD(dwIOAddress, dwData) {                     \
    volatile DWORD* pdwAddr = ((PDWORD)(dwIOAddress));          \
    writel((DWORD)dwData, pdwAddr);					    \
}

#endif


//
// ALWAYS IO-Mapped IO when in 16-bit/32-bit environment
//
#define PCBvInPortB(dwIOAddress, pbyData) {     \
	    *(pbyData) = inb(dwIOAddress);          \
}

#define PCBvInPortW(dwIOAddress, pwData) {      \
	    *(pwData) = inw(dwIOAddress);           \
}

#define PCBvInPortD(dwIOAddress, pdwData) {     \
	    *(pdwData) = inl(dwIOAddress);          \
}

#define PCBvOutPortB(dwIOAddress, byData) {     \
        outb(byData, dwIOAddress);              \
}

#define PCBvOutPortW(dwIOAddress, wData) {      \
        outw(wData, dwIOAddress);               \
}

#define PCBvOutPortD(dwIOAddress, dwData) {     \
        outl(dwData, dwIOAddress);              \
}


#define PCAvDelayByIO(uDelayUnit) {             \
    BYTE    byData;                             \
    ULONG   ii;                                 \
                                                \
    if (uDelayUnit <= 50) {                     \
        udelay(uDelayUnit);                     \
    }                                           \
    else {                                      \
        for (ii = 0; ii < (uDelayUnit); ii++)   \
		     byData = inb(0x61);				\
    }                                           \
}


/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/




#endif // __UPC_H__

