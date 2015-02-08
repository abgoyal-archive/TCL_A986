
#include "cstring.h"
#include "ustrfmt.h"


U_CAPI int32_t U_EXPORT2
uprv_itou (UChar * buffer, int32_t capacity,
           uint32_t i, uint32_t radix, int32_t minwidth)
{
    int32_t length = 0;
    int digit;
    int32_t j;
    UChar temp;

    do{
        digit = (int)(i % radix);
        buffer[length++]=(UChar)(digit<=9?(0x0030+digit):(0x0030+digit+7));
        i=i/radix;
    } while(i && length<capacity);

    while (length < minwidth){
        buffer[length++] = (UChar) 0x0030;/*zero padding */
    }
    /* null terminate the buffer */
    if(length<capacity){
        buffer[length] = (UChar) 0x0000;
    }

    /* Reverses the string */
    for (j = 0; j < (length / 2); j++){
        temp = buffer[(length-1) - j];
        buffer[(length-1) - j] = buffer[j];
        buffer[j] = temp;
    }
    return length;
}
