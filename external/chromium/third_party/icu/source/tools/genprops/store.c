
#include <stdio.h>
#include "unicode/utypes.h"
#include "unicode/uchar.h"
#include "cmemory.h"
#include "cstring.h"
#include "utrie.h"
#include "unicode/udata.h"
#include "unewdata.h"
#include "writesrc.h"
#include "uprops.h"
#include "genprops.h"

#define DO_DEBUG_OUT 0


/* UDataInfo cf. udata.h */
static UDataInfo dataInfo={
    sizeof(UDataInfo),
    0,

    U_IS_BIG_ENDIAN,
    U_CHARSET_FAMILY,
    U_SIZEOF_UCHAR,
    0,

    { 0x55, 0x50, 0x72, 0x6f },                 /* dataFormat="UPro" */
    { 5, 0, UTRIE_SHIFT, UTRIE_INDEX_SHIFT },   /* formatVersion */
    { 5, 1, 0, 0 }                              /* dataVersion */
};

static UNewTrie *pTrie=NULL;

/* -------------------------------------------------------------------------- */

extern void
setUnicodeVersion(const char *v) {
    UVersionInfo version;
    u_versionFromString(version, v);
    uprv_memcpy(dataInfo.dataVersion, version, 4);
}

extern void
initStore() {
    pTrie=utrie_open(NULL, NULL, 40000, 0, 0, TRUE);
    if(pTrie==NULL) {
        fprintf(stderr, "error: unable to create a UNewTrie\n");
        exit(U_MEMORY_ALLOCATION_ERROR);
    }

    initAdditionalProperties();
}

extern void
exitStore() {
    utrie_close(pTrie);
    exitAdditionalProperties();
}

static uint32_t printNumericTypeValueError(Props *p) {
    fprintf(stderr, "genprops error: unable to encode numeric type & value %d  %ld/%lu E%d\n",
            (int)p->numericType, (long)p->numericValue, (unsigned long)p->denominator, p->exponent);
    exit(U_ILLEGAL_ARGUMENT_ERROR);
    return 0;
}

/* store a character's properties ------------------------------------------- */

extern uint32_t
makeProps(Props *p) {
    uint32_t den;
    int32_t type, value, exp;

    /* encode numeric type & value */
    type=p->numericType;
    value=p->numericValue;
    den=p->denominator;
    exp=p->exponent;

    if(den!=0) {
        /* fraction */
        if( type!=U_NT_NUMERIC ||
            value<-1 || value==0 || value>UPROPS_FRACTION_MAX_NUM ||
            den<UPROPS_FRACTION_MIN_DEN || UPROPS_FRACTION_MAX_DEN<den ||
            exp!=0
        ) {
            return printNumericTypeValueError(p);
        }
        type=UPROPS_NT_FRACTION;

        if(value==-1) {
            value=0;
        }
        den-=UPROPS_FRACTION_DEN_OFFSET;
        value=(value<<UPROPS_FRACTION_NUM_SHIFT)|den;
    } else if(exp!=0) {
        /* very large value */
        if( type!=U_NT_NUMERIC ||
            value<1 || 9<value ||
            exp<UPROPS_LARGE_MIN_EXP || UPROPS_LARGE_MAX_EXP_EXTRA<exp
        ) {
            return printNumericTypeValueError(p);
        }
        type=UPROPS_NT_LARGE;

        if(exp<=UPROPS_LARGE_MAX_EXP) {
            /* 1..9 * 10^(2..17) */
            exp-=UPROPS_LARGE_EXP_OFFSET;
        } else {
            /* 1 * 10^(18..33) */
            if(value!=1) {
                return printNumericTypeValueError(p);
            }
            value=0;
            exp-=UPROPS_LARGE_EXP_OFFSET_EXTRA;
        }
        value=(value<<UPROPS_LARGE_MANT_SHIFT)|exp;
    } else if(value>UPROPS_MAX_SMALL_NUMBER) {
        /* large value */
        if(type!=U_NT_NUMERIC) {
            return printNumericTypeValueError(p);
        }
        type=UPROPS_NT_LARGE;

        /* split the value into mantissa and exponent, base 10 */
        while((value%10)==0) {
            value/=10;
            ++exp;
        }
        if(value>9) {
            return printNumericTypeValueError(p);
        }

        exp-=UPROPS_LARGE_EXP_OFFSET;
        value=(value<<UPROPS_LARGE_MANT_SHIFT)|exp;
    } else if(value<0) {
        /* unable to encode negative values, other than fractions -1/x */
        return printNumericTypeValueError(p);

    /* } else normal value=0..0xff { */
    }

    /* encode the properties */
    return
        (uint32_t)p->generalCategory |
        ((uint32_t)type<<UPROPS_NUMERIC_TYPE_SHIFT) |
        ((uint32_t)value<<UPROPS_NUMERIC_VALUE_SHIFT);
}

extern void
addProps(uint32_t c, uint32_t x) {
    if(!utrie_set32(pTrie, (UChar32)c, x)) {
        fprintf(stderr, "error: too many entries for the properties trie\n");
        exit(U_BUFFER_OVERFLOW_ERROR);
    }
}

extern uint32_t
getProps(uint32_t c) {
    return utrie_get32(pTrie, (UChar32)c, NULL);
}

/* areas of same properties ------------------------------------------------- */

extern void
repeatProps(uint32_t first, uint32_t last, uint32_t x) {
    if(!utrie_setRange32(pTrie, (UChar32)first, (UChar32)(last+1), x, FALSE)) {
        fprintf(stderr, "error: too many entries for the properties trie\n");
        exit(U_BUFFER_OVERFLOW_ERROR);
    }
}

/* generate output data ----------------------------------------------------- */

extern void
generateData(const char *dataDir, UBool csource) {
    static int32_t indexes[UPROPS_INDEX_COUNT]={
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0
    };
    static uint8_t trieBlock[40000];
    static uint8_t additionalProps[120000];

    UNewDataMemory *pData;
    UErrorCode errorCode=U_ZERO_ERROR;
    uint32_t size = 0;
    int32_t trieSize, additionalPropsSize, offset;
    long dataLength;

    trieSize=utrie_serialize(pTrie, trieBlock, sizeof(trieBlock), NULL, TRUE, &errorCode);
    if(U_FAILURE(errorCode)) {
        fprintf(stderr, "error: utrie_serialize failed: %s (length %ld)\n", u_errorName(errorCode), (long)trieSize);
        exit(errorCode);
    }

    offset=sizeof(indexes)/4;               /* uint32_t offset to the properties trie */

    /* round up trie size to 4-alignment */
    trieSize=(trieSize+3)&~3;
    offset+=trieSize>>2;
    indexes[UPROPS_PROPS32_INDEX]=          /* set indexes to the same offsets for empty */
    indexes[UPROPS_EXCEPTIONS_INDEX]=       /* structures from the old format version 3 */
    indexes[UPROPS_EXCEPTIONS_TOP_INDEX]=   /* so that less runtime code has to be changed */
    indexes[UPROPS_ADDITIONAL_TRIE_INDEX]=offset;

    if(beVerbose) {
        printf("trie size in bytes:                    %5u\n", (int)trieSize);
    }

    if(csource) {
        /* write .c file for hardcoded data */
        UTrie trie={ NULL };
        UTrie2 *trie2;
        FILE *f;

        utrie_unserialize(&trie, trieBlock, trieSize, &errorCode);
        if(U_FAILURE(errorCode)) {
            fprintf(
                stderr,
                "genprops error: failed to utrie_unserialize(uprops.icu main trie) - %s\n",
                u_errorName(errorCode));
            exit(errorCode);
        }

        /* use UTrie2 */
        dataInfo.formatVersion[0]=6;
        dataInfo.formatVersion[2]=0;
        dataInfo.formatVersion[3]=0;
        trie2=utrie2_fromUTrie(&trie, 0, &errorCode);
        if(U_FAILURE(errorCode)) {
            fprintf(
                stderr,
                "genprops error: utrie2_fromUTrie() failed - %s\n",
                u_errorName(errorCode));
            exit(errorCode);
        }
        {
            /* delete lead surrogate code unit values */
            UChar lead;
            trie2=utrie2_cloneAsThawed(trie2, &errorCode);
            for(lead=0xd800; lead<0xdc00; ++lead) {
                utrie2_set32ForLeadSurrogateCodeUnit(trie2, lead, trie2->initialValue, &errorCode);
            }
            utrie2_freeze(trie2, UTRIE2_16_VALUE_BITS, &errorCode);
            if(U_FAILURE(errorCode)) {
                fprintf(
                    stderr,
                    "genprops error: deleting lead surrogate code unit values failed - %s\n",
                    u_errorName(errorCode));
                exit(errorCode);
            }
        }

        f=usrc_create(dataDir, "uchar_props_data.c");
        if(f!=NULL) {
            usrc_writeArray(f,
                "static const UVersionInfo formatVersion={",
                dataInfo.formatVersion, 8, 4,
                "};\n\n");
            usrc_writeArray(f,
                "static const UVersionInfo dataVersion={",
                dataInfo.dataVersion, 8, 4,
                "};\n\n");
            usrc_writeUTrie2Arrays(f,
                "static const uint16_t propsTrie_index[%ld]={\n", NULL,
                trie2,
                "\n};\n\n");
            usrc_writeUTrie2Struct(f,
                "static const UTrie2 propsTrie={\n",
                trie2, "propsTrie_index", NULL,
                "};\n\n");

            additionalPropsSize=writeAdditionalData(f, additionalProps, sizeof(additionalProps), indexes);
            size=4*offset+additionalPropsSize;      /* total size of data */

            usrc_writeArray(f,
                "static const int32_t indexes[UPROPS_INDEX_COUNT]={",
                indexes, 32, UPROPS_INDEX_COUNT,
                "};\n\n");
            fclose(f);
        }
        utrie2_close(trie2);
    } else {
        /* write the data */
        pData=udata_create(dataDir, DATA_TYPE, DATA_NAME, &dataInfo,
                        haveCopyright ? U_COPYRIGHT_STRING : NULL, &errorCode);
        if(U_FAILURE(errorCode)) {
            fprintf(stderr, "genprops: unable to create data memory, %s\n", u_errorName(errorCode));
            exit(errorCode);
        }

        additionalPropsSize=writeAdditionalData(NULL, additionalProps, sizeof(additionalProps), indexes);
        size=4*offset+additionalPropsSize;      /* total size of data */

        udata_writeBlock(pData, indexes, sizeof(indexes));
        udata_writeBlock(pData, trieBlock, trieSize);
        udata_writeBlock(pData, additionalProps, additionalPropsSize);

        /* finish up */
        dataLength=udata_finish(pData, &errorCode);
        if(U_FAILURE(errorCode)) {
            fprintf(stderr, "genprops: error %d writing the output file\n", errorCode);
            exit(errorCode);
        }

        if(dataLength!=(long)size) {
            fprintf(stderr, "genprops: data length %ld != calculated size %lu\n",
                dataLength, (unsigned long)size);
            exit(U_INTERNAL_PROGRAM_ERROR);
        }
    }

    if(beVerbose) {
        printf("data size:                            %6lu\n", (unsigned long)size);
    }
}

