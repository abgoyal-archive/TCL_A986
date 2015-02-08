#ifndef PROPNAME_H
#define PROPNAME_H

#include "unicode/utypes.h"
#include "unicode/uchar.h"
#include "udataswp.h"
#include "uprops.h"


/* low-level char * property name comparison -------------------------------- */

U_CDECL_BEGIN


U_CAPI int32_t U_EXPORT2
uprv_compareASCIIPropertyNames(const char *name1, const char *name2);

U_CAPI int32_t U_EXPORT2
uprv_compareEBCDICPropertyNames(const char *name1, const char *name2);

#if U_CHARSET_FAMILY==U_ASCII_FAMILY
#   define uprv_comparePropertyNames uprv_compareASCIIPropertyNames
#elif U_CHARSET_FAMILY==U_EBCDIC_FAMILY
#   define uprv_comparePropertyNames uprv_compareEBCDICPropertyNames
#else
#   error U_CHARSET_FAMILY is not valid
#endif

U_CDECL_END

/* UDataMemory structure and signatures ------------------------------------- */

#define PNAME_DATA_NAME "pnames"
#define PNAME_DATA_TYPE "icu"

/* Fields in UDataInfo: */

/* PNAME_SIG[] is encoded as numeric literals for compatibility with the HP compiler */
#define PNAME_SIG_0 ((uint8_t)0x70) /* p */
#define PNAME_SIG_1 ((uint8_t)0x6E) /* n */
#define PNAME_SIG_2 ((uint8_t)0x61) /* a */
#define PNAME_SIG_3 ((uint8_t)0x6D) /* m */

#define PNAME_FORMAT_VERSION ((int8_t)1) /* formatVersion[0] */

U_CAPI int32_t U_EXPORT2
upname_swap(const UDataSwapper *ds,
            const void *inData, int32_t length, void *outData,
            UErrorCode *pErrorCode);


#ifdef XP_CPLUSPLUS

class Builder;

U_NAMESPACE_BEGIN

typedef int16_t Offset; /*  must be signed */

#define MAX_OFFSET 0x7FFF

typedef int32_t EnumValue;

/* ---------------------------------------------------------------------- */
/*  ValueMap */

struct ValueMap {

    /*  -- begin pnames data -- */
    /*  Enum=>name EnumToOffset / NonContiguousEnumToOffset objects. */
    /*  Exactly one of these will be nonzero. */
    Offset enumToName_offset;
    Offset ncEnumToName_offset;

    Offset nameToEnum_offset; /*  Name=>enum data */
    /*  -- end pnames data -- */
};

/* ---------------------------------------------------------------------- */
/*  PropertyAliases class */

class PropertyAliases {

    /*  -- begin pnames data -- */
    /*  Enum=>name EnumToOffset object for binary and enumerated */
    /*  properties */
    Offset enumToName_offset;

    /*  Name=>enum data for binary & enumerated properties */
    Offset nameToEnum_offset;

    /*  Enum=>offset EnumToOffset object mapping enumerated properties */
    /*  to ValueMap objects */
    Offset enumToValue_offset;

    /*  The following are needed by external readers of this data. */
    /*  We don't use them ourselves. */
    int16_t total_size; /*  size in bytes excluding the udata header */
    Offset valueMap_offset; /*  offset to start of array */
    int16_t valueMap_count; /*  number of entries */
    Offset nameGroupPool_offset; /*  offset to start of array */
    int16_t nameGroupPool_count; /*  number of entries (not groups) */
    Offset stringPool_offset; /*  offset to start of pool */
    int16_t stringPool_count; /*  number of strings (not size in bytes) */

    /*  -- end pnames data -- */

    friend class ::Builder;

    const ValueMap* getValueMap(EnumValue prop) const;

    const char* chooseNameInGroup(Offset offset,
                                  UPropertyNameChoice choice) const;

 public:

    inline const int8_t* getPointer(Offset o) const {
        return ((const int8_t*) this) + o;
    }

    inline const int8_t* getPointerNull(Offset o) const {
        return o ? getPointer(o) : NULL;
    }

    inline const char* getPropertyName(EnumValue prop,
                                       UPropertyNameChoice choice) const;
    
    inline EnumValue getPropertyEnum(const char* alias) const;

    inline const char* getPropertyValueName(EnumValue prop, EnumValue value,
                                            UPropertyNameChoice choice) const;
    
    inline EnumValue getPropertyValueEnum(EnumValue prop,
                                          const char* alias) const;

    static int32_t
    swap(const UDataSwapper *ds,
         const uint8_t *inBytes, int32_t length, uint8_t *outBytes,
         UErrorCode *pErrorCode);
};

/* ---------------------------------------------------------------------- */
/*  EnumToOffset */

class EnumToOffset {

    /*  -- begin pnames data -- */
    EnumValue enumStart;
    EnumValue enumLimit;
    Offset _offsetArray; /*  [array of enumLimit-enumStart] */
    /*  -- end pnames data -- */

    friend class ::Builder;

    Offset* getOffsetArray() {
        return &_offsetArray;
    }

    const Offset* getOffsetArray() const {
        return &_offsetArray;
    }

    static int32_t getSize(int32_t n) {
        return sizeof(EnumToOffset) + sizeof(Offset) * (n - 1);
    }

    int32_t getSize() {
        return getSize(enumLimit - enumStart);
    }

 public:

    Offset getOffset(EnumValue enumProbe) const {
        if (enumProbe < enumStart ||
            enumProbe >= enumLimit) {
            return 0; /*  not found */
        }
        const Offset* p = getOffsetArray();
        return p[enumProbe - enumStart];
    }

    static int32_t
    swap(const UDataSwapper *ds,
         const uint8_t *inBytes, int32_t length, uint8_t *outBytes,
         uint8_t *temp, int32_t pos,
         UErrorCode *pErrorCode);
};

/* ---------------------------------------------------------------------- */
/*  NonContiguousEnumToOffset */

class NonContiguousEnumToOffset {

    /*  -- begin pnames data -- */
    int32_t count;
    EnumValue _enumArray; /*  [array of count] */
    /*  Offset _offsetArray; // [array of count] after enumValue[count-1] */
    /*  -- end pnames data -- */

    friend class ::Builder;

    EnumValue* getEnumArray() {
        return &_enumArray;
    }

    const EnumValue* getEnumArray() const {
        return &_enumArray;
    }
    
    Offset* getOffsetArray() {
        return (Offset*) (getEnumArray() + count);
    }

    const Offset* getOffsetArray() const {
        return (Offset*) (getEnumArray() + count);
    }

    static int32_t getSize(int32_t n) {
        return sizeof(int32_t) + (sizeof(EnumValue) + sizeof(Offset)) * n;
    }

    int32_t getSize() {
        return getSize(count);
    }

 public:

    Offset getOffset(EnumValue enumProbe) const {
        const EnumValue* e = getEnumArray();
        const Offset* p = getOffsetArray();
        /*  linear search; binary later if warranted */
        /*  (binary is not faster for short lists) */
        for (int32_t i=0; i<count; ++i) {
            if (e[i] < enumProbe) continue;
            if (e[i] > enumProbe) break;
            return p[i];
        }
        return 0; /*  not found */
    }

    static int32_t
    swap(const UDataSwapper *ds,
         const uint8_t *inBytes, int32_t length, uint8_t *outBytes,
         uint8_t *temp, int32_t pos,
         UErrorCode *pErrorCode);
};

/* ---------------------------------------------------------------------- */
/*  NameToEnum */

class NameToEnum {

    /*  -- begin pnames data -- */
    int32_t count;       /*  number of entries */
    EnumValue _enumArray; /*  [array of count] EnumValues */
    /*  Offset _nameArray; // [array of count] offsets to names */
    /*  -- end pnames data -- */

    friend class ::Builder;

    EnumValue* getEnumArray() {
        return &_enumArray;
    }

    const EnumValue* getEnumArray() const {
        return &_enumArray;
    }

    Offset* getNameArray() {
        return (Offset*) (getEnumArray() + count);
    }

    const Offset* getNameArray() const {
        return (Offset*) (getEnumArray() + count);
    }

    static int32_t getSize(int32_t n) {
        return sizeof(int32_t) + (sizeof(Offset) + sizeof(EnumValue)) * n;
    }

    int32_t getSize() {
        return getSize(count);
    }

 public:
  
    EnumValue getEnum(const char* alias, const PropertyAliases& data) const {

        const Offset* n = getNameArray();
        const EnumValue* e = getEnumArray();

        /*  linear search; binary later if warranted */
        /*  (binary is not faster for short lists) */
        for (int32_t i=0; i<count; ++i) {
            const char* name = (const char*) data.getPointer(n[i]);
            int32_t c = uprv_comparePropertyNames(alias, name);
            if (c > 0) continue;
            if (c < 0) break;
            return e[i];
        }
        
        return UCHAR_INVALID_CODE;
    }

    static int32_t
    swap(const UDataSwapper *ds,
         const uint8_t *inBytes, int32_t length, uint8_t *outBytes,
         uint8_t *temp, int32_t pos,
         UErrorCode *pErrorCode);
};

U_NAMESPACE_END

#endif /* C++ */

#endif
