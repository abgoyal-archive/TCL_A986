

#ifndef _INC_SBE_PROMFORMAT_H_
#define _INC_SBE_PROMFORMAT_H_






#ifdef __cplusplus
extern      "C"
{
#endif


#define STRUCT_OFFSET(type, symbol)  ((long)&(((type *)0)->symbol))

#define PROM_FORMAT_Unk   (-1)
#define PROM_FORMAT_TYPE1   1
#define PROM_FORMAT_TYPE2   2


/****** bit fields  for a type 1 formatted seeprom **************************/
    typedef struct
    {
        char        type;       /* 0x00 */
        char        Id[2];      /* 0x01-0x02 */
        char        SubId[2];   /* 0x03-0x04 */
        char        Serial[6];  /* 0x05-0x0a */
        char        CreateTime[4];      /* 0x0b-0x0e */
        char        HeatRunTime[4];     /* 0x0f-0x12 */
        char        HeatRunIterations[4];       /* 0x13-0x16 */
        char        HeatRunErrors[4];   /* 0x17-0x1a */
        char        Crc32[4];   /* 0x1b-0x1e */
    }           FLD_TYPE1;


/****** bit fields  for a type 2 formatted seeprom **************************/
    typedef struct
    {
        char        type;       /* 0x00 */
        char        length[2];  /* 0x01-0x02 */
        char        Crc32[4];   /* 0x03-0x06 */
        char        Id[2];      /* 0x07-0x08 */
        char        SubId[2];   /* 0x09-0x0a */
        char        Serial[6];  /* 0x0b-0x10 */
        char        CreateTime[4];      /* 0x11-0x14 */
        char        HeatRunTime[4];     /* 0x15-0x18 */
        char        HeatRunIterations[4];       /* 0x19-0x1c */
        char        HeatRunErrors[4];   /* 0x1d-0x20 */
    }           FLD_TYPE2;



/***** this union allows us to access the seeprom as an array of bytes ***/
/***** or as individual fields                                         ***/

#define SBE_EEPROM_SIZE    128
#define SBE_MFG_INFO_SIZE  sizeof(FLD_TYPE2)

    typedef union
    {
        char        bytes[128];
        FLD_TYPE1   fldType1;
        FLD_TYPE2   fldType2;
    }           PROMFORMAT;

#ifdef __cplusplus
}
#endif

#endif                          /*** _INC_SBE_PROMFORMAT_H_ ***/
