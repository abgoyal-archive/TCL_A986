
/*	BSDI sys_info.h,v 1.6 1998/06/03 19:14:59 karels Exp	*/


#ifndef         __SYS_INFO_H
#define         __SYS_INFO_H



/*Include Files ------------------------------------------------------------- */

#include        "osd_util.h"

#ifndef NO_PACK
#if defined (_DPT_AIX)
#pragma options align=packed
#else
#pragma pack(1)
#endif  /* aix */
#endif  // no unpack



#ifdef  __cplusplus
   struct driveParam_S {
#else
   typedef struct  {
#endif

   uSHORT       cylinders;      /* Upto 1024 */
   uCHAR        heads;          /* Upto 255 */
   uCHAR        sectors;        /* Upto 63 */

#ifdef  __cplusplus

//---------- Portability Additions ----------- in sp_sinfo.cpp
#ifdef DPT_PORTABLE
	uSHORT		netInsert(dptBuffer_S *buffer);
	uSHORT		netExtract(dptBuffer_S *buffer);
#endif // DPT PORTABLE
//--------------------------------------------

   };
#else
   } driveParam_S;
#endif
/*driveParam_S - end */



/*flags - bit definitions */
#define SI_CMOS_Valid           0x0001
#define SI_NumDrivesValid       0x0002
#define SI_ProcessorValid       0x0004
#define SI_MemorySizeValid      0x0008
#define SI_DriveParamsValid     0x0010
#define SI_SmartROMverValid     0x0020
#define SI_OSversionValid       0x0040
#define SI_OSspecificValid      0x0080  /* 1 if OS structure returned */
#define SI_BusTypeValid         0x0100

#define SI_ALL_VALID            0x0FFF  /* All Std SysInfo is valid */
#define SI_NO_SmartROM          0x8000

/*busType - definitions */
#define SI_ISA_BUS      0x00
#define SI_MCA_BUS      0x01
#define SI_EISA_BUS     0x02
#define SI_PCI_BUS      0x04

#ifdef  __cplusplus
   struct sysInfo_S {
#else
   typedef struct  {
#endif

   uCHAR        drive0CMOS;             /* CMOS Drive 0 Type */
   uCHAR        drive1CMOS;             /* CMOS Drive 1 Type */
   uCHAR        numDrives;              /* 0040:0075 contents */
   uCHAR        processorFamily;        /* Same as DPTSIG's definition */
   uCHAR        processorType;          /* Same as DPTSIG's definition */
   uCHAR        smartROMMajorVersion;
   uCHAR        smartROMMinorVersion;   /* SmartROM version */
   uCHAR        smartROMRevision;
   uSHORT       flags;                  /* See bit definitions above */
   uSHORT       conventionalMemSize;    /* in KB */
   uINT         extendedMemSize;        /* in KB */
   uINT         osType;                 /* Same as DPTSIG's definition */
   uCHAR        osMajorVersion;
   uCHAR        osMinorVersion;         /* The OS version */
   uCHAR        osRevision;
#ifdef _SINIX_ADDON
   uCHAR        busType;                /* See defininitions above */
   uSHORT       osSubRevision;
   uCHAR        pad[2];                 /* For alignment */
#else
   uCHAR        osSubRevision;
   uCHAR        busType;                /* See defininitions above */
   uCHAR        pad[3];                 /* For alignment */
#endif
   driveParam_S drives[16];             /* SmartROM Logical Drives */

#ifdef  __cplusplus

//---------- Portability Additions ----------- in sp_sinfo.cpp
#ifdef DPT_PORTABLE
	uSHORT		netInsert(dptBuffer_S *buffer);
	uSHORT		netExtract(dptBuffer_S *buffer);
#endif // DPT PORTABLE
//--------------------------------------------

   };
#else
   } sysInfo_S;
#endif
/*sysInfo_S - end */



/*flags - bit definitions */
#define DI_DOS_HIGH             0x01    /* DOS is loaded high */
#define DI_DPMI_VALID           0x02    /* DPMI version is valid */

#ifdef  __cplusplus
   struct DOS_Info_S {
#else
   typedef struct {
#endif

   uCHAR        flags;          /* See bit definitions above */
   uSHORT       driverLocation; /* SmartROM BIOS address */
   uSHORT       DOS_version;
   uSHORT       DPMI_version;

#ifdef  __cplusplus

//---------- Portability Additions ----------- in sp_sinfo.cpp
#ifdef DPT_PORTABLE
	uSHORT		netInsert(dptBuffer_S *buffer);
	uSHORT		netExtract(dptBuffer_S *buffer);
#endif // DPT PORTABLE
//--------------------------------------------

   };
#else
   } DOS_Info_S;
#endif
/*DOS_Info_S - end */



#ifdef  __cplusplus
   struct Netware_Info_S {
#else
   typedef struct {
#endif

   uCHAR        driverName[13];         /* ie PM12NW31.DSK */
   uCHAR        serverName[48];
   uCHAR        netwareVersion;         /* The Netware OS version */
   uCHAR        netwareSubVersion;
   uCHAR        netwareRevision;
   uSHORT       maxConnections;         /* Probably  250 or 1000 */
   uSHORT       connectionsInUse;
   uSHORT       maxVolumes;
   uCHAR        unused;
   uCHAR        SFTlevel;
   uCHAR        TTSlevel;

   uCHAR        clibMajorVersion;       /* The CLIB.NLM version */
   uCHAR        clibMinorVersion;
   uCHAR        clibRevision;

#ifdef  __cplusplus

//---------- Portability Additions ----------- in sp_sinfo.cpp
#ifdef DPT_PORTABLE
	uSHORT		netInsert(dptBuffer_S *buffer);
	uSHORT		netExtract(dptBuffer_S *buffer);
#endif // DPT PORTABLE
//--------------------------------------------

   };
#else
   } Netware_Info_S;
#endif
/*Netware_Info_S - end */



#ifdef  __cplusplus
   struct OS2_Info_S {
#else
   typedef struct {
#endif

   uCHAR        something;

#ifdef  __cplusplus

//---------- Portability Additions ----------- in sp_sinfo.cpp
#ifdef DPT_PORTABLE
	uSHORT		netInsert(dptBuffer_S *buffer);
	uSHORT		netExtract(dptBuffer_S *buffer);
#endif // DPT PORTABLE
//--------------------------------------------

   };
#else
   } OS2_Info_S;
#endif
/*OS2_Info_S - end */



#ifdef  __cplusplus
   struct WinNT_Info_S {
#else
   typedef struct {
#endif

   uCHAR        something;

#ifdef  __cplusplus

//---------- Portability Additions ----------- in sp_sinfo.cpp
#ifdef DPT_PORTABLE
	uSHORT		netInsert(dptBuffer_S *buffer);
	uSHORT		netExtract(dptBuffer_S *buffer);
#endif // DPT PORTABLE
//--------------------------------------------

   };
#else
   } WinNT_Info_S;
#endif
/*WinNT_Info_S - end */



#ifdef  __cplusplus
   struct SCO_Info_S {
#else
   typedef struct {
#endif

   uCHAR        something;

#ifdef  __cplusplus

//---------- Portability Additions ----------- in sp_sinfo.cpp
#ifdef DPT_PORTABLE
	uSHORT		netInsert(dptBuffer_S *buffer);
	uSHORT		netExtract(dptBuffer_S *buffer);
#endif // DPT PORTABLE
//--------------------------------------------

   };
#else
   } SCO_Info_S;
#endif
/*SCO_Info_S - end */



#ifdef  __cplusplus
   struct USL_Info_S {
#else
   typedef struct {
#endif

   uCHAR        something;

#ifdef  __cplusplus

//---------- Portability Additions ----------- in sp_sinfo.cpp
#ifdef DPT_PORTABLE
	uSHORT		netInsert(dptBuffer_S *buffer);
	uSHORT		netExtract(dptBuffer_S *buffer);
#endif // DPT PORTABLE
//--------------------------------------------

   };
#else
   } USL_Info_S;
#endif
/*USL_Info_S - end */


  /* Restore default structure packing */
#ifndef NO_UNPACK
#if defined (_DPT_AIX)
#pragma options align=reset
#elif defined (UNPACK_FOUR)
#pragma pack(4)
#else
#pragma pack()
#endif  /* aix */
#endif  // no unpack

#endif  // __SYS_INFO_H

