
#ifndef ANDROID_AVD_INFO_H
#define ANDROID_AVD_INFO_H

#include "android/utils/ini.h"
#include "android/avd/hw-config.h"


#define  AVD_IMAGE_LIST \
    _AVD_IMG(KERNEL,"kernel-qemu","kernel") \
    _AVD_IMG(RAMDISK,"ramdisk.img","ramdisk") \
    _AVD_IMG(INITSYSTEM,"system.img","init system") \
    _AVD_IMG(INITDATA,"userdata.img","init data") \
    _AVD_IMG(USERSYSTEM,"system-qemu.img","user system") \
    _AVD_IMG(USERDATA,"userdata-qemu.img", "user data") \
    _AVD_IMG(CACHE,"cache.img","cache") \
    _AVD_IMG(SDCARD,"sdcard.img","SD Card") \

#define _AVD_IMG(x,y,z)   AVD_IMAGE_##x ,
typedef enum {
    AVD_IMAGE_LIST
    AVD_IMAGE_MAX /* do not remove */
} AvdImageType;
#undef  _AVD_IMG

typedef struct AvdInfo  AvdInfo;

/* various flags used when creating an AvdInfo object */
typedef enum {
    /* use to force a data wipe */
    AVDINFO_WIPE_DATA = (1 << 0),
    /* use to ignore the cache partition */
    AVDINFO_NO_CACHE  = (1 << 1),
    /* use to wipe cache partition, ignored if NO_CACHE is set */
    AVDINFO_WIPE_CACHE = (1 << 2),
    /* use to ignore ignore SDCard image (default or provided) */
    AVDINFO_NO_SDCARD = (1 << 3),
    /* use to wipe the system image with new initial values */
    AVDINFO_WIPE_SYSTEM = (1 << 4),
} AvdFlags;

typedef struct {
    unsigned     flags;
    const char*  skinName;
    const char*  skinRootPath;
    const char*  forcePaths[AVD_IMAGE_MAX];
} AvdInfoParams;

AvdInfo*  avdInfo_new( const char*  name, AvdInfoParams*  params );

AvdInfo*  avdInfo_newForAndroidBuild( const char*     androidBuildRoot,
                                      const char*     androidOut,
                                      AvdInfoParams*  params );

void        avdInfo_free( AvdInfo*  i );

const char*  avdInfo_getName( AvdInfo*  i );

const char*  avdInfo_getImageFile( AvdInfo*  i, AvdImageType  imageType );

uint64_t     avdInfo_getImageFileSize( AvdInfo*  i, AvdImageType  imageType );

int          avdInfo_isImageReadOnly( AvdInfo*  i, AvdImageType  imageType );

int          avdInfo_lockImageFile( AvdInfo*  i, AvdImageType  imageType, int  abortOnError);

/* Manually set the path of a given image file. */
void         avdInfo_setImageFile( AvdInfo*  i, AvdImageType  imageType, const char*  imagePath );

/* Returns the path of the skin directory */
/* the string belongs to the AvdInfo object */
const char*  avdInfo_getSkinPath( AvdInfo*  i );

/* Returns the name of the virtual device's skin */
const char*  avdInfo_getSkinName( AvdInfo*  i );

/* Returns the root skin directory for this device */
const char*  avdInfo_getSkinDir ( AvdInfo*  i );

/* Returns the content path of the virtual device */
const char*  avdInfo_getContentPath( AvdInfo*  i );

// +MTK03764_2011_03_23
/* Returns the content target of the virtual device */
const char*  avdInfo_getContentTarget( AvdInfo*  i );
// -MTK03764_2011_03_23

/* Returns TRUE iff in the Android build system */
int          avdInfo_inAndroidBuild( AvdInfo*  i );

/* Reads the AVD's hardware configuration into 'hw'. returns -1 on error, 0 otherwise */
int          avdInfo_getHwConfig( AvdInfo*  i, AndroidHwConfig*  hw );

/* Returns a *copy* of the path used to store trace 'foo'. result must be freed by caller */
char*        avdInfo_getTracePath( AvdInfo*  i, const char*  traceName );

/* */

#endif /* ANDROID_AVD_INFO_H */
