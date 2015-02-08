


#ifndef __TYPES_H__
#define __TYPES_H__

#include <bfa_os_inc.h>

#define wwn_t u64
#define lun_t u64

#define WWN_NULL	(0)
#define FC_SYMNAME_MAX	256	/*  max name server symbolic name size */
#define FC_ALPA_MAX	128

#pragma pack(1)

#define MAC_ADDRLEN	(6)
struct mac_s { u8 mac[MAC_ADDRLEN]; };
#define mac_t struct mac_s

#pragma pack()

#endif
