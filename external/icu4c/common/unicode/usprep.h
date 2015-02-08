
#ifndef __USPREP_H__
#define __USPREP_H__


#include "unicode/utypes.h"
#include "unicode/localpointer.h"

#if !UCONFIG_NO_IDNA

#include "unicode/parseerr.h"

typedef struct UStringPrepProfile UStringPrepProfile;


#define USPREP_DEFAULT 0x0000

#define USPREP_ALLOW_UNASSIGNED 0x0001

typedef enum UStringPrepProfileType {
    /**
     * RFC3491 Nameprep
     * @stable ICU 4.2
     */
    USPREP_RFC3491_NAMEPREP,
    /**
     * RFC3530 nfs4_cs_prep
     * @stable ICU 4.2
     */
	USPREP_RFC3530_NFS4_CS_PREP,
    /**
     * RFC3530 nfs4_cs_prep with case insensitive option
     * @stable ICU 4.2
     */
	USPREP_RFC3530_NFS4_CS_PREP_CI,
    /**
     * RFC3530 nfs4_cis_prep
     * @stable ICU 4.2
     */
	USPREP_RFC3530_NFS4_CIS_PREP,
    /**
     * RFC3530 nfs4_mixed_prep for prefix
     * @stable ICU 4.2
     */
	USPREP_RFC3530_NFS4_MIXED_PREP_PREFIX,
    /**
     * RFC3530 nfs4_mixed_prep for suffix
     * @stable ICU 4.2
     */
	USPREP_RFC3530_NFS4_MIXED_PREP_SUFFIX,
    /**
     * RFC3722 iSCSI
     * @stable ICU 4.2
     */
	USPREP_RFC3722_ISCSI,
    /**
     * RFC3920 XMPP Nodeprep
     * @stable ICU 4.2
     */
	USPREP_RFC3920_NODEPREP,
    /**
     * RFC3920 XMPP Resourceprep
     * @stable ICU 4.2
     */
	USPREP_RFC3920_RESOURCEPREP,
    /**
     * RFC4011 Policy MIB Stringprep
     * @stable ICU 4.2
     */
	USPREP_RFC4011_MIB,
    /**
     * RFC4013 SASLprep
     * @stable ICU 4.2
     */
    USPREP_RFC4013_SASLPREP,
    /**
     * RFC4505 trace
     * @stable ICU 4.2
     */
	USPREP_RFC4505_TRACE,
    /**
     * RFC4518 LDAP
     * @stable ICU 4.2
     */
	USPREP_RFC4518_LDAP,
    /**
     * RFC4518 LDAP for case ignore, numeric and stored prefix
     * matching rules
     * @stable ICU 4.2
     */
	USPREP_RFC4518_LDAP_CI
} UStringPrepProfileType;

U_STABLE UStringPrepProfile* U_EXPORT2
usprep_open(const char* path, 
            const char* fileName,
            UErrorCode* status);

U_STABLE UStringPrepProfile* U_EXPORT2
usprep_openByType(UStringPrepProfileType type,
				  UErrorCode* status);

U_STABLE void U_EXPORT2
usprep_close(UStringPrepProfile* profile);

#if U_SHOW_CPLUSPLUS_API

U_NAMESPACE_BEGIN

U_DEFINE_LOCAL_OPEN_POINTER(LocalUStringPrepProfilePointer, UStringPrepProfile, usprep_close);

U_NAMESPACE_END

#endif


U_STABLE int32_t U_EXPORT2
usprep_prepare(   const UStringPrepProfile* prep,
                  const UChar* src, int32_t srcLength, 
                  UChar* dest, int32_t destCapacity,
                  int32_t options,
                  UParseError* parseError,
                  UErrorCode* status );


#endif /* #if !UCONFIG_NO_IDNA */

#endif
