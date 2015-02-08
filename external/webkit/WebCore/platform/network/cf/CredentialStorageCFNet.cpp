

#include "config.h"
#include "CredentialStorage.h"

#include "AuthenticationCF.h"
#include "Credential.h"
#include "ProtectionSpace.h"
#include <WebKitSystemInterface/WebKitSystemInterface.h>
#include <wtf/RetainPtr.h>

namespace WebCore {

Credential CredentialStorage::getFromPersistentStorage(const ProtectionSpace& protectionSpace)
{
    RetainPtr<CFURLProtectionSpaceRef> protectionSpaceCF(AdoptCF, createCF(protectionSpace));
    RetainPtr<CFURLCredentialRef> credentialCF(AdoptCF, wkCopyCredentialFromCFPersistentStorage(protectionSpaceCF.get()));
    return core(credentialCF.get());
}

} // namespace WebCore
