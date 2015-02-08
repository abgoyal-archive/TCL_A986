//* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */

// NB: Modelled after Mozilla's code (originally written by Pamela Greene,
// later modified by others), but almost entirely rewritten for Chrome.


#ifndef NET_BASE_REGISTRY_CONTROLLED_DOMAIN_H_
#define NET_BASE_REGISTRY_CONTROLLED_DOMAIN_H_

#include <string>

#include "base/basictypes.h"

class GURL;

template <typename T>
struct DefaultSingletonTraits;
struct DomainRule;

namespace net {

struct RegistryControlledDomainServiceSingletonTraits;

// This class is a singleton.
class RegistryControlledDomainService {
 public:
   ~RegistryControlledDomainService() { }

  // Returns the registered, organization-identifying host and all its registry
  // information, but no subdomains, from the given GURL.  Returns an empty
  // string if the GURL is invalid, has no host (e.g. a file: URL), has multiple
  // trailing dots, is an IP address, has only one subcomponent (i.e. no dots
  // other than leading/trailing ones), or is itself a recognized registry
  // identifier.  If no matching rule is found in the effective-TLD data (or in
  // the default data, if the resource failed to load), the last subcomponent of
  // the host is assumed to be the registry.
  //
  // Examples:
  //   http://www.google.com/file.html -> "google.com"  (com)
  //   http://..google.com/file.html   -> "google.com"  (com)
  //   http://google.com./file.html    -> "google.com." (com)
  //   http://a.b.co.uk/file.html      -> "b.co.uk"     (co.uk)
  //   file:///C:/bar.html             -> ""            (no host)
  //   http://foo.com../file.html      -> ""            (multiple trailing dots)
  //   http://192.168.0.1/file.html    -> ""            (IP address)
  //   http://bar/file.html            -> ""            (no subcomponents)
  //   http://co.uk/file.html          -> ""            (host is a registry)
  //   http://foo.bar/file.html        -> "foo.bar"     (no rule; assume bar)
  static std::string GetDomainAndRegistry(const GURL& gurl);

  // Like the GURL version, but takes a host (which is canonicalized internally)
  // instead of a full GURL.
  static std::string GetDomainAndRegistry(const std::string& host);
  static std::string GetDomainAndRegistry(const std::wstring& host);

  // This convenience function returns true if the two GURLs both have hosts
  // and one of the following is true:
  // * They each have a known domain and registry, and it is the same for both
  //   URLs.  Note that this means the trailing dot, if any, must match too.
  // * They don't have known domains/registries, but the hosts are identical.
  // Effectively, callers can use this function to check whether the input URLs
  // represent hosts "on the same site".
  static bool SameDomainOrHost(const GURL& gurl1, const GURL& gurl2);

  // Finds the length in bytes of the registrar portion of the host in the
  // given GURL.  Returns std::string::npos if the GURL is invalid or has no
  // host (e.g. a file: URL).  Returns 0 if the GURL has multiple trailing dots,
  // is an IP address, has no subcomponents, or is itself a recognized registry
  // identifier.  If no matching rule is found in the effective-TLD data (or in
  // the default data, if the resource failed to load), returns 0 if
  // |allow_unknown_registries| is false, or the length of the last subcomponent
  // if |allow_unknown_registries| is true.
  //
  // Examples:
  //   http://www.google.com/file.html -> 3                 (com)
  //   http://..google.com/file.html   -> 3                 (com)
  //   http://google.com./file.html    -> 4                 (com)
  //   http://a.b.co.uk/file.html      -> 5                 (co.uk)
  //   file:///C:/bar.html             -> std::string::npos (no host)
  //   http://foo.com../file.html      -> 0                 (multiple trailing
  //                                                         dots)
  //   http://192.168.0.1/file.html    -> 0                 (IP address)
  //   http://bar/file.html            -> 0                 (no subcomponents)
  //   http://co.uk/file.html          -> 0                 (host is a registry)
  //   http://foo.bar/file.html        -> 0 or 3, depending (no rule; assume
  //                                                         bar)
  static size_t GetRegistryLength(const GURL& gurl,
                                  bool allow_unknown_registries);

  // Like the GURL version, but takes a host (which is canonicalized internally)
  // instead of a full GURL.
  static size_t GetRegistryLength(const std::string& host,
                                  bool allow_unknown_registries);
  static size_t GetRegistryLength(const std::wstring& host,
                                  bool allow_unknown_registries);

 protected:
  // The entire protected API is only for unit testing.  I mean it.  Don't make
  // me come over there!
  RegistryControlledDomainService();

  // Set the RegistryControledDomainService instance to be used internally.
  // |instance| will supersede the singleton instance normally used.  If
  // |instance| is NULL, normal behavior is restored, and internal operations
  // will return to using the singleton.  This function always returns the
  // instance set by the most recent call to SetInstance.
  static RegistryControlledDomainService* SetInstance(
      RegistryControlledDomainService* instance);

  typedef const struct DomainRule* (*FindDomainPtr)(const char *, unsigned int);

  // Used for unit tests, so that a different perfect hash map from the full
  // list is used.
  static void UseFindDomainFunction(FindDomainPtr function);

 private:
  // To allow construction of the internal singleton instance.
  friend struct DefaultSingletonTraits<RegistryControlledDomainService>;

  // Returns the singleton instance, after attempting to initialize it.
  // NOTE that if the effective-TLD data resource can't be found, the instance
  // will be initialized and continue operation with simple default TLD data.
  static RegistryControlledDomainService* GetInstance();

  // Internal workings of the static public methods.  See above.
  static std::string GetDomainAndRegistryImpl(const std::string& host);
  size_t GetRegistryLengthImpl(const std::string& host,
                               bool allow_unknown_registries);

  // Function that returns a DomainRule given a domain.
  FindDomainPtr find_domain_function_;

  DISALLOW_COPY_AND_ASSIGN(RegistryControlledDomainService);
};

}  // namespace net

#endif  // NET_BASE_REGISTRY_CONTROLLED_DOMAIN_H_
