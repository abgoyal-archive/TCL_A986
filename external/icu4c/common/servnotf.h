#ifndef ICUNOTIF_H
#define ICUNOTIF_H

#include "unicode/utypes.h"

#if UCONFIG_NO_SERVICE

U_NAMESPACE_BEGIN

class ICUNotifier;

U_NAMESPACE_END

#else

#include "unicode/uobject.h"
#include "unicode/unistr.h"

#include "mutex.h"
#include "uvector.h"

U_NAMESPACE_BEGIN

class U_COMMON_API EventListener : public UObject {
public: 
    virtual ~EventListener();

public:
    static UClassID U_EXPORT2 getStaticClassID();

    virtual UClassID getDynamicClassID() const;

public:
#ifdef SERVICE_DEBUG
    virtual UnicodeString& debug(UnicodeString& result) const {
      return debugClass(result);
    }

    virtual UnicodeString& debugClass(UnicodeString& result) const {
      return result.append("Key");
    }
#endif
};


class U_COMMON_API ICUNotifier : public UMemory  {
private: UMTX notifyLock;
private: UVector* listeners;
         
public: 
    ICUNotifier(void);
    
    virtual ~ICUNotifier(void);
    
    /**
     * Add a listener to be notified when notifyChanged is called.
     * The listener must not be null. AcceptsListener must return
     * true for the listener.  Attempts to concurrently
     * register the identical listener more than once will be
     * silently ignored.  
     */
    virtual void addListener(const EventListener* l, UErrorCode& status);
    
    /**
     * Stop notifying this listener.  The listener must
     * not be null.  Attemps to remove a listener that is
     * not registered will be silently ignored.
     */
    virtual void removeListener(const EventListener* l, UErrorCode& status);
    
    /**
     * ICU doesn't spawn its own threads.  All listeners are notified in
     * the thread of the caller.  Misbehaved listeners can therefore
     * indefinitely block the calling thread.  Callers should beware of
     * deadlock situations.  
     */
    virtual void notifyChanged(void);
    
protected: 
    /**
     * Subclasses implement this to return TRUE if the listener is
     * of the appropriate type.
     */
    virtual UBool acceptsListener(const EventListener& l) const = 0;
    
    /**
     * Subclasses implement this to notify the listener.
     */
    virtual void notifyListener(EventListener& l) const = 0;
};

U_NAMESPACE_END

/* UCONFIG_NO_SERVICE */
#endif

/* ICUNOTIF_H */
#endif
