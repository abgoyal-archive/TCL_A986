
#ifndef __UOBJECT_H__
#define __UOBJECT_H__

#include "unicode/utypes.h"

U_NAMESPACE_BEGIN


#ifndef U_OVERRIDE_CXX_ALLOCATION
#define U_OVERRIDE_CXX_ALLOCATION 1
#endif

#ifndef U_HAVE_PLACEMENT_NEW
#define U_HAVE_PLACEMENT_NEW 1
#endif


#ifndef U_HAVE_DEBUG_LOCATION_NEW
#define U_HAVE_DEBUG_LOCATION_NEW 0
#endif

#ifndef U_NO_THROW
#define U_NO_THROW throw()
#endif

class U_COMMON_API UMemory {
public:

/* test versions for debugging shaper heap memory problems */
#ifdef SHAPER_MEMORY_DEBUG  
    static void * NewArray(int size, int count);
    static void * GrowArray(void * array, int newSize );
    static void   FreeArray(void * array );
#endif

#if U_OVERRIDE_CXX_ALLOCATION
    /**
     * Override for ICU4C C++ memory management.
     * simple, non-class types are allocated using the macros in common/cmemory.h
     * (uprv_malloc(), uprv_free(), uprv_realloc());
     * they or something else could be used here to implement C++ new/delete
     * for ICU4C C++ classes
     * @stable ICU 2.4
     */
    static void * U_EXPORT2 operator new(size_t size) U_NO_THROW;

    /**
     * Override for ICU4C C++ memory management.
     * See new().
     * @stable ICU 2.4
     */
    static void * U_EXPORT2 operator new[](size_t size) U_NO_THROW;

    /**
     * Override for ICU4C C++ memory management.
     * simple, non-class types are allocated using the macros in common/cmemory.h
     * (uprv_malloc(), uprv_free(), uprv_realloc());
     * they or something else could be used here to implement C++ new/delete
     * for ICU4C C++ classes
     * @stable ICU 2.4
     */
    static void U_EXPORT2 operator delete(void *p) U_NO_THROW;

    /**
     * Override for ICU4C C++ memory management.
     * See delete().
     * @stable ICU 2.4
     */
    static void U_EXPORT2 operator delete[](void *p) U_NO_THROW;

#if U_HAVE_PLACEMENT_NEW
    /**
     * Override for ICU4C C++ memory management for STL.
     * See new().
     * @stable ICU 2.6
     */
    static inline void * U_EXPORT2 operator new(size_t, void *ptr) U_NO_THROW { return ptr; }

    /**
     * Override for ICU4C C++ memory management for STL.
     * See delete().
     * @stable ICU 2.6
     */
    static inline void U_EXPORT2 operator delete(void *, void *) U_NO_THROW {}
#endif /* U_HAVE_PLACEMENT_NEW */
#if U_HAVE_DEBUG_LOCATION_NEW
    /**
      * This method overrides the MFC debug version of the operator new
      * 
      * @param size   The requested memory size
      * @param file   The file where the allocation was requested
      * @param line   The line where the allocation was requested 
      */ 
    static void * U_EXPORT2 operator new(size_t size, const char* file, int line) U_NO_THROW;
    /**
      * This method provides a matching delete for the MFC debug new
      * 
      * @param p      The pointer to the allocated memory
      * @param file   The file where the allocation was requested
      * @param line   The line where the allocation was requested 
      */ 
    static void U_EXPORT2 operator delete(void* p, const char* file, int line) U_NO_THROW;
#endif /* U_HAVE_DEBUG_LOCATION_NEW */
#endif /* U_OVERRIDE_CXX_ALLOCATION */

    /*
     * Assignment operator not declared. The compiler will provide one
     * which does nothing since this class does not contain any data members.
     * API/code coverage may show the assignment operator as present and
     * untested - ignore.
     * Subclasses need this assignment operator if they use compiler-provided
     * assignment operators of their own. An alternative to not declaring one
     * here would be to declare and empty-implement a protected or public one.
    UMemory &UMemory::operator=(const UMemory &);
     */
};

class U_COMMON_API UObject : public UMemory {
public:
    /**
     * Destructor.
     *
     * @stable ICU 2.2
     */
    virtual ~UObject();

    /**
     * ICU4C "poor man's RTTI", returns a UClassID for the actual ICU class.
     *
     * @stable ICU 2.2
     */
    virtual UClassID getDynamicClassID() const = 0;

protected:
    // the following functions are protected to prevent instantiation and
    // direct use of UObject itself

    // default constructor
    // commented out because UObject is abstract (see getDynamicClassID)
    // inline UObject() {}

    // copy constructor
    // commented out because UObject is abstract (see getDynamicClassID)
    // inline UObject(const UObject &other) {}

#if 0
    // TODO Sometime in the future. Implement operator==().
    // (This comment inserted in 2.2)
    // some or all of the following "boilerplate" functions may be made public
    // in a future ICU4C release when all subclasses implement them

    // assignment operator
    // (not virtual, see "Taligent's Guide to Designing Programs" pp.73..74)
    // commented out because the implementation is the same as a compiler's default
    // UObject &operator=(const UObject &other) { return *this; }

    // comparison operators
    virtual inline UBool operator==(const UObject &other) const { return this==&other; }
    inline UBool operator!=(const UObject &other) const { return !operator==(other); }

    // clone() commented out from the base class:
    // some compilers do not support co-variant return types
    // (i.e., subclasses would have to return UObject * as well, instead of SubClass *)
    // see also UObject class documentation.
    // virtual UObject *clone() const;
#endif

    /*
     * Assignment operator not declared. The compiler will provide one
     * which does nothing since this class does not contain any data members.
     * API/code coverage may show the assignment operator as present and
     * untested - ignore.
     * Subclasses need this assignment operator if they use compiler-provided
     * assignment operators of their own. An alternative to not declaring one
     * here would be to declare and empty-implement a protected or public one.
    UObject &UObject::operator=(const UObject &);
     */

// Future implementation for RTTI that support subtyping. [alan]
// 
//  public:
//     /**
//      * @internal
//      */
//     static UClassID getStaticClassID();
// 
//     /**
//      * @internal
//      */
//     UBool instanceOf(UClassID type) const;
};

#define UOBJECT_DEFINE_RTTI_IMPLEMENTATION(myClass) \
    UClassID U_EXPORT2 myClass::getStaticClassID() { \
        static char classID = 0; \
        return (UClassID)&classID; \
    } \
    UClassID myClass::getDynamicClassID() const \
    { return myClass::getStaticClassID(); }


#define UOBJECT_DEFINE_ABSTRACT_RTTI_IMPLEMENTATION(myClass) \
    UClassID U_EXPORT2 myClass::getStaticClassID() { \
        static char classID = 0; \
        return (UClassID)&classID; \
    }

// /**
//  * This macro adds ICU RTTI to an ICU concrete class implementation.
//  * This macro should be invoked in *.cpp files.  The corresponding
//  * header should declare getDynamicClassID and getStaticClassID.
//  *
//  * @param myClass The name of the class that needs RTTI defined.
//  * @param myParent The name of the myClass's parent.
//  * @internal
//  */


U_NAMESPACE_END

#endif
