
#ifndef __LOCALPOINTER_H__
#define __LOCALPOINTER_H__


#include "unicode/utypes.h"

#if U_SHOW_CPLUSPLUS_API

U_NAMESPACE_BEGIN

template<typename T>
class LocalPointerBase {
public:
    /**
     * Constructor takes ownership.
     * @param p simple pointer to an object that is adopted
     * @draft ICU 4.4
     */
    explicit LocalPointerBase(T *p=NULL) : ptr(p) {}
    /**
     * Destructor deletes the object it owns.
     * Subclass must override: Base class does nothing.
     * @draft ICU 4.4
     */
    ~LocalPointerBase() { /* delete ptr; */ }
    /**
     * NULL check.
     * @return TRUE if ==NULL
     * @draft ICU 4.4
     */
    UBool isNull() const { return ptr==NULL; }
    /**
     * NULL check.
     * @return TRUE if !=NULL
     * @draft ICU 4.4
     */
    UBool isValid() const { return ptr!=NULL; }
    /**
     * Comparison with a simple pointer, so that existing code
     * with ==NULL need not be changed.
     * @param other simple pointer for comparison
     * @return true if this pointer value equals other
     * @draft ICU 4.4
     */
    bool operator==(const T *other) const { return ptr==other; }
    /**
     * Comparison with a simple pointer, so that existing code
     * with !=NULL need not be changed.
     * @param other simple pointer for comparison
     * @return true if this pointer value differs from other
     * @draft ICU 4.4
     */
    bool operator!=(const T *other) const { return ptr!=other; }
    /**
     * Access without ownership change.
     * @return the pointer value
     * @draft ICU 4.4
     */
    T *getAlias() const { return ptr; }
    /**
     * Access without ownership change.
     * @return the pointer value as a reference
     * @draft ICU 4.4
     */
    T &operator*() const { return *ptr; }
    /**
     * Access without ownership change.
     * @return the pointer value
     * @draft ICU 4.4
     */
    T *operator->() const { return ptr; }
    /**
     * Gives up ownership; the internal pointer becomes NULL.
     * @return the pointer value;
     *         caller becomes responsible for deleting the object
     * @draft ICU 4.4
     */
    T *orphan() {
        T *p=ptr;
        ptr=NULL;
        return p;
    }
    /**
     * Deletes the object it owns,
     * and adopts (takes ownership of) the one passed in.
     * Subclass must override: Base class does not delete the object.
     * @param p simple pointer to an object that is adopted
     * @draft ICU 4.4
     */
    void adoptInstead(T *p) {
        // delete ptr;
        ptr=p;
    }
protected:
    T *ptr;
private:
    // No comparison operators with other LocalPointerBases.
    bool operator==(const LocalPointerBase &other);
    bool operator!=(const LocalPointerBase &other);
    // No ownership transfer: No copy constructor, no assignment operator.
    LocalPointerBase(const LocalPointerBase &other);
    void operator=(const LocalPointerBase &other);
    // No heap allocation. Use only on the stack.
    static void * U_EXPORT2 operator new(size_t size);
    static void * U_EXPORT2 operator new[](size_t size);
#if U_HAVE_PLACEMENT_NEW
    static void * U_EXPORT2 operator new(size_t, void *ptr);
#endif
};

template<typename T>
class LocalPointer : public LocalPointerBase<T> {
public:
    /**
     * Constructor takes ownership.
     * @param p simple pointer to an object that is adopted
     * @draft ICU 4.4
     */
    explicit LocalPointer(T *p=NULL) : LocalPointerBase<T>(p) {}
    /**
     * Destructor deletes the object it owns.
     * @draft ICU 4.4
     */
    ~LocalPointer() {
        delete LocalPointerBase<T>::ptr;
    }
    /**
     * Deletes the object it owns,
     * and adopts (takes ownership of) the one passed in.
     * @param p simple pointer to an object that is adopted
     * @draft ICU 4.4
     */
    void adoptInstead(T *p) {
        delete LocalPointerBase<T>::ptr;
        LocalPointerBase<T>::ptr=p;
    }
};

template<typename T>
class LocalArray : public LocalPointerBase<T> {
public:
    /**
     * Constructor takes ownership.
     * @param p simple pointer to an array of T objects that is adopted
     * @draft ICU 4.4
     */
    explicit LocalArray(T *p=NULL) : LocalPointerBase<T>(p) {}
    /**
     * Destructor deletes the array it owns.
     * @draft ICU 4.4
     */
    ~LocalArray() {
        delete[] LocalPointerBase<T>::ptr;
    }
    /**
     * Deletes the array it owns,
     * and adopts (takes ownership of) the one passed in.
     * @param p simple pointer to an array of T objects that is adopted
     * @draft ICU 4.4
     */
    void adoptInstead(T *p) {
        delete[] LocalPointerBase<T>::ptr;
        LocalPointerBase<T>::ptr=p;
    }
    /**
     * Array item access (writable).
     * No index bounds check.
     * @param i array index
     * @return reference to the array item
     * @draft ICU 4.4
     */
    T &operator[](ptrdiff_t i) const { return LocalPointerBase<T>::ptr[i]; }
};

#define U_DEFINE_LOCAL_OPEN_POINTER(LocalPointerClassName, Type, closeFunction) \
    class LocalPointerClassName : public LocalPointerBase<Type> { \
    public: \
        explicit LocalPointerClassName(Type *p=NULL) : LocalPointerBase<Type>(p) {} \
        ~LocalPointerClassName() { closeFunction(ptr); } \
        void adoptInstead(Type *p) { \
            closeFunction(ptr); \
            ptr=p; \
        } \
    }

U_NAMESPACE_END

#endif  /* U_SHOW_CPLUSPLUS_API */
#endif  /* __LOCALPOINTER_H__ */
