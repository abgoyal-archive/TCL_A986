
#ifndef CMEMORY_H
#define CMEMORY_H

#include "unicode/utypes.h"
#include "unicode/localpointer.h"
#include <stddef.h>
#include <string.h>


#define uprv_memcpy(dst, src, size) U_STANDARD_CPP_NAMESPACE memcpy(dst, src, size)
#define uprv_memmove(dst, src, size) U_STANDARD_CPP_NAMESPACE memmove(dst, src, size)
#define uprv_memset(buffer, mark, size) U_STANDARD_CPP_NAMESPACE memset(buffer, mark, size)
#define uprv_memcmp(buffer1, buffer2, size) U_STANDARD_CPP_NAMESPACE memcmp(buffer1, buffer2,size)

U_CAPI void * U_EXPORT2
uprv_malloc(size_t s);

U_CAPI void * U_EXPORT2
uprv_realloc(void *mem, size_t size);

U_CAPI void U_EXPORT2
uprv_free(void *mem);

typedef union {
    long    t1;
    double  t2;
    void   *t3;
} UAlignedMemory;

#define U_POINTER_MASK_LSB(ptr, mask) (((ptrdiff_t)(char *)(ptr)) & (mask))

#define U_ALIGNMENT_OFFSET(ptr) U_POINTER_MASK_LSB(ptr, sizeof(UAlignedMemory) - 1)

#define U_ALIGNMENT_OFFSET_UP(ptr) (sizeof(UAlignedMemory) - U_ALIGNMENT_OFFSET(ptr))

U_CFUNC UBool 
cmemory_inUse(void);

U_CFUNC UBool 
cmemory_cleanup(void);

#ifdef XP_CPLUSPLUS

U_NAMESPACE_BEGIN

template<typename T>
class LocalMemory : public LocalPointerBase<T> {
public:
    /**
     * Constructor takes ownership.
     * @param p simple pointer to an array of T items that is adopted
     */
    explicit LocalMemory(T *p=NULL) : LocalPointerBase<T>(p) {}
    /**
     * Destructor deletes the memory it owns.
     */
    ~LocalMemory() {
        uprv_free(LocalPointerBase<T>::ptr);
    }
    /**
     * Deletes the array it owns,
     * and adopts (takes ownership of) the one passed in.
     * @param p simple pointer to an array of T items that is adopted
     */
    void adoptInstead(T *p) {
        uprv_free(LocalPointerBase<T>::ptr);
        LocalPointerBase<T>::ptr=p;
    }
    /**
     * Deletes the array it owns, allocates a new one and reset its bytes to 0.
     * Returns the new array pointer.
     * If the allocation fails, then the current array is unchanged and
     * this method returns NULL.
     * @param newCapacity must be >0
     * @return the allocated array pointer, or NULL if the allocation failed
     */
    inline T *allocateInsteadAndReset(int32_t newCapacity=1);
    /**
     * Deletes the array it owns and allocates a new one, copying length T items.
     * Returns the new array pointer.
     * If the allocation fails, then the current array is unchanged and
     * this method returns NULL.
     * @param newCapacity must be >0
     * @param length number of T items to be copied from the old array to the new one;
     *               must be no more than the capacity of the old array,
     *               which the caller must track because the LocalMemory does not track it
     * @return the allocated array pointer, or NULL if the allocation failed
     */
    inline T *allocateInsteadAndCopy(int32_t newCapacity=1, int32_t length=0);
    /**
     * Array item access (writable).
     * No index bounds check.
     * @param i array index
     * @return reference to the array item
     */
    T &operator[](ptrdiff_t i) const { return LocalPointerBase<T>::ptr[i]; }
};

template<typename T, int32_t stackCapacity>
class MaybeStackArray {
public:
    /**
     * Default constructor initializes with internal T[stackCapacity] buffer.
     */
    MaybeStackArray() : needToRelease(FALSE), capacity(stackCapacity), ptr(stackArray) {}
    /**
     * Destructor deletes the array (if owned).
     */
    ~MaybeStackArray() { releaseArray(); }
    /**
     * Returns the array capacity (number of T items).
     * @return array capacity
     */
    int32_t getCapacity() const { return capacity; }
    /**
     * Access without ownership change.
     * @return the array pointer
     */
    T *getAlias() const { return ptr; }
    /**
     * Returns the array limit. Simple convenience method.
     * @return getAlias()+getCapacity()
     */
    T *getArrayLimit() const { return getAlias()+capacity; }
    /**
     * Access without ownership change. Same as getAlias().
     * A class instance can be used directly in expressions that take a T *.
     * @return the array pointer
     */
    operator T *() const { return ptr; }
    /**
     * Array item access (writable).
     * No index bounds check.
     * @param i array index
     * @return reference to the array item
     */
    T &operator[](ptrdiff_t i) { return ptr[i]; }
    /**
     * Deletes the array (if owned) and aliases another one, no transfer of ownership.
     * If the arguments are illegal, then the current array is unchanged.
     * @param otherArray must not be NULL
     * @param otherCapacity must be >0
     */
    void aliasInstead(T *otherArray, int32_t otherCapacity) {
        if(otherArray!=NULL && otherCapacity>0) {
            releaseArray();
            ptr=otherArray;
            capacity=otherCapacity;
            needToRelease=FALSE;
        }
    };
    /**
     * Deletes the array (if owned) and allocates a new one, copying length T items.
     * Returns the new array pointer.
     * If the allocation fails, then the current array is unchanged and
     * this method returns NULL.
     * @param newCapacity can be less than or greater than the current capacity;
     *                    must be >0
     * @param length number of T items to be copied from the old array to the new one
     * @return the allocated array pointer, or NULL if the allocation failed
     */
    inline T *resize(int32_t newCapacity, int32_t length=0);
    /**
     * Gives up ownership of the array if owned, or else clones it,
     * copying length T items; resets itself to the internal stack array.
     * Returns NULL if the allocation failed.
     * @param length number of T items to copy when cloning,
     *        and capacity of the clone when cloning
     * @param resultCapacity will be set to the returned array's capacity (output-only)
     * @return the array pointer;
     *         caller becomes responsible for deleting the array
     * @draft ICU 4.4
     */
    inline T *orphanOrClone(int32_t length, int32_t &resultCapacity);
private:
    UBool needToRelease;
    int32_t capacity;
    T *ptr;
    T stackArray[stackCapacity];
    void releaseArray() {
        if(needToRelease) {
            uprv_free(ptr);
        }
    }
    /* No comparison operators with other MaybeStackArray's. */
    bool operator==(const MaybeStackArray & /*other*/) {return FALSE;};
    bool operator!=(const MaybeStackArray & /*other*/) {return TRUE;};
    /* No ownership transfer: No copy constructor, no assignment operator. */
    MaybeStackArray(const MaybeStackArray & /*other*/) {};
    void operator=(const MaybeStackArray & /*other*/) {};

    // No heap allocation. Use only on the stack.
    //   (Declaring these functions private triggers a cascade of problems:
    //      MSVC insists on exporting an instantiation of MaybeStackArray, which
    //      requires that all functions be defined.
    //      An empty implementation of new() is rejected, it must return a value.
    //      Returning NULL is rejected by gcc for operator new.
    //      The expedient thing is just not to override operator new.
    //      While relatively pointless, heap allocated instances will function.
    // static void * U_EXPORT2 operator new(size_t size); 
    // static void * U_EXPORT2 operator new[](size_t size);
#if U_HAVE_PLACEMENT_NEW
    // static void * U_EXPORT2 operator new(size_t, void *ptr);
#endif
};

template<typename T>
inline T *LocalMemory<T>::allocateInsteadAndReset(int32_t newCapacity) {
    if(newCapacity>0) {
        T *p=(T *)uprv_malloc(newCapacity*sizeof(T));
        if(p!=NULL) {
            uprv_memset(p, 0, newCapacity*sizeof(T));
            uprv_free(LocalPointerBase<T>::ptr);
            LocalPointerBase<T>::ptr=p;
        }
        return p;
    } else {
        return NULL;
    }
}


template<typename T>
inline T *LocalMemory<T>::allocateInsteadAndCopy(int32_t newCapacity, int32_t length) {
    if(newCapacity>0) {
        T *p=(T *)uprv_malloc(newCapacity*sizeof(T));
        if(p!=NULL) {
            if(length>0) {
                if(length>newCapacity) {
                    length=newCapacity;
                }
                uprv_memcpy(p, LocalPointerBase<T>::ptr, length*sizeof(T));
            }
            uprv_free(LocalPointerBase<T>::ptr);
            LocalPointerBase<T>::ptr=p;
        }
        return p;
    } else {
        return NULL;
    }
}

template<typename T, int32_t stackCapacity>
inline T *MaybeStackArray<T, stackCapacity>::resize(int32_t newCapacity, int32_t length) {
    if(newCapacity>0) {
        T *p=(T *)uprv_malloc(newCapacity*sizeof(T));
        if(p!=NULL) {
            if(length>0) {
                if(length>capacity) {
                    length=capacity;
                }
                if(length>newCapacity) {
                    length=newCapacity;
                }
                uprv_memcpy(p, ptr, length*sizeof(T));
            }
            releaseArray();
            ptr=p;
            capacity=newCapacity;
            needToRelease=TRUE;
        }
        return p;
    } else {
        return NULL;
    }
}

template<typename T, int32_t stackCapacity>
inline T *MaybeStackArray<T, stackCapacity>::orphanOrClone(int32_t length, int32_t &resultCapacity) {
    T *p;
    if(needToRelease) {
        p=ptr;
    } else if(length<=0) {
        return NULL;
    } else {
        if(length>capacity) {
            length=capacity;
        }
        p=(T *)uprv_malloc(length*sizeof(T));
        if(p==NULL) {
            return NULL;
        }
        uprv_memcpy(p, ptr, length*sizeof(T));
    }
    resultCapacity=length;
    ptr=stackArray;
    capacity=stackCapacity;
    needToRelease=FALSE;
    return p;
}

U_NAMESPACE_END

#endif  /* XP_CPLUSPLUS */
#endif  /* CMEMORY_H */
