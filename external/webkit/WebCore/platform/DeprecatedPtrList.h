

#ifndef DeprecatedPtrList_h
#define DeprecatedPtrList_h

#include "DeprecatedPtrListImpl.h"
#include <wtf/FastAllocBase.h>

namespace WebCore {

template <class T> class DeprecatedPtrListIterator;

template <class T> class DeprecatedPtrList : public FastAllocBase {
public:
    DeprecatedPtrList() : impl(deleteFunc), del_item(false) { }
    ~DeprecatedPtrList() { impl.clear(del_item); }
    
    DeprecatedPtrList(const DeprecatedPtrList& l) : impl(l.impl), del_item(false) { }
    DeprecatedPtrList& operator=(const DeprecatedPtrList &l) { impl.assign(l.impl, del_item); return *this; }

    bool isEmpty() const { return impl.isEmpty(); }
    unsigned count() const { return impl.count(); }
    void clear() { impl.clear(del_item); }

    T *at(unsigned n) { return (T *)impl.at(n); }

    bool insert(unsigned n, const T *item) { return impl.insert(n, item); }
    bool remove() { return impl.remove(del_item); }
    bool remove(unsigned n) { return impl.remove(n, del_item); }
    bool remove(const T *item) { return impl.removeRef(item, del_item); }
    bool removeFirst() { return impl.removeFirst(del_item); }
    bool removeLast() { return impl.removeLast(del_item); }
    bool removeRef(const T *item) { return impl.removeRef(item, del_item); }

    T *getFirst() const { return (T *)impl.getFirst(); }
    T *getLast() const { return (T *)impl.getLast(); }
    T *getNext() const { return (T *)impl.getNext(); }
    T *getPrev() const { return (T *)impl.getPrev(); }
    T *current() const { return (T *)impl.current(); }
    T *first() { return (T *)impl.first(); }
    T *last() { return (T *)impl.last(); }
    T *next() { return (T *)impl.next(); }
    T *prev() { return (T *)impl.prev(); }
    T *take(unsigned n) { return (T *)impl.take(n); }
    T *take() { return (T *)impl.take(); }

    void append(const T *item) { impl.append(item); }
    void prepend(const T *item) { impl.prepend(item); }

    unsigned containsRef(const T *item) const { return impl.containsRef(item); }
    int findRef(const T *item) { return impl.findRef(item); }

    typedef DeprecatedPtrListIterator<T> Iterator;
    typedef DeprecatedPtrListIterator<T> ConstIterator;
    ConstIterator begin() const { return ConstIterator(*this); }
    ConstIterator end() const { ConstIterator itr(*this); itr.toLast(); ++itr; return itr; }

    bool autoDelete() const { return del_item; }
    void setAutoDelete(bool autoDelete) { del_item = autoDelete; }

 private:
    static void deleteFunc(void *item) { delete (T *)item; }

    friend class DeprecatedPtrListIterator<T>;

    DeprecatedPtrListImpl impl;
    bool del_item;
};

template <class T> class DeprecatedPtrListIterator {
public:
    DeprecatedPtrListIterator() { }
    DeprecatedPtrListIterator(const DeprecatedPtrList<T> &l) : impl(l.impl) { }

    unsigned count() const { return impl.count(); }
    T *toFirst() { return (T *)impl.toFirst(); }
    T *toLast() { return (T *)impl.toLast(); }
    T *current() const { return (T *)impl.current(); }

    operator T *() const { return (T *)impl.current(); }
    T *operator*() const { return (T *)impl.current(); }
    T *operator--() { return (T *)--impl; }
    T *operator++()  { return (T *)++impl; }

private:
    DeprecatedPtrListImplIterator impl;
};

}

#endif
