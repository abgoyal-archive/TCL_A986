

#ifndef DeprecatedPtrListImpl_h
#define DeprecatedPtrListImpl_h

namespace WebCore {

class DeprecatedListNode;
class DeprecatedPtrListImplIterator;

class DeprecatedPtrListImpl
{
public:
    
    DeprecatedPtrListImpl(void (*deleteFunc)(void *));
    DeprecatedPtrListImpl(const DeprecatedPtrListImpl &impl);
    ~DeprecatedPtrListImpl();
     
    bool isEmpty() const { return nodeCount == 0; }
    unsigned count() const { return nodeCount; }
    void clear(bool deleteItems);

    void *at(unsigned n);

    bool insert(unsigned n, const void *item);
    bool remove(bool deleteItem);
    bool remove(unsigned n, bool deleteItem);
    bool removeFirst(bool deleteItem);
    bool removeLast(bool deleteItem);
    bool removeRef(const void *item, bool deleteItem);

    void *getFirst() const;
    void *getLast() const;
    void *getNext() const;
    void *getPrev() const;
    void *current() const;
    void *first();
    void *last();
    void *next();
    void *prev();
    void *take(unsigned n);
    void *take();

    void append(const void *item);
    void prepend(const void *item);

    unsigned containsRef(const void *item) const;
    int findRef(const void *item);

    DeprecatedPtrListImpl &assign(const DeprecatedPtrListImpl &impl, bool deleteItems);

 private:
    DeprecatedPtrListImpl &operator =(const DeprecatedPtrListImpl &impl);

    void swap(DeprecatedPtrListImpl &impl);

    void addIterator(DeprecatedPtrListImplIterator *iter) const;
    void removeIterator(DeprecatedPtrListImplIterator *iter) const;

    DeprecatedListNode *head;
    DeprecatedListNode *tail;
    DeprecatedListNode *cur;
    unsigned nodeCount;
    void (*deleteItem)(void *);
    mutable DeprecatedPtrListImplIterator *iterators;

    friend class DeprecatedPtrListImplIterator;
};


class DeprecatedPtrListImplIterator {
public:
    DeprecatedPtrListImplIterator();
    DeprecatedPtrListImplIterator(const DeprecatedPtrListImpl &impl);
    ~DeprecatedPtrListImplIterator();

    DeprecatedPtrListImplIterator(const DeprecatedPtrListImplIterator &impl);
    DeprecatedPtrListImplIterator &operator=(const DeprecatedPtrListImplIterator &impl);

    unsigned count() const;
    void *toFirst();
    void *toLast();
    void *current() const;

    void *operator--();
    void *operator++();

private:
    const DeprecatedPtrListImpl *list;
    DeprecatedListNode *node;
    DeprecatedPtrListImplIterator *next;
    DeprecatedPtrListImplIterator *prev;

    friend class DeprecatedPtrListImpl;
};

}

#endif
