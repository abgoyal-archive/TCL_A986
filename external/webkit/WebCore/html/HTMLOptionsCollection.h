

#ifndef HTMLOptionsCollection_h
#define HTMLOptionsCollection_h

#include "HTMLCollection.h"

namespace WebCore {

class HTMLOptionElement;
class HTMLSelectElement;

typedef int ExceptionCode;

class HTMLOptionsCollection : public HTMLCollection {
public:
    static PassRefPtr<HTMLOptionsCollection> create(PassRefPtr<HTMLSelectElement>);

    void add(PassRefPtr<HTMLOptionElement>, ExceptionCode&);
    void add(PassRefPtr<HTMLOptionElement>, int index, ExceptionCode&);
    void remove(int index);

    int selectedIndex() const;
    void setSelectedIndex(int);

    void setLength(unsigned, ExceptionCode&);

private:
    HTMLOptionsCollection(PassRefPtr<HTMLSelectElement>);
};

} //namespace

#endif
