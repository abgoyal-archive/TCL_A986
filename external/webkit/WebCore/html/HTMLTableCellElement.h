

#ifndef HTMLTableCellElement_h
#define HTMLTableCellElement_h

#include "HTMLTablePartElement.h"

namespace WebCore {

class HTMLTableCellElement : public HTMLTablePartElement {
public:
    HTMLTableCellElement(const QualifiedName&, Document*);
    ~HTMLTableCellElement();

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusOptional; }
    virtual int tagPriority() const { return 6; }

    int cellIndex() const;

    int col() const { return _col; }
    void setCol(int col) { _col = col; }
    int row() const { return _row; }
    void setRow(int r) { _row = r; }

    int colSpan() const { return cSpan; }
    int rowSpan() const { return rSpan; }

    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    // used by table cells to share style decls created by the enclosing table.
    virtual bool canHaveAdditionalAttributeStyleDecls() const { return true; }
    virtual void additionalAttributeStyleDecls(Vector<CSSMutableStyleDeclaration*>&);
    
    virtual bool isURLAttribute(Attribute*) const;

    void setCellIndex(int);

    String abbr() const;
    void setAbbr(const String&);

    String align() const;
    void setAlign(const String&);

    String axis() const;
    void setAxis(const String&);

    String bgColor() const;
    void setBgColor(const String&);

    String ch() const;
    void setCh(const String&);

    String chOff() const;
    void setChOff(const String&);

    void setColSpan(int);

    String headers() const;
    void setHeaders(const String&);

    String height() const;
    void setHeight(const String&);

    bool noWrap() const;
    void setNoWrap(bool);

    void setRowSpan(int);

    String scope() const;
    void setScope(const String&);

    String vAlign() const;
    void setVAlign(const String&);

    String width() const;
    void setWidth(const String&);

    virtual void addSubresourceAttributeURLs(ListHashSet<KURL>&) const;

protected:
    int _row;
    int _col;
    int rSpan;
    int cSpan;
    int rowHeight;
    bool m_solid;
};

} //namespace

#endif
