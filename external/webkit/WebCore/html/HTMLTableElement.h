

#ifndef HTMLTableElement_h
#define HTMLTableElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLCollection;
class HTMLTableCaptionElement;
class HTMLTableSectionElement;

class HTMLTableElement : public HTMLElement {
public:
    HTMLTableElement(const QualifiedName&, Document*);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 9; }
    virtual bool checkDTD(const Node*);

    HTMLTableCaptionElement* caption() const;
    void setCaption(PassRefPtr<HTMLTableCaptionElement>, ExceptionCode&);

    HTMLTableSectionElement* tHead() const;
    void setTHead(PassRefPtr<HTMLTableSectionElement>, ExceptionCode&);

    HTMLTableSectionElement* tFoot() const;
    void setTFoot(PassRefPtr<HTMLTableSectionElement>, ExceptionCode&);

    PassRefPtr<HTMLElement> createTHead();
    void deleteTHead();
    PassRefPtr<HTMLElement> createTFoot();
    void deleteTFoot();
    PassRefPtr<HTMLElement> createCaption();
    void deleteCaption();
    PassRefPtr<HTMLElement> insertRow(int index, ExceptionCode&);
    void deleteRow(int index, ExceptionCode&);

    PassRefPtr<HTMLCollection> rows();
    PassRefPtr<HTMLCollection> tBodies();

    String align() const;
    void setAlign(const String&);

    String bgColor() const;
    void setBgColor(const String&);

    String border() const;
    void setBorder(const String&);

    String cellPadding() const;
    void setCellPadding(const String&);

    String cellSpacing() const;
    void setCellSpacing(const String&);

    String frame() const;
    void setFrame(const String&);

    String rules() const;
    void setRules(const String&);

    String summary() const;
    void setSummary(const String&);

    String width() const;
    void setWidth(const String&);

    virtual ContainerNode* addChild(PassRefPtr<Node>);
    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);
    virtual void attach();
    virtual bool isURLAttribute(Attribute*) const;

    // Used to obtain either a solid or outset border decl and to deal with the frame
    // and rules attributes.
    virtual bool canHaveAdditionalAttributeStyleDecls() const { return true; }
    virtual void additionalAttributeStyleDecls(Vector<CSSMutableStyleDeclaration*>&);
    void addSharedCellDecls(Vector<CSSMutableStyleDeclaration*>&);
    void addSharedGroupDecls(bool rows, Vector<CSSMutableStyleDeclaration*>&);

    virtual void addSubresourceAttributeURLs(ListHashSet<KURL>&) const;

private:
    void addSharedCellBordersDecl(Vector<CSSMutableStyleDeclaration*>&);
    void addSharedCellPaddingDecl(Vector<CSSMutableStyleDeclaration*>&);
    
    enum TableRules { UnsetRules, NoneRules, GroupsRules, RowsRules, ColsRules, AllRules };
    enum CellBorders { NoBorders, SolidBorders, InsetBorders, SolidBordersColsOnly, SolidBordersRowsOnly };

    CellBorders cellBorders() const;

    HTMLTableSectionElement* lastBody() const;

    bool m_borderAttr;          // Sets a precise border width and creates an outset border for the table and for its cells.
    bool m_borderColorAttr;     // Overrides the outset border and makes it solid for the table and cells instead.
    bool m_frameAttr;           // Implies a thin border width if no border is set and then a certain set of solid/hidden borders based off the value.
    TableRules m_rulesAttr;     // Implies a thin border width, a collapsing border model, and all borders on the table becoming set to hidden (if frame/border
                                // are present, to none otherwise).
   
    unsigned short m_padding;
    RefPtr<CSSMappedAttributeDeclaration> m_paddingDecl;
};

} //namespace

#endif
