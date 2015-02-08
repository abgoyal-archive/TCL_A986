

#ifndef HTMLElement_h
#define HTMLElement_h

#include "StyledElement.h"

namespace WebCore {

class DocumentFragment;
class HTMLCollection;
class HTMLFormElement;

enum HTMLTagStatus { TagStatusOptional, TagStatusRequired, TagStatusForbidden };
                       
class HTMLElement : public StyledElement {
public:
    static PassRefPtr<HTMLElement> create(const QualifiedName& tagName, Document*);

    PassRefPtr<HTMLCollection> children();
    
    virtual String title() const;

    virtual short tabIndex() const;
    void setTabIndex(int);

    String innerHTML() const;
    String outerHTML() const;
    PassRefPtr<DocumentFragment> createContextualFragment(const String&, FragmentScriptingPermission = FragmentScriptingAllowed);
    void setInnerHTML(const String&, ExceptionCode&);
    void setOuterHTML(const String&, ExceptionCode&);
    void setInnerText(const String&, ExceptionCode&);
    void setOuterText(const String&, ExceptionCode&);

    Element* insertAdjacentElement(const String& where, Element* newChild, ExceptionCode&);
    void insertAdjacentHTML(const String& where, const String& html, ExceptionCode&);
    void insertAdjacentText(const String& where, const String& text, ExceptionCode&);

    virtual bool supportsFocus() const;
    
    virtual bool isContentEditable() const;
    virtual bool isContentRichlyEditable() const;

    String contentEditable() const;
    void setContentEditable(const String&);

    virtual bool draggable() const;
    void setDraggable(bool);

    void click();

    virtual void accessKeyAction(bool sendToAnyElement);

    virtual HTMLTagStatus endTagRequirement() const;
    virtual int tagPriority() const;

    virtual bool rendererIsNeeded(RenderStyle*);
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);

    HTMLFormElement* form() const { return virtualForm(); }

    static void addHTMLAlignmentToStyledElement(StyledElement*, MappedAttribute*);

protected:
    HTMLElement(const QualifiedName& tagName, Document*, ConstructionType = CreateElementZeroRefCount);

    void addHTMLAlignment(MappedAttribute*);

    virtual bool mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    virtual bool childAllowed(Node* newChild); // Error-checking during parsing that checks the DTD

    // Helper function to check the DTD for a given child node.
    virtual bool checkDTD(const Node*);

    static bool inEitherTagList(const Node*);
    static bool inInlineTagList(const Node*);
    static bool inBlockTagList(const Node*);
    static bool isRecognizedTagName(const QualifiedName&);

    HTMLFormElement* findFormAncestor() const;

private:
    virtual bool isHTMLElement() const { return true; }

    virtual String nodeName() const;

    void setContentEditable(MappedAttribute*);

    virtual HTMLFormElement* virtualForm() const;

    Node* insertAdjacent(const String& where, Node* newChild, ExceptionCode&);
};

inline HTMLElement::HTMLElement(const QualifiedName& tagName, Document* document, ConstructionType type)
    : StyledElement(tagName, document, type)
{
    ASSERT(tagName.localName().impl());
}

} // namespace WebCore

#endif // HTMLElement_h
