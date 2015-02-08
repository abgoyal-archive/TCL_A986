

#ifndef HTMLAnchorElement_h
#define HTMLAnchorElement_h

#include "HTMLElement.h"

namespace WebCore {

// Link relation bitmask values.
// FIXME: Uncomment as the various link relations are implemented.
enum {
//     RelationAlternate   = 0x00000001,
//     RelationArchives    = 0x00000002,
//     RelationAuthor      = 0x00000004,
//     RelationBoomark     = 0x00000008,
//     RelationExternal    = 0x00000010,
//     RelationFirst       = 0x00000020,
//     RelationHelp        = 0x00000040,
//     RelationIndex       = 0x00000080,
//     RelationLast        = 0x00000100,
//     RelationLicense     = 0x00000200,
//     RelationNext        = 0x00000400,
//     RelationNoFolow    = 0x00000800,
    RelationNoReferrer     = 0x00001000,
//     RelationPrev        = 0x00002000,
//     RelationSearch      = 0x00004000,
//     RelationSidebar     = 0x00008000,
//     RelationTag         = 0x00010000,
//     RelationUp          = 0x00020000,
};

class HTMLAnchorElement : public HTMLElement {
public:
    static PassRefPtr<HTMLAnchorElement> create(Document*);
    static PassRefPtr<HTMLAnchorElement> create(const QualifiedName&, Document*);

    KURL href() const;
    void setHref(const AtomicString&);

    const AtomicString& name() const;

    String hash() const;
    void setHash(const String&);

    String host() const;
    void setHost(const String&);

    String hostname() const;
    void setHostname(const String&);

    String pathname() const;
    void setPathname(const String&);

    String port() const;
    void setPort(const String&);

    String protocol() const;
    void setProtocol(const String&);

    String search() const;
    void setSearch(const String&);

    String text() const;

    String toString() const;

    bool isLiveLink() const;

    bool hasRel(uint32_t relation) const;
    void setRel(const String&);

protected:
    HTMLAnchorElement(const QualifiedName&, Document*);

    virtual void parseMappedAttribute(MappedAttribute*);

private:
    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 1; }
    virtual bool supportsFocus() const;
    virtual bool isMouseFocusable() const;
    virtual bool isKeyboardFocusable(KeyboardEvent*) const;
    virtual void defaultEventHandler(Event*);
    virtual void setActive(bool active = true, bool pause = false);
    virtual void accessKeyAction(bool fullAction);
    virtual bool isURLAttribute(Attribute*) const;
    virtual bool canStartSelection() const;
    virtual String target() const;
    virtual short tabIndex() const;
    virtual bool draggable() const;

    RefPtr<Element> m_rootEditableElementForSelectionOnMouseDown;
    bool m_wasShiftKeyDownOnMouseDown;
    uint32_t m_linkRelations;
};

} // namespace WebCore

#endif // HTMLAnchorElement_h
