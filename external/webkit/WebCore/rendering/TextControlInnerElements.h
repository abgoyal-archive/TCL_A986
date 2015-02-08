
 
#ifndef TextControlInnerElements_h
#define TextControlInnerElements_h

#include "HTMLDivElement.h"

namespace WebCore {

class String;

class TextControlInnerElement : public HTMLDivElement {
public:
    TextControlInnerElement(Document*, Node* shadowParent = 0);
    
    virtual bool isMouseFocusable() const { return false; } 
    virtual bool isShadowNode() const { return m_shadowParent; }
    virtual Node* shadowParentNode() { return m_shadowParent; }
    void setShadowParentNode(Node* node) { m_shadowParent = node; }
    void attachInnerElement(Node*, PassRefPtr<RenderStyle>, RenderArena*);
    
private:
    Node* m_shadowParent;
};

class TextControlInnerTextElement : public TextControlInnerElement {
public:
    TextControlInnerTextElement(Document*, Node* shadowParent);
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);  
    virtual void defaultEventHandler(Event*);
};

class SearchFieldResultsButtonElement : public TextControlInnerElement {
public:
    SearchFieldResultsButtonElement(Document*);
    virtual void defaultEventHandler(Event*);
};

class SearchFieldCancelButtonElement : public TextControlInnerElement {
public:
    SearchFieldCancelButtonElement(Document*);
    virtual void defaultEventHandler(Event*);
    virtual void detach();
private:
    bool m_capturing;
};

} //namespace

#endif
