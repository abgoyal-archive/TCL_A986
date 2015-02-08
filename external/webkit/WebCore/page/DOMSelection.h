


#ifndef DOMSelection_h
#define DOMSelection_h

#include <wtf/Forward.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    class Frame;
    class Range;
    class Node;
    class String;
    class VisibleSelection;

    typedef int ExceptionCode;

    class DOMSelection : public RefCounted<DOMSelection> {
    public:
        static PassRefPtr<DOMSelection> create(Frame* frame) { return adoptRef(new DOMSelection(frame)); }

        Frame* frame() const;
        void disconnectFrame();

        // Safari Selection Object API
        // These methods return the valid equivalents of internal editing positions.
        Node* baseNode() const;
        Node* extentNode() const;
        int baseOffset() const;
        int extentOffset() const;
        String type() const;
        void setBaseAndExtent(Node* baseNode, int baseOffset, Node* extentNode, int extentOffset, ExceptionCode&);
        void setPosition(Node*, int offset, ExceptionCode&);
        void modify(const String& alter, const String& direction, const String& granularity);

        // Mozilla Selection Object API
        // In Firefox, anchor/focus are the equal to the start/end of the selection,
        // but reflect the direction in which the selection was made by the user.  That does
        // not mean that they are base/extent, since the base/extent don't reflect
        // expansion.
        // These methods return the valid equivalents of internal editing positions.
        Node* anchorNode() const;
        int anchorOffset() const;
        Node* focusNode() const;
        int focusOffset() const;
        bool isCollapsed() const;
        int rangeCount() const;
        void collapse(Node*, int offset, ExceptionCode&);
        void collapseToEnd();
        void collapseToStart();
        void extend(Node*, int offset, ExceptionCode&);
        PassRefPtr<Range> getRangeAt(int, ExceptionCode&);
        void removeAllRanges();
        void addRange(Range*);
        void deleteFromDocument();
        bool containsNode(const Node*, bool partlyContained) const;
        void selectAllChildren(Node*, ExceptionCode&);

        String toString();

        // Microsoft Selection Object API
        void empty();

    private:
        DOMSelection(Frame*);

        // Convenience method for accessors, does not NULL check m_frame.
        const VisibleSelection& visibleSelection() const;

        Frame* m_frame;
    };

} // namespace WebCore

#endif // DOMSelection_h
