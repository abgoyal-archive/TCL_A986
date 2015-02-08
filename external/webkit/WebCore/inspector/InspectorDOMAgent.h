

#ifndef InspectorDOMAgent_h
#define InspectorDOMAgent_h

#include "AtomicString.h"
#include "EventListener.h"
#include "EventTarget.h"
#include "ScriptArray.h"
#include "ScriptObject.h"
#include "ScriptState.h"

#include <wtf/ListHashSet.h>
#include <wtf/HashMap.h>
#include <wtf/HashSet.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {
    class ContainerNode;
    class Element;
    class Event;
    class Document;
    class InspectorFrontend;
    class NameNodeMap;
    class Node;
    class Page;

    struct EventListenerInfo {
        EventListenerInfo(Node* node, const AtomicString& eventType, const EventListenerVector& eventListenerVector)
            : node(node)
            , eventType(eventType)
            , eventListenerVector(eventListenerVector)
        {
        }

        Node* node;
        const AtomicString eventType;
        const EventListenerVector eventListenerVector;
    };

    class InspectorDOMAgent : public EventListener {
    public:
        static PassRefPtr<InspectorDOMAgent> create(InspectorFrontend* frontend)
        {
            return adoptRef(new InspectorDOMAgent(frontend));
        }

        static const InspectorDOMAgent* cast(const EventListener* listener)
        {
            return listener->type() == InspectorDOMAgentType
                ? static_cast<const InspectorDOMAgent*>(listener)
                : 0;
        }

        InspectorDOMAgent(InspectorFrontend* frontend);
        ~InspectorDOMAgent();

        void reset();

        virtual bool operator==(const EventListener& other);

        // Methods called from the frontend.
        void getChildNodes(long callId, long nodeId);
        void setAttribute(long callId, long elementId, const String& name, const String& value);
        void removeAttribute(long callId, long elementId, const String& name);
        void setTextNodeValue(long callId, long nodeId, const String& value);
        void getEventListenersForNode(long callId, long nodeId);

        // Methods called from the InspectorController.
        void setDocument(Document* document);
        void releaseDanglingNodes();

        void didInsertDOMNode(Node*);
        void didRemoveDOMNode(Node*);
        void didModifyDOMAttr(Element*);

        Node* nodeForId(long nodeId);
        Node* nodeForPath(const String& path);
        long pushNodePathToFrontend(Node* node);
        void pushChildNodesToFrontend(long nodeId);

   private:
        void startListening(Document* document);
        void stopListening(Document* document);

        virtual void handleEvent(ScriptExecutionContext*, Event* event);

        typedef HashMap<RefPtr<Node>, long> NodeToIdMap;
        long bind(Node* node, NodeToIdMap* nodesMap);
        void unbind(Node* node, NodeToIdMap* nodesMap);

        bool pushDocumentToFrontend();

        ScriptObject buildObjectForNode(Node* node, int depth, NodeToIdMap* nodesMap);
        ScriptArray buildArrayForElementAttributes(Element* element);
        ScriptArray buildArrayForContainerChildren(Node* container, int depth, NodeToIdMap* nodesMap);

        ScriptObject buildObjectForEventListener(const RegisteredEventListener& registeredEventListener, const AtomicString& eventType, Node* node);

        // We represent embedded doms as a part of the same hierarchy. Hence we treat children of frame owners differently.
        // We also skip whitespace text nodes conditionally. Following methods encapsulate these specifics.
        Node* innerFirstChild(Node* node);
        Node* innerNextSibling(Node* node);
        Node* innerPreviousSibling(Node* node);
        unsigned innerChildNodeCount(Node* node);
        Node* innerParentNode(Node* node);
        bool isWhitespace(Node* node);

        Document* mainFrameDocument() const;
        String documentURLString(Document* document) const;
        void discardBindings();

        InspectorFrontend* m_frontend;
        NodeToIdMap m_documentNodeToIdMap;
        // Owns node mappings for dangling nodes.
        Vector<NodeToIdMap*> m_danglingNodeToIdMaps;
        HashMap<long, Node*> m_idToNode;
        HashMap<long, NodeToIdMap*> m_idToNodesMap;
        HashSet<long> m_childrenRequested;
        long m_lastNodeId;
        ListHashSet<RefPtr<Document> > m_documents;
    };


} // namespace WebCore

#endif // !defined(InspectorDOMAgent_h)
