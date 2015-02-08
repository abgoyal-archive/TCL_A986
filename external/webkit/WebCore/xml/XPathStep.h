

#ifndef XPathStep_h
#define XPathStep_h

#if ENABLE(XPATH)

#include "Node.h"
#include "XPathExpressionNode.h"
#include "XPathNodeSet.h"

namespace WebCore {

    namespace XPath {

        class Predicate;
        
        class Step : public ParseNode, public Noncopyable {
        public:
            enum Axis {
                AncestorAxis, AncestorOrSelfAxis, AttributeAxis,
                ChildAxis, DescendantAxis, DescendantOrSelfAxis,
                FollowingAxis, FollowingSiblingAxis, NamespaceAxis,
                ParentAxis, PrecedingAxis, PrecedingSiblingAxis,
                SelfAxis
            };
            
            class NodeTest : public FastAllocBase {
            public:
                enum Kind {
                    TextNodeTest, CommentNodeTest, ProcessingInstructionNodeTest, AnyNodeTest, NameTest
                };
                
                NodeTest(Kind kind) : m_kind(kind) {}
                NodeTest(Kind kind, const String& data) : m_kind(kind), m_data(data) {}
                NodeTest(Kind kind, const String& data, const String& namespaceURI) : m_kind(kind), m_data(data), m_namespaceURI(namespaceURI) {}
                
                Kind kind() const { return m_kind; }
                const AtomicString& data() const { return m_data; }
                const AtomicString& namespaceURI() const { return m_namespaceURI; }
                Vector<Predicate*>& mergedPredicates() { return m_mergedPredicates; }
                const Vector<Predicate*>& mergedPredicates() const { return m_mergedPredicates; }
                
            private:
                Kind m_kind;
                AtomicString m_data;
                AtomicString m_namespaceURI;

                // When possible, we merge some or all predicates with node test for better performance.
                Vector<Predicate*> m_mergedPredicates;
            };

            Step(Axis, const NodeTest& nodeTest, const Vector<Predicate*>& predicates = Vector<Predicate*>());
            ~Step();

            void optimize();

            void evaluate(Node* context, NodeSet&) const;

            Axis axis() const { return m_axis; }
            const NodeTest& nodeTest() const { return m_nodeTest; }

        private:
            friend void optimizeStepPair(Step*, Step*, bool&);
            bool predicatesAreContextListInsensitive() const;

            void parseNodeTest(const String&);
            void nodesInAxis(Node* context, NodeSet&) const;
            String namespaceFromNodetest(const String& nodeTest) const;

            Axis m_axis;
            NodeTest m_nodeTest;
            Vector<Predicate*> m_predicates;
        };

        void optimizeStepPair(Step*, Step*, bool& dropSecondStep);
    }

}

#endif // ENABLE(XPATH)

#endif // XPathStep_h
