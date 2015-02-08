

#ifndef XPathParser_h
#define XPathParser_h

#if ENABLE(XPATH)

#include "XPathStep.h"
#include "XPathPredicate.h"

namespace WebCore {

    typedef int ExceptionCode;

    class XPathNSResolver;

    namespace XPath {

        class Expression;
        class ParseNode;
        class Predicate;

        struct Token {
            int type;
            String str;
            Step::Axis axis;
            NumericOp::Opcode numop;
            EqTestOp::Opcode eqop;
            
            Token(int t) : type(t) {}
            Token(int t, const String& v): type(t), str(v) {}
            Token(int t, Step::Axis v): type(t), axis(v) {}
            Token(int t, NumericOp::Opcode v): type(t), numop(v) {}
            Token(int t, EqTestOp::Opcode v): type(t), eqop(v) {}
        };

        class Parser : public Noncopyable {
        public:
            Parser();

            XPathNSResolver* resolver() const { return m_resolver.get(); }
            bool expandQName(const String& qName, String& localName, String& namespaceURI);

            Expression* parseStatement(const String& statement, PassRefPtr<XPathNSResolver>, ExceptionCode&);

            static Parser* current() { return currentParser; }

            int lex(void* yylval);

            Expression* m_topExpr;
            bool m_gotNamespaceError;

            void registerParseNode(ParseNode*);
            void unregisterParseNode(ParseNode*);

            void registerPredicateVector(Vector<Predicate*>*);
            void deletePredicateVector(Vector<Predicate*>*);

            void registerExpressionVector(Vector<Expression*>*);
            void deleteExpressionVector(Vector<Expression*>*);

            void registerString(String*);
            void deleteString(String*);

            void registerNodeTest(Step::NodeTest*);
            void deleteNodeTest(Step::NodeTest*);

        private:
            bool isOperatorContext() const;

            void skipWS();
            Token makeTokenAndAdvance(int type, int advance = 1);
            Token makeTokenAndAdvance(int type, NumericOp::Opcode, int advance = 1);
            Token makeTokenAndAdvance(int type, EqTestOp::Opcode, int advance = 1);
            char peekAheadHelper();
            char peekCurHelper();

            Token lexString();
            Token lexNumber();
            bool lexNCName(String&);
            bool lexQName(String&);

            Token nextToken();
            Token nextTokenInternal();

            void reset(const String& data);

            static Parser* currentParser;

            unsigned m_nextPos;
            String m_data;
            int m_lastTokenType;
            RefPtr<XPathNSResolver> m_resolver;

            HashSet<ParseNode*> m_parseNodes;
            HashSet<Vector<Predicate*>*> m_predicateVectors;
            HashSet<Vector<Expression*>*> m_expressionVectors;
            HashSet<String*> m_strings;
            HashSet<Step::NodeTest*> m_nodeTests;
        };

    }
}

#endif // ENABLE(XPATH)

#endif
