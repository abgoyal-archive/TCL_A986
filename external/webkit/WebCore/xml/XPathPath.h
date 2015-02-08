

#ifndef XPathPath_h
#define XPathPath_h

#if ENABLE(XPATH)

#include "XPathExpressionNode.h"
#include "XPathNodeSet.h"

namespace WebCore {

    namespace XPath {

        class Predicate;
        class Step;

        class Filter : public Expression {
        public:
            Filter(Expression*, const Vector<Predicate*>& = Vector<Predicate*>());
            virtual ~Filter();

            virtual Value evaluate() const;

        private:
            virtual Value::Type resultType() const { return Value::NodeSetValue; }

            Expression* m_expr;
            Vector<Predicate*> m_predicates;
        };

        class LocationPath : public Expression {
        public:
            LocationPath();
            virtual ~LocationPath();
            void setAbsolute(bool value) { m_absolute = value; setIsContextNodeSensitive(!m_absolute); }

            virtual Value evaluate() const;
            void evaluate(NodeSet& nodes) const; // nodes is an input/output parameter

            void appendStep(Step* step);
            void insertFirstStep(Step* step);

        private:
            virtual Value::Type resultType() const { return Value::NodeSetValue; }

            Vector<Step*> m_steps;
            bool m_absolute;
        };

        class Path : public Expression {
        public:
            Path(Filter*, LocationPath*);
            virtual ~Path();

            virtual Value evaluate() const;

        private:
            virtual Value::Type resultType() const { return Value::NodeSetValue; }

            Filter* m_filter;
            LocationPath* m_path;
        };

    }
}

#endif // ENABLE(XPATH)

#endif // XPath_Path_H
