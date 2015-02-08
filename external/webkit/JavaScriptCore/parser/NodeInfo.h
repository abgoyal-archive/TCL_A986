

#ifndef NodeInfo_h
#define NodeInfo_h

#include "Nodes.h"
#include "Parser.h"

namespace JSC {

    template <typename T> struct NodeInfo {
        T m_node;
        CodeFeatures m_features;
        int m_numConstants;
    };

    typedef NodeInfo<FuncDeclNode*> FuncDeclNodeInfo;    
    typedef NodeInfo<FuncExprNode*> FuncExprNodeInfo;
    typedef NodeInfo<ExpressionNode*> ExpressionNodeInfo;
    typedef NodeInfo<ArgumentsNode*> ArgumentsNodeInfo;
    typedef NodeInfo<ConstDeclNode*> ConstDeclNodeInfo;
    typedef NodeInfo<PropertyNode*> PropertyNodeInfo;
    typedef NodeInfo<PropertyList> PropertyListInfo;
    typedef NodeInfo<ElementList> ElementListInfo;
    typedef NodeInfo<ArgumentList> ArgumentListInfo;
    
    template <typename T> struct NodeDeclarationInfo {
        T m_node;
        ParserArenaData<DeclarationStacks::VarStack>* m_varDeclarations;
        ParserArenaData<DeclarationStacks::FunctionStack>* m_funcDeclarations;
        CodeFeatures m_features;
        int m_numConstants;
    };
    
    typedef NodeDeclarationInfo<StatementNode*> StatementNodeInfo;
    typedef NodeDeclarationInfo<CaseBlockNode*> CaseBlockNodeInfo;
    typedef NodeDeclarationInfo<CaseClauseNode*> CaseClauseNodeInfo;
    typedef NodeDeclarationInfo<SourceElements*> SourceElementsInfo;
    typedef NodeDeclarationInfo<ClauseList> ClauseListInfo;
    typedef NodeDeclarationInfo<ExpressionNode*> VarDeclListInfo;
    typedef NodeDeclarationInfo<ConstDeclList> ConstDeclListInfo;
    typedef NodeDeclarationInfo<ParameterList> ParameterListInfo;

} // namespace JSC

#endif // NodeInfo_h
