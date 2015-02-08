

#include "config.h"
#include "Profile.h"

#include "ProfileNode.h"
#include <stdio.h>

namespace JSC {

PassRefPtr<Profile> Profile::create(const UString& title, unsigned uid)
{
    return adoptRef(new Profile(title, uid));
}

Profile::Profile(const UString& title, unsigned uid)
    : m_title(title)
    , m_uid(uid)
{
    // FIXME: When multi-threading is supported this will be a vector and calls
    // into the profiler will need to know which thread it is executing on.
    m_head = ProfileNode::create(CallIdentifier("Thread_1", UString(), 0), 0, 0);
}

Profile::~Profile()
{
}

void Profile::forEach(void (ProfileNode::*function)())
{
    ProfileNode* currentNode = m_head->firstChild();
    for (ProfileNode* nextNode = currentNode; nextNode; nextNode = nextNode->firstChild())
        currentNode = nextNode;

    if (!currentNode)
        currentNode = m_head.get();

    ProfileNode* endNode = m_head->traverseNextNodePostOrder();
    while (currentNode && currentNode != endNode) {
        (currentNode->*function)();
        currentNode = currentNode->traverseNextNodePostOrder();
    } 
}

void Profile::focus(const ProfileNode* profileNode)
{
    if (!profileNode || !m_head)
        return;

    bool processChildren;
    const CallIdentifier& callIdentifier = profileNode->callIdentifier();
    for (ProfileNode* currentNode = m_head.get(); currentNode; currentNode = currentNode->traverseNextNodePreOrder(processChildren))
        processChildren = currentNode->focus(callIdentifier);

    // Set the visible time of all nodes so that the %s display correctly.
    forEach(&ProfileNode::calculateVisibleTotalTime);
}

void Profile::exclude(const ProfileNode* profileNode)
{
    if (!profileNode || !m_head)
        return;

    const CallIdentifier& callIdentifier = profileNode->callIdentifier();

    for (ProfileNode* currentNode = m_head.get(); currentNode; currentNode = currentNode->traverseNextNodePreOrder())
        currentNode->exclude(callIdentifier);

    // Set the visible time of the head so the %s display correctly.
    m_head->setVisibleTotalTime(m_head->totalTime() - m_head->selfTime());
    m_head->setVisibleSelfTime(0.0);
}

void Profile::restoreAll()
{
    forEach(&ProfileNode::restore);
}

#ifndef NDEBUG
void Profile::debugPrintData() const
{
    printf("Call graph:\n");
    m_head->debugPrintData(0);
}

typedef pair<UString::Rep*, unsigned> NameCountPair;

static inline bool functionNameCountPairComparator(const NameCountPair& a, const NameCountPair& b)
{
    return a.second > b.second;
}

void Profile::debugPrintDataSampleStyle() const
{
    typedef Vector<NameCountPair> NameCountPairVector;

    FunctionCallHashCount countedFunctions;
    printf("Call graph:\n");
    m_head->debugPrintDataSampleStyle(0, countedFunctions);

    printf("\nTotal number in stack:\n");
    NameCountPairVector sortedFunctions(countedFunctions.size());
    copyToVector(countedFunctions, sortedFunctions);

    std::sort(sortedFunctions.begin(), sortedFunctions.end(), functionNameCountPairComparator);
    for (NameCountPairVector::iterator it = sortedFunctions.begin(); it != sortedFunctions.end(); ++it)
        printf("        %-12d%s\n", (*it).second, UString((*it).first).UTF8String().c_str());

    printf("\nSort by top of stack, same collapsed (when >= 5):\n");
}
#endif

} // namespace JSC
