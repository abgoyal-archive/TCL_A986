

#ifndef Label_h
#define Label_h

#include "CodeBlock.h"
#include "Instruction.h"
#include <wtf/Assertions.h>
#include <wtf/Vector.h>
#include <limits.h>

namespace JSC {

    class Label {
    public:
        explicit Label(CodeBlock* codeBlock)
            : m_refCount(0)
            , m_location(invalidLocation)
            , m_codeBlock(codeBlock)
        {
        }

        void setLocation(unsigned location)
        {
            m_location = location;

            unsigned size = m_unresolvedJumps.size();
            for (unsigned i = 0; i < size; ++i)
                m_codeBlock->instructions()[m_unresolvedJumps[i].second].u.operand = m_location - m_unresolvedJumps[i].first;
        }

        int bind(int opcode, int offset) const
        {
            if (m_location == invalidLocation) {
                m_unresolvedJumps.append(std::make_pair(opcode, offset));
                return 0;
            }
            return m_location - opcode;
        }

        void ref() { ++m_refCount; }
        void deref()
        {
            --m_refCount;
            ASSERT(m_refCount >= 0);
        }
        int refCount() const { return m_refCount; }

        bool isForward() const { return m_location == invalidLocation; }

    private:
        typedef Vector<std::pair<int, int>, 8> JumpVector;

        static const unsigned invalidLocation = UINT_MAX;

        int m_refCount;
        unsigned m_location;
        CodeBlock* m_codeBlock;
        mutable JumpVector m_unresolvedJumps;
    };

} // namespace JSC

#endif // Label_h
