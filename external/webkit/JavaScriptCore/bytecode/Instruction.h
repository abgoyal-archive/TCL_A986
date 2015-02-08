

#ifndef Instruction_h
#define Instruction_h

#include "MacroAssembler.h"
#include "Opcode.h"
#include "Structure.h"
#include <wtf/VectorTraits.h>

#define POLYMORPHIC_LIST_CACHE_SIZE 8

namespace JSC {

    // *Sigh*, If the JIT is enabled we need to track the stubRountine (of type CodeLocationLabel),
    // If the JIT is not in use we don't actually need the variable (that said, if the JIT is not in use we don't
    // curently actually use PolymorphicAccessStructureLists, which we should).  Anyway, this seems like the best
    // solution for now - will need to something smarter if/when we actually want mixed-mode operation.
#if ENABLE(JIT)
    typedef CodeLocationLabel PolymorphicAccessStructureListStubRoutineType;
#else
    typedef void* PolymorphicAccessStructureListStubRoutineType;
#endif

    class JSCell;
    class Structure;
    class StructureChain;

    // Structure used by op_get_by_id_self_list and op_get_by_id_proto_list instruction to hold data off the main opcode stream.
    struct PolymorphicAccessStructureList : FastAllocBase {
        struct PolymorphicStubInfo {
            bool isChain;
            PolymorphicAccessStructureListStubRoutineType stubRoutine;
            Structure* base;
            union {
                Structure* proto;
                StructureChain* chain;
            } u;

            void set(PolymorphicAccessStructureListStubRoutineType _stubRoutine, Structure* _base)
            {
                stubRoutine = _stubRoutine;
                base = _base;
                u.proto = 0;
                isChain = false;
            }
            
            void set(PolymorphicAccessStructureListStubRoutineType _stubRoutine, Structure* _base, Structure* _proto)
            {
                stubRoutine = _stubRoutine;
                base = _base;
                u.proto = _proto;
                isChain = false;
            }
            
            void set(PolymorphicAccessStructureListStubRoutineType _stubRoutine, Structure* _base, StructureChain* _chain)
            {
                stubRoutine = _stubRoutine;
                base = _base;
                u.chain = _chain;
                isChain = true;
            }
        } list[POLYMORPHIC_LIST_CACHE_SIZE];
        
        PolymorphicAccessStructureList(PolymorphicAccessStructureListStubRoutineType stubRoutine, Structure* firstBase)
        {
            list[0].set(stubRoutine, firstBase);
        }

        PolymorphicAccessStructureList(PolymorphicAccessStructureListStubRoutineType stubRoutine, Structure* firstBase, Structure* firstProto)
        {
            list[0].set(stubRoutine, firstBase, firstProto);
        }

        PolymorphicAccessStructureList(PolymorphicAccessStructureListStubRoutineType stubRoutine, Structure* firstBase, StructureChain* firstChain)
        {
            list[0].set(stubRoutine, firstBase, firstChain);
        }

        void derefStructures(int count)
        {
            for (int i = 0; i < count; ++i) {
                PolymorphicStubInfo& info = list[i];

                ASSERT(info.base);
                info.base->deref();

                if (info.u.proto) {
                    if (info.isChain)
                        info.u.chain->deref();
                    else
                        info.u.proto->deref();
                }
            }
        }
    };

    struct Instruction {
        Instruction(Opcode opcode)
        {
#if !HAVE(COMPUTED_GOTO)
            // We have to initialize one of the pointer members to ensure that
            // the entire struct is initialized, when opcode is not a pointer.
            u.jsCell = 0;
#endif
            u.opcode = opcode;
        }

        Instruction(int operand)
        {
            // We have to initialize one of the pointer members to ensure that
            // the entire struct is initialized in 64-bit.
            u.jsCell = 0;
            u.operand = operand;
        }

        Instruction(Structure* structure) { u.structure = structure; }
        Instruction(StructureChain* structureChain) { u.structureChain = structureChain; }
        Instruction(JSCell* jsCell) { u.jsCell = jsCell; }
        Instruction(PolymorphicAccessStructureList* polymorphicStructures) { u.polymorphicStructures = polymorphicStructures; }

        union {
            Opcode opcode;
            int operand;
            Structure* structure;
            StructureChain* structureChain;
            JSCell* jsCell;
            PolymorphicAccessStructureList* polymorphicStructures;
        } u;
    };

} // namespace JSC

namespace WTF {

    template<> struct VectorTraits<JSC::Instruction> : VectorTraitsBase<true, JSC::Instruction> { };

} // namespace WTF

#endif // Instruction_h
