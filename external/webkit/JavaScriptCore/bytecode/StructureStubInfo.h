

#ifndef StructureStubInfo_h
#define StructureStubInfo_h

#if ENABLE(JIT)

#include "Instruction.h"
#include "MacroAssembler.h"
#include "Opcode.h"
#include "Structure.h"

namespace JSC {

    enum AccessType {
        access_get_by_id_self,
        access_get_by_id_proto,
        access_get_by_id_chain,
        access_get_by_id_self_list,
        access_get_by_id_proto_list,
        access_put_by_id_transition,
        access_put_by_id_replace,
        access_get_by_id,
        access_put_by_id,
        access_get_by_id_generic,
        access_put_by_id_generic,
        access_get_array_length,
        access_get_string_length,
    };

    struct StructureStubInfo {
        StructureStubInfo(AccessType accessType)
            : accessType(accessType)
            , seen(false)
        {
        }

        void initGetByIdSelf(Structure* baseObjectStructure)
        {
            accessType = access_get_by_id_self;

            u.getByIdSelf.baseObjectStructure = baseObjectStructure;
            baseObjectStructure->ref();
        }

        void initGetByIdProto(Structure* baseObjectStructure, Structure* prototypeStructure)
        {
            accessType = access_get_by_id_proto;

            u.getByIdProto.baseObjectStructure = baseObjectStructure;
            baseObjectStructure->ref();

            u.getByIdProto.prototypeStructure = prototypeStructure;
            prototypeStructure->ref();
        }

        void initGetByIdChain(Structure* baseObjectStructure, StructureChain* chain)
        {
            accessType = access_get_by_id_chain;

            u.getByIdChain.baseObjectStructure = baseObjectStructure;
            baseObjectStructure->ref();

            u.getByIdChain.chain = chain;
            chain->ref();
        }

        void initGetByIdSelfList(PolymorphicAccessStructureList* structureList, int listSize)
        {
            accessType = access_get_by_id_self_list;

            u.getByIdProtoList.structureList = structureList;
            u.getByIdProtoList.listSize = listSize;
        }

        void initGetByIdProtoList(PolymorphicAccessStructureList* structureList, int listSize)
        {
            accessType = access_get_by_id_proto_list;

            u.getByIdProtoList.structureList = structureList;
            u.getByIdProtoList.listSize = listSize;
        }

        // PutById*

        void initPutByIdTransition(Structure* previousStructure, Structure* structure, StructureChain* chain)
        {
            accessType = access_put_by_id_transition;

            u.putByIdTransition.previousStructure = previousStructure;
            previousStructure->ref();

            u.putByIdTransition.structure = structure;
            structure->ref();

            u.putByIdTransition.chain = chain;
            chain->ref();
        }

        void initPutByIdReplace(Structure* baseObjectStructure)
        {
            accessType = access_put_by_id_replace;
    
            u.putByIdReplace.baseObjectStructure = baseObjectStructure;
            baseObjectStructure->ref();
        }

        void deref();

        bool seenOnce()
        {
            return seen;
        }

        void setSeen()
        {
            seen = true;
        }

        int accessType : 31;
        int seen : 1;

        union {
            struct {
                Structure* baseObjectStructure;
            } getByIdSelf;
            struct {
                Structure* baseObjectStructure;
                Structure* prototypeStructure;
            } getByIdProto;
            struct {
                Structure* baseObjectStructure;
                StructureChain* chain;
            } getByIdChain;
            struct {
                PolymorphicAccessStructureList* structureList;
                int listSize;
            } getByIdSelfList;
            struct {
                PolymorphicAccessStructureList* structureList;
                int listSize;
            } getByIdProtoList;
            struct {
                Structure* previousStructure;
                Structure* structure;
                StructureChain* chain;
            } putByIdTransition;
            struct {
                Structure* baseObjectStructure;
            } putByIdReplace;
        } u;

        CodeLocationLabel stubRoutine;
        CodeLocationCall callReturnLocation;
        CodeLocationLabel hotPathBegin;
    };

} // namespace JSC

#endif

#endif // StructureStubInfo_h
