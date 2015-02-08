

#include "config.h"
#include "StructureStubInfo.h"

namespace JSC {

#if ENABLE(JIT)
void StructureStubInfo::deref()
{
    switch (accessType) {
    case access_get_by_id_self:
        u.getByIdSelf.baseObjectStructure->deref();
        return;
    case access_get_by_id_proto:
        u.getByIdProto.baseObjectStructure->deref();
        u.getByIdProto.prototypeStructure->deref();
        return;
    case access_get_by_id_chain:
        u.getByIdChain.baseObjectStructure->deref();
        u.getByIdChain.chain->deref();
        return;
    case access_get_by_id_self_list: {
        PolymorphicAccessStructureList* polymorphicStructures = u.getByIdSelfList.structureList;
        polymorphicStructures->derefStructures(u.getByIdSelfList.listSize);
        delete polymorphicStructures;
        return;
    }
    case access_get_by_id_proto_list: {
        PolymorphicAccessStructureList* polymorphicStructures = u.getByIdProtoList.structureList;
        polymorphicStructures->derefStructures(u.getByIdProtoList.listSize);
        delete polymorphicStructures;
        return;
    }
    case access_put_by_id_transition:
        u.putByIdTransition.previousStructure->deref();
        u.putByIdTransition.structure->deref();
        u.putByIdTransition.chain->deref();
        return;
    case access_put_by_id_replace:
        u.putByIdReplace.baseObjectStructure->deref();
        return;
    case access_get_by_id:
    case access_put_by_id:
    case access_get_by_id_generic:
    case access_put_by_id_generic:
    case access_get_array_length:
    case access_get_string_length:
        // These instructions don't ref their Structures.
        return;
    default:
        ASSERT_NOT_REACHED();
    }
}
#endif

} // namespace JSC
