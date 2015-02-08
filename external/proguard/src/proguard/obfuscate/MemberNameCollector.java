
package proguard.obfuscate;

import proguard.classfile.*;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.MemberVisitor;

import java.util.Map;

public class MemberNameCollector
extends      SimplifiedVisitor
implements   MemberVisitor
{
    private final boolean allowAggressiveOverloading;
    private final Map     descriptorMap;


    /**
     * Creates a new MemberNameCollector.
     * @param allowAggressiveOverloading a flag that specifies whether class
     *                                   members can be overloaded aggressively.
     * @param descriptorMap              the map of descriptors to
     *                                   [new name - old name] maps.
     */
    public MemberNameCollector(boolean allowAggressiveOverloading,
                               Map     descriptorMap)
    {
        this.allowAggressiveOverloading = allowAggressiveOverloading;
        this.descriptorMap              = descriptorMap;
    }


    // Implementations for MemberVisitor.

    public void visitAnyMember(Clazz clazz, Member member)
    {
        // Special cases: <clinit> and <init> are always kept unchanged.
        // We can ignore them here.
        String name = member.getName(clazz);
        if (name.equals(ClassConstants.INTERNAL_METHOD_NAME_CLINIT) ||
            name.equals(ClassConstants.INTERNAL_METHOD_NAME_INIT))
        {
            return;
        }

        // Get the member's new name.
        String newName = MemberObfuscator.newMemberName(member);

        // Remember it, if it has already been set.
        if (newName != null)
        {
            // Get the member's descriptor.
            String descriptor = member.getDescriptor(clazz);

            // Check whether we're allowed to do aggressive overloading
            if (!allowAggressiveOverloading)
            {
                // Trim the return argument from the descriptor if not.
                // Works for fields and methods alike.
                descriptor = descriptor.substring(0, descriptor.indexOf(')')+1);
            }

            // Put the [descriptor - new name] in the map,
            // creating a new [new name - old name] map if necessary.
            Map nameMap = MemberObfuscator.retrieveNameMap(descriptorMap, descriptor);

            // Isn't there another original name for this new name, or should
            // this original name get priority?
            String otherName = (String)nameMap.get(newName);
            if (otherName == null                              ||
                MemberObfuscator.hasFixedNewMemberName(member) ||
                name.compareTo(otherName) < 0)
            {
                // Remember not to use the new name again in this name space.
                nameMap.put(newName, name);
            }
        }
    }
}
