
package proguard.classfile.visitor;

import proguard.classfile.*;
import proguard.classfile.util.SimplifiedVisitor;

import java.util.Set;

public class MemberCollector
extends      SimplifiedVisitor
implements   MemberVisitor
{
    private final Set set;


    /**
     * Creates a new MemberCollector.
     * @param set the <code>Set</code> in which all method names/descriptor
     *            strings will be collected.
     */
    public MemberCollector(Set set)
    {
        this.set = set;
    }


    // Implementations for MemberVisitor.


    public void visitAnyMember(Clazz clazz, Member member)
    {
        set.add(member.getName(clazz) + member.getDescriptor(clazz));
    }
}