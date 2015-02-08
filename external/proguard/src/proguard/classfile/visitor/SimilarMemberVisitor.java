
package proguard.classfile.visitor;

import proguard.classfile.*;

public class SimilarMemberVisitor
implements   MemberVisitor
{
    private final Clazz         targetClass;
    private final boolean       visitThisMember;
    private final boolean       visitSuperMembers;
    private final boolean       visitInterfaceMembers;
    private final boolean       visitOverridingMembers;
    private final MemberVisitor memberVisitor;


    /**
     * Creates a new SimilarMemberVisitor.
     * @param targetClass            the class in whose hierarchy to look for
     *                               the visited class members.
     * @param visitThisMember        specifies whether to visit the class
     *                               members in the target class itself.
     * @param visitSuperMembers      specifies whether to visit the class
     *                               members in the super classes of the target
     *                               class.
     * @param visitInterfaceMembers  specifies whether to visit the class
     *                               members in the interface classes of the
     *                               target class.
     * @param visitOverridingMembers specifies whether to visit the class
     *                               members in the subclasses of the target
     *                               class.
     * @param memberVisitor          the <code>MemberVisitor</code> to which
     *                               visits will be delegated.
     */
    public SimilarMemberVisitor(Clazz         targetClass,
                                boolean       visitThisMember,
                                boolean       visitSuperMembers,
                                boolean       visitInterfaceMembers,
                                boolean       visitOverridingMembers,
                                MemberVisitor memberVisitor)
    {
        this.targetClass            = targetClass;
        this.visitThisMember        = visitThisMember;
        this.visitSuperMembers      = visitSuperMembers;
        this.visitInterfaceMembers  = visitInterfaceMembers;
        this.visitOverridingMembers = visitOverridingMembers;
        this.memberVisitor          = memberVisitor;
    }


    // Implementations for MemberVisitor.

    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {
        targetClass.hierarchyAccept(visitThisMember,
                                    visitSuperMembers,
                                    visitInterfaceMembers,
                                    visitOverridingMembers,
                                    new NamedFieldVisitor(programField.getName(programClass),
                                                          programField.getDescriptor(programClass),
                                                          memberVisitor));
    }


    public void visitLibraryField(LibraryClass libraryClass, LibraryField libraryField)
    {
        targetClass.hierarchyAccept(visitThisMember,
                                    visitSuperMembers,
                                    visitInterfaceMembers,
                                    visitOverridingMembers,
                                    new NamedFieldVisitor(libraryField.getName(libraryClass),
                                                          libraryField.getDescriptor(libraryClass),
                                                          memberVisitor));
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        targetClass.hierarchyAccept(visitThisMember,
                                    visitSuperMembers,
                                    visitInterfaceMembers,
                                    visitOverridingMembers,
                                    new NamedMethodVisitor(programMethod.getName(programClass),
                                                           programMethod.getDescriptor(programClass),
                                                           memberVisitor));
    }


    public void visitLibraryMethod(LibraryClass libraryClass, LibraryMethod libraryMethod)
    {
        targetClass.hierarchyAccept(visitThisMember,
                                    visitSuperMembers,
                                    visitInterfaceMembers,
                                    visitOverridingMembers,
                                    new NamedMethodVisitor(libraryMethod.getName(libraryClass),
                                                           libraryMethod.getDescriptor(libraryClass),
                                                           memberVisitor));
    }
}