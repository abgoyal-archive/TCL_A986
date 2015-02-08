
package proguard.classfile.visitor;

import proguard.classfile.*;
import proguard.classfile.util.SimplifiedVisitor;

public class MethodImplementationTraveler
extends      SimplifiedVisitor
implements   MemberVisitor
{
    private final boolean       visitThisMethod;
    private final boolean       visitSuperMethods;
    private final boolean       visitInterfaceMethods;
    private final boolean       visitOverridingMethods;
    private final MemberVisitor memberVisitor;


    /**
     * Creates a new MethodImplementationTraveler.
     * @param visitThisMethod        specifies whether to visit the originally
     *                               visited methods.
     * @param visitSuperMethods      specifies whether to visit the method in
     *                               the super classes.
     * @param visitInterfaceMethods  specifies whether to visit the method in
     *                               the interface classes.
     * @param visitOverridingMethods specifies whether to visit the method in
     *                               the subclasses.
     * @param memberVisitor          the <code>MemberVisitor</code> to which
     *                               visits will be delegated.
     */
    public MethodImplementationTraveler(boolean       visitThisMethod,
                                        boolean       visitSuperMethods,
                                        boolean       visitInterfaceMethods,
                                        boolean       visitOverridingMethods,
                                        MemberVisitor memberVisitor)
    {
        this.visitThisMethod        = visitThisMethod;
        this.visitSuperMethods      = visitSuperMethods;
        this.visitInterfaceMethods  = visitInterfaceMethods;
        this.visitOverridingMethods = visitOverridingMethods;
        this.memberVisitor          = memberVisitor;
    }


    // Implementations for MemberVisitor.

    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        if (visitThisMethod)
        {
            programMethod.accept(programClass, memberVisitor);
        }

        if (!isSpecial(programClass, programMethod))
        {
            programClass.hierarchyAccept(false,
                                         visitSuperMethods,
                                         visitInterfaceMethods,
                                         visitOverridingMethods,
                                         new NamedMethodVisitor(programMethod.getName(programClass),
                                                                programMethod.getDescriptor(programClass),
                                         new MemberAccessFilter(0,
                                                                ClassConstants.INTERNAL_ACC_PRIVATE |
                                                                ClassConstants.INTERNAL_ACC_STATIC,
                                         memberVisitor)));
        }
    }


    public void visitLibraryMethod(LibraryClass libraryClass, LibraryMethod libraryMethod)
    {
        if (visitThisMethod)
        {
            libraryMethod.accept(libraryClass, memberVisitor);
        }

        if (!isSpecial(libraryClass, libraryMethod))
        {
            libraryClass.hierarchyAccept(false,
                                         visitSuperMethods,
                                         visitInterfaceMethods,
                                         visitOverridingMethods,
                                         new NamedMethodVisitor(libraryMethod.getName(libraryClass),
                                                                libraryMethod.getDescriptor(libraryClass),
                                         new MemberAccessFilter(0,
                                                                ClassConstants.INTERNAL_ACC_PRIVATE |
                                                                ClassConstants.INTERNAL_ACC_STATIC,
                                         memberVisitor)));
        }
    }


    // Small utility methods.

    private boolean isSpecial(Clazz clazz, Method method)
    {
        return (method.getAccessFlags() &
                (ClassConstants.INTERNAL_ACC_PRIVATE |
                 ClassConstants.INTERNAL_ACC_STATIC)) != 0 ||
               method.getName(clazz).equals(ClassConstants.INTERNAL_METHOD_NAME_INIT);
    }
}
