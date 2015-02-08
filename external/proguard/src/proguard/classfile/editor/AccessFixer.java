
package proguard.classfile.editor;

import proguard.classfile.*;
import proguard.classfile.constant.*;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.util.*;
import proguard.classfile.visitor.*;

public class AccessFixer
extends      SimplifiedVisitor
implements   ConstantVisitor,
             ClassVisitor,
             MemberVisitor
{
    private MyReferencedClassFinder referencedClassFinder = new MyReferencedClassFinder();

    private Clazz referencingClass;
    private Clazz referencedClass;


    // Implementations for ConstantVisitor.

    public void visitAnyConstant(Clazz clazz, Constant constant) {}


    public void visitStringConstant(Clazz clazz, StringConstant stringConstant)
    {
        referencingClass = clazz;
        referencedClass  = stringConstant.referencedClass;

        // Make sure the access flags of the referenced class or class member,
        // if any, are acceptable.
        stringConstant.referencedClassAccept(this);
        stringConstant.referencedMemberAccept(this);
    }


    public void visitAnyRefConstant(Clazz clazz, RefConstant refConstant)
    {
        referencingClass = clazz;

        // Remember the specified class, since it might be different from
        // the referenced class that acutally contains the class member.
        clazz.constantPoolEntryAccept(refConstant.u2classIndex, referencedClassFinder);

        // Make sure the access flags of the referenced class member are
        // acceptable.
        refConstant.referencedMemberAccept(this);
    }


    public void visitClassConstant(Clazz clazz, ClassConstant classConstant)
    {
        referencingClass = clazz;

        // Make sure the access flags of the referenced class are acceptable.
        classConstant.referencedClassAccept(this);
    }


    // Implementations for ClassVisitor.

    public void visitLibraryClass(LibraryClass libraryClass) {}


    public void visitProgramClass(ProgramClass programClass)
    {
        int currentAccessFlags  = programClass.getAccessFlags();
        int currentAccessLevel  = AccessUtil.accessLevel(currentAccessFlags);

        // Compute the required access level.
        Clazz referencingClass = this.referencingClass;
        int requiredAccessLevel =
            inSamePackage(programClass, referencingClass) ? AccessUtil.PACKAGE_VISIBLE :
                                                            AccessUtil.PUBLIC;

        // Fix the class access flags if necessary.
        if (currentAccessLevel < requiredAccessLevel)
        {
            programClass.u2accessFlags =
                AccessUtil.replaceAccessFlags(currentAccessFlags,
                                              AccessUtil.accessFlags(requiredAccessLevel));
        }
    }


    // Implementations for MemberVisitor.

    public void visitLibraryMember(LibraryClass libraryClass, LibraryMember libraryMember) {}


    public void visitProgramMember(ProgramClass programClass, ProgramMember programMember)
    {
        int currentAccessFlags  = programMember.getAccessFlags();
        int currentAccessLevel  = AccessUtil.accessLevel(currentAccessFlags);

        // Compute the required access level.
        int requiredAccessLevel =
            programClass.equals(referencingClass)         ? AccessUtil.PRIVATE         :
            inSamePackage(programClass, referencingClass) ? AccessUtil.PACKAGE_VISIBLE :
            referencedClass.extends_(referencingClass) &&
            referencingClass.extends_(programClass)       ? AccessUtil.PROTECTED       :
                                                            AccessUtil.PUBLIC;

        // Fix the class member access flags if necessary.
        if (currentAccessLevel < requiredAccessLevel)
        {
            programMember.u2accessFlags =
                AccessUtil.replaceAccessFlags(currentAccessFlags,
                                              AccessUtil.accessFlags(requiredAccessLevel));
        }
    }


    /**
     * This ConstantVisitor returns the referenced class of the class constant
     * that it visits.
     */
    private class MyReferencedClassFinder
    extends       SimplifiedVisitor
    implements    ConstantVisitor
    {
        // Implementations for ConstantVisitor.
        public void visitClassConstant(Clazz clazz, ClassConstant classConstant)
        {
            referencedClass = classConstant.referencedClass;
        }
    }


    // Small utility methods.

    private boolean inSamePackage(ProgramClass class1, Clazz class2)
    {
        return ClassUtil.internalPackageName(class1.getName()).equals(
               ClassUtil.internalPackageName(class2.getName()));
    }
}
