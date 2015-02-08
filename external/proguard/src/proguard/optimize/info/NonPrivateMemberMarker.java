
package proguard.optimize.info;

import proguard.classfile.*;
import proguard.classfile.constant.*;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.*;

public class NonPrivateMemberMarker
extends      SimplifiedVisitor
implements   ClassVisitor,
             ConstantVisitor,
             MemberVisitor
{
    private final MethodImplementationFilter filteredMethodMarker = new MethodImplementationFilter(this);


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        // Mark all referenced class members in different classes.
        programClass.constantPoolEntriesAccept(this);

        // Explicitly mark the <clinit> method.
        programClass.methodAccept(ClassConstants.INTERNAL_METHOD_NAME_CLINIT,
                                  ClassConstants.INTERNAL_METHOD_TYPE_CLINIT,
                                  this);

        // Explicitly mark the parameterless <init> method.
        programClass.methodAccept(ClassConstants.INTERNAL_METHOD_NAME_INIT,
                                  ClassConstants.INTERNAL_METHOD_TYPE_INIT,
                                  this);

        // Mark all methods that may have implementations.
        programClass.methodsAccept(filteredMethodMarker);
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        // Go over all methods.
        libraryClass.methodsAccept(this);
    }


    // Implementations for ConstantVisitor.

    public void visitAnyConstant(Clazz clazz, Constant constant) {}


    public void visitStringConstant(Clazz clazz, StringConstant stringConstant)
    {
        Clazz referencedClass = stringConstant.referencedClass;

        // Is it refering to another class or class member?
        if (referencedClass != null &&
            !referencedClass.equals(clazz))
        {
            // The referenced class member, if any, can never be made private.
            stringConstant.referencedMemberAccept(this);
        }
    }


    public void visitAnyRefConstant(Clazz clazz, RefConstant refConstant)
    {
        Clazz referencedClass = refConstant.referencedClass;

        // Is it refering to a class member in another class?
        // The class member might be in another class, or
        // it may be referenced through another class.
        if (referencedClass != null &&
            !referencedClass.equals(clazz) ||
            !refConstant.getClassName(clazz).equals(clazz.getName()))
        {
            // The referenced class member can never be made private.
            refConstant.referencedMemberAccept(this);
        }
    }


    // Implementations for MemberVisitor.

    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {
        markCanNotBeMadePrivate(programField);
    }


    public void visitLibraryField(LibraryClass libraryClass, LibraryField libraryField)
    {
        markCanNotBeMadePrivate(libraryField);
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        markCanNotBeMadePrivate(programMethod);
    }


    public void visitLibraryMethod(LibraryClass libraryClass, LibraryMethod libraryMethod)
    {
        markCanNotBeMadePrivate(libraryMethod);
    }


    // Small utility methods.

    private static void markCanNotBeMadePrivate(Field field)
    {
        FieldOptimizationInfo info = FieldOptimizationInfo.getFieldOptimizationInfo(field);
        if (info != null)
        {
            info.setCanNotBeMadePrivate();
        }
    }


    /**
     * Returns whether the given field can be made private.
     */
    public static boolean canBeMadePrivate(Field field)
    {
        FieldOptimizationInfo info = FieldOptimizationInfo.getFieldOptimizationInfo(field);
        return info != null &&
               info.canBeMadePrivate();
    }


    private static void markCanNotBeMadePrivate(Method method)
    {
        MethodOptimizationInfo info = MethodOptimizationInfo.getMethodOptimizationInfo(method);
        if (info != null)
        {
            info.setCanNotBeMadePrivate();
        }
    }


    /**
     * Returns whether the given method can be made private.
     */
    public static boolean canBeMadePrivate(Method method)
    {
        MethodOptimizationInfo info = MethodOptimizationInfo.getMethodOptimizationInfo(method);
        return info != null &&
               info.canBeMadePrivate();
    }
}
