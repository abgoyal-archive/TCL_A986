
package proguard.optimize.info;

import proguard.classfile.*;
import proguard.classfile.attribute.CodeAttribute;
import proguard.classfile.constant.*;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.instruction.*;
import proguard.classfile.instruction.visitor.InstructionVisitor;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.*;

public class AccessMethodMarker
extends      SimplifiedVisitor
implements   InstructionVisitor,
             ConstantVisitor,
             ClassVisitor,
             MemberVisitor
{
    private Method invokingMethod;


    // Implementations for InstructionVisitor.

    public void visitAnyInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, Instruction instruction) {}


    public void visitConstantInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, ConstantInstruction constantInstruction)
    {
        invokingMethod = method;

        clazz.constantPoolEntryAccept(constantInstruction.constantIndex, this);
    }


    // Implementations for ConstantVisitor.

    public void visitAnyConstant(Clazz clazz, Constant constant) {}


    public void visitStringConstant(Clazz clazz, StringConstant stringConstant)
    {
        // Check the referenced class or class member, if any.
       stringConstant.referencedClassAccept(this);
       stringConstant.referencedMemberAccept(this);
    }


    public void visitAnyRefConstant(Clazz clazz, RefConstant refConstant)
    {
        // Check the referenced class.
        clazz.constantPoolEntryAccept(refConstant.u2classIndex, this);

        // Check the referenced class member itself.
        refConstant.referencedClassAccept(this);
        refConstant.referencedMemberAccept(this);
    }


    public void visitClassConstant(Clazz clazz, ClassConstant classConstant)
    {
        // Check the referenced class.
       classConstant.referencedClassAccept(this);
    }


    // Implementations for ClassVisitor.

    public void visitAnyClass(Clazz clazz)
    {
        int accessFlags = clazz.getAccessFlags();

        if ((accessFlags & ClassConstants.INTERNAL_ACC_PUBLIC) == 0)
        {
            setAccessesPackageCode(invokingMethod);
        }
    }


    // Implementations for MemberVisitor.

    public void visitAnyMember(Clazz clazz, Member member)
    {
        int accessFlags = member.getAccessFlags();

        if      ((accessFlags & ClassConstants.INTERNAL_ACC_PRIVATE)   != 0)
        {
            setAccessesPrivateCode(invokingMethod);
        }
        else if ((accessFlags & ClassConstants.INTERNAL_ACC_PROTECTED) != 0)
        {
            setAccessesProtectedCode(invokingMethod);
        }
        else if ((accessFlags & ClassConstants.INTERNAL_ACC_PUBLIC)    == 0)
        {
            setAccessesPackageCode(invokingMethod);
        }
    }


    // Small utility methods.

    private static void setAccessesPrivateCode(Method method)
    {
        MethodOptimizationInfo info = MethodOptimizationInfo.getMethodOptimizationInfo(method);
        if (info != null)
        {
            info.setAccessesPrivateCode();
        }
    }


    /**
     * Returns whether the given method accesses private class members.
     */
    public static boolean accessesPrivateCode(Method method)
    {
        MethodOptimizationInfo info = MethodOptimizationInfo.getMethodOptimizationInfo(method);
        return info == null || info.accessesPrivateCode();
    }


    private static void setAccessesPackageCode(Method method)
    {
        MethodOptimizationInfo info = MethodOptimizationInfo.getMethodOptimizationInfo(method);
        if (info != null)
        {
            info.setAccessesPackageCode();
        }
    }


    /**
     * Returns whether the given method accesses package visible classes or class
     * members.
     */
    public static boolean accessesPackageCode(Method method)
    {
        MethodOptimizationInfo info = MethodOptimizationInfo.getMethodOptimizationInfo(method);
        return info == null || info.accessesPackageCode();
    }


    private static void setAccessesProtectedCode(Method method)
    {
        MethodOptimizationInfo info = MethodOptimizationInfo.getMethodOptimizationInfo(method);
        if (info != null)
        {
            info.setAccessesProtectedCode();
        }
    }


    /**
     * Returns whether the given method accesses protected class members.
     */
    public static boolean accessesProtectedCode(Method method)
    {
        MethodOptimizationInfo info = MethodOptimizationInfo.getMethodOptimizationInfo(method);
        return info == null || info.accessesProtectedCode();
    }
}
