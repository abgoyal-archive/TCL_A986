
package proguard.optimize.info;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.instruction.*;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.*;

public class SideEffectMethodMarker
extends      SimplifiedVisitor
implements   ClassPoolVisitor,
             ClassVisitor,
             MemberVisitor,
             AttributeVisitor
{
    // A reusable object for checking whether instructions have side effects.
    private final SideEffectInstructionChecker sideEffectInstructionChecker = new SideEffectInstructionChecker(false);

    // Parameters and values for visitor methods.
    private int     newSideEffectCount;
    private boolean hasSideEffects;


    // Implementations for ClassPoolVisitor.

    public void visitClassPool(ClassPool classPool)
    {
        // Go over all classes and their methods, marking if they have side
        // effects, until no new cases can be found.
        do
        {
            newSideEffectCount = 0;

            // Go over all classes and their methods once.
            classPool.classesAccept(this);
        }
        while (newSideEffectCount > 0);
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        // Go over all methods.
        programClass.methodsAccept(this);
    }


    // Implementations for MemberVisitor.

    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        if (!hasSideEffects(programMethod) &&
            !NoSideEffectMethodMarker.hasNoSideEffects(programMethod))
        {
            // Initialize the return value.
            hasSideEffects =
                (programMethod.getAccessFlags() &
                 (ClassConstants.INTERNAL_ACC_NATIVE |
                  ClassConstants.INTERNAL_ACC_SYNCHRONIZED)) != 0;

            // Look further if the method hasn't been marked yet.
            if (!hasSideEffects)
            {
                // Investigate the actual code.
                programMethod.attributesAccept(programClass, this);
            }

            // Mark the method depending on the return value.
            if (hasSideEffects)
            {
                markSideEffects(programMethod);

                newSideEffectCount++;
            }
        }
    }


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute) {}


    public void visitCodeAttribute(Clazz clazz, Method method, CodeAttribute codeAttribute)
    {
        // Remember whether the code has any side effects.
        hasSideEffects = hasSideEffects(clazz, method, codeAttribute);
    }


    // Small utility methods.

    /**
     * Returns whether the given code has any side effects.
     */
    private boolean hasSideEffects(Clazz         clazz,
                                   Method        method,
                                   CodeAttribute codeAttribute)
    {
        byte[] code   = codeAttribute.code;
        int    length = codeAttribute.u4codeLength;

        // Go over all instructions.
        int offset = 0;
        do
        {
            // Get the current instruction.
            Instruction instruction = InstructionFactory.create(code, offset);

            // Check if it may be throwing exceptions.
            if (sideEffectInstructionChecker.hasSideEffects(clazz,
                                                            method,
                                                            codeAttribute,
                                                            offset,
                                                            instruction))
            {
                return true;
            }

            // Go to the next instruction.
            offset += instruction.length(offset);
        }
        while (offset < length);

        return false;
    }


    private static void markSideEffects(Method method)
    {
        MethodOptimizationInfo info = MethodOptimizationInfo.getMethodOptimizationInfo(method);
        if (info != null)
        {
            info.setSideEffects();
        }
    }


    public static boolean hasSideEffects(Method method)
    {
        MethodOptimizationInfo info = MethodOptimizationInfo.getMethodOptimizationInfo(method);
        return info == null ||
               info.hasSideEffects();
    }
}
