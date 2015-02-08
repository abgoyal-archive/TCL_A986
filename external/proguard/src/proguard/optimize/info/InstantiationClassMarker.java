
package proguard.optimize.info;

import proguard.classfile.*;
import proguard.classfile.attribute.CodeAttribute;
import proguard.classfile.constant.ClassConstant;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.instruction.*;
import proguard.classfile.instruction.visitor.InstructionVisitor;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.ClassVisitor;

public class InstantiationClassMarker
extends      SimplifiedVisitor
implements   InstructionVisitor,
             ConstantVisitor,
             ClassVisitor
{
    // Implementations for InstructionVisitor.

    public void visitAnyInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, Instruction instruction) {}


    public void visitConstantInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, ConstantInstruction constantInstruction)
    {
        if (constantInstruction.opcode == InstructionConstants.OP_NEW)
        {
            clazz.constantPoolEntryAccept(constantInstruction.constantIndex, this);
        }
    }


    // Implementations for ConstantVisitor.

    public void visitClassConstant(Clazz clazz, ClassConstant classConstant)
    {
        classConstant.referencedClassAccept(this);
    }


    // Implementations for ClassVisitor.

    public void visitLibraryClass(LibraryClass libraryClass) {}

    public void visitProgramClass(ProgramClass programClass)
    {
        setInstantiated(programClass);
    }


    // Small utility methods.

    private static void setInstantiated(Clazz clazz)
    {
        ClassOptimizationInfo info = ClassOptimizationInfo.getClassOptimizationInfo(clazz);
        if (info != null)
        {
            info.setInstantiated();
        }
    }


    public static boolean isInstantiated(Clazz clazz)
    {
        ClassOptimizationInfo info = ClassOptimizationInfo.getClassOptimizationInfo(clazz);
        return info == null || info.isInstantiated();
    }
}