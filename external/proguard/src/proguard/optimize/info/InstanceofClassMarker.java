
package proguard.optimize.info;

import proguard.classfile.*;
import proguard.classfile.attribute.CodeAttribute;
import proguard.classfile.constant.ClassConstant;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.instruction.*;
import proguard.classfile.instruction.visitor.InstructionVisitor;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.ClassVisitor;

public class InstanceofClassMarker
extends      SimplifiedVisitor
implements   InstructionVisitor,
             ConstantVisitor,
             ClassVisitor
{
    // Implementations for InstructionVisitor.

    public void visitAnyInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, Instruction instruction) {}


    public void visitConstantInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, ConstantInstruction constantInstruction)
    {
        if (constantInstruction.opcode == InstructionConstants.OP_INSTANCEOF)
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
        setInstanceofed(programClass);
    }


    // Small utility methods.

    private static void setInstanceofed(Clazz clazz)
    {
        ClassOptimizationInfo info = ClassOptimizationInfo.getClassOptimizationInfo(clazz);
        if (info != null)
        {
            info.setInstanceofed();
        }
    }


    public static boolean isInstanceofed(Clazz clazz)
    {
        ClassOptimizationInfo info = ClassOptimizationInfo.getClassOptimizationInfo(clazz);
        return info == null || info.isInstanceofed();
    }
}