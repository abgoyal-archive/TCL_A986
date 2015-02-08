
package proguard.classfile.visitor;

import proguard.classfile.*;
import proguard.classfile.attribute.CodeAttribute;
import proguard.classfile.constant.*;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.instruction.*;
import proguard.classfile.instruction.visitor.InstructionVisitor;
import proguard.classfile.util.SimplifiedVisitor;


public class DotClassClassVisitor
extends      SimplifiedVisitor
implements   InstructionVisitor,
             ConstantVisitor
{
    private final ClassVisitor classVisitor;


    /**
     * Creates a new ClassHierarchyTraveler.
     * @param classVisitor the <code>ClassVisitor</code> to which visits will
     *                     be delegated.
     */
    public DotClassClassVisitor(ClassVisitor classVisitor)
    {
        this.classVisitor = classVisitor;
    }


    // Implementations for InstructionVisitor.

    public void visitAnyInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, Instruction instruction) {}


    public void visitConstantInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, ConstantInstruction constantInstruction)
    {
        byte opcode = constantInstruction.opcode;

        // Could this instruction be a .class construct?
        if (opcode == InstructionConstants.OP_LDC ||
            opcode == InstructionConstants.OP_LDC_W)
        {
            clazz.constantPoolEntryAccept(constantInstruction.constantIndex,
                                          this);
        }
    }


    // Implementations for ConstantVisitor.

    public void visitAnyConstant(Clazz clazz, Constant constant) {}


    public void visitClassConstant(Clazz clazz, ClassConstant classConstant)
    {
        // Visit the referenced class from the .class construct.
        classConstant.referencedClassAccept(classVisitor);
    }
}
