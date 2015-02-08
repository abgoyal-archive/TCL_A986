
package proguard.classfile.editor;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.instruction.visitor.InstructionVisitor;
import proguard.classfile.instruction.*;
import proguard.classfile.util.SimplifiedVisitor;

public class InstructionAdder
extends      SimplifiedVisitor
implements   InstructionVisitor
{
    private final ConstantAdder         constantAdder;
    private final CodeAttributeComposer codeAttributeComposer;


    /**
     * Creates a new InstructionAdder that will copy classes into the given
     * target code attribute.
     */
    public InstructionAdder(ProgramClass          targetClass,
                            CodeAttributeComposer targetComposer)
    {
        constantAdder         = new ConstantAdder(targetClass);
        codeAttributeComposer = targetComposer;
    }


    // Implementations for InstructionVisitor.


    public void visitAnyInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, Instruction instruction)
    {
        // Add the instruction.
        codeAttributeComposer.appendInstruction(offset, instruction);
    }


    public void visitConstantInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, ConstantInstruction constantInstruction)
    {
        // Create a copy of the instruction.
        Instruction newConstantInstruction =
            new ConstantInstruction(constantInstruction.opcode,
                                    constantAdder.addConstant(clazz, constantInstruction.constantIndex),
                                    constantInstruction.constant).shrink();

        // Add the instruction.
        codeAttributeComposer.appendInstruction(offset, newConstantInstruction);
    }
}