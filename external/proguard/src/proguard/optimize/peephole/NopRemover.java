
package proguard.optimize.peephole;

import proguard.classfile.*;
import proguard.classfile.attribute.CodeAttribute;
import proguard.classfile.editor.CodeAttributeEditor;
import proguard.classfile.instruction.*;
import proguard.classfile.instruction.visitor.InstructionVisitor;
import proguard.classfile.util.SimplifiedVisitor;

public class NopRemover
extends      SimplifiedVisitor
implements   InstructionVisitor
{
    private final CodeAttributeEditor codeAttributeEditor;
    private final InstructionVisitor  extraInstructionVisitor;


    /**
     * Creates a new NopRemover.
     * @param codeAttributeEditor a code editor that can be used for
     *                            accumulating changes to the code.
     */
    public NopRemover(CodeAttributeEditor codeAttributeEditor)
    {
        this(codeAttributeEditor, null);
    }


    /**
     * Creates a new NopRemover.
     * @param codeAttributeEditor     a code editor that can be used for
     *                                accumulating changes to the code.
     * @param extraInstructionVisitor an optional extra visitor for all removed
     *                                nop instructions.
     */
    public NopRemover(CodeAttributeEditor codeAttributeEditor,
                      InstructionVisitor  extraInstructionVisitor)
    {
        this.codeAttributeEditor     = codeAttributeEditor;
        this.extraInstructionVisitor = extraInstructionVisitor;
    }


    // Implementations for InstructionVisitor.

    public void visitAnyInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, Instruction instruction) {}


    public void visitSimpleInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, SimpleInstruction simpleInstruction)
    {
        // Check if the instruction is a nop instruction.
        if (simpleInstruction.opcode == InstructionConstants.OP_NOP &&
            !codeAttributeEditor.isModified(offset))
        {
            codeAttributeEditor.deleteInstruction(offset);

            // Visit the instruction, if required.
            if (extraInstructionVisitor != null)
            {
                extraInstructionVisitor.visitSimpleInstruction(clazz, method, codeAttribute, offset, simpleInstruction);
            }
        }
    }
}
