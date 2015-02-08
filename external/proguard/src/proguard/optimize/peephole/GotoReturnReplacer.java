
package proguard.optimize.peephole;

import proguard.classfile.*;
import proguard.classfile.attribute.CodeAttribute;
import proguard.classfile.editor.CodeAttributeEditor;
import proguard.classfile.instruction.*;
import proguard.classfile.instruction.visitor.InstructionVisitor;
import proguard.classfile.util.SimplifiedVisitor;

public class GotoReturnReplacer
extends      SimplifiedVisitor
implements   InstructionVisitor
{
    private final CodeAttributeEditor codeAttributeEditor;
    private final InstructionVisitor  extraInstructionVisitor;


    /**
     * Creates a new GotoReturnReplacer.
     * @param codeAttributeEditor     a code editor that can be used for
     *                                accumulating changes to the code.
     */
    public GotoReturnReplacer(CodeAttributeEditor codeAttributeEditor)
    {
        this(codeAttributeEditor, null);
    }


    /**
     * Creates a new GotoReturnReplacer.
     * @param codeAttributeEditor     a code editor that can be used for
     *                                accumulating changes to the code.
     * @param extraInstructionVisitor an optional extra visitor for all replaced
     *                                goto instructions.
     */
    public GotoReturnReplacer(CodeAttributeEditor codeAttributeEditor,
                              InstructionVisitor  extraInstructionVisitor)
    {
        this.codeAttributeEditor     = codeAttributeEditor;
        this.extraInstructionVisitor = extraInstructionVisitor;
    }


    // Implementations for InstructionVisitor.

    public void visitAnyInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, Instruction instruction) {}


    public void visitBranchInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, BranchInstruction branchInstruction)
    {
        // Check if the instruction is an unconditional goto instruction.
        byte opcode = branchInstruction.opcode;
        if (opcode == InstructionConstants.OP_GOTO ||
            opcode == InstructionConstants.OP_GOTO_W)
        {
            // Check if the goto instruction points to a return instruction.
            int targetOffset = offset + branchInstruction.branchOffset;

            if (!codeAttributeEditor.isModified(offset) &&
                !codeAttributeEditor.isModified(targetOffset))
            {
                Instruction targetInstruction = InstructionFactory.create(codeAttribute.code,
                                                                          targetOffset);
                switch (targetInstruction.opcode)
                {
                    case InstructionConstants.OP_IRETURN:
                    case InstructionConstants.OP_LRETURN:
                    case InstructionConstants.OP_FRETURN:
                    case InstructionConstants.OP_DRETURN:
                    case InstructionConstants.OP_ARETURN:
                    case InstructionConstants.OP_RETURN:
                        // Replace the goto instruction by the return instruction.
                        Instruction returnInstruction =
                             new SimpleInstruction(targetInstruction.opcode);
                        codeAttributeEditor.replaceInstruction(offset,
                                                               returnInstruction);

                        // Visit the instruction, if required.
                        if (extraInstructionVisitor != null)
                        {
                            extraInstructionVisitor.visitBranchInstruction(clazz, method, codeAttribute, offset, branchInstruction);
                        }

                        break;
                }
            }
        }
    }
}
