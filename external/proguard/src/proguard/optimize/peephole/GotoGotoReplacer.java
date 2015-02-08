
package proguard.optimize.peephole;

import proguard.classfile.*;
import proguard.classfile.attribute.CodeAttribute;
import proguard.classfile.editor.CodeAttributeEditor;
import proguard.classfile.instruction.*;
import proguard.classfile.instruction.visitor.InstructionVisitor;
import proguard.classfile.util.SimplifiedVisitor;

public class GotoGotoReplacer
extends      SimplifiedVisitor
implements   InstructionVisitor
{
    private final CodeAttributeEditor codeAttributeEditor;
    private final InstructionVisitor  extraInstructionVisitor;


    /**
     * Creates a new GotoGotoReplacer.
     * @param codeAttributeEditor     a code editor that can be used for
     *                                accumulating changes to the code.
     */
    public GotoGotoReplacer(CodeAttributeEditor codeAttributeEditor)
    {
        this(codeAttributeEditor, null);
    }


    /**
     * Creates a new GotoGotoReplacer.
     * @param codeAttributeEditor     a code editor that can be used for
     *                                accumulating changes to the code.
     * @param extraInstructionVisitor an optional extra visitor for all replaced
     *                                goto instructions.
     */
    public GotoGotoReplacer(CodeAttributeEditor codeAttributeEditor,
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
            // Check if the goto instruction points to another simple goto
            // instruction.
            int branchOffset = branchInstruction.branchOffset;
            int targetOffset = offset + branchOffset;

            if (branchOffset != branchInstruction.length(offset) &&
                !codeAttributeEditor.isModified(offset) &&
                !codeAttributeEditor.isModified(targetOffset))
            {
                Instruction targetInstruction =
                    InstructionFactory.create(codeAttribute.code, targetOffset);

                if (targetInstruction.opcode == InstructionConstants.OP_GOTO)
                {
                    // Simplify the goto instruction.
                    int targetBranchOffset = ((BranchInstruction)targetInstruction).branchOffset;

                    Instruction newBranchInstruction =
                         new BranchInstruction(opcode,
                                               (branchOffset + targetBranchOffset));
                    codeAttributeEditor.replaceInstruction(offset,
                                                           newBranchInstruction);

                    // Visit the instruction, if required.
                    if (extraInstructionVisitor != null)
                    {
                        extraInstructionVisitor.visitBranchInstruction(clazz, method, codeAttribute, offset, branchInstruction);
                    }
                }
            }
        }
    }
}
