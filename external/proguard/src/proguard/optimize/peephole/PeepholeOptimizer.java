
package proguard.optimize.peephole;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.editor.CodeAttributeEditor;
import proguard.classfile.instruction.visitor.InstructionVisitor;
import proguard.classfile.util.SimplifiedVisitor;

public class PeepholeOptimizer
extends      SimplifiedVisitor
implements   AttributeVisitor
{
    private final BranchTargetFinder  branchTargetFinder;
    private final CodeAttributeEditor codeAttributeEditor;
    private final InstructionVisitor  instructionVisitor;


    /**
     * Creates a new PeepholeOptimizer.
     * @param codeAttributeEditor the code attribute editor that will be reset
     *                            and then executed.
     * @param instructionVisitor  the instruction visitor that performs
     *                            peephole optimizations using the above code
     *                            attribute editor.
     */
    public PeepholeOptimizer(CodeAttributeEditor codeAttributeEditor,
                             InstructionVisitor  instructionVisitor)
    {
        this(null, codeAttributeEditor, instructionVisitor);
    }


    /**
     * Creates a new PeepholeOptimizer.
     * @param branchTargetFinder  branch target finder that will be initialized
     *                            to indicate branch targets in the visited code.
     * @param codeAttributeEditor the code attribute editor that will be reset
     *                            and then executed.
     * @param instructionVisitor  the instruction visitor that performs
     *                            peephole optimizations using the above code
     *                            attribute editor.
     */
    public PeepholeOptimizer(BranchTargetFinder  branchTargetFinder,
                             CodeAttributeEditor codeAttributeEditor,
                             InstructionVisitor  instructionVisitor)
    {
        this.branchTargetFinder  = branchTargetFinder;
        this.codeAttributeEditor = codeAttributeEditor;
        this.instructionVisitor  = instructionVisitor;
    }


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute) {}


    public void visitCodeAttribute(Clazz clazz, Method method, CodeAttribute codeAttribute)
    {
        if (branchTargetFinder != null)
        {
            // Set up the branch target finder.
            branchTargetFinder.visitCodeAttribute(clazz, method, codeAttribute);
        }

        // Set up the code attribute editor.
        codeAttributeEditor.reset(codeAttribute.u4codeLength);

        // Find the peephole optimizations.
        codeAttribute.instructionsAccept(clazz, method, instructionVisitor);

        // Apply the peephole optimizations.
        codeAttributeEditor.visitCodeAttribute(clazz, method, codeAttribute);
    }
}
