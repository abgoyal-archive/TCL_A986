
package proguard.optimize.peephole;

import proguard.classfile.constant.Constant;
import proguard.classfile.editor.CodeAttributeEditor;
import proguard.classfile.instruction.Instruction;
import proguard.classfile.instruction.visitor.*;

public class InstructionSequencesReplacer
extends      MultiInstructionVisitor
implements   InstructionVisitor
{
    private static final int PATTERN_INDEX     = 0;
    private static final int REPLACEMENT_INDEX = 1;


    /**
     * Creates a new InstructionSequencesReplacer.
     * @param patternConstants        any constants referenced by the pattern
     *                                instruction.
     * @param instructionSequences    the instruction sequences to be replaced,
     *                                with subsequently the sequence pair index,
     *                                the patten/replacement index (0 or 1),
     *                                and the instruction index in the sequence.
     * @param branchTargetFinder      a branch target finder that has been
     *                                initialized to indicate branch targets
     *                                in the visited code.
     * @param codeAttributeEditor     a code editor that can be used for
     *                                accumulating changes to the code.
     */
    public InstructionSequencesReplacer(Constant[]          patternConstants,
                                        Instruction[][][]   instructionSequences,
                                        BranchTargetFinder  branchTargetFinder,
                                        CodeAttributeEditor codeAttributeEditor)
    {
        this(patternConstants,
             instructionSequences,
             branchTargetFinder,
             codeAttributeEditor,
             null);
    }


    /**
     * Creates a new InstructionSequenceReplacer.
     * @param patternConstants        any constants referenced by the pattern
     *                                instruction.
     * @param instructionSequences    the instruction sequences to be replaced,
     *                                with subsequently the sequence pair index,
     *                                the patten/replacement index (0 or 1),
     *                                and the instruction index in the sequence.
     * @param branchTargetFinder      a branch target finder that has been
     *                                initialized to indicate branch targets
     *                                in the visited code.
     * @param codeAttributeEditor     a code editor that can be used for
     *                                accumulating changes to the code.
     * @param extraInstructionVisitor an optional extra visitor for all deleted
     *                                load instructions.
     */
    public InstructionSequencesReplacer(Constant[]          patternConstants,
                                        Instruction[][][]   instructionSequences,
                                        BranchTargetFinder  branchTargetFinder,
                                        CodeAttributeEditor codeAttributeEditor,
                                        InstructionVisitor  extraInstructionVisitor)
    {
        super(createInstructionSequenceReplacers(patternConstants,
                                                 instructionSequences,
                                                 branchTargetFinder,
                                                 codeAttributeEditor,
                                                 extraInstructionVisitor));
    }


    /**
     * Creates an array of InstructionSequenceReplacer instances.
     * @param patternConstants        any constants referenced by the pattern
     *                                instruction.
     * @param instructionSequences    the instruction sequences to be replaced,
     *                                with subsequently the sequence pair index,
     *                                the from/to index (0 or 1), and the
     *                                instruction index in the sequence.
     * @param branchTargetFinder      a branch target finder that has been
     *                                initialized to indicate branch targets
     *                                in the visited code.
     * @param codeAttributeEditor     a code editor that can be used for
     *                                accumulating changes to the code.
     * @param extraInstructionVisitor an optional extra visitor for all deleted
     *                                load instructions.
     */
    private static InstructionVisitor[] createInstructionSequenceReplacers(Constant[]          patternConstants,
                                                                           Instruction[][][]   instructionSequences,
                                                                           BranchTargetFinder  branchTargetFinder,
                                                                           CodeAttributeEditor codeAttributeEditor,
                                                                           InstructionVisitor  extraInstructionVisitor)
    {
        InstructionVisitor[] instructionSequenceReplacers =
            new InstructionSequenceReplacer[instructionSequences.length];

        for (int index = 0; index < instructionSequenceReplacers.length; index++)
        {
            Instruction[][] instructionSequencePair = instructionSequences[index];
            instructionSequenceReplacers[index] =
                new InstructionSequenceReplacer(patternConstants,
                                                instructionSequencePair[PATTERN_INDEX],
                                                instructionSequencePair[REPLACEMENT_INDEX],
                                                branchTargetFinder,
                                                codeAttributeEditor,
                                                extraInstructionVisitor);
        }

        return instructionSequenceReplacers;
    }
}
