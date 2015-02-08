
package proguard.evaluation;

import proguard.classfile.Clazz;
import proguard.classfile.attribute.CodeAttribute;
import proguard.evaluation.value.InstructionOffsetValue;

public class BasicBranchUnit
implements   BranchUnit
{
    private boolean                wasCalled;
    private InstructionOffsetValue traceBranchTargets;


    /**
     * Resets the flag that tells whether any of the branch unit commands was
     * called.
     */
    public void resetCalled()
    {
        wasCalled = false;
    }

    /**
     * Sets the flag that tells whether any of the branch unit commands was
     * called.
     */
    protected void setCalled()
    {
        wasCalled = true;
    }

    /**
     * Returns whether any of the branch unit commands was called.
     */
    public boolean wasCalled()
    {
        return wasCalled;
    }


    /**
     * Sets the initial branch targets, which will be updated as the branch
     * methods of the branch unit are called.
     */
    public void setTraceBranchTargets(InstructionOffsetValue branchTargets)
    {
        this.traceBranchTargets = branchTargets;
    }

    public InstructionOffsetValue getTraceBranchTargets()
    {
        return traceBranchTargets;
    }


    // Implementations for BranchUnit.

    public void branch(Clazz         clazz,
                       CodeAttribute codeAttribute,
                       int           offset,
                       int           branchTarget)
    {
        // Override the branch targets.
        traceBranchTargets = new InstructionOffsetValue(branchTarget);

        wasCalled = true;
    }


    public void branchConditionally(Clazz         clazz,
                                    CodeAttribute codeAttribute,
                                    int           offset,
                                    int           branchTarget,
                                    int           conditional)
    {
        // Accumulate the branch targets.
        traceBranchTargets =
            traceBranchTargets.generalize(new InstructionOffsetValue(branchTarget)).instructionOffsetValue();

        wasCalled = true;
    }


    public void returnFromMethod()
    {
        // Stop processing this block.
        traceBranchTargets = InstructionOffsetValue.EMPTY_VALUE;

        wasCalled = true;
    }


    public void throwException()
    {
        // Stop processing this block.
        traceBranchTargets = InstructionOffsetValue.EMPTY_VALUE;

        wasCalled = true;
    }
}
