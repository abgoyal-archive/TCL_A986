
package proguard.optimize.evaluation;

import proguard.classfile.Clazz;
import proguard.classfile.attribute.CodeAttribute;
import proguard.evaluation.BasicBranchUnit;
import proguard.evaluation.value.Value;

class   TracedBranchUnit
extends BasicBranchUnit
{
    // Implementations for BranchUnit.

    public void branchConditionally(Clazz         clazz,
                                    CodeAttribute codeAttribute,
                                    int           offset,
                                    int           branchTarget,
                                    int           conditional)
    {
        if      (conditional == Value.ALWAYS)
        {
            // Always branch.
            super.branch(clazz, codeAttribute, offset, branchTarget);
        }
        else if (conditional != Value.NEVER)
        {
            // Maybe branch.
            super.branchConditionally(clazz, codeAttribute, offset, branchTarget, conditional);
        }
        else
        {
            super.setCalled();
        }
    }
}
