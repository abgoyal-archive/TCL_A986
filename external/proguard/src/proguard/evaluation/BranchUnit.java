
package proguard.evaluation;

import proguard.classfile.Clazz;
import proguard.classfile.attribute.CodeAttribute;

public interface BranchUnit
{
    /**
     * Sets the new instruction offset.
     */
    public void branch(Clazz         clazz,
                       CodeAttribute codeAttribute,
                       int           offset,
                       int           branchTarget);


    /**
     * Sets the new instruction offset, depending on the certainty of the
     * conditional branch.
     */
    public void branchConditionally(Clazz         clazz,
                                    CodeAttribute codeAttribute,
                                    int           offset,
                                    int           branchTarget,
                                    int           conditional);


    /**
     * Returns from the method with the given value.
     */
    public void returnFromMethod();


    /**
     * Handles the throwing of an exception.
     */
    public void throwException();
}
