
package proguard.evaluation;

import proguard.classfile.*;
import proguard.classfile.attribute.CodeAttribute;
import proguard.classfile.instruction.ConstantInstruction;
import proguard.evaluation.value.Value;

public interface InvocationUnit
{
    /**
     * Sets up the given variables for entering the given method.
     */
    public void enterMethod(Clazz     clazz,
                            Method    method,
                            Variables variables);


    /**
     * Exits the given method with the given return value.
     */
    public void exitMethod(Clazz  clazz,
                           Method method,
                           Value  returnValue);


    /**
     * Updates the given stack corresponding to the execution of the given
     * field or method reference instruction.
     */
    public void invokeMember(Clazz               clazz,
                             Method              method,
                             CodeAttribute       codeAttribute,
                             int                 offset,
                             ConstantInstruction constantInstruction,
                             Stack               stack);
}
