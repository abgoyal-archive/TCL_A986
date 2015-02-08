
package proguard.optimize.peephole;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.editor.VariableEditor;
import proguard.classfile.util.*;
import proguard.classfile.visitor.MemberVisitor;
import proguard.optimize.*;
import proguard.optimize.info.*;

public class VariableShrinker
extends      SimplifiedVisitor
implements   AttributeVisitor
{
    private static final boolean DEBUG = false;


    private final MemberVisitor extraVariableMemberVisitor;

    private final VariableUsageMarker variableUsageMarker = new VariableUsageMarker();
    private final VariableEditor      variableEditor      = new VariableEditor();


    /**
     * Creates a new VariableShrinker.
     */
    public VariableShrinker()
    {
        this(null);
    }


    /**
     * Creates a new VariableShrinker with an extra visitor.
     * @param extraVariableMemberVisitor an optional extra visitor for all
     *                                   removed variables.
     */
    public VariableShrinker(MemberVisitor extraVariableMemberVisitor)
    {
        this.extraVariableMemberVisitor = extraVariableMemberVisitor;
    }


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute) {}


    public void visitCodeAttribute(Clazz clazz, Method method, CodeAttribute codeAttribute)
    {
        if ((method.getAccessFlags() & ClassConstants.INTERNAL_ACC_ABSTRACT) == 0)
        {
            // Compute the parameter size.
            int parameterSize =
                ClassUtil.internalMethodParameterSize(method.getDescriptor(clazz),
                                                      method.getAccessFlags());

            // Get the total size of the local variable frame.
            int maxLocals = codeAttribute.u2maxLocals;

            if (DEBUG)
            {
                System.out.println("VariableShrinker: "+clazz.getName()+"."+method.getName(clazz)+method.getDescriptor(clazz));
                System.out.println("  Parameter size = " + parameterSize);
                System.out.println("  Max locals     = " + maxLocals);
            }

            // Figure out the local variables that are used by the code.
            variableUsageMarker.visitCodeAttribute(clazz, method, codeAttribute);

            // Delete unused local variables from the local variable frame.
            variableEditor.reset(maxLocals);

            for (int variableIndex = parameterSize; variableIndex < maxLocals; variableIndex++)
            {
                // Is the variable not required?
                if (!variableUsageMarker.isVariableUsed(variableIndex))
                {
                    if (DEBUG)
                    {
                        System.out.println("  Deleting local variable #"+variableIndex);
                    }

                    // Delete the unused variable.
                    variableEditor.deleteVariable(variableIndex);

                    // Visit the method, if required.
                    if (extraVariableMemberVisitor != null)
                    {
                        method.accept(clazz, extraVariableMemberVisitor);
                    }
                }
            }

            // Shift all remaining parameters and variables in the byte code.
            variableEditor.visitCodeAttribute(clazz, method, codeAttribute);
        }
    }
}
