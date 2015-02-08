
package proguard.optimize;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.editor.*;
import proguard.classfile.util.*;
import proguard.classfile.visitor.MemberVisitor;
import proguard.optimize.info.ParameterUsageMarker;

public class ParameterShrinker
extends      SimplifiedVisitor
implements   AttributeVisitor
{
    private static final boolean DEBUG = false;


    private final MemberVisitor extraVariableMemberVisitor;

    private final VariableRemapper variableRemapper = new VariableRemapper();


    /**
     * Creates a new ParameterShrinker.
     */
    public ParameterShrinker()
    {
        this(null);
    }


    /**
     * Creates a new ParameterShrinker with an extra visitor.
     * @param extraVariableMemberVisitor an optional extra visitor for all
     *                                   removed parameters.
     */
    public ParameterShrinker(MemberVisitor extraVariableMemberVisitor)
    {
        this.extraVariableMemberVisitor = extraVariableMemberVisitor;
    }


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute) {}


    public void visitCodeAttribute(Clazz clazz, Method method, CodeAttribute codeAttribute)
    {
        // Get the original parameter size that was saved.
        int oldParameterSize = ParameterUsageMarker.getParameterSize(method);

        // Compute the new parameter size from the shrunk descriptor.
        int newParameterSize =
            ClassUtil.internalMethodParameterSize(method.getDescriptor(clazz),
                                                  method.getAccessFlags());

        if (oldParameterSize > newParameterSize)
        {
            // Get the total size of the local variable frame.
            int maxLocals = codeAttribute.u2maxLocals;

            if (DEBUG)
            {
                System.out.println("ParameterShrinker: "+clazz.getName()+"."+method.getName(clazz)+method.getDescriptor(clazz));
                System.out.println("  Old parameter size = " + oldParameterSize);
                System.out.println("  New parameter size = " + newParameterSize);
                System.out.println("  Max locals         = " + maxLocals);
            }

            // Create a variable map.
            int[] variableMap = new int[maxLocals];

            // Move unused parameters right after the parameter block.
            int usedParameterIndex   = 0;
            int unusedParameterIndex = newParameterSize;
            for (int parameterIndex = 0; parameterIndex < oldParameterSize; parameterIndex++)
            {
                // Is the variable required as a parameter?
                if (ParameterUsageMarker.isParameterUsed(method, parameterIndex))
                {
                    // Keep the variable as a parameter.
                    variableMap[parameterIndex] = usedParameterIndex++;
                }
                else
                {
                    if (DEBUG)
                    {
                        System.out.println("  Deleting parameter #"+parameterIndex);
                    }

                    // Shift the variable to the unused parameter block,
                    // in case it is still used as a variable.
                    variableMap[parameterIndex] = unusedParameterIndex++;

                    // Visit the method, if required.
                    if (extraVariableMemberVisitor != null)
                    {
                        method.accept(clazz, extraVariableMemberVisitor);
                    }
                }
            }

            // Fill out the remainder of the map.
            for (int variableIndex = oldParameterSize; variableIndex < maxLocals; variableIndex++)
            {
                variableMap[variableIndex] = variableIndex;
            }

            // Set the map.
            variableRemapper.setVariableMap(variableMap);

            // Remap the variables.
            variableRemapper.visitCodeAttribute(clazz, method, codeAttribute);
        }
    }
}
