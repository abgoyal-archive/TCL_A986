
package proguard.classfile.editor;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.util.SimplifiedVisitor;

public class VariableEditor
extends      SimplifiedVisitor
implements   AttributeVisitor
{
    private boolean   modified;

    private boolean[] deleted     = new boolean[ClassConstants.TYPICAL_VARIABLES_SIZE];
    private int[]     variableMap = new int[ClassConstants.TYPICAL_VARIABLES_SIZE];

    private final VariableRemapper variableRemapper = new VariableRemapper();


    /**
     * Resets the accumulated code changes.
     * @param maxLocals the length of the local variable frame that will be
     *                  edited next.
     */
    public void reset(int maxLocals)
    {
        // Try to reuse the previous array.
        if (deleted.length < maxLocals)
        {
            deleted = new boolean[maxLocals];
        }
        else
        {
            for (int index = 0; index < maxLocals; index++)
            {
                deleted[index] = false;
            }
        }

        modified = false;
    }


    /**
     * Remembers to delete the given variable.
     * @param variableIndex the index of the variable to be deleted.
     */
    public void deleteVariable(int variableIndex)
    {
        deleted[variableIndex] = true;

        modified = true;
    }


    /**
     * Returns whether the given variable at the given offset has deleted.
     */
    public boolean isDeleted(int instructionOffset)
    {
        return deleted[instructionOffset];
    }


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute) {}


    public void visitCodeAttribute(Clazz clazz, Method method, CodeAttribute codeAttribute)
    {
        // Avoid doing any work if nothing is changing anyway.
        if (!modified)
        {
            return;
        }

        int oldMaxLocals = codeAttribute.u2maxLocals;

        // Make sure there is a sufficiently large variable map.
        if (variableMap.length < oldMaxLocals)
        {
            variableMap = new int[oldMaxLocals];
        }

        // Fill out the variable map.
        int newVariableIndex = 0;
        for (int oldVariableIndex = 0; oldVariableIndex < oldMaxLocals; oldVariableIndex++)
        {
            variableMap[oldVariableIndex] = deleted[oldVariableIndex] ?
                -1 : newVariableIndex++;
        }

        // Set the map.
        variableRemapper.setVariableMap(variableMap);

        // Remap the variables.
        variableRemapper.visitCodeAttribute(clazz, method, codeAttribute);

        // Update the length of local variable frame.
        codeAttribute.u2maxLocals = newVariableIndex;
    }
}
