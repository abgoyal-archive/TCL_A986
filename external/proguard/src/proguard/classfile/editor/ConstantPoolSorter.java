
package proguard.classfile.editor;

import proguard.classfile.*;
import proguard.classfile.constant.Constant;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.ClassVisitor;

import java.util.Arrays;

public class ConstantPoolSorter
extends      SimplifiedVisitor
implements   ClassVisitor
{
    private int[]                constantIndexMap       = new int[ClassConstants.TYPICAL_CONSTANT_POOL_SIZE];
    private ComparableConstant[] comparableConstantPool = new ComparableConstant[ClassConstants.TYPICAL_CONSTANT_POOL_SIZE];
    private Constant[]           newConstantPool        = new Constant[ClassConstants.TYPICAL_CONSTANT_POOL_SIZE];

    private final ConstantPoolRemapper constantPoolRemapper = new ConstantPoolRemapper();


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        int constantPoolCount = programClass.u2constantPoolCount;

        // Sort the constant pool and set up an index map.
        if (constantIndexMap.length < constantPoolCount)
        {
            constantIndexMap       = new int[constantPoolCount];
            comparableConstantPool = new ComparableConstant[constantPoolCount];
            newConstantPool        = new Constant[constantPoolCount];
        }

        // Initialize an array whose elements can be compared.
        int sortLength = 0;
        for (int oldIndex = 1; oldIndex < constantPoolCount; oldIndex++)
        {
            Constant constant = programClass.constantPool[oldIndex];
            if (constant != null)
            {
                comparableConstantPool[sortLength++] =
                    new ComparableConstant(programClass, oldIndex, constant);
            }
        }

        // Sort the array.
        Arrays.sort(comparableConstantPool, 0, sortLength);

        // Save the sorted elements.
        int newLength = 1;
        int newIndex  = 1;
        ComparableConstant previousComparableConstant = null;
        for (int sortIndex = 0; sortIndex < sortLength; sortIndex++)
        {
            ComparableConstant comparableConstant = comparableConstantPool[sortIndex];

            // Isn't this a duplicate of the previous constant?
            if (!comparableConstant.equals(previousComparableConstant))
            {
                // Remember the index of the new entry.
                newIndex = newLength;

                // Copy the sorted constant pool entry over to the constant pool.
                Constant constant = comparableConstant.getConstant();

                newConstantPool[newLength++] = constant;

                // Long entries take up two slots, the second of which is null.
                int tag = constant.getTag();
                if (tag == ClassConstants.CONSTANT_Long ||
                    tag == ClassConstants.CONSTANT_Double)
                {
                    newConstantPool[newLength++] = null;
                }

                previousComparableConstant = comparableConstant;
            }

            // Fill out the map array.
            constantIndexMap[comparableConstant.getIndex()] = newIndex;
        }

        // Copy the new constant pool over.
        System.arraycopy(newConstantPool, 0, programClass.constantPool, 0, newLength);

        // Clear any remaining entries.
        for (int index = newLength; index < constantPoolCount; index++)
        {
            programClass.constantPool[index] = null;
        }

        programClass.u2constantPoolCount = newLength;

        // Remap all constant pool references.
        constantPoolRemapper.setConstantIndexMap(constantIndexMap);
        constantPoolRemapper.visitProgramClass(programClass);
    }
}
