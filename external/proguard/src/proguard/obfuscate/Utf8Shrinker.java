
package proguard.obfuscate;

import proguard.classfile.*;
import proguard.classfile.constant.Constant;
import proguard.classfile.editor.ConstantPoolRemapper;
import proguard.classfile.visitor.ClassVisitor;


public class Utf8Shrinker implements ClassVisitor
{
    private int[]                constantIndexMap     = new int[ClassConstants.TYPICAL_CONSTANT_POOL_SIZE];
    private final ConstantPoolRemapper constantPoolRemapper = new ConstantPoolRemapper();


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        // Shift the used constant pool entries together, filling out the
        // index map.
        programClass.u2constantPoolCount =
            shrinkConstantPool(programClass.constantPool,
                               programClass.u2constantPoolCount);

        // Remap all constant pool references.
        constantPoolRemapper.setConstantIndexMap(constantIndexMap);
        constantPoolRemapper.visitProgramClass(programClass);
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
    }


    // Small utility methods.

    /**
     * Removes all UTF-8 entries that are not marked as being used
     * from the given constant pool.
     * @return the new number of entries.
     */
    private int shrinkConstantPool(Constant[] constantPool, int length)
    {
        // Create a new index map, if necessary.
        if (constantIndexMap.length < length)
        {
            constantIndexMap = new int[length];
        }

        int     counter = 1;
        boolean isUsed  = false;

        // Shift the used constant pool entries together.
        for (int index = 1; index < length; index++)
        {
            constantIndexMap[index] = counter;

            Constant constant = constantPool[index];

            // Don't update the flag if this is the second half of a long entry.
            if (constant != null)
            {
                isUsed = constant.getTag() != ClassConstants.CONSTANT_Utf8 ||
                         Utf8UsageMarker.isUsed(constant);
            }

            if (isUsed)
            {
                constantPool[counter++] = constant;
            }
        }

        // Clear the remaining constant pool elements.
        for (int index = counter; index < length; index++)
        {
            constantPool[index] = null;
        }

        return counter;
    }
}
