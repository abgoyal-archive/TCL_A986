
package proguard.classfile.attribute;

import proguard.classfile.*;
import proguard.classfile.attribute.visitor.*;

public class LineNumberTableAttribute extends Attribute
{
    public int              u2lineNumberTableLength;
    public LineNumberInfo[] lineNumberTable;


    /**
     * Creates an uninitialized LineNumberTableAttribute.
     */
    public LineNumberTableAttribute()
    {
    }


    /**
     * Creates an initialized LineNumberTableAttribute.
     */
    public LineNumberTableAttribute(int              u2attributeNameIndex,
                                    int              u2lineNumberTableLength,
                                    LineNumberInfo[] lineNumberTable)
    {
        super(u2attributeNameIndex);

        this.u2lineNumberTableLength = u2lineNumberTableLength;
        this.lineNumberTable         = lineNumberTable;
    }


    /**
     * Returns the line number corresponding to the given byte code program
     * counter.
     */
    public int getLineNumber(int pc)
    {
        for (int index = u2lineNumberTableLength-1 ; index >= 0 ; index--)
        {
            LineNumberInfo info = lineNumberTable[index];
            if (pc >= info.u2startPC)
            {
                return info.u2lineNumber;
            }
        }

        return u2lineNumberTableLength > 0 ?
            lineNumberTable[0].u2lineNumber :
            0;
    }


    // Implementations for Attribute.

    public void accept(Clazz clazz, Method method, CodeAttribute codeAttribute, AttributeVisitor attributeVisitor)
    {
        attributeVisitor.visitLineNumberTableAttribute(clazz, method, codeAttribute, this);
    }


    /**
     * Applies the given visitor to all line numbers.
     */
    public void lineNumbersAccept(Clazz clazz, Method method, CodeAttribute codeAttribute, LineNumberInfoVisitor lineNumberInfoVisitor)
    {
        for (int index = 0; index < u2lineNumberTableLength; index++)
        {
            // We don't need double dispatching here, since there is only one
            // type of LineNumberInfo.
            lineNumberInfoVisitor.visitLineNumberInfo(clazz, method, codeAttribute, lineNumberTable[index]);
        }
    }
}
