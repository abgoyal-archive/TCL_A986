
package proguard.classfile.editor;

import proguard.classfile.attribute.*;

public class LineNumberTableAttributeEditor
{
    private LineNumberTableAttribute targetLineNumberTableAttribute;


    /**
     * Creates a new LineNumberTableAttributeEditor that will edit line numbers
     * in the given line number table attribute.
     */
    public LineNumberTableAttributeEditor(LineNumberTableAttribute targetLineNumberTableAttribute)
    {
        this.targetLineNumberTableAttribute = targetLineNumberTableAttribute;
    }


    /**
     * Adds a given line number to the line number table attribute.
     */
    public void addLineNumberInfo(LineNumberInfo lineNumberInfo)
    {
        int              lineNumberTableLength = targetLineNumberTableAttribute.u2lineNumberTableLength;
        LineNumberInfo[] lineNumberTable       = targetLineNumberTableAttribute.lineNumberTable;

        // Make sure there is enough space for the new lineNumberInfo.
        if (lineNumberTable.length <= lineNumberTableLength)
        {
            targetLineNumberTableAttribute.lineNumberTable = new LineNumberInfo[lineNumberTableLength+1];
            System.arraycopy(lineNumberTable, 0,
                             targetLineNumberTableAttribute.lineNumberTable, 0,
                             lineNumberTableLength);
            lineNumberTable = targetLineNumberTableAttribute.lineNumberTable;
        }

        // Add the lineNumberInfo.
        lineNumberTable[targetLineNumberTableAttribute.u2lineNumberTableLength++] = lineNumberInfo;
    }
}