
package proguard.classfile.editor;

import proguard.classfile.attribute.visitor.LineNumberInfoVisitor;
import proguard.classfile.attribute.*;
import proguard.classfile.*;

public class LineNumberInfoAdder
implements   LineNumberInfoVisitor
{
    private final LineNumberTableAttributeEditor lineNumberTableAttributeEditor;


    /**
     * Creates a new LineNumberInfoAdder that will copy line numbers into the
     * given target line number table.
     */
    public LineNumberInfoAdder(LineNumberTableAttribute targetLineNumberTableAttribute)
    {
        this.lineNumberTableAttributeEditor = new LineNumberTableAttributeEditor(targetLineNumberTableAttribute);
    }


    // Implementations for LineNumberInfoVisitor.

    public void visitLineNumberInfo(Clazz clazz, Method method, CodeAttribute codeAttribute, LineNumberInfo lineNumberInfo)
    {
        // Create a new line number.
        LineNumberInfo newLineNumberInfo =
            new LineNumberInfo(lineNumberInfo.u2startPC,
                               lineNumberInfo.u2lineNumber);

        // Add it to the target.
        lineNumberTableAttributeEditor.addLineNumberInfo(newLineNumberInfo);
    }
}
