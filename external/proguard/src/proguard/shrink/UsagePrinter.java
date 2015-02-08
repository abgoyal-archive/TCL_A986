
package proguard.shrink;

import proguard.classfile.*;
import proguard.classfile.util.*;
import proguard.classfile.visitor.*;

import java.io.PrintStream;


public class UsagePrinter
extends      SimplifiedVisitor
implements   ClassVisitor,
             MemberVisitor
{
    private final UsageMarker usageMarker;
    private final boolean     printUnusedItems;
    private final PrintStream ps;

    // A field to remember the class name, if a header is needed for class members.
    private String      className;


    /**
     * Creates a new UsagePrinter that prints to <code>System.out</code>.
     * @param usageMarker      the usage marker that was used to mark the
     *                         classes and class members.
     * @param printUnusedItems a flag that indicates whether only unused items
     *                         should be printed, or alternatively, only used
     *                         items.
     */
    public UsagePrinter(UsageMarker usageMarker,
                        boolean     printUnusedItems)
    {
        this(usageMarker, printUnusedItems, System.out);
    }


    /**
     * Creates a new UsagePrinter that prints to the given stream.
     * @param usageMarker      the usage marker that was used to mark the
     *                         classes and class members.
     * @param printUnusedItems a flag that indicates whether only unused items
     *                         should be printed, or alternatively, only used
     *                         items.
     * @param printStream      the stream to which to print.
     */
    public UsagePrinter(UsageMarker usageMarker,
                        boolean     printUnusedItems,
                        PrintStream printStream)
    {
        this.usageMarker      = usageMarker;
        this.printUnusedItems = printUnusedItems;
        this.ps               = printStream;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        if (usageMarker.isUsed(programClass))
        {
            if (printUnusedItems)
            {
                className = programClass.getName();

                programClass.fieldsAccept(this);
                programClass.methodsAccept(this);

                className = null;
            }
            else
            {
                ps.println(ClassUtil.externalClassName(programClass.getName()));
            }
        }
        else
        {
            if (printUnusedItems)
            {
                ps.println(ClassUtil.externalClassName(programClass.getName()));
            }
        }
    }


    // Implementations for MemberVisitor.

    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {
        if (usageMarker.isUsed(programField) ^ printUnusedItems)
        {
            printClassNameHeader();

            ps.println("    " +
                       lineNumberRange(programClass, programField) +
                       ClassUtil.externalFullFieldDescription(
                           programField.getAccessFlags(),
                           programField.getName(programClass),
                           programField.getDescriptor(programClass)));
        }
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        if (usageMarker.isUsed(programMethod) ^ printUnusedItems)
        {
            printClassNameHeader();

            ps.println("    " +
                       lineNumberRange(programClass, programMethod) +
                       ClassUtil.externalFullMethodDescription(
                           programClass.getName(),
                           programMethod.getAccessFlags(),
                           programMethod.getName(programClass),
                           programMethod.getDescriptor(programClass)));
        }
    }


    // Small utility methods.

    /**
     * Prints the class name field. The field is then cleared, so it is not
     * printed again.
     */
    private void printClassNameHeader()
    {
        if (className != null)
        {
            ps.println(ClassUtil.externalClassName(className) + ":");
            className = null;
        }
    }


    /**
     * Returns the line number range of the given class member, followed by a
     * colon, or just an empty String if no range is available.
     */
    private static String lineNumberRange(ProgramClass programClass, ProgramMember programMember)
    {
        String range = programMember.getLineNumberRange(programClass);
        return range != null ?
            (range + ":") :
            "";
    }
}
