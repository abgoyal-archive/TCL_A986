
package proguard.shrink;

import proguard.classfile.*;
import proguard.classfile.util.ClassUtil;
import proguard.classfile.visitor.*;

import java.io.PrintStream;


public class ShortestUsagePrinter
implements   ClassVisitor,
             MemberVisitor
{
    private final ShortestUsageMarker shortestUsageMarker;
    private final boolean             verbose;
    private final PrintStream         ps;


    /**
     * Creates a new UsagePrinter that prints verbosely to <code>System.out</code>.
     * @param shortestUsageMarker the usage marker that was used to mark the
     *                            classes and class members.
     */
    public ShortestUsagePrinter(ShortestUsageMarker shortestUsageMarker)
    {
        this(shortestUsageMarker, true);
    }


    /**
     * Creates a new UsagePrinter that prints to the given stream.
     * @param shortestUsageMarker the usage marker that was used to mark the
     *                            classes and class members.
     * @param verbose             specifies whether the output should be verbose.
     */
    public ShortestUsagePrinter(ShortestUsageMarker shortestUsageMarker,
                                boolean             verbose)
    {
        this(shortestUsageMarker, verbose, System.out);
    }

    /**
     * Creates a new UsagePrinter that prints to the given stream.
     * @param shortestUsageMarker the usage marker that was used to mark the
     *                            classes and class members.
     * @param verbose             specifies whether the output should be verbose.
     * @param printStream         the stream to which to print.
     */
    public ShortestUsagePrinter(ShortestUsageMarker shortestUsageMarker,
                                boolean             verbose,
                                PrintStream         printStream)
    {
        this.shortestUsageMarker = shortestUsageMarker;
        this.verbose             = verbose;
        this.ps                  = printStream;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        // Print the name of this class.
        ps.println(ClassUtil.externalClassName(programClass.getName()));

        // Print the reason for keeping this class.
        printReason(programClass);
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        // Print the name of this class.
        ps.println(ClassUtil.externalClassName(libraryClass.getName()));

        // Print the reason for keeping this class.
        ps.println("  is a library class.\n");
    }


    // Implementations for MemberVisitor.

    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {
        // Print the name of this field.
        String name = programField.getName(programClass);
        String type = programField.getDescriptor(programClass);

        ps.println(ClassUtil.externalClassName(programClass.getName()) +
                   (verbose ?
                        ": " + ClassUtil.externalFullFieldDescription(0, name, type):
                        "."  + name) +
                   lineNumberRange(programClass, programField));

        // Print the reason for keeping this method.
        printReason(programField);
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        // Print the name of this method.
        String name = programMethod.getName(programClass);
        String type = programMethod.getDescriptor(programClass);

        ps.println(ClassUtil.externalClassName(programClass.getName()) +
                   (verbose ?
                        ": " + ClassUtil.externalFullMethodDescription(programClass.getName(), 0, name, type):
                        "."  + name) +
                   lineNumberRange(programClass, programMethod));

        // Print the reason for keeping this method.
        printReason(programMethod);
    }


    public void visitLibraryField(LibraryClass libraryClass, LibraryField libraryField)
    {
        // Print the name of this field.
        String name = libraryField.getName(libraryClass);
        String type = libraryField.getDescriptor(libraryClass);

        ps.println(ClassUtil.externalClassName(libraryClass.getName()) +
                   (verbose ?
                        ": " + ClassUtil.externalFullFieldDescription(0, name, type):
                        "."  + name));

        // Print the reason for keeping this field.
        ps.println("  is a library field.\n");
    }


    public void visitLibraryMethod(LibraryClass libraryClass, LibraryMethod libraryMethod)
    {
        // Print the name of this method.
        String name = libraryMethod.getName(libraryClass);
        String type = libraryMethod.getDescriptor(libraryClass);

        ps.println(ClassUtil.externalClassName(libraryClass.getName()) +
                   (verbose ?
                        ": " + ClassUtil.externalFullMethodDescription(libraryClass.getName(), 0, name, type):
                        "."  + name));

        // Print the reason for keeping this method.
        ps.println("  is a library method.\n");
    }


    // Small utility methods.

    private void printReason(VisitorAccepter visitorAccepter)
    {
        if (shortestUsageMarker.isUsed(visitorAccepter))
        {
            ShortestUsageMark shortestUsageMark = shortestUsageMarker.getShortestUsageMark(visitorAccepter);

            // Print the reason for keeping this class.
            ps.print("  " + shortestUsageMark.getReason());

            // Print the class or method that is responsible, with its reasons.
            shortestUsageMark.acceptClassVisitor(this);
            shortestUsageMark.acceptMemberVisitor(this);
        }
        else
        {
            ps.println("  is not being kept.\n");
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
            (" (" + range + ")") :
            "";
    }
}
