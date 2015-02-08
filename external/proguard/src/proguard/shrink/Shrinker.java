
package proguard.shrink;

import proguard.*;
import proguard.classfile.ClassPool;
import proguard.classfile.attribute.visitor.*;
import proguard.classfile.visitor.*;

import java.io.*;

public class Shrinker
{
    private final Configuration configuration;


    /**
     * Creates a new Shrinker.
     */
    public Shrinker(Configuration configuration)
    {
        this.configuration = configuration;
    }


    /**
     * Performs shrinking of the given program class pool.
     */
    public ClassPool execute(ClassPool programClassPool,
                             ClassPool libraryClassPool) throws IOException
    {
        // Check if we have at least some keep commands.
        if (configuration.keep == null)
        {
            throw new IOException("You have to specify '-keep' options for the shrinking step.");
        }

        // Clean up any old visitor info.
        programClassPool.classesAccept(new ClassCleaner());
        libraryClassPool.classesAccept(new ClassCleaner());

        // Create a visitor for marking the seeds.
        UsageMarker usageMarker = configuration.whyAreYouKeeping == null ?
            new UsageMarker() :
            new ShortestUsageMarker();

        ClassPoolVisitor classPoolvisitor =
            ClassSpecificationVisitorFactory.createClassPoolVisitor(configuration.keep,
                                                                    usageMarker,
                                                                    usageMarker,
                                                                    true,
                                                                    false,
                                                                    false);
        // Mark the seeds.
        programClassPool.accept(classPoolvisitor);
        libraryClassPool.accept(classPoolvisitor);

        // Mark interfaces that have to be kept.
        programClassPool.classesAccept(new InterfaceUsageMarker(usageMarker));

        // Mark the inner class and annotation information that has to be kept.
        programClassPool.classesAccept(
            new UsedClassFilter(usageMarker,
            new AllAttributeVisitor(true,
            new MultiAttributeVisitor(new AttributeVisitor[]
            {
                new InnerUsageMarker(usageMarker),
                new AnnotationUsageMarker(usageMarker),
            }))));

        // Should we explain ourselves?
        if (configuration.whyAreYouKeeping != null)
        {
            System.out.println();

            // Create a visitor for explaining classes and class members.
            ShortestUsagePrinter shortestUsagePrinter =
                new ShortestUsagePrinter((ShortestUsageMarker)usageMarker,
                                         configuration.verbose);

            ClassPoolVisitor whyClassPoolvisitor =
                ClassSpecificationVisitorFactory.createClassPoolVisitor(configuration.whyAreYouKeeping,
                                                                        shortestUsagePrinter,
                                                                        shortestUsagePrinter);

            // Mark the seeds.
            programClassPool.accept(whyClassPoolvisitor);
            libraryClassPool.accept(whyClassPoolvisitor);
        }

        if (configuration.printUsage != null)
        {
            PrintStream ps = isFile(configuration.printUsage) ?
                new PrintStream(new BufferedOutputStream(new FileOutputStream(configuration.printUsage))) :
                System.out;

            // Print out items that will be removed.
            programClassPool.classesAcceptAlphabetically(
                new UsagePrinter(usageMarker, true, ps));

            if (ps != System.out)
            {
                ps.close();
            }
        }

        // Discard unused program classes.
        int originalProgramClassPoolSize = programClassPool.size();

        ClassPool newProgramClassPool = new ClassPool();
        programClassPool.classesAccept(
            new UsedClassFilter(usageMarker,
            new MultiClassVisitor(
            new ClassVisitor[] {
                new ClassShrinker(usageMarker),
                new ClassPoolFiller(newProgramClassPool)
            })));

        programClassPool.clear();

        // Check if we have at least some output classes.
        int newProgramClassPoolSize = newProgramClassPool.size();
        if (newProgramClassPoolSize == 0)
        {
            throw new IOException("The output jar is empty. Did you specify the proper '-keep' options?");
        }

        if (configuration.verbose)
        {
            System.out.println("Removing unused program classes and class elements...");
            System.out.println("  Original number of program classes: " + originalProgramClassPoolSize);
            System.out.println("  Final number of program classes:    " + newProgramClassPoolSize);
        }

        return newProgramClassPool;
    }


    /**
     * Returns whether the given file is actually a file, or just a placeholder
     * for the standard output.
     */
    private boolean isFile(File file)
    {
        return file.getPath().length() > 0;
    }
}
