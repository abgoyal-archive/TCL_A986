
package proguard.ant;

import org.apache.tools.ant.*;
import org.apache.tools.ant.types.*;
import proguard.*;
import proguard.util.ListUtil;

import java.io.File;

public class ClassPathElement extends Path
{
    private String filter;
    private String jarFilter;
    private String warFilter;
    private String earFilter;
    private String zipFilter;


    /**
     * @see Path#Path(Project)
     */
    public ClassPathElement(Project project)
    {
        super(project);
    }


    /**
     * Adds the contents of this class path element to the given class path.
     * @param classPath the class path to be extended.
     * @param output    specifies whether this is an output entry or not.
     */
    public void appendClassPathEntriesTo(ClassPath classPath, boolean output)
    {
        File     baseDir = getProject().getBaseDir();
        String[] fileNames;

        if (isReference())
        {
            // Get the referenced path or file set.
            Object referencedObject = getCheckedRef(DataType.class,
                                                    DataType.class.getName());

            if (referencedObject instanceof Path)
            {
                Path path = (Path)referencedObject;

                // Get the names of the files in the referenced path.
                fileNames = path.list();
            }
            else if (referencedObject instanceof AbstractFileSet)
            {
                AbstractFileSet fileSet = (AbstractFileSet)referencedObject;

                // Get the names of the existing input files in the referenced file set.
                DirectoryScanner scanner = fileSet.getDirectoryScanner(getProject());
                baseDir   = scanner.getBasedir();
                fileNames = scanner.getIncludedFiles();
            }
            else
            {
                throw new BuildException("The refid attribute doesn't point to a <path> element or a <fileset> element");
            }
        }
        else
        {
            // Get the names of the files in this path.
            fileNames = list();
        }

        if (output)
        {
            if (fileNames.length != 1)
            {
                throw new BuildException("The <outjar> element must specify exactly one file or directory ["+fileNames.length+"]");
            }
        }
        //else
        //{
        //    if (fileNames.length < 1)
        //    {
        //        throw new BuildException("The <injar> element must specify at least one file or directory");
        //    }
        //}

        for (int index = 0; index < fileNames.length; index++)
        {
            // Create a new class path entry, with the proper file name and
            // any filters.
            String fileName = fileNames[index];
            File   file     = new File(fileName);

            ClassPathEntry entry =
                new ClassPathEntry(file.isAbsolute() ? file : new File(baseDir, fileName),
                                   output);
            entry.setFilter(ListUtil.commaSeparatedList(filter));
            entry.setJarFilter(ListUtil.commaSeparatedList(jarFilter));
            entry.setWarFilter(ListUtil.commaSeparatedList(warFilter));
            entry.setEarFilter(ListUtil.commaSeparatedList(earFilter));
            entry.setZipFilter(ListUtil.commaSeparatedList(zipFilter));

            // Add it to the class path.
            classPath.add(entry);
        }
    }


    // Ant task attributes.

    /**
     * @deprecated Use {@link #setLocation(File)} instead.
     */
    public void setFile(File file)
    {
        setLocation(file);
    }


    /**
     * @deprecated Use {@link #setLocation(File)} instead.
     */
    public void setDir(File file)
    {
        setLocation(file);
    }


    /**
     * @deprecated Use {@link #setLocation(File)} instead.
     */
    public void setName(File file)
    {
        setLocation(file);
    }


    public void setFilter(String filter)
    {
        this.filter = filter;
    }


    public void setJarfilter(String jarFilter)
    {
        this.jarFilter = jarFilter;
    }


    public void setWarfilter(String warFilter)
    {
        this.warFilter = warFilter;
    }


    public void setEarfilter(String earFilter)
    {
        this.earFilter = earFilter;
    }


    public void setZipfilter(String zipFilter)
    {
        this.zipFilter = zipFilter;
    }
}
