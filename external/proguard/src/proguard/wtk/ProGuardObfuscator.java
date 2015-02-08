
package proguard.wtk;

import com.sun.kvem.environment.Obfuscator;
import proguard.*;

import java.io.*;


public class ProGuardObfuscator implements Obfuscator
{
    private static final String DEFAULT_CONFIGURATION = "default.pro";


    // Implementations for Obfuscator.

    public void createScriptFile(File jadFile,
                                 File projectDir)
    {
        // We don't really need to create a script file;
        // we'll just fill out all options in the run method.
    }


    public void run(File   obfuscatedJarFile,
                    String wtkBinDir,
                    String wtkLibDir,
                    String jarFileName,
                    String projectDirName,
                    String classPath,
                    String emptyAPI)
    throws IOException
    {
        // Create the ProGuard configuration.
        Configuration configuration = new Configuration();

        // Parse the default configuration file.
        ConfigurationParser parser = new ConfigurationParser(this.getClass().getResource(DEFAULT_CONFIGURATION));

        try
        {
            parser.parse(configuration);

            // Fill out the library class path.
            configuration.libraryJars = classPath(classPath);

            // Fill out the program class path (input and output).
            configuration.programJars = new ClassPath();
            configuration.programJars.add(new ClassPathEntry(new File(jarFileName), false));
            configuration.programJars.add(new ClassPathEntry(obfuscatedJarFile, true));

            // The preverify tool seems to unpack the resulting classes,
            // so we must not use mixed-case class names on Windows.
            configuration.useMixedCaseClassNames =
                !System.getProperty("os.name").regionMatches(true, 0, "windows", 0, 7);

            // Run ProGuard with these options.
            ProGuard proGuard = new ProGuard(configuration);
            proGuard.execute();

        }
        catch (ParseException ex)
        {
            throw new IOException(ex.getMessage());
        }
        finally
        {
            parser.close();
        }
    }


    /**
     * Converts the given class path String into a ClassPath object.
     */
    private ClassPath classPath(String classPathString)
    {
        ClassPath classPath = new ClassPath();

        String separator = System.getProperty("path.separator");

        int index = 0;
        while (index < classPathString.length())
        {
            // Find the next separator, or the end of the String.
            int next_index = classPathString.indexOf(separator, index);
            if (next_index < 0)
            {
                next_index = classPathString.length();
            }

            // Create and add the found class path entry.
            ClassPathEntry classPathEntry =
                new ClassPathEntry(new File(classPathString.substring(index, next_index)),
                                   false);

            classPath.add(classPathEntry);

            // Continue after the separator.
            index = next_index + 1;
        }

        return classPath;
    }
}
