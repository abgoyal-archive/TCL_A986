
package proguard.ant;

import org.apache.tools.ant.BuildException;
import org.apache.tools.ant.types.DataType;
import proguard.Configuration;

public class ConfigurationElement extends DataType
{
    /**
     * Adds the contents of this configuration task to the given configuration.
     * @param configuration the configuration to be extended.
     */
    public void appendTo(Configuration configuration)
    {
        // Get the referenced element.
        if (!isReference())
        {
            throw new BuildException("Nested element <configuration> must have a refid attribute");
        }

        ConfigurationTask configurationTask =
            (ConfigurationTask)getCheckedRef(ConfigurationTask.class,
                                             ConfigurationTask.class.getName());

        // Append the referenced configuration entries to the given configuration.
        configurationTask.appendTo(configuration);
    }
}
