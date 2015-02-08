
import proguard.annotation.*;

@KeepApplication
public class NativeCallBack
{
    /**
     * Suppose this is a native method that computes an answer.
     *
     * The -keep option regular ProGuard configuration will make sure it is
     * not renamed when processing this code.
     */
    public native int computeAnswer();


    /**
     * Suppose this method is called back from the above native method.
     *
     * ProGuard would remove it, because it is not referenced from java.
     * The annotation will make sure it is preserved anyhow.
     */
    @Keep
    public int getAnswer()
    {
        return 42;
    }


    public static void main(String[] args)
    {
        int answer = new NativeCallBack().computeAnswer();

        System.out.println("The answer is " + answer);
    }
}
