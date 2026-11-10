import java.lang.instrument.Instrumentation;
import java.net.URL;
import java.net.URLClassLoader;
import java.lang.reflect.Method;

/** Reloadable entry point; avoids retaining obsolete translation implementations. */
public final class VivadoZhBootstrap {
    private static URLClassLoader loader;
    private static Method entry;
    public static synchronized void agentmain(String args, Instrumentation inst) throws Exception {
        if(entry!=null) {
            entry.invoke(null,"disable",inst);
            entry=null;
            loader.close();loader=null;
        }
        if(args.equals("disable")) return;
        String[] fields=args.split("\\|",-1);
        if(fields.length!=4) throw new IllegalArgumentException("enable|dictionary|output|payload.jar");
        URL jar=new java.io.File(fields[3]).toURI().toURL();
        String payloadArgs=String.join("|",fields[0],fields[1],fields[2]);
        loader=new URLClassLoader(new URL[]{jar},ClassLoader.getPlatformClassLoader());
        entry=Class.forName("VivadoZhAgent",true,loader).getMethod("agentmain",String.class,Instrumentation.class);
        try {entry.invoke(null,payloadArgs,inst);}
        catch(Exception e) {
            try(java.io.PrintWriter log=new java.io.PrintWriter(new java.io.File(fields[2],"bootstrap-error.txt"),"UTF-8")) {e.printStackTrace(log);}
            try {entry.invoke(null,"disable",inst);} catch(Exception ignored) {}
            entry=null;loader.close();loader=null;
            throw e;
        }
    }
}
