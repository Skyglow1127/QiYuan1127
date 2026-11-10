import java.lang.instrument.Instrumentation;
import java.lang.reflect.*;
import java.nio.file.*;
import java.util.*;
import javax.swing.*;

/** Restore this prototype's text before renderer layout after a failed hot reload. */
public final class VivadoZhRecovery {
    public static void agentmain(String output,Instrumentation instrumentation) throws Exception {
        ArrayList<String> log=new ArrayList<>();
        SwingUtilities.invokeAndWait(() -> {
            for(Class<?> cls:instrumentation.getAllLoadedClasses()) {
                if(!cls.getName().equals("VivadoZhAgent")) continue;
                try {
                    Field timer=cls.getDeclaredField("timer");timer.setAccessible(true);
                    javax.swing.Timer active=(javax.swing.Timer)timer.get(null);
                    if(active!=null) active.stop();
                    Field changes=cls.getDeclaredField("changes");changes.setAccessible(true);
                    Map<?,?> all=(Map<?,?>)changes.get(null);
                    for(Object group:new ArrayList<>(all.values())) for(Object change:new ArrayList<>(((Map<?,?>)group).values())) {
                        try {Method restore=change.getClass().getDeclaredMethod("restore");restore.setAccessible(true);restore.invoke(change);}
                        catch(Exception ex) {log.add("property: "+ex);}
                    }
                    Method disable=cls.getDeclaredMethod("disable");disable.setAccessible(true);disable.invoke(null);
                    log.add("restored "+cls.getClassLoader());
                } catch(Exception ex) {log.add("agent: "+ex);}
            }
        });
        Files.write(Paths.get(output),log);
    }
}
