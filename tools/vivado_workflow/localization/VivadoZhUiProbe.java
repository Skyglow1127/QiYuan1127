import java.awt.*;
import java.lang.instrument.Instrumentation;
import java.nio.file.*;
import java.util.*;
import javax.swing.*;
import javax.swing.text.JTextComponent;

/** Read-only UI inventory, plus selecting an existing Settings page for validation. */
public final class VivadoZhUiProbe {
    private static final Set<String> SETTINGS=new HashSet<>(Arrays.asList("General","Simulation","Elaboration","Synthesis","Implementation","Bitstream","Project","IP Defaults","Source File","Display","Help","Selection Rules","Shortcuts","Window Behavior"));
    public static void agentmain(String args,Instrumentation unused) throws Exception {
        String[] fields=args.split("\\|",-1);Path out=Paths.get(fields[0]);
        ArrayList<String> lines=new ArrayList<>();
        if(fields.length>1&&!fields[1].isEmpty()) {
            if(!SETTINGS.contains(fields[1])) throw new IllegalArgumentException("Only an existing Settings page may be selected");
            SwingUtilities.invokeAndWait(() -> {
                for(Window w:Window.getWindows()) if(w.isShowing()&&w instanceof Dialog && Arrays.asList("Settings","设置").contains(((Dialog)w).getTitle())) select(w,fields[1],lines);
            });
        }
        SwingUtilities.invokeAndWait(() -> {for(Window w:Window.getWindows()) if(w.isShowing()) snapshot(w,lines);});
        Files.write(out,lines);
    }
    private static void select(Component c,String title,ArrayList<String> log) {
        if(c instanceof JTree) {
            JTree tree=(JTree)c;
            for(int i=0;i<tree.getRowCount();i++) {
                Object node=tree.getPathForRow(i).getLastPathComponent();
                String value=tree.convertValueToText(node,false,false,false,i,false);
                if(value.equals(title)) {tree.setSelectionRow(i);tree.scrollRowToVisible(i);log.add("SELECTED\t"+title+"\t"+i);return;}
            }
            return;
        }
        if(c instanceof Container) for(Component child:((Container)c).getComponents()) select(child,title,log);
    }
    private static void snapshot(Component c,ArrayList<String> lines) {
        if(!c.isShowing()) return;
        String text=null;
        if(c instanceof JLabel) text=((JLabel)c).getText();
        else if(c instanceof AbstractButton) text=((AbstractButton)c).getText();
        else if(c instanceof JTextComponent&&!((JTextComponent)c).isEditable()) {
            JTextComponent t=(JTextComponent)c;
            if(t.getDocument().getLength()<12000) try{text=t.getDocument().getText(0,t.getDocument().getLength());}catch(Exception ignored){}
        }
        Point p=c.getLocationOnScreen();
        if(text!=null) lines.add(c.getClass().getName()+"\t"+p.x+","+p.y+","+c.getWidth()+","+c.getHeight()+"\t"+text.replace("\n","\\n").replace("\t"," "));
        if(c instanceof JTree) {
            JTree tree=(JTree)c;
            for(int i=0;i<tree.getRowCount()&&i<100;i++) {
                Object node=tree.getPathForRow(i).getLastPathComponent();
                lines.add("TREE\t"+i+"\t"+tree.getRowBounds(i)+"\t"+tree.convertValueToText(node,false,false,false,i,false));
            }
            return;
        }
        if(c instanceof Container && !(c instanceof JTextComponent)) for(Component child:((Container)c).getComponents()) snapshot(child,lines);
    }
}
