import java.awt.*;
import java.awt.event.MouseEvent;
import java.lang.instrument.Instrumentation;
import java.nio.file.*;
import java.util.*;
import javax.swing.*;
import javax.swing.tree.*;

/** Local, one-shot UI inspection/navigation for translation verification. */
public final class VivadoZhProbe {
    private static String clean(String s) {return s==null?"":s.replace("\t"," ").replace("\n","\\n").replace("\r","");}
    public static void agentmain(String args,Instrumentation ignored) throws Exception {
        String[] a=args.split("\\|",-1);
        if(a.length!=3) throw new IllegalArgumentException("snapshot/click/tree/tab/tree-double/table-double|label|output.tsv");
        ArrayList<String> lines=new ArrayList<>();
        SwingUtilities.invokeAndWait(() -> {
            ArrayList<Component> items=new ArrayList<>();
            Set<Component> seen=Collections.newSetFromMap(new IdentityHashMap<>());
            for(Window w:Window.getWindows()) if(w.isShowing()) collect(w,items,seen);
            if(!a[0].equals("snapshot")) {
                ArrayList<Runnable> actions=new ArrayList<>();
                for(Component c:items) {
                    if(a[0].equals("click") && c instanceof AbstractButton) {
                        AbstractButton b=(AbstractButton)c;
                        if(b.isEnabled() && (a[1].equals(b.getText()) || a[1].equals(b.getActionCommand()))) actions.add(() -> b.doClick());
                    }
                    if(a[0].startsWith("tree") && c instanceof JTree) {
                        JTree tree=(JTree)c;
                        for(int row=0;row<tree.getRowCount();row++) {
                            TreePath path=tree.getPathForRow(row);
                            if(path!=null && a[1].equals(path.getLastPathComponent().toString())) {
                                actions.add(() -> {tree.setSelectionPath(path);tree.scrollPathToVisible(path);if(a[0].equals("tree-double")) doubleClick(tree,tree.getPathBounds(path));});
                            }
                        }
                    }
                    if(a[0].equals("tab") && c instanceof JTabbedPane) {
                        JTabbedPane tabs=(JTabbedPane)c;
                        for(int n=0;n<tabs.getTabCount();n++) {final int index=n;if(a[1].equals(tabs.getTitleAt(n))) actions.add(() -> tabs.setSelectedIndex(index));}
                    }
                    if(a[0].equals("table-double") && c instanceof JTable) {
                        JTable table=(JTable)c;
                        for(int r=0;r<Math.min(table.getRowCount(),1000);r++) for(int col=0;col<table.getColumnCount();col++) {
                            final int row=r,column=col;
                            if(a[1].equals(String.valueOf(table.getValueAt(r,col)))) actions.add(() -> {table.setRowSelectionInterval(row,row);Rectangle rect=table.getCellRect(row,column,true);table.scrollRectToVisible(rect);doubleClick(table,rect);});
                        }
                    }
                }
                if(actions.size()!=1) {lines.add("ACTION_REFUSED matches="+actions.size());}
                else {lines.add("ACTION_SENT "+a[0]+" "+a[1]);SwingUtilities.invokeLater(actions.get(0));}
            }
            for(Component c:items) {
                String text="";
                if(c instanceof JLabel) text=((JLabel)c).getText();
                else if(c instanceof AbstractButton) text=((AbstractButton)c).getText();
                else if(c instanceof javax.swing.text.JTextComponent) {
                    javax.swing.text.JTextComponent t=(javax.swing.text.JTextComponent)c;
                    if(!(t instanceof JPasswordField) && t.getDocument().getLength()<4000) text="editable="+t.isEditable()+" opaque="+t.isOpaque()+" text="+t.getText();
                } else if(c instanceof Dialog) text=((Dialog)c).getTitle();
                if(text!=null && !text.isEmpty()) lines.add(c.getClass().getName()+"\t"+clean(c.getName())+"\t"+clean(text));
                if(c instanceof JTree) {
                    JTree tree=(JTree)c;
                    for(int r=0;r<Math.min(tree.getRowCount(),300);r++) lines.add("TREE "+c.getClass().getName()+"\t"+r+"\t"+clean(String.valueOf(tree.getPathForRow(r).getLastPathComponent())));
                }
                if(c instanceof JTable) {
                    JTable table=(JTable)c;
                    for(int r=0;r<Math.min(table.getRowCount(),100);r++) {
                        StringJoiner row=new StringJoiner(" | ");
                        for(int col=0;col<Math.min(table.getColumnCount(),12);col++) row.add(clean(String.valueOf(table.getValueAt(r,col))));
                        lines.add("TABLE "+c.getClass().getName()+"\t"+r+"\t"+row);
                    }
                }
                if(c instanceof JTabbedPane) {JTabbedPane tabs=(JTabbedPane)c;for(int n=0;n<tabs.getTabCount();n++)lines.add("TAB "+c.getClass().getName()+"\t"+n+"\t"+clean(tabs.getTitleAt(n)));}
            }
        });
        Files.write(Paths.get(a[2]),lines);
    }
    private static void collect(Component c,ArrayList<Component> items,Set<Component> seen) {
        if(!c.isShowing() || !seen.add(c)) return;
        items.add(c);
        if(c instanceof Container) for(Component child:((Container)c).getComponents()) collect(child,items,seen);
    }
    private static void doubleClick(Component c,Rectangle r) {
        if(r==null)return;int x=r.x+Math.min(r.width/2,80),y=r.y+r.height/2;
        for(int type:new int[]{MouseEvent.MOUSE_PRESSED,MouseEvent.MOUSE_RELEASED,MouseEvent.MOUSE_CLICKED})
            c.dispatchEvent(new MouseEvent(c,type,System.currentTimeMillis(),0,x,y,2,false,MouseEvent.BUTTON1));
    }
}
