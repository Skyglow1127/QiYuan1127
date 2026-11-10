import java.awt.Component;
import java.lang.reflect.Field;
import java.net.URLClassLoader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import javax.swing.JLabel;
import javax.swing.JTable;
import javax.swing.SwingUtilities;
import javax.swing.table.DefaultTableCellRenderer;
import javax.swing.table.TableCellEditor;
import javax.swing.table.TableCellRenderer;
import javax.swing.table.TableModel;
import javax.tools.JavaCompiler;
import javax.tools.ToolProvider;

/** Contract tests with a small class-shape fixture; no Vivado JNI is loaded. */
public final class VivadoZhIpTablesTest {
    private static int checks;
    private static void check(boolean success,String message) {
        if(!success) throw new AssertionError(message);checks++;
    }
    private static Object field(Object target,String name) throws Exception {
        Field field=target.getClass().getDeclaredField(name);field.setAccessible(true);return field.get(target);
    }
    @SuppressWarnings("unchecked")
    private static ArrayList<Object> entries(Object table,String field) throws Exception {
        return (ArrayList<Object>)field(table,field);
    }
    private static void set(Object target,String name,Object value) throws Exception {
        Field field=target.getClass().getDeclaredField(name);field.setAccessible(true);field.set(target,value);
    }

    public static void main(String[] args) throws Exception {
        JavaCompiler compiler=ToolProvider.getSystemJavaCompiler();
        if(compiler==null) throw new IllegalStateException("Tests require a JDK compiler");
        Path dir=Files.createTempDirectory("vivado-zh-iptable-fixture-");
        Path pkg=dir.resolve("ui/data/coregen/xpegua");Files.createDirectories(pkg);
        String prefix="package ui.data.coregen.xpegua; import java.awt.*; import java.util.*; import javax.swing.*; import javax.swing.table.*; ";
        Files.writeString(pkg.resolve("ba.java"),prefix+
            "public class ba extends JLabel { public ba(String value){super(value);} }",StandardCharsets.UTF_8);
        Files.writeString(pkg.resolve("bc.java"),prefix+
            "public class bc extends AbstractCellEditor implements TableCellRenderer,TableCellEditor {"+
            "ba fEg=new ba(\"Probe Width\"); private final String nativeText=\"Probe Width\";"+
            "public Component getTableCellRendererComponent(JTable t,Object v,boolean s,boolean f,int r,int c){return fEg;}"+
            "public Component getTableCellEditorComponent(JTable t,Object v,boolean s,int r,int c){return fEg;}"+
            "public Object getCellEditorValue(){return nativeText;} }",StandardCharsets.UTF_8);
        Files.writeString(pkg.resolve("as.java"),prefix+
            "public class as extends DefaultTableCellRenderer { public final String imageIdentity=\"unchanged\"; }",StandardCharsets.UTF_8);
        Files.writeString(pkg.resolve("aD.java"),prefix+
            "public class aD extends JTable { ArrayList<TableCellRenderer> fDi=new ArrayList<>();"+
            "ArrayList<TableCellEditor> fDj=new ArrayList<>(); public aD(){super(1,3);"+
            "bc text=new bc(); fDi.add(text); fDi.add(new as()); fDi.add(new DefaultTableCellRenderer());fDj.add(text);}"+
            "public TableCellRenderer getCellRenderer(int r,int c){return fDi.get(c);}"+
            "public void nativeRefresh(){bc text=(bc)fDi.get(0);text.fEg.setText(\"Probe Width\");as image=(as)fDi.get(1);"+
            "if(!image.imageIdentity.equals(\"unchanged\"))throw new AssertionError();}"+
            "}",StandardCharsets.UTF_8);
        for(String[] variant:new String[][]{{"ba","cJ"},{"bc","cK"},{"as","cw"},{"aD","cC"}}) {
            String source=Files.readString(pkg.resolve(variant[0]+".java"));
            for(String[] name:new String[][]{{"ba","cJ"},{"bc","cK"},{"as","cw"},{"aD","cC"},{"fEg","fGq"}})
                source=source.replaceAll("\\b"+name[0]+"\\b",name[1]);
            Files.writeString(pkg.resolve(variant[1]+".java"),source);
        }
        int exit=compiler.run(null,null,null,"--release","11","-d",dir.toString(),
            pkg.resolve("cJ.java").toString(),pkg.resolve("cK.java").toString(),pkg.resolve("cw.java").toString(),pkg.resolve("cC.java").toString(),pkg.resolve("ba.java").toString(),pkg.resolve("bc.java").toString(),pkg.resolve("as.java").toString(),pkg.resolve("aD.java").toString());
        check(exit==0,"Fixture must compile");
        try(URLClassLoader loader=new URLClassLoader(new java.net.URL[]{dir.toUri().toURL()},ClassLoader.getPlatformClassLoader())) {
            Class<?> tableClass=loader.loadClass("ui.data.coregen.xpegua.aD");
            Class<?> textClass=loader.loadClass("ui.data.coregen.xpegua.bc");
            SwingUtilities.invokeAndWait(() -> {
                try {run(tableClass,textClass,"fEg");run(loader.loadClass("ui.data.coregen.xpegua.cC"),loader.loadClass("ui.data.coregen.xpegua.cK"),"fGq");} catch(Exception error) {throw new RuntimeException(error);}
            });
        }
        boolean rejected=false;
        try {VivadoZhIpTables.install(new JTable(),c -> c);} catch(IllegalStateException expected) {rejected=true;}
        check(rejected,"Off-EDT discovery rejected");
        rejected=false;
        try {VivadoZhIpTables.restoreAll();} catch(IllegalStateException expected) {rejected=true;}
        check(rejected,"Off-EDT restoration rejected");
        System.out.println("PASS VivadoZhIpTablesTest: "+checks+" checks (mock class-shape contract; no native JNI)");
    }

    private static void run(Class<?> tableClass,Class<?> textClass,String displayField) throws Exception {
        JTable table=(JTable)tableClass.getConstructor().newInstance();
        TableModel model=table.getModel();table.setValueAt("C_PROBE0_WIDTH",0,0);
        ArrayList<Object> renderers=entries(table,"fDi"),editors=entries(table,"fDj");
        Object text=renderers.get(0),image=renderers.get(1),other=renderers.get(2),editor=editors.get(0);
        JLabel label=(JLabel)field(text,displayField);String name="C_PROBE0_WIDTH";label.setName(name);
        Set<Component> visited=Collections.newSetFromMap(new IdentityHashMap<>());
        java.util.function.UnaryOperator<Component> translate=c -> {visited.add(c);((JLabel)c).setText("探针宽度");return c;};
        VivadoZhIpTables.install(table,translate);
        check(visited.size()==1&&visited.contains(label),"Only bc.fEg label visited");
        check(label.getText().equals("探针宽度"),"In-place callback applied");
        check(label.getName().equals(name),"Component identity name remains intact");
        check(field(table,"fDi")==renderers&&field(table,"fDj")==editors,"Collection identities preserved");
        check(renderers.get(0)==text&&renderers.get(1)==image&&renderers.get(2)==other,"All renderers retained");
        check(editors.get(0)==editor,"Editor retained");
        check(((TableCellEditor)editor).getCellEditorValue().equals("Probe Width"),"Backend editor value unchanged");
        check(table.getModel()==model&&table.getValueAt(0,0).equals("C_PROBE0_WIDTH"),"Table model and parameter value unchanged");
        check(table.getCellRenderer(0,0)==text,"Custom getCellRenderer still returns concrete bc");
        tableClass.getMethod("nativeRefresh").invoke(table);
        check(label.getText().equals("Probe Width"),"Concrete bc/as casts and refresh still work");
        VivadoZhIpTables.install(table,translate);
        check(label.getText().equals("探针宽度"),"Refreshed labels translated on repeated scan");
        Object replacement=textClass.getConstructor().newInstance();
        JLabel replacementLabel=(JLabel)field(replacement,displayField);
        ArrayList<Object> rebuilt=new ArrayList<>(Arrays.asList(replacement,image,other));set(table,"fDi",rebuilt);
        visited.clear();VivadoZhIpTables.install(table,translate);
        check(visited.size()==1&&visited.contains(replacementLabel),"Rebuilt collection discovered");
        check(field(table,"fDi")==rebuilt&&rebuilt.get(0)==replacement,"External renderer replacement retained");
        check(editors.get(0)==editor,"Old editor collection is never rewritten");
        JLabel externalLabel=(JLabel)replacementLabel.getClass().getConstructor(String.class).newInstance("Probe Width");
        set(replacement,displayField,externalLabel);visited.clear();VivadoZhIpTables.install(table,translate);
        check(visited.contains(externalLabel)&&!visited.contains(replacementLabel),"External display-label replacement discovered");
        boolean rejected=false;
        try {VivadoZhIpTables.install(table,c -> new JLabel("replacement"));}
        catch(IllegalStateException expected) {rejected=expected.getMessage().contains("same label");}
        check(rejected,"Replacement callback rejected clearly");
        check(field(replacement,displayField)==externalLabel,"Bad callback cannot replace native display field");
        AtomicInteger calls=new AtomicInteger();
        VivadoZhIpTables.install(new JTable(1,1),c -> {calls.incrementAndGet();return c;});
        check(calls.get()==0,"Unrelated JTable ignored");
        VivadoZhIpTables.restoreAll();
        check(field(table,"fDi")==rebuilt&&rebuilt.get(0)==replacement&&field(replacement,displayField)==externalLabel,
            "Helper restoration changes no application objects");
        check(externalLabel.getText().equals("探针宽度"),"Text rollback remains exclusively the caller's responsibility");
    }
}
