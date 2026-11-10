import java.awt.Component;
import java.lang.reflect.*;
import java.util.*;
import javax.swing.*;
import javax.swing.text.Document;

public final class VivadoZhTextTest {
    private static int checks;
    private static void check(boolean value,String message) {checks++;if(!value)throw new AssertionError(message);}
    @SuppressWarnings("unchecked")
    public static void main(String[] args) throws Exception {
        Class<?> agent=Class.forName("VivadoZhAgent");
        Field wf=agent.getDeclaredField("words");wf.setAccessible(true);
        Map<String,String> words=(Map<String,String>)wf.get(null);
        words.put("Open","打开");words.put("Design Sources","设计源文件");
        words.put("This wizard creates a project.","此向导用于创建工程。");
        Method translate=agent.getDeclaredMethod("translate",String.class);translate.setAccessible(true);
        Method html=agent.getDeclaredMethod("translateHtml",String.class,boolean.class);html.setAccessible(true);
        Method visit=agent.getDeclaredMethod("visit",Component.class,Set.class);visit.setAccessible(true);
        Method disable=agent.getDeclaredMethod("disable");disable.setAccessible(true);
        check(translate.invoke(null,"Design Sources (12)").equals("设计源文件 (12)"),"numeric count preserved");
        check(translate.invoke(null,"This wizard\n creates   a project.").equals("此向导用于创建工程。"),"whitespace normalized");
        check(translate.invoke(null,"Probe Ports(0..7)").equals("探针端口（0..7）"),"ILA probe range");
        check(translate.invoke(null,"Probe_Ports(8..15)").equals("探针端口（8..15）"),"ILA underscore range");
        check(translate.invoke(null,"Monitor Interface(00..15)").equals("监测接口（00..15）"),"monitor indices retained verbatim");
        check(translate.invoke(null,"Monitor Interface0").equals("监测接口 0"),"monitor instance caption");
        check(translate.invoke(null,"VCO Freq = 1200.00000 MHz").equals("VCO 频率 = 1200.00000 MHz"),"VCO exact numeric precision retained");
        check(translate.invoke(null,"VCO Freq = 960.125 MHz").equals("VCO 频率 = 960.125 MHz"),"VCO live values are not cached snapshots");
        check(translate.invoke(null,"Total heap size: 812M Used: 171M").equals("堆内存总量：812M 已使用：171M"),"live heap metrics retained");
        check(translate.invoke(null,"Total heap size: 812M Used: 178M").equals("堆内存总量：812M 已使用：178M"),"heap update not stale");
        for(String identifier:new String[]{"PROBE0","SLOT0","clk_out1","CONFIG.C_PROBE0_WIDTH","Probe Ports(foo..7)"})
            check(translate.invoke(null,identifier).equals(identifier),"identifier unchanged: "+identifier);
        Field mf=agent.getDeclaredField("machineWords");mf.setAccessible(true);
        ((Map<String,String>)mf.get(null)).put("This unreviewed machine translation should remain English.","未经校对的译文");
        Method withDrafts=agent.getDeclaredMethod("translate",String.class,boolean.class);withDrafts.setAccessible(true);
        check(withDrafts.invoke(null,"This unreviewed machine translation should remain English.",true).equals("This unreviewed machine translation should remain English."),"unreviewed machine draft never displayed");
        String source="<html><head><style>Open</style></head><body><a href=\"action:Open?id=3\">Open</a><pre>Open</pre></body></html>";
        String translated=(String)html.invoke(null,source,false);
        check(translated.equals("<html><head><style>Open</style></head><body><a href=\"action:Open?id=3\">打开</a><pre>Open</pre></body></html>"),"HTML links, styles and code preserved");
        SwingUtilities.invokeAndWait(() -> {
            try {
                JEditorPane prose=new JEditorPane("text/html",source);prose.setEditable(false);
                Document original=prose.getDocument();String before=prose.getText();
                visit.invoke(null,prose,Collections.newSetFromMap(new IdentityHashMap<Component,Boolean>()));
                check(prose.getDocument()!=original,"separate presentation document");
                check(prose.getDocument().getText(0,prose.getDocument().getLength()).contains("打开"),"read-only prose translated");
                check(prose.getText().contains("action:Open?id=3"),"link action unchanged");
                check(original.getText(0,original.getLength()).contains("Open"),"original document unchanged");
                JEditorPane editor=new JEditorPane("text/html",source);editor.setEditable(true);Document input=editor.getDocument();
                visit.invoke(null,editor,Collections.newSetFromMap(new IdentityHashMap<Component,Boolean>()));
                check(editor.getDocument()==input&&!editor.getText().contains("打开"),"editable text unchanged");
                JEditorPane dynamic=new JEditorPane("text/html","<html><body>Open</body></html>");dynamic.setEditable(false);
                visit.invoke(null,dynamic,Collections.newSetFromMap(new IdentityHashMap<Component,Boolean>()));
                Document displayed=dynamic.getDocument();
                dynamic.setText("<html><body>New native status</body></html>");
                check(dynamic.getDocument()==displayed,"fixture updates existing display document");
                disable.invoke(null);
                check(dynamic.getDocument()==displayed && dynamic.getText().contains("New native status"),"disable does not overwrite newer in-place application text");
                check(prose.getDocument()==original && prose.getText().equals(before),"same original document restored");
            } catch(Exception ex) {throw new RuntimeException(ex);}
        });
        System.out.println("TEST_PASS VivadoZhTextTest checks="+checks);
    }
}
