import java.awt.*;
import java.lang.ref.*;
import java.lang.reflect.*;
import java.util.*;
import javax.swing.*;
import javax.swing.table.*;
public class VivadoZhAgentTest {
 static Method visit,disable;
 static Map<String,String> words;
 static JLabel live;
 static JButton button;
 static JTable table;
 static Object model,headerValue,identifier;
 static TableCellRenderer original;
 static WeakReference<JLabel> dead;
 static void check(boolean b,String m){if(!b)throw new AssertionError(m);}
 static void scan(Component c)throws Exception {visit.invoke(null,c,Collections.newSetFromMap(new IdentityHashMap<Component,Boolean>()));}
 static class NameSettingLabel extends JLabel {
  NameSettingLabel(String s){super(s);setName(null);}
  public void setText(String s){super.setText(s);if(getName()==null)setName(s);}
 }
 public static void main(String[] args)throws Exception {
  Class<?> a=Class.forName("VivadoZhAgent");
  Field wf=a.getDeclaredField("words");wf.setAccessible(true);words=(Map<String,String>)wf.get(null);
  words.put("Open","打开");words.put("Name","名称");
  Method html=a.getDeclaredMethod("translateFlowHtml",String.class);html.setAccessible(true);
  check(html.invoke(null,"<html><span style=\"color:gray\">Open</span></html>").equals("<html><span style=\"color:gray\">打开</span></html>"),"flow HTML markup preserved");
  check(html.invoke(null,"<html><b>Open device</b></html>").equals("<html><b>Open device</b></html>"),"no substring translation");
  visit=a.getDeclaredMethod("visit",Component.class,Set.class);visit.setAccessible(true);
  disable=a.getDeclaredMethod("disable");disable.setAccessible(true);
  SwingUtilities.invokeAndWait(()->{try{
   live=new NameSettingLabel("Open");scan(live);
   check(live.getText().equals("打开"),"label translated");check(live.getName()==null,"name preserved");
   button=new JButton("Open");check(button.getModel().getActionCommand()==null,"implicit action originally");scan(button);
   check(button.getText().equals("打开"),"button translated");check(button.getActionCommand().equals("Open"),"dispatch preserved");
   table=new JTable(new Object[][]{{"Open"}},new String[]{"Name"});model=table.getModel();
   TableColumn c=table.getColumnModel().getColumn(0);headerValue=c.getHeaderValue();identifier=c.getIdentifier();original=c.getHeaderRenderer();
   scan(table);
   Component result=c.getHeaderRenderer().getTableCellRendererComponent(table,c.getHeaderValue(),false,false,-1,0);
   check(result instanceof JLabel && ((JLabel)result).getText().equals("名称"),"header translated");
   check(c.getHeaderValue()==headerValue && c.getIdentifier()==identifier && table.getModel()==model && table.getValueAt(0,0).equals("Open"),"table data unchanged");
   JLabel gone=new JLabel("Open");scan(gone);dead=new WeakReference<>(gone);
  }catch(Exception e){throw new RuntimeException(e);}});
  for(int i=0;i<10 && dead.get()!=null;i++){System.gc();Thread.sleep(20);}
  check(dead.get()==null,"translated component not retained");
  SwingUtilities.invokeAndWait(()->{try{
   disable.invoke(null);
   check(live.getText().equals("Open") && live.getName()==null,"label fully restored");
   check(button.getText().equals("Open") && button.getModel().getActionCommand()==null,"button text and implicit action restored");
   check(table.getColumnModel().getColumn(0).getHeaderRenderer()==original,"header renderer restored");
  }catch(Exception e){throw new RuntimeException(e);}});
  System.out.println("PASS: name preserved; command preserved/restored; table values and identifiers unchanged; header renderer restored; translated component GC succeeds");
 }
}
