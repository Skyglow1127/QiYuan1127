import java.awt.*;
import java.lang.instrument.Instrumentation;
import java.lang.ref.WeakReference;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.io.StringReader;
import java.io.StringWriter;
import java.util.*;
import java.util.concurrent.*;
import java.util.regex.*;
import javax.swing.*;
import javax.swing.border.TitledBorder;
import javax.swing.table.*;
import javax.swing.text.JTextComponent;
import javax.swing.text.Document;
import javax.swing.text.html.HTMLDocument;
import javax.swing.tree.TreeCellRenderer;

/** Experimental, session-only translation of static Swing presentation text. */
public final class VivadoZhAgent {
    private static final Map<String,String> words = new LinkedHashMap<>();
    private static final Map<Object,Map<String,Change>> changes = new WeakHashMap<>();
    // Wrappers may retain delegates which retain their UI. Registry values must be weak too.
    private static final Set<String> observed = new TreeSet<>();
    private static final Set<String> applied = new TreeSet<>();
    private static final Set<String> queued = new LinkedHashSet<>();
    private static final Queue<String> pending = new ConcurrentLinkedQueue<>();
    private static final Map<String,String> machineWords = new ConcurrentHashMap<>();
    private static final Map<String,String> normalizedWords = new HashMap<>();
    private static final int OBSERVED_LIMIT=20000;
    private static ScheduledExecutorService cacheWorker;
    private static long cacheModified;
    private static volatile long cacheGeneration;
    private static javax.swing.Timer timer;
    private static Path output;
    private static int ticks;
    private static long updates;
    private static boolean dirty=true;
    private static String savedState;

    // Identity alone is insufficient: Vivado may update a displayed document in
    // place. Do not restore an old document over a newer application update.
    private static final class HtmlState {
        final Document document;
        final String text;
        HtmlState(Document source) {
            document=source;
            try {text=source.getText(0,source.getLength());}
            catch(javax.swing.text.BadLocationException error) {throw new IllegalStateException(error);}
        }
        @Override public boolean equals(Object other) {
            return other instanceof HtmlState && document==((HtmlState)other).document && text.equals(((HtmlState)other).text);
        }
        @Override public int hashCode() {return System.identityHashCode(document)*31+text.hashCode();}
    }

    // These functions must not capture their target; Change holds only a weak target.
    private interface Getter { Object get(Object target); }
    private interface Setter { void set(Object target,Object value); }
    private static final class Change {
        final WeakReference<Object> target;
        final Object before;
        Object after;
        final Getter getter;
        final Setter setter;
        Change(Object t,Object b,Object a,Getter g,Setter s) {
            target=new WeakReference<>(t);before=b;after=a;getter=g;setter=s;
        }
        void restore() {
            Object t=target.get();
            if(t!=null && Objects.equals(getter.get(t),after)) setter.set(t,before);
        }
    }

    public static void agentmain(String args, Instrumentation ignored) throws Exception {
        String[] a=args.split("\\|",-1);
        if(a[0].equals("disable")) {
            SwingUtilities.invokeAndWait(VivadoZhAgent::disable);
            return;
        }
        if(a.length!=3 || !a[0].equals("enable")) throw new IllegalArgumentException("enable|dictionary.tsv|output-directory or disable");
        Map<String,String> loaded=new LinkedHashMap<>();
        for(String line:Files.readAllLines(Paths.get(a[1]),StandardCharsets.UTF_8)) {
            if(line.isEmpty() || line.startsWith("#")) continue;
            String[] fields=line.split("\t",-1);
            if(fields.length!=2 || fields[0].isEmpty() || fields[1].isEmpty()) throw new IllegalArgumentException("Invalid dictionary line: "+line);
            loaded.put(fields[0],fields[1]);
        }
        Path out=Paths.get(a[2]);Files.createDirectories(out);
        SwingUtilities.invokeAndWait(() -> {
            // Re-enabling with another dictionary restores the previous presentation first.
            if(timer!=null) disable();
            words.clear();words.putAll(loaded);output=out;dirty=true;savedState=null;
            normalizedWords.clear();
            words.forEach((en,zh) -> normalizedWords.put(normalize(en).toLowerCase(Locale.ROOT),zh));
            startCacheWorker();
            scan();
            timer=new javax.swing.Timer(800,e -> {
                try {scan();} catch(Exception ex) {writeError(ex);}
            });
            timer.start();save("enabled");
        });
    }

    private static void change(Object target,String property,Object before,Object after,Getter getter,Setter setter) {
        if(Objects.equals(before,after)) return;
        Map<String,Change> byProperty=changes.computeIfAbsent(target,k -> new LinkedHashMap<>());
        Change prev=byProperty.get(property);
        if(prev==null || !Objects.equals(prev.after,before))
            byProperty.put(property,new Change(target,before,after,getter,setter));
        else prev.after=after;
        setter.set(target,after);updates++;dirty=true;
    }

    private static void recordTranslation(String original,String translated) {
        if(applied.add(original+"\t"+translated)) dirty=true;
    }
    private static String normalize(String s) {return s.replace('\u00a0',' ').replaceAll("\\s+"," ").trim();}

    private static String translate(String s) {return translate(s,false);}

    private static String translate(String s,boolean allowMachine) {
        if(s==null || s.isEmpty() || s.length()>12000) return s;
        if(s.trim().toLowerCase(Locale.ROOT).startsWith("<html")) return translateHtml(s,allowMachine);
        String exact=words.get(s);
        if(exact!=null) {recordTranslation(s,exact);return exact;}
        String trim=normalize(s),translated=words.get(trim);
        if(translated==null) translated=normalizedWords.get(trim.toLowerCase(Locale.ROOT));
        if(translated!=null) {
            recordTranslation(trim,translated);
            return translated;
        }
        for(String suffix:new String[]{"...","…",":"}) if(trim.endsWith(suffix)) {
            String key=trim.substring(0,trim.length()-suffix.length()).trim();
            translated=words.get(key);
            if(translated==null) translated=normalizedWords.get(key.toLowerCase(Locale.ROOT));
            if(translated!=null) {recordTranslation(trim,translated+suffix);return translated+suffix;}
        }
        Matcher count=Pattern.compile("^(.+?)(\\s*\\([0-9]+\\))$").matcher(trim);
        if(count.matches()) {
            String head=translate(count.group(1),false);
            if(!head.equals(count.group(1))) return head+count.group(2);
        }
        // Reviewed ILA display templates; port identifiers such as PROBE0 stay intact.
        Matcher ipRange=Pattern.compile("^(Probe[ _]Ports|Monitor Interface)\\s*\\((\\d+)\\.\\.(\\d+)\\)$").matcher(trim);
        if(ipRange.matches()) {
            String zh=(ipRange.group(1).startsWith("Probe")?"探针端口":"监测接口")+"（"+ipRange.group(2)+".."+ipRange.group(3)+"）";
            recordTranslation(trim,zh);return zh;
        }
        Matcher monitor=Pattern.compile("^Monitor Interface(\\d+)$").matcher(trim);
        if(monitor.matches()) {String zh="监测接口 "+monitor.group(1);recordTranslation(trim,zh);return zh;}
        Matcher vco=Pattern.compile("^VCO Freq(\\s*=\\s*)([0-9]+(?:\\.[0-9]+)?)(\\s+MHz)$").matcher(trim);
        if(vco.matches()) {String zh="VCO 频率"+vco.group(1)+vco.group(2)+vco.group(3);recordTranslation(trim,zh);return zh;}
        Matcher heap=Pattern.compile("^Total heap size: ([0-9]+(?:\\.[0-9]+)?[KMG]?B?) Used: ([0-9]+(?:\\.[0-9]+)?[KMG]?B?)$").matcher(trim);
        if(heap.matches()) return "堆内存总量："+heap.group(1)+" 已使用："+heap.group(2);
        // Offline output is a draft for human review only. Never display unchecked
        // model translations or cache snapshots of changing values such as heap usage.
        if(allowMachine && prose(trim)) {
            if(queued.size()<OBSERVED_LIMIT && queued.add(trim)) pending.add(trim);
        }
        return s;
    }

    private static boolean prose(String s) {
        return s.length()>24 && s.length()<=4000 && s.matches("(?s).*[A-Za-z]{3} [A-Za-z]{2}.*")
            && !s.matches("(?s).*[/\\\\][A-Za-z0-9_.-]+[/\\\\].*")
            && !s.contains("${") && !s.contains("set_property") && !s.contains("<")
            && !s.matches("(?s).*[\\u3400-\\u9fff].*");
    }

    private static String jsonString(String s) {
        return "\""+s.replace("\\","\\\\").replace("\"","\\\"").replace("\n","\\n").replace("\r","\\r").replace("\t","\\t")+"\"";
    }

    private static void startCacheWorker() {
        cacheWorker=Executors.newSingleThreadScheduledExecutor(r -> {Thread t=new Thread(r,"Vivado Chinese cache");t.setDaemon(true);return t;});
        cacheWorker.scheduleWithFixedDelay(() -> {
            try {
                ArrayList<String> batch=new ArrayList<>();String line;
                while((line=pending.poll())!=null) batch.add(jsonString(line));
                if(!batch.isEmpty()) Files.write(output.resolve("translation-queue.jsonl"),batch,StandardCharsets.UTF_8,StandardOpenOption.CREATE,StandardOpenOption.APPEND);
                Path cache=output.resolve("machine-translations.tsv");
                if(Files.exists(cache)) {
                    long modified=Files.getLastModifiedTime(cache).toMillis();
                    if(modified!=cacheModified) {
                        Map<String,String> fresh=new HashMap<>();
                        for(String row:Files.readAllLines(cache,StandardCharsets.UTF_8)) {
                            String[] f=row.split("\t",-1);
                            if(f.length==2 && !f[1].isEmpty() && f[1].matches("(?s).*[\\u3400-\\u9fff].*")) fresh.put(f[0],f[1]);
                        }
                        machineWords.putAll(fresh);cacheModified=modified;cacheGeneration++;
                    }
                }
            } catch(Exception ex) {writeError(ex);}
        },0,600,TimeUnit.MILLISECONDS);
    }

    private static void text(Component c,String value,Getter getter,Setter setter) {
        displayText(c,value,translate(value,true),getter,setter);
    }

    private static void displayText(Component c,String value,String zh,Getter getter,Setter setter) {
        if(value==null || value.isEmpty()) return;
        observe(c,value);
        if(zh.equals(value)) return;
        if(VivadoZhStyledLabel.isStyledLabel(c)) {
            VivadoZhStyledLabel.Snapshot original=VivadoZhStyledLabel.read((JLabel)c);
            VivadoZhStyledLabel.Snapshot translated=VivadoZhStyledLabel.translated(original,zh);
            change(c,"styled-state",original,translated,t -> VivadoZhStyledLabel.read((JLabel)t),
                (t,v) -> VivadoZhStyledLabel.apply((JLabel)t,(VivadoZhStyledLabel.Snapshot)v));
            ensureFont(c,zh);return;
        }
        Setter displaySetter=(target,v) -> {
            Component component=(Component)target;
            String originalName=component.getName();
            setter.set(target,v);
            // Some Vivado JLabel subclasses set Component.name from setText().
            if(!Objects.equals(originalName,component.getName())) component.setName(originalName);
        };
        change(c,"text",value,zh,getter,displaySetter);
        ensureFont(c,zh);
    }

    private static void ensureFont(Component c,String zh) {
        Font f=c.getFont();
        if(f!=null && f.canDisplayUpTo(zh)>=0) {
            Font translated=new Font("Noto Sans CJK SC",f.getStyle(),f.getSize());
            change(c,"font",f,translated,t -> ((Component)t).getFont(),(t,v) -> ((Component)t).setFont((Font)v));
        }
    }

    private static void observe(Component c,String value) {
        if(value!=null && observed.size()<OBSERVED_LIMIT && value.length()<12000
            && observed.add(c.getClass().getName()+"\t"+value.replace("\t"," ").replace("\n","\\n").replace("\r",""))) dirty=true;
    }

    private static void buttonText(AbstractButton b) {
        String before=b.getText();observe(b,before);String after=translate(before,true);
        // In native IP checkboxes these exact captions are port names, not actions.
        if(b instanceof JCheckBox && b.getClass().getName().equals("ui.utils.swing.c.k")
            && ("reset".equals(before)||"resetn".equals(before)||"locked".equals(before))) return;
        if(Objects.equals(before,after)) return;
        // Preserve the effective dispatch string without losing the original nullable model value.
        String effective=b.getActionCommand(),raw=b.getModel().getActionCommand();
        text(b,before,t -> ((AbstractButton)t).getText(),(t,v) -> ((AbstractButton)t).setText((String)v));
        if(!Objects.equals(effective,b.getActionCommand()))
            change(b.getModel(),"action-command",raw,effective,
                t -> ((ButtonModel)t).getActionCommand(),(t,v) -> ((ButtonModel)t).setActionCommand((String)v));
    }

    private static void scan() {
        Set<Component> visited=Collections.newSetFromMap(new IdentityHashMap<Component,Boolean>());
        for(Window w:Window.getWindows()) if(w.isDisplayable()) visit(w,visited);
        VivadoZhOverlay.refresh();
        VivadoZhSmartConnectOverlay.refresh();
        if(++ticks%5==0) save("enabled");
    }

    private static void visit(Component c,Set<Component> seen) {
        if(!seen.add(c)) return;
        VivadoZhSmartConnectOverlay.install(c);
        if(c instanceof JComponent) VivadoZhBorders.install((JComponent)c,value -> {observe(c,value);return translate(value,false);});
        if(c instanceof JTable) VivadoZhIpTables.install((JTable)c,VivadoZhAgent::inPlaceIpText);
        if(c instanceof JTable || c instanceof JTree || c instanceof JComboBox || c instanceof JList) {
            VivadoZhRenderers.install(c,VivadoZhAgent::rendererText);return;
        }
        if(c instanceof JEditorPane) {readOnlyHtml((JEditorPane)c);return;}
        if(c instanceof JTextField && !(c instanceof JPasswordField) && (!((JTextField)c).isEditable() || !c.isEnabled())) {
            JTextField field=(JTextField)c;String original=field.getText();observe(c,original);
            String translated=translate(original,false);
            if(!Objects.equals(original,translated)) VivadoZhOverlay.offer(field,translated);
            return;
        }
        // Input fields, code editors and backend consoles are not display labels.
        if(c instanceof JTextComponent || c instanceof CellRendererPane) return;
        if(c instanceof AbstractButton) buttonText((AbstractButton)c);
        else if(c instanceof JLabel) {
            JLabel label=(JLabel)c;
            text(label,label.getText(),t -> ((JLabel)t).getText(),(t,v) -> ((JLabel)t).setText((String)v));
        }
        if(c instanceof Dialog) {
            Dialog d=(Dialog)c;String before=d.getTitle(),after=translate(before);
            if(!Objects.equals(before,after)) change(d,"title",before,after,
                t -> ((Dialog)t).getTitle(),(t,v) -> ((Dialog)t).setTitle((String)v));
        }
        if(c instanceof JTabbedPane) {
            JTabbedPane pane=(JTabbedPane)c;
            final WeakReference<JTabbedPane> weakPane=new WeakReference<>(pane);
            for(int i=0;i<pane.getTabCount();i++) {
                Component child=pane.getComponentAt(i);
                String before=pane.getTitleAt(i),after=translate(before);
                if(!Objects.equals(before,after)) change(child,"tab-title",before,after,
                    t -> {JTabbedPane p=weakPane.get();int n=p==null?-1:p.indexOfComponent((Component)t);return n<0?null:p.getTitleAt(n);},
                    (t,v) -> {JTabbedPane p=weakPane.get();int n=p==null?-1:p.indexOfComponent((Component)t);if(n>=0)p.setTitleAt(n,(String)v);});
            }
        }
        if(c instanceof JComponent) {
            JComponent jc=(JComponent)c;
            String tip=jc.getToolTipText();
            String zhTip=translate(tip,true);
            if(!Objects.equals(tip,zhTip)) change(jc,"tooltip",tip,zhTip,t -> ((JComponent)t).getToolTipText(),(t,v) -> ((JComponent)t).setToolTipText((String)v));

        }
        if(c instanceof JMenu) for(Component child:((JMenu)c).getMenuComponents()) visit(child,seen);
        if(c instanceof Container) for(Component child:((Container)c).getComponents()) visit(child,seen);
    }

    private static Component rendererText(Component rendered) {
        if(rendered instanceof JLabel) {
            JLabel label=(JLabel)rendered;
            String translated=translate(label.getText(),false);
            if(VivadoZhStyledLabel.isStyledLabel(label)) {
                observe(label,label.getText());
                return Objects.equals(label.getText(),translated)?label:VivadoZhStyledLabel.translatedCopy(label,translated);
            }
            displayText(label,label.getText(),translated,t -> ((JLabel)t).getText(),(t,v) -> ((JLabel)t).setText((String)v));
        }
        return rendered;
    }

    // Coregen keeps concrete renderer identities in private lists and casts them
    // during updates. Translate their existing presentation label in place.
    private static Component inPlaceIpText(Component rendered) {
        if(rendered instanceof JLabel) {
            JLabel label=(JLabel)rendered;
            displayText(label,label.getText(),translate(label.getText(),false),
                t -> ((JLabel)t).getText(),(t,v) -> ((JLabel)t).setText((String)v));
        }
        return rendered;
    }

    private static void readOnlyHtml(JEditorPane pane) {
        if(pane.isEditable() || !pane.getContentType().equalsIgnoreCase("text/html")) return;
        String cls=pane.getClass().getName();
        if(!cls.startsWith("ui.utils.swing.") && !cls.equals("ui.data.coregen.xpegua.aW") && !cls.equals("javax.swing.JEditorPane") && !cls.equals("javax.swing.JTextPane")) return;
        Document current=pane.getDocument();
        if(!(current instanceof HTMLDocument) || current.getLength()>200000) return;
        HtmlState currentState=new HtmlState(current);
        Map<String,Change> previous=changes.get(pane);
        Change prior=previous==null?null:previous.get("html-document");
        if(prior!=null && currentState.equals(prior.after)) return;
        observe(pane,pane.getText());
        try {
            HTMLDocument translated=VivadoZhHtml.translatedCopy((HTMLDocument)current,s -> translate(s,true));
            if(translated==null) return;
            int caret=pane.getCaretPosition();
            change(pane,"html-document",currentState,new HtmlState(translated),
                t -> new HtmlState(((JEditorPane)t).getDocument()),(t,v) -> ((JEditorPane)t).setDocument(((HtmlState)v).document));
            pane.setCaretPosition(Math.min(caret,translated.getLength()));
        } catch(Exception ex) {writeError(ex);}
    }

    private static String translateFlowHtml(String value) {
        return translateHtml(value,false);
    }

    private static String translateHtml(String value,boolean allowMachine) {
        if(value==null || !value.trim().toLowerCase(Locale.ROOT).startsWith("<html")) return translate(value,allowMachine);
        StringBuilder out=new StringBuilder();
        int pos=0;String skip=null;
        while(pos<value.length()) {
            if(value.charAt(pos)=='<') {
                int end=value.indexOf('>',pos);
                if(end<0) return value;
                String tag=value.substring(pos,end+1).toLowerCase(Locale.ROOT);
                if(skip!=null && tag.matches("</"+skip+"\\s*>")) skip=null;
                else if(tag.matches("<(head|style|script|code|pre)(\\s[^>]*)?>")) skip=tag.substring(1).split("[ >]",2)[0];
                out.append(value,pos,end+1);pos=end+1;
            } else {
                int end=value.indexOf('<',pos);
                if(end<0) end=value.length();
                String raw=value.substring(pos,end);
                String plain=raw.replace("&nbsp;"," ").replace("&amp;","&").replace("&quot;","\"").replace("&#39;","'");
                String zh=skip==null?translate(plain,allowMachine):plain;
                out.append(zh.equals(plain)?raw:zh.replace("&","&amp;").replace("<","&lt;").replace(">","&gt;"));pos=end;
            }
        }
        return out.toString();
    }
    private static void disable() {
        if(timer!=null) {timer.stop();timer=null;}
        if(cacheWorker!=null) {cacheWorker.shutdownNow();cacheWorker=null;}
        VivadoZhRenderers.restoreAll();VivadoZhBorders.restoreAll();
        VivadoZhIpTables.restoreAll();VivadoZhOverlay.restoreAll();
        VivadoZhSmartConnectOverlay.restoreAll();
        // Restore model dispatch state last, after button text has returned to its original value.
        ArrayList<Change> actionCommands=new ArrayList<>();
        for(Map<String,Change> group:new ArrayList<>(changes.values()))
            for(Map.Entry<String,Change> entry:group.entrySet()) {
                if(entry.getKey().equals("action-command")) actionCommands.add(entry.getValue());
                else try {entry.getValue().restore();} catch(Exception e) {writeError(e);}
            }
        for(Change ch:actionCommands) try {ch.restore();}catch(Exception e){writeError(e);}
        changes.clear();dirty=true;save("disabled");
        for(Window w:Window.getWindows()) if(w.isDisplayable()) w.repaint();
    }

    private static void save(String state) {
        if(output==null || (!dirty && Objects.equals(state,savedState))) return;
        try {
            Files.write(output.resolve("observed-ui.tsv"),observed,StandardCharsets.UTF_8);
            Files.write(output.resolve("translated-ui.tsv"),applied,StandardCharsets.UTF_8);
            Files.write(output.resolve("agent-status.txt"),Arrays.asList("state="+state,"dictionary_entries="+words.size(),"distinct_translations="+applied.size(),"property_updates="+updates,"scan_count="+ticks,"observed_limit="+OBSERVED_LIMIT,"machine_cache_entries="+machineWords.size(),"machine_queue_entries="+queued.size(),"machine_cache_policy=review_only","scope=Swing presentation, IP table/border adapters, read-only HTML and bounded overlays; original engineering data and commands preserved"),StandardCharsets.UTF_8);
            dirty=false;savedState=state;
        } catch(Exception e) {writeError(e);}
    }
    private static void writeError(Exception e) {
        if(output==null) return;
        try {Files.write(output.resolve("agent-errors.log"),Arrays.asList(e.toString()),StandardCharsets.UTF_8,StandardOpenOption.CREATE,StandardOpenOption.APPEND);}catch(Exception ignored){}
    }
}
