import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.lang.ref.WeakReference;
import java.lang.reflect.Field;
import java.net.URLClassLoader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import javax.imageio.ImageIO;
import javax.swing.*;
import javax.tools.JavaCompiler;
import javax.tools.ToolProvider;

/** Headless class-shape + pixel contracts. This does not load or exercise Vivado JNI. */
public final class VivadoZhSmartConnectOverlayTest {
    private static final String PREFIX="ui.data.coregen.xpegua.ipwidgets.y.";
    private static final String TEMPLATE="DOMAIN: clock_0\nFREQ: 100000000\nPHASE: 0.000";
    private static final ArrayList<String> checks=new ArrayList<>();
    private static ClassLoader loader;
    private static Path evidence;
    private static void check(boolean result,String message) {
        if(!result) throw new AssertionError(message);checks.add(message);
    }
    private static Object get(Object target,String name) throws Exception {
        for(Class<?> type=target.getClass();type!=null;type=type.getSuperclass()) {
            try {Field field=type.getDeclaredField(name);field.setAccessible(true);return field.get(target);}
            catch(NoSuchFieldException absent) {}
        }
        throw new NoSuchFieldException(name);
    }
    private static void set(Object target,String name,Object value) throws Exception {
        for(Class<?> type=target.getClass();type!=null;type=type.getSuperclass()) {
            try {Field field=type.getDeclaredField(name);field.setAccessible(true);field.set(target,value);return;}
            catch(NoSuchFieldException absent) {}
        }
        throw new NoSuchFieldException(name);
    }
    @SuppressWarnings("unchecked") private static List<Object> list(Object target,String name) throws Exception {return (List<Object>)get(target,name);}
    private static Object node(Rectangle box,String name,String annotation) throws Exception {
        return loader.loadClass(PREFIX+"p").getConstructor(Rectangle.class,String.class,String.class).newInstance(box,name,annotation);
    }
    private static Object edge(Object from,Object to) throws Exception {
        return loader.loadClass(PREFIX+"s").getConstructor(loader.loadClass(PREFIX+"p"),loader.loadClass(PREFIX+"p")).newInstance(from,to);
    }
    private static final class Fixture {
        final JRootPane root=new JRootPane();final JPanel content=new JPanel(null);
        final JComponent canvas;final Object first;final List<Object> nodes,edges;
        Fixture() throws Exception {
            canvas=(JComponent)loader.loadClass(PREFIX+"q").getConstructor().newInstance();
            root.setContentPane(content);root.setSize(800,500);root.doLayout();
            content.setBackground(Color.WHITE);content.add(canvas);canvas.setBounds(20,20,700,420);
            canvas.setOpaque(true);canvas.setBackground(Color.WHITE);canvas.setForeground(Color.BLACK);
            canvas.setFont(new Font("Noto Sans CJK SC",Font.PLAIN,14));
            nodes=list(canvas,"gSG");edges=list(canvas,"gSH");
            first=node(new Rectangle(60,60,180,45),"S00_Entry",TEMPLATE);nodes.add(first);
        }
        JComponent overlay() {
            for(Component component:root.getLayeredPane().getComponents())
                if(component.getClass().getName().equals("VivadoZhSmartConnectOverlay$Overlay")) return (JComponent)component;
            return null;
        }
        Rectangle prefix(int row) {
            FontMetrics metrics=canvas.getFontMetrics(canvas.getFont());String[] labels={"DOMAIN: ","FREQ: ","PHASE: "};
            int baseline=105+(row+1)*metrics.getHeight();
            return new Rectangle(70,baseline-metrics.getAscent(),metrics.stringWidth(labels[row]),metrics.getAscent()+metrics.getDescent());
        }
        void install() {VivadoZhSmartConnectOverlay.install(canvas);}
    }
    private static BufferedImage paint(JComponent component) {
        BufferedImage image=new BufferedImage(component.getWidth(),component.getHeight(),BufferedImage.TYPE_INT_ARGB);
        Graphics2D graphics=image.createGraphics();component.paint(graphics);graphics.dispose();return image;
    }
    private static int alpha(BufferedImage image) {
        int count=0;for(int y=0;y<image.getHeight();y++) for(int x=0;x<image.getWidth();x++) if((image.getRGB(x,y)>>>24)!=0) count++;
        return count;
    }
    private static int alpha(BufferedImage image,Rectangle box) {
        int count=0;Rectangle clipped=box.intersection(new Rectangle(0,0,image.getWidth(),image.getHeight()));
        for(int y=clipped.y;y<clipped.y+clipped.height;y++) for(int x=clipped.x;x<clipped.x+clipped.width;x++) if((image.getRGB(x,y)>>>24)!=0) count++;
        return count;
    }
    private static String signature(Fixture f) throws Exception {
        StringBuilder text=new StringBuilder();text.append(System.identityHashCode(f.nodes)).append('/').append(System.identityHashCode(f.edges));
        for(Object n:f.nodes) {
            text.append('|').append(System.identityHashCode(n));
            for(String field:new String[]{"name","gTo","gTn","eyM","gTj","gTi","gTl","gTm","gSS","gST","gTE"}) text.append('/').append(get(n,field));
        }
        for(Object e:f.edges) {
            text.append('|').append(System.identityHashCode(e));
            for(String field:new String[]{"gTJ","gTK","gTL","gTM","gTP","gTQ","gTN","gTO"}) text.append('/').append(get(e,field));
        }
        return text.toString();
    }
    private static void modelAndPixels() throws Exception {
        Fixture f=new Fixture();Object other=node(new Rectangle(360,60,180,45),"M00_Exit","");
        f.nodes.add(other);Object connection=edge(f.first,other);f.edges.add(connection);
        String signature=signature(f);Component glass=f.root.getGlassPane();Font font=f.canvas.getFont();
        Object parent=f.canvas.getParent();int childCount=f.root.getLayeredPane().getComponentCount();
        AtomicInteger events=new AtomicInteger();f.canvas.addPropertyChangeListener(event -> events.incrementAndGet());
        BufferedImage before=paint(f.root);f.install();JComponent overlay=f.overlay();
        check(overlay!=null,"exact Clocking canvas receives overlay");
        BufferedImage layer=paint(overlay);check(alpha(layer)>0,"verified Clocking template paints Chinese prefixes");
        for(int row=0;row<3;row++) check(alpha(layer,f.prefix(row))>0,"prefix "+row+" is painted");
        int unexpected=0;
        for(int y=0;y<layer.getHeight();y++) for(int x=0;x<layer.getWidth();x++) if((layer.getRGB(x,y)>>>24)!=0) {
            boolean allowed=false;for(int row=0;row<3;row++) allowed|=f.prefix(row).contains(x,y);if(!allowed) unexpected++;
        }
        check(unexpected==0,"all overlay pixels remain inside three original prefix rectangles");
        BufferedImage after=paint(f.root);int changed=0,outside=0;
        for(int y=0;y<before.getHeight();y++) for(int x=0;x<before.getWidth();x++) if(before.getRGB(x,y)!=after.getRGB(x,y)) {
            changed++;boolean allowed=false;
            for(int row=0;row<3;row++) {Rectangle region=f.prefix(row);region.translate(20,20);allowed|=region.contains(x,y);}
            if(!allowed) outside++;
        }
        check(changed>0&&outside==0,"rendered pixel difference is restricted to prefixes; suffixes and topology pixels unchanged");
        check(signature.equals(signature(f)),"node identities, fields, annotations, edge values and collections unchanged");
        check(f.canvas.getParent()==parent&&f.canvas.getFont()==font&&events.get()==0,"canvas parent, font and properties unchanged");
        check(f.root.getGlassPane()==glass,"original glass pane is preserved");
        check(!overlay.contains(75,120)&&!overlay.isFocusable(),"overlay is mouse-transparent and cannot take focus");
        Point hit=SwingUtilities.convertPoint(f.canvas,75,120,f.root.getLayeredPane());
        check(SwingUtilities.getDeepestComponentAt(f.root.getLayeredPane(),hit.x,hit.y)==f.canvas,"mouse hit testing still reaches canvas");
        f.install();VivadoZhSmartConnectOverlay.refresh();
        check(f.overlay()==overlay&&f.root.getLayeredPane().getComponentCount()==childCount+1,"repeated install and refresh are idempotent");
        if(evidence!=null) {ImageIO.write(before,"png",evidence.resolve("smartconnect-fixture-before.png").toFile());ImageIO.write(after,"png",evidence.resolve("smartconnect-fixture-after.png").toFile());}
        VivadoZhSmartConnectOverlay.restoreAll();
        check(f.overlay()==null&&f.root.getLayeredPane().getComponentCount()==childCount,"restore removes overlay without touching original hierarchy");
        check(signature.equals(signature(f)),"restoration changes no model state");
        BufferedImage restored=paint(f.root);boolean identical=true;
        for(int y=0;y<before.getHeight();y++) for(int x=0;x<before.getWidth();x++) identical&=before.getRGB(x,y)==restored.getRGB(x,y);
        check(identical,"restored image equals original image pixel-for-pixel");
    }
    private static void templateAndLifecycle() throws Exception {
        Fixture f=new Fixture();f.install();JComponent overlay=f.overlay();
        set(f.first,"gTo","DOMAIN: clock_0\nFREQ: 100000000");check(alpha(paint(overlay))==0,"incomplete template excluded");
        set(f.first,"gTo",TEMPLATE+"\nEXTRA: value");check(alpha(paint(overlay))==0,"extra template line excluded");
        set(f.first,"gTo",TEMPLATE.replace("DOMAIN:","OBJECT:"));check(alpha(paint(overlay))==0,"unreviewed template prefix excluded");
        set(f.first,"gTo",TEMPLATE.replace("clock_0","clock_1").replace("100000000","200000000"));
        check(alpha(paint(overlay))>0,"changed suffix data is read live without model mutation");
        set(f.first,"gTo",TEMPLATE);f.canvas.setLocation(50,20);check(alpha(paint(overlay))==0,"stale overlay position is never painted");
        VivadoZhSmartConnectOverlay.refresh();check(alpha(paint(overlay))>0,"refresh follows canvas position");
        set(f.canvas,"showing",false);check(alpha(paint(overlay))==0,"hidden canvas excluded");set(f.canvas,"showing",true);
        set(f.canvas,"gSR","functional");check(alpha(paint(overlay))==0,"mode change suppresses incompatible canvas");set(f.canvas,"gSR","clocking");
        f.content.remove(f.canvas);VivadoZhSmartConnectOverlay.refresh();check(f.overlay()==null,"detached canvas overlay removed");
        VivadoZhSmartConnectOverlay.restoreAll();
        Fixture unrelated=new Fixture();JPanel normal=new JPanel();unrelated.content.add(normal);VivadoZhSmartConnectOverlay.install(normal);
        check(unrelated.overlay()==null,"ordinary JPanel is excluded even if it could draw matching text");
    }
    private static void safetyConditions() throws Exception {
        Fixture f=new Fixture();f.install();JComponent overlay=f.overlay();
        f.canvas.setOpaque(false);check(alpha(paint(overlay))==0,"transparent canvas excluded");f.canvas.setOpaque(true);
        f.canvas.setBackground(new Color(255,255,255,100));check(alpha(paint(overlay))==0,"translucent background excluded");f.canvas.setBackground(Color.WHITE);
        f.canvas.setComponentOrientation(ComponentOrientation.RIGHT_TO_LEFT);check(alpha(paint(overlay))==0,"unverified right-to-left canvas excluded");
        f.canvas.setComponentOrientation(ComponentOrientation.LEFT_TO_RIGHT);
        set(f.first,"gTE",new Color(0,0,0,100));check(alpha(paint(overlay))==0,"translucent annotation text excluded");set(f.first,"gTE",Color.BLACK);
        Font original=f.canvas.getFont();f.canvas.setFont(new Font("Liberation Sans",Font.PLAIN,14));
        check(alpha(paint(overlay))>0,"CJK fallback font paints inside physical Latin-font prefix bounds");
        f.canvas.setFont(new Font("Liberation Sans",Font.PLAIN,14).deriveFont(AffineTransform.getScaleInstance(0.1,1)));
        check(alpha(paint(overlay))==0,"insufficient English prefix width suppresses translation without widening it");f.canvas.setFont(original);
        Rectangle first=f.prefix(0);JPanel child=new JPanel();child.setBounds(first);f.canvas.add(child);
        BufferedImage image=paint(overlay);check(alpha(image,first)==0&&alpha(image,f.prefix(1))>0,"child-control obstruction excludes entire affected prefix");f.canvas.remove(child);
        JPanel sibling=new JPanel();Rectangle cover=new Rectangle(first);cover.translate(20,20);sibling.setBounds(cover);f.content.add(sibling,0);
        image=paint(overlay);check(alpha(image,first)==0&&alpha(image,f.prefix(1))>0,"foreground sibling obstruction excluded");f.content.remove(sibling);
        JPanel popup=new JPanel();popup.setBounds(0,0,800,500);f.root.getLayeredPane().add(popup,JLayeredPane.POPUP_LAYER);
        check(alpha(paint(overlay))==0,"foreground popup prevents painting through it");f.root.getLayeredPane().remove(popup);
        Object obstruction=node(new Rectangle(60,105,180,90),"obstruction","");f.nodes.add(obstruction);
        check(alpha(paint(overlay))==0,"other node or port shapes are never erased");f.nodes.remove(obstruction);
        Object source=node(new Rectangle(0,60,45,45),"edge_source","");Object destination=node(new Rectangle(180,180,45,45),"edge_target","");
        f.nodes.add(source);f.nodes.add(destination);Object crossing=edge(source,destination);f.edges.add(crossing);
        image=paint(overlay);check(alpha(image,first)==0,"crossing connection line suppresses affected prefix");
        f.edges.clear();f.nodes.remove(source);f.nodes.remove(destination);
        Object param=loader.loadClass(PREFIX+"v").getConstructor().newInstance();set(param,"gUh",new Rectangle(first));set(param,"gUc",1);
        list(f.first,"gTb").add(param);check(alpha(paint(overlay),first)==0,"property rectangle/text obstruction excluded");list(f.first,"gTb").clear();
        set(param,"gUh",null);set(param,"gUc",0);list(f.first,"gTb").add(param);
        check(alpha(paint(overlay))>0,"hidden native property may have null unpainted geometry");
        set(param,"gUc",1);check(alpha(paint(overlay))==0,"visible property without completed geometry suppresses overlay");list(f.first,"gTb").clear();
        check(alpha(paint(overlay))>0,"translation resumes after obstruction removal");
        VivadoZhSmartConnectOverlay.restoreAll();
    }
    private static void clipping() throws Exception {
        Fixture f=new Fixture();JPanel viewport=new JPanel(null);viewport.setBounds(20,20,100,420);
        f.content.remove(f.canvas);f.content.add(viewport);viewport.add(f.canvas);f.canvas.setLocation(0,0);
        f.install();check(alpha(paint(f.overlay()))==0,"partially clipped prefixes are left wholly unchanged");
        viewport.setSize(300,420);VivadoZhSmartConnectOverlay.refresh();check(alpha(paint(f.overlay()))>0,"fully visible prefixes paint after viewport expansion");
        VivadoZhSmartConnectOverlay.restoreAll();
    }
    private static long diagnosticCount(ByteArrayOutputStream output,String detail) {
        return output.toString(StandardCharsets.UTF_8).lines().filter(line -> line.contains(detail)).count();
    }
    private static void nullablePropertiesAndFailureIsolation() throws Exception {
        Fixture broken=new Fixture(),healthy=new Fixture();
        Object properties=get(broken.first,"gTb");
        set(broken.first,"gTb",null);broken.install();
        check(broken.overlay()!=null&&alpha(paint(broken.overlay()))>0,"native null Node.gTb means an empty property list");
        check(get(broken.first,"gTb")==null,"accepting null properties does not initialize or mutate native model");
        set(broken.first,"gTb",properties);broken.install();
        check(alpha(paint(broken.overlay()))>0,"empty properties and null properties both paint safely");
        PrintStream previous=System.err;ByteArrayOutputStream output=new ByteArrayOutputStream();
        try(PrintStream capture=new PrintStream(output,true,StandardCharsets.UTF_8)) {
            System.setErr(capture);
            JComponent original=broken.overlay();set(broken.canvas,"gSG",null);
            for(Component component:new Component[]{broken.canvas,healthy.canvas}) VivadoZhSmartConnectOverlay.install(component);
            check(broken.overlay()==null,"missing required node collection removes the previously installed overlay");
            check(alpha(paint(original))==0,"retired overlay cannot repaint stale text even through a retained reference");
            check(healthy.overlay()!=null&&alpha(paint(healthy.overlay()))>0,"one unsupported canvas does not interrupt installation on the next canvas");
            broken.install();broken.install();
            check(diagnosticCount(output,"collection: gSG")==1,"identical install errors log once per canvas");
            set(broken.canvas,"gSG",broken.nodes);broken.install();
            check(broken.overlay()!=null&&broken.overlay()!=original&&alpha(paint(broken.overlay()))>0,"restored structure retries with a new functioning overlay");
            set(broken.canvas,"gSG",null);broken.install();
            check(diagnosticCount(output,"collection: gSG")==1,"same diagnostic stays suppressed after recovery and another failure");
            set(broken.canvas,"gSG",broken.nodes);broken.install();
            set(broken.canvas,"gSH",null);VivadoZhSmartConnectOverlay.refresh();
            check(broken.overlay()==null,"required edge collection remains strict during refresh");
            check(healthy.overlay()!=null&&alpha(paint(healthy.overlay()))>0,"refresh failure is isolated from a healthy canvas");
            broken.install();check(diagnosticCount(output,"collection: gSH")==1,"refresh and install share per-canvas diagnostic deduplication");
            set(broken.canvas,"gSH",broken.edges);broken.install();
            check(broken.overlay()!=null&&alpha(paint(broken.overlay()))>0,"canvas reinstalls after required edge collection returns");
            JComponent stale=broken.overlay();set(broken.first,"gTE",null);
            check(alpha(paint(stale))==0&&broken.overlay()==null,"paint-time structural failure removes its overlay and paints no stale translation");
            check(alpha(paint(stale))==0&&diagnosticCount(output,"annotation color")==1,"retired paint callbacks do not repeat diagnostics");
            set(broken.first,"gTE",Color.BLACK);broken.install();
            check(alpha(paint(broken.overlay()))>0,"paint-time failure can recover on the next scan");
            list(broken.first,"gTb").add(new Object());broken.install();
            check(broken.overlay()==null,"non-null property collections still reject unsupported members");
            list(broken.first,"gTb").clear();broken.install();
            check(alpha(paint(broken.overlay()))>0,"corrected property contents recover without replacing native collections");
            check(get(broken.first,"gTb")==properties&&get(broken.canvas,"gSG")==broken.nodes&&get(broken.canvas,"gSH")==broken.edges,
                "failure handling preserves all restored native collection identities");
        } finally {System.setErr(previous);VivadoZhSmartConnectOverlay.restoreAll();}
    }
    private static WeakReference<JComponent> detached(JRootPane[] retained) throws Exception {
        Fixture f=new Fixture();f.install();f.content.remove(f.canvas);retained[0]=f.root;return new WeakReference<>(f.canvas);
    }
    private static URLClassLoader fixtures() throws Exception {
        JavaCompiler compiler=ToolProvider.getSystemJavaCompiler();if(compiler==null) throw new IllegalStateException("Tests require a JDK compiler");
        Path dir=Files.createTempDirectory("vivado-smartconnect-fixture-");Path pkg=dir.resolve("ui/data/coregen/xpegua/ipwidgets/y");Files.createDirectories(pkg);
        String header="package ui.data.coregen.xpegua.ipwidgets.y; import java.awt.*; import java.util.*; import javax.swing.*; ";
        Files.writeString(pkg.resolve("p.java"),header+"public class p {"+
            "Rectangle gTn; String name,gTo; Color gTE=Color.BLACK; int eyM,gTj,gTi=15,gTl,gTm=3,gSS=1,gST=1; java.util.List<v> gTb=new ArrayList<>();"+
            "public p(Rectangle r,String n,String a){gTn=new Rectangle(r);name=n;gTo=a;eyM=r.x/15-1;gTj=r.y/15-1;gTl=r.width/15;} }");
        Files.writeString(pkg.resolve("v.java"),header+"public class v {Rectangle gUh=new Rectangle();String name=\"REGSLICE\",drG=\"1\";int gUc=0;public v(){} }");
        Files.writeString(pkg.resolve("s.java"),header+"public class s {p gTJ,gTK;int gTL,gTM;String gTP=\"\",gTQ=\"\";v gTN,gTO;public s(p a,p b){gTJ=a;gTK=b;} }");
        Files.writeString(pkg.resolve("n.java"),header+"public class n extends JPanel {ArrayList<p>gSG=new ArrayList<>();ArrayList<s>gSH=new ArrayList<>();String gSR=\"clocking\";public n(){setLayout(null);} }");
        Files.writeString(pkg.resolve("q.java"),header+"public class q extends n {public boolean showing=true;public q(){} public boolean isShowing(){return showing;}"+
            "protected void paintComponent(Graphics raw){super.paintComponent(raw);Graphics2D g=(Graphics2D)raw;FontMetrics m=g.getFontMetrics();"+
            "for(p n:gSG){g.setColor(Color.LIGHT_GRAY);g.fill(n.gTn);g.setColor(Color.BLACK);g.draw(n.gTn);"+
            "g.drawString(n.name,n.gTn.x+10,n.gTn.y-m.getHeight()+9);String[]lines=n.gTo.split(\"\\n\",-1);"+
            "for(int i=0;i<lines.length;i++)g.drawString(lines[i],n.gTn.x+10,n.gTn.y+n.gTn.height+(i+1)*m.getHeight());}"+
            "for(s e:gSH){p a=e.gTJ,b=e.gTK;g.setColor(Color.BLACK);g.drawLine((a.eyM+a.gTl+2)*a.gTi,(a.gTj+2+e.gTL*a.gTm)*a.gTi+a.gTi/2,"+
            "b.eyM*b.gTi,(b.gTj+2+e.gTM*b.gTm)*b.gTi+b.gTi/2);} } }");
        int exit=compiler.run(null,null,null,"--release","11","-classpath",dir.toString(),"-d",dir.toString(),pkg.resolve("p.java").toString(),pkg.resolve("v.java").toString(),pkg.resolve("s.java").toString(),pkg.resolve("n.java").toString(),pkg.resolve("q.java").toString());
        check(exit==0,"independent SmartConnect-shaped fixture compiles without AMD classes or JNI");
        return new URLClassLoader(new java.net.URL[]{dir.toUri().toURL()},ClassLoader.getPlatformClassLoader());
    }
    public static void main(String[] args) throws Exception {
        if(args.length>0) {evidence=Path.of(args[0]).toAbsolutePath();Files.createDirectories(evidence);}
        try(URLClassLoader fixtureLoader=fixtures()) {
            loader=fixtureLoader;
            boolean rejected=false;try {VivadoZhSmartConnectOverlay.install(new JPanel());} catch(IllegalStateException expected){rejected=true;}
            check(rejected,"off-EDT install rejected");
            rejected=false;try {VivadoZhSmartConnectOverlay.refresh();} catch(IllegalStateException expected){rejected=true;}
            check(rejected,"off-EDT refresh rejected");
            rejected=false;try {VivadoZhSmartConnectOverlay.restoreAll();} catch(IllegalStateException expected){rejected=true;}
            check(rejected,"off-EDT restore rejected");
            SwingUtilities.invokeAndWait(() -> {try {modelAndPixels();templateAndLifecycle();safetyConditions();clipping();nullablePropertiesAndFailureIsolation();} catch(Exception error){throw new RuntimeException(error);}});
            JRootPane[] retained=new JRootPane[1];WeakReference<?>[] weak=new WeakReference<?>[1];
            SwingUtilities.invokeAndWait(() -> {try {weak[0]=detached(retained);} catch(Exception error){throw new RuntimeException(error);}});
            for(int i=0;i<20&&weak[0].get()!=null;i++){System.gc();Thread.sleep(30);}
            check(weak[0].get()==null,"adapter does not retain a detached canvas or topology");
            SwingUtilities.invokeAndWait(() -> {VivadoZhSmartConnectOverlay.refresh();check(retained[0].getLayeredPane().getComponentCount()==1,"orphan overlay removed after target collection");VivadoZhSmartConnectOverlay.restoreAll();});
        }
        if(evidence!=null) {
            ArrayList<String> rows=new ArrayList<>();rows.add("{");rows.add("  \"result\": \"PASS\",");rows.add("  \"scope\": \"headless independent class-shape and pixel contracts; no Vivado JNI or GUI\",");
            rows.add("  \"native_gui_verified\": false,");rows.add("  \"checks_passed\": "+checks.size()+",");rows.add("  \"checks\": [");
            for(int i=0;i<checks.size();i++) rows.add("    \""+checks.get(i).replace("\\","\\\\").replace("\"","\\\"")+"\""+(i+1<checks.size()?",":""));
            rows.add("  ]");rows.add("}");Files.write(evidence.resolve("smartconnect-overlay-contract.json"),rows,StandardCharsets.UTF_8);
        }
        System.out.println("VivadoZhSmartConnectOverlayTest PASS: "+checks.size()+" checks (mock class-shape/pixel contracts; native GUI not verified)");
    }
}
