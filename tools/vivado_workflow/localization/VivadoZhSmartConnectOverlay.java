import java.awt.*;
import java.awt.font.GlyphVector;
import java.awt.geom.Area;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.lang.ref.WeakReference;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.HashSet;
import java.util.Set;
import java.util.WeakHashMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import javax.swing.*;

/**
 * Experimental display-only adapter for Vivado 2025.2 SmartConnect Clocking.
 * Only DOMAIN/FREQ/PHASE prefixes in the verified three-line annotation template
 * are painted. Node/port/property names, values and all native models stay intact.
 * No application setter, renderer replacement, bytecode patch or paint replay is
 * used: reflection only reads a current geometry snapshot on the Swing EDT.
 */
public final class VivadoZhSmartConnectOverlay {
    private static final String PACKAGE="ui.data.coregen.xpegua.ipwidgets.y.";
    private static final String CANVAS=PACKAGE+"q",BASE=PACKAGE+"n",NODE=PACKAGE+"p",EDGE=PACKAGE+"s",PARAM=PACKAGE+"v";
    private static final String[] EN={"DOMAIN: ","FREQ: ","PHASE: "};
    private static final String[] ZH={"时钟域: ","频率: ","相位: "};
    private static final Pattern TEMPLATE=Pattern.compile("\\ADOMAIN: ([^\\r\\n]*)\\nFREQ: ([^\\r\\n]*)\\nPHASE: ([^\\r\\n]*)\\z");
    private static final Map<JComponent,WeakReference<Overlay>> canvases=new WeakHashMap<>();
    // Keep deduplication through recovery/reinstallation. Weak keys retain no canvas.
    private static final Map<JComponent,Set<String>> diagnostics=new WeakHashMap<>();
    private static final ArrayList<WeakReference<Overlay>> layers=new ArrayList<>();
    private static final ClassValue<Map<String,Field>> fields=new ClassValue<Map<String,Field>>() {
        @Override protected Map<String,Field> computeValue(Class<?> type) {
            Map<String,Field> result=new java.util.HashMap<>();
            for(Field field:type.getDeclaredFields()) {
                if(java.lang.reflect.Modifier.isStatic(field.getModifiers())) continue;
                field.setAccessible(true);result.put(field.getName(),field);
            }
            return result;
        }
    };

    private VivadoZhSmartConnectOverlay() {}

    /** Safe to offer every scanned component. Only the exact supported canvas is accepted. */
    public static void install(Component component) {
        requireEdt();
        if(!(component instanceof JComponent)||!component.getClass().getName().equals(CANVAS)) return;
        JComponent canvas=(JComponent)component;
        Overlay overlay=deref(canvases.get(canvas));
        try {
            // An incomplete native scene is local to this canvas, never a scan failure.
            Scene.read(canvas);
            JRootPane root=SwingUtilities.getRootPane(canvas);
            if(root==null||root.getLayeredPane()==null) {discard(canvas,overlay);return;}
            JLayeredPane host=root.getLayeredPane();
            if(overlay==null||overlay.getParent()!=host) {
                discard(canvas,overlay);
                int layer=host.getLayer(root.getContentPane())+1;
                if(layer>=JLayeredPane.PALETTE_LAYER) return;
                overlay=new Overlay(canvas);host.add(overlay,Integer.valueOf(layer),0);
                canvases.put(canvas,new WeakReference<>(overlay));layers.add(new WeakReference<>(overlay));
            }
            overlay.refresh();
        } catch(RuntimeException unsupported) {
            failed(canvas,overlay,unsupported);
        }
    }

    public static void refresh() {
        requireEdt();
        for(Iterator<WeakReference<Overlay>> iterator=layers.iterator();iterator.hasNext();) {
            Overlay overlay=iterator.next().get();
            if(overlay==null) {iterator.remove();continue;}
            JComponent canvas=overlay.canvas.get();JRootPane root=canvas==null?null:SwingUtilities.getRootPane(canvas);
            if(root==null||overlay.getParent()==null||root.getLayeredPane()!=overlay.getParent()) {
                overlay.remove();iterator.remove();
                if(canvas!=null&&deref(canvases.get(canvas))==overlay) canvases.remove(canvas);
            } else {
                try {Scene.read(canvas);overlay.refresh();}
                catch(RuntimeException unsupported) {failed(canvas,overlay,unsupported);iterator.remove();}
            }
        }
    }

    public static void restoreAll() {
        requireEdt();
        for(WeakReference<Overlay> reference:layers) {Overlay overlay=reference.get();if(overlay!=null) overlay.remove();}
        layers.clear();canvases.clear();
    }

    private static Overlay deref(WeakReference<Overlay> ref) {return ref==null?null:ref.get();}
    private static void discard(JComponent canvas,Overlay overlay) {
        if(overlay!=null) overlay.remove();
        if(deref(canvases.get(canvas))==overlay) canvases.remove(canvas);
    }
    private static void failed(JComponent canvas,Overlay overlay,RuntimeException error) {
        discard(canvas,overlay);
        String detail=error.getClass().getName()+": "+String.valueOf(error.getMessage());
        if(diagnostics.computeIfAbsent(canvas,key -> new HashSet<>()).add(detail))
            System.err.println("[VivadoZhSmartConnectOverlay] Canvas "+Integer.toHexString(System.identityHashCode(canvas))+" skipped: "+detail);
    }
    private static void requireEdt() {
        if(!SwingUtilities.isEventDispatchThread()) throw new IllegalStateException("SmartConnect display adapter requires the Swing EDT");
    }
    private static Object read(Object object,Class<?> owner,String name) {
        Field field=fields.get(owner).get(name);
        if(field==null) throw new IllegalStateException("Unsupported SmartConnect display field: "+owner.getName()+"."+name);
        try {return field.get(object);} catch(IllegalAccessException error) {throw new IllegalStateException("Cannot read SmartConnect display geometry",error);}
    }
    private static Object read(Object object,String name) {return read(object,object.getClass(),name);}
    private static int number(Object object,String name) {
        Object value=read(object,name);if(!(value instanceof Integer)) throw new IllegalStateException("Unsupported SmartConnect integer field: "+name);
        return (Integer)value;
    }
    private static String string(Object object,String name) {
        Object value=read(object,name);if(value==null) return "";
        if(!(value instanceof String)) throw new IllegalStateException("Unsupported SmartConnect text field: "+name);return (String)value;
    }
    private static Rectangle rectangle(Object object,String name) {
        Object value=read(object,name);
        if(!(value instanceof Rectangle)) throw new IllegalStateException("Unsupported SmartConnect rectangle: "+name);
        return new Rectangle((Rectangle)value);
    }
    private static List<?> list(Object object,Class<?> owner,String name) {
        Object value=read(object,owner,name);
        if(!(value instanceof List<?>)) throw new IllegalStateException("Unsupported SmartConnect collection: "+name);
        return new ArrayList<>((List<?>)value);
    }
    private static void exact(Object value,String name) {
        if(value==null||!value.getClass().getName().equals(name)) throw new IllegalStateException("Unsupported SmartConnect display object: "+name);
    }

    private static final class Parameter {
        final Rectangle box;final String name,value;final int mode;
        Parameter(Object raw) {
            exact(raw,PARAM);name=string(raw,"name");value=string(raw,"drG");mode=number(raw,"gUc");
            // Native v.gUh starts null and is assigned only when a visible
            // property is painted. An absent layout is never invented here.
            Object geometry=read(raw,"gUh");
            if(geometry!=null&&!(geometry instanceof Rectangle)) throw new IllegalStateException("Unsupported SmartConnect property geometry");
            box=geometry==null?null:new Rectangle((Rectangle)geometry);
        }
    }
    private static final class Node {
        final Rectangle box;final String name,annotation;final Color color;
        final int x,y,grid,width,spacing,inputs,outputs;final ArrayList<Parameter> parameters=new ArrayList<>();
        Node(Object raw) {
            exact(raw,NODE);box=rectangle(raw,"gTn");name=string(raw,"name");annotation=string(raw,"gTo");
            Object ink=read(raw,"gTE");if(!(ink instanceof Color)) throw new IllegalStateException("Unsupported SmartConnect annotation color");color=(Color)ink;
            x=number(raw,"eyM");y=number(raw,"gTj");grid=number(raw,"gTi");width=number(raw,"gTl");spacing=number(raw,"gTm");
            inputs=number(raw,"gSS");outputs=number(raw,"gST");
            // Native p.d(Graphics2D) explicitly skips both property passes when gTb
            // is null. Only this collection is optional; scene lists remain required.
            Object rawParameters=read(raw,"gTb");
            if(rawParameters!=null) {
                if(!(rawParameters instanceof List<?>)) throw new IllegalStateException("Unsupported SmartConnect collection: gTb");
                for(Object parameter:new ArrayList<>((List<?>)rawParameters)) parameters.add(new Parameter(parameter));
            }
            if(grid<=0||grid>1000||width<=0||spacing<=0||inputs<0||outputs<0||inputs>4096||outputs>4096)
                throw new IllegalStateException("Invalid SmartConnect display geometry");
        }
        Point input(int index) {return new Point(x*grid,(y+2+index*spacing)*grid+grid/2);}
        Point output(int index) {return new Point((x+width+2)*grid,(y+2+index*spacing)*grid+grid/2);}
    }
    private static final class Edge {
        final Point source,destination;final String upper,lower;
        Edge(Object raw,IdentityHashMap<Object,Node> nodes) {
            exact(raw,EDGE);Node from=nodes.get(read(raw,"gTJ")),to=nodes.get(read(raw,"gTK"));
            if(from==null||to==null) throw new IllegalStateException("Unknown SmartConnect connection endpoint");
            int output=number(raw,"gTL"),input=number(raw,"gTM");
            if(output<0||output>=from.outputs||input<0||input>=to.inputs)
                throw new IllegalStateException("Unverified SmartConnect connection index");
            source=from.output(output);destination=to.input(input);
            upper=annotation(raw,"gTN","gTP");lower=annotation(raw,"gTO","gTQ");
        }
        private static String annotation(Object raw,String param,String fallback) {
            Object parameter=read(raw,param);if(parameter==null) return string(raw,fallback);
            exact(parameter,PARAM);return string(parameter,"drG");
        }
    }
    private static final class Scene {
        final ArrayList<Node> nodes=new ArrayList<>();final ArrayList<Edge> edges=new ArrayList<>();
        static Scene read(JComponent canvas) {
            exact(canvas,CANVAS);Class<?> base=canvas.getClass().getSuperclass();
            if(!base.getName().equals(BASE)||!"clocking".equals(VivadoZhSmartConnectOverlay.read(canvas,base,"gSR")))
                throw new IllegalStateException("Unsupported SmartConnect canvas mode");
            Scene result=new Scene();IdentityHashMap<Object,Node> index=new IdentityHashMap<>();
            List<?> rawNodes=list(canvas,base,"gSG"),rawEdges=list(canvas,base,"gSH");
            if(rawNodes.size()>4096||rawEdges.size()>16384) throw new IllegalStateException("SmartConnect display snapshot exceeds bounded scope");
            for(Object raw:rawNodes) {Node node=new Node(raw);result.nodes.add(node);index.put(raw,node);}
            for(Object raw:rawEdges) result.edges.add(new Edge(raw,index));
            return result;
        }
    }

    private static Rectangle textBox(FontMetrics metrics,String text,int x,int baseline) {
        return new Rectangle(x,baseline-metrics.getAscent(),metrics.stringWidth(text),metrics.getAscent()+metrics.getDescent());
    }
    private static void addBox(Area area,Rectangle box,int pad) {
        if(box.width<=0||box.height<=0) return;
        Rectangle expanded=new Rectangle(box);expanded.grow(pad,pad);area.add(new Area(expanded));
    }

    private static final class Overlay extends JComponent {
        final WeakReference<JComponent> canvas;
        private boolean retired;
        Overlay(JComponent target) {canvas=new WeakReference<>(target);setOpaque(false);setFocusable(false);setRequestFocusEnabled(false);setDoubleBuffered(false);}
        @Override public boolean contains(int x,int y) {return false;}
        void refresh() {
            JComponent target=canvas.get();Container host=getParent();if(target==null||host==null) return;
            Rectangle old=getBounds(),next=SwingUtilities.convertRectangle(target,new Rectangle(0,0,target.getWidth(),target.getHeight()),host);
            if(!old.equals(next)) {setBounds(next);host.repaint(old.x,old.y,old.width,old.height);}
            repaint();
        }
        void remove() {
            retired=true;
            Container host=getParent();if(host!=null) {Rectangle old=getBounds();host.remove(this);host.repaint(old.x,old.y,old.width,old.height);}
        }
        private Area visibleArea(JComponent target) {
            Area area=new Area(new Rectangle(0,0,getWidth(),getHeight()));
            for(Component child:target.getComponents()) if(child.isVisible())
                area.subtract(new Area(SwingUtilities.convertRectangle(target,child.getBounds(),this)));
            Component child=target;
            for(Container parent=child.getParent();parent!=null;child=parent,parent=parent.getParent()) {
                if(!parent.isVisible()) return new Area();
                area.intersect(new Area(SwingUtilities.convertRectangle(parent,new Rectangle(0,0,parent.getWidth(),parent.getHeight()),this)));
                int order=parent.getComponentZOrder(child);
                for(int i=0;i<order;i++) {
                    Component sibling=parent.getComponent(i);
                    if(sibling==this||sibling instanceof Overlay||!sibling.isVisible()) continue;
                    area.subtract(new Area(SwingUtilities.convertRectangle(parent,sibling.getBounds(),this)));
                }
                if(parent==getParent()) return area;
            }
            return new Area();
        }
        @Override protected void paintComponent(Graphics graphics) {
            if(retired) return;
            JComponent target=canvas.get();Container host=getParent();
            if(target==null||host==null||!target.isShowing()||!target.isOpaque()
                    ||!target.getComponentOrientation().isLeftToRight()) return;
            JRootPane root=SwingUtilities.getRootPane(target);
            if(root==null||root.getLayeredPane()!=host) return;
            Rectangle current=SwingUtilities.convertRectangle(target,new Rectangle(0,0,target.getWidth(),target.getHeight()),host);
            if(!current.equals(getBounds())) return;
            Color background=target.getBackground();Font original=target.getFont();
            if(background==null||background.getAlpha()!=255||original==null) return;
            final Scene scene;
            try {scene=Scene.read(target);} catch(RuntimeException unsafe) {failed(target,this,unsafe);return;}
            Graphics2D g=(Graphics2D)graphics.create();
            try {
                FontMetrics metrics=g.getFontMetrics(original);int height=metrics.getHeight();
                if(height<=0) return;
                Area visible=visibleArea(target);if(visible.isEmpty()) return;
                Area obstacles=obstacles(scene,g,original,metrics);if(obstacles==null) return;
                for(Node node:scene.nodes) {
                    if(node.box.width<=0||node.box.height<=0||node.color.getAlpha()!=255) continue;
                    Matcher template=TEMPLATE.matcher(node.annotation);if(!template.matches()) continue;
                    int x=node.box.x+10;
                    for(int row=0;row<3;row++) {
                        int baseline=node.box.y+node.box.height+(row+1)*height;
                        Rectangle erase=textBox(metrics,EN[row],x,baseline);
                        // A partially clipped or covered prefix is left wholly in English.
                        if(erase.width<=0||!visible.contains(erase)||obstacles.intersects(erase)) continue;
                        if(intersectsOtherAnnotations(scene,node,row,erase,metrics)) continue;
                        Font translated=original.canDisplayUpTo(ZH[row])<0?original:
                            new Font("Noto Sans CJK SC",original.getStyle(),original.getSize()).deriveFont(original.getSize2D());
                        if(translated.canDisplayUpTo(ZH[row])>=0||g.getFontMetrics(translated).stringWidth(ZH[row])>erase.width) continue;
                        GlyphVector glyphs=translated.createGlyphVector(g.getFontRenderContext(),ZH[row]);
                        Rectangle2D ink=glyphs.getVisualBounds();
                        Rectangle2D placed=new Rectangle2D.Double(x+ink.getX(),baseline+ink.getY(),ink.getWidth(),ink.getHeight());
                        if(!erase.contains(placed)) continue;
                        g.setColor(background);g.fill(erase);g.setFont(translated);g.setColor(node.color);g.drawString(ZH[row],x,baseline);
                    }
                }
            } finally {g.dispose();}
        }

        private boolean intersectsOtherAnnotations(Scene scene,Node own,int ownRow,Rectangle erase,FontMetrics metrics) {
            for(Node node:scene.nodes) {
                String[] lines=node.annotation.split("\\n",-1);
                for(int row=0;row<lines.length;row++) {
                    if(node==own&&row==ownRow) continue;
                    Rectangle text=textBox(metrics,lines[row],node.box.x+10,node.box.y+node.box.height+(row+1)*metrics.getHeight());
                    if(text.intersects(erase)) return true;
                }
            }
            return false;
        }

        private Area obstacles(Scene scene,Graphics2D g,Font font,FontMetrics metrics) {
            Area result=new Area();
            Font small=font.deriveFont(font.getSize()*0.9f);FontMetrics smallMetrics=g.getFontMetrics(small);
            for(Node node:scene.nodes) {
                if(node.box.width<=0||node.box.height<=0) return null;
                addBox(result,node.box,2);
                addBox(result,textBox(metrics,node.name,node.box.x+10,node.box.y-metrics.getHeight()+9),1);
                for(int i=0;i<node.inputs;i++) addBox(result,new Rectangle(node.box.x-node.grid,node.box.y+node.grid+i*node.spacing*node.grid,node.grid,node.grid),2);
                for(int i=0;i<node.outputs;i++) addBox(result,new Rectangle(node.box.x+node.box.width,node.box.y+node.grid+i*node.spacing*node.grid,node.grid,node.grid),2);
                for(Parameter parameter:node.parameters) if(parameter.mode==1) {
                    if(parameter.box==null||parameter.box.width<=0||parameter.box.height<=0) return null;
                    addBox(result,parameter.box,2);
                    addBox(result,textBox(smallMetrics,parameter.name,parameter.box.x+parameter.box.width+node.grid,parameter.box.y+parameter.box.height-3),1);
                    addBox(result,textBox(smallMetrics,parameter.value,parameter.box.x+5,parameter.box.y+smallMetrics.getHeight()-2),1);
                }
            }
            for(Edge edge:scene.edges) {
                Line2D line=new Line2D.Double(edge.source,edge.destination);
                result.add(new Area(new BasicStroke(3f).createStrokedShape(line)));
                int dx=edge.destination.x-edge.source.x;
                addBox(result,textBox(metrics,edge.upper,edge.source.x+(dx-metrics.stringWidth(edge.upper))/2,edge.source.y-2),1);
                addBox(result,textBox(metrics,edge.lower,edge.source.x+(dx-metrics.stringWidth(edge.lower))/2,edge.source.y+metrics.getHeight()),1);
            }
            return result;
        }
    }
}
