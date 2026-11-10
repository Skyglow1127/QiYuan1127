import java.awt.Font;
import java.io.*;
import java.net.URL;
import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import javax.swing.*;
import javax.swing.event.*;
import javax.swing.text.*;
import javax.swing.text.html.*;

/** Headless regression tests for lossless read-only HTML presentation copies. */
public final class VivadoZhHtmlTest {
    private static int checks;
    private static void check(boolean condition,String description) {
        checks++;if(!condition) throw new AssertionError(description);
    }
    private static final Map<String,String> WORDS=new LinkedHashMap<>();
    static {
        WORDS.put("Twelve","十二点");WORDS.put("Seven","七点");WORDS.put("Legacy","传统字号");
        WORDS.put("Nested","嵌套");WORDS.put("Link","链接");WORDS.put("Open","打开");
        WORDS.put("Header","标题");WORDS.put("Plain","正文");WORDS.put("First","第一");WORDS.put("Second","第二");
    }
    private static String translate(String text) {return WORDS.getOrDefault(text,text);}
    private static HTMLDocument document(String source) {
        return (HTMLDocument)new JEditorPane("text/html",source).getDocument();
    }
    private static String text(Document document) throws BadLocationException {
        return document.getText(0,document.getLength());
    }
    private static AttributeSet attributes(HTMLDocument document,String word) throws BadLocationException {
        int index=text(document).indexOf(word);check(index>=0,"expected text exists: "+word);
        return document.getCharacterElement(index).getAttributes();
    }
    private static void fontUnchanged(HTMLDocument source,HTMLDocument copy,String before,String after) throws Exception {
        AttributeSet a=attributes(source,before),b=attributes(copy,after);
        Font original=source.getFont(a),translated=copy.getFont(b);
        check(original.equals(translated),"font family/style/size preserved for "+before+": "+original+" / "+translated);
        Object cssBefore=a.getAttribute(CSS.Attribute.FONT_SIZE),cssAfter=b.getAttribute(CSS.Attribute.FONT_SIZE);
        check(Objects.equals(String.valueOf(cssBefore),String.valueOf(cssAfter)),"CSS font-size representation preserved for "+before);
    }
    private static void fontAndStyles() throws Exception {
        HTMLDocument source=document("<html><head><style>p {color:#126234} .note {font-style:italic}</style></head><body>"
            +"<p><span style='font-size:12 ; font-family:Dialog'>Twelve</span> "
            +"<span style='font-size:7pt'>Seven</span> <font size='3'>Legacy</font> "
            +"<span style='font-size:9pt;font-family:Monospaced'><b><span style='font-size:12pt'>Nested</span></b></span></p>"
            +"<p class='note'>Plain</p></body></html>");
        String original=text(source);AtomicInteger changed=new AtomicInteger();
        source.addDocumentListener(new DocumentListener() {
            public void insertUpdate(DocumentEvent e) {changed.incrementAndGet();}
            public void removeUpdate(DocumentEvent e) {changed.incrementAndGet();}
            public void changedUpdate(DocumentEvent e) {changed.incrementAndGet();}
        });
        int listenerCount=source.getDocumentListeners().length;
        HTMLDocument copy=VivadoZhHtml.translatedCopy(source,VivadoZhHtmlTest::translate);
        check(copy!=null&&copy!=source,"separate translated document produced");
        check(text(source).equals(original)&&changed.get()==0,"source document/content/events unchanged");
        check(source.getDocumentListeners().length==listenerCount,"source listeners were never detached");
        check(copy.getDocumentListeners().length==0&&copy.getUndoableEditListeners().length==0,"copy has no external document listeners");
        check(source.getStyleSheet()!=copy.getStyleSheet(),"stylesheet copied independently");
        fontUnchanged(source,copy,"Twelve","十二点");
        fontUnchanged(source,copy,"Seven","七点");
        fontUnchanged(source,copy,"Legacy","传统字号");
        fontUnchanged(source,copy,"Nested","嵌套");
        check(copy.getFont(attributes(copy,"十二点")).getSize()==12,"unitless CSS 12 stays 12 points, never HTML maximum size");
        check(copy.getFont(attributes(copy,"七点")).getSize()==7,"CSS 7pt stays 7 points");
        AttributeSet legacy=(AttributeSet)attributes(copy,"传统字号").getAttribute(HTML.Tag.FONT);
        check("3".equals(legacy.getAttribute(HTML.Attribute.SIZE)),"explicit HTML font size=3 kept as HTML size=3");
        check(copy.getStyleSheet().getRule("p").getAttribute(CSS.Attribute.COLOR).toString().equals("#126234"),"stylesheet color rule retained");
        copy.getStyleSheet().addRule("p {color:#ff0000}");
        check(source.getStyleSheet().getRule("p").getAttribute(CSS.Attribute.COLOR).toString().equals("#126234"),"changing copied styles does not mutate source styles");
        check(source.getFont(attributes(source,"Twelve")).getSize()==12&&changed.get()==0,"source style and document still unchanged after copy edits");
    }
    private static void linksAndStructure() throws Exception {
        HTMLDocument source=document("<html><head><title>Header</title></head><body><p><a href='action:Open?id=3#part' target='_blank'><b>Link</b></a>"
            +" <code>Open</code> <span>First</span><i>Second</i></p><pre>Open</pre><p>Open</p></body></html>");
        URL base=new URL("https://example.invalid/ip/help/");source.setBase(base);
        HTMLDocument copy=VivadoZhHtml.translatedCopy(source,VivadoZhHtmlTest::translate);
        check(base.equals(copy.getBase()),"hyperlink base URL retained");
        AttributeSet link=(AttributeSet)attributes(copy,"链接").getAttribute(HTML.Tag.A);
        check("action:Open?id=3#part".equals(link.getAttribute(HTML.Attribute.HREF)),"hyperlink action, query and fragment exact");
        check("_blank".equals(link.getAttribute(HTML.Attribute.TARGET)),"hyperlink target preserved");
        check(copy.getFont(attributes(copy,"链接")).isBold(),"nested hyperlink emphasis preserved");
        check(text(copy).contains("第一第二"),"adjacent differently styled leaves translated independently in place");
        check(copy.getFont(attributes(copy,"第二")).isItalic(),"second leaf italic formatting preserved");
        check(text(copy).contains("Open")&&text(copy).contains("打开"),"code/pre left English while ordinary prose translated");
        check(Objects.equals(source.getProperty(Document.TitleProperty),copy.getProperty(Document.TitleProperty)),"head/title property preserved");
        check(count(source.getDefaultRootElement())==count(copy.getDefaultRootElement()),"element structure remains unchanged");
        check(VivadoZhHtml.translatedCopy(source,value->value)==null,"no-op translator does not create/swap a document");
        HTMLDocument empty=VivadoZhHtml.translatedCopy(document("<html><p>Open</p></html>"),value->"bad\nnew paragraph");
        check(empty==null,"translator cannot introduce paragraph structure");
    }
    private static int count(Element element) {
        int result=1;for(int i=0;i<element.getElementCount();i++) result+=count(element.getElement(i));return result;
    }
    private static final class ForbiddenListener implements DocumentListener,UndoableEditListener,Serializable {
        private final byte[] graph=new byte[8*1024*1024];
        private void writeObject(ObjectOutputStream out) {throw new AssertionError("External listener graph must never be traversed");}
        public void insertUpdate(DocumentEvent e) {throw new AssertionError("Original/copy listener invoked");}
        public void removeUpdate(DocumentEvent e) {throw new AssertionError("Original/copy listener invoked");}
        public void changedUpdate(DocumentEvent e) {throw new AssertionError("Original/copy listener invoked");}
        public void undoableEditHappened(UndoableEditEvent e) {throw new AssertionError("Original/copy undo listener invoked");}
    }
    private static final class UnknownObject implements Serializable {
        private void writeObject(ObjectOutputStream out) {throw new AssertionError("Unknown object must be rejected before traversal");}
    }
    private static void boundaries() throws Exception {
        HTMLDocument source=document("<html><body>Open</body></html>");
        ForbiddenListener listener=new ForbiddenListener();source.addDocumentListener(listener);source.addUndoableEditListener(listener);
        HTMLDocument copy=VivadoZhHtml.translatedCopy(source,VivadoZhHtmlTest::translate);
        check(text(copy).contains("打开"),"8 MiB external listener graph excluded before copying");
        check(Arrays.asList(source.getDocumentListeners()).contains(listener)&&Arrays.asList(source.getUndoableEditListeners()).contains(listener),"original external listeners remain attached");
        source.putProperty("unsupported-object",new UnknownObject());
        try {VivadoZhHtml.translatedCopy(source,VivadoZhHtmlTest::translate);throw new AssertionError("Unknown property accepted");}
        catch(IOException expected) {check(true,"unknown/native object graph fails closed before traversal");}
        source.putProperty("unsupported-object",new JPanel());
        try {VivadoZhHtml.translatedCopy(source,VivadoZhHtmlTest::translate);throw new AssertionError("UI graph accepted");}
        catch(IOException expected) {check(true,"UI component graph rejected");}
        check(text(source).contains("Open")&&!text(source).contains("打开"),"copy failures leave source English intact");
        source.putProperty("unsupported-object",null);source.putProperty("oversized-object",new byte[5*1024*1024]);
        try {VivadoZhHtml.translatedCopy(source,VivadoZhHtmlTest::translate);throw new AssertionError("Oversized graph accepted");}
        catch(IOException expected) {check(true,"bounded output rejects excessive byte graph");}
        source.putProperty("oversized-object",null);List<Object> deep=new ArrayList<>(),current=deep;
        for(int i=0;i<80;i++) {List<Object> next=new ArrayList<>();current.add(next);current=next;}
        source.putProperty("deep-object",deep);
        try {VivadoZhHtml.translatedCopy(source,VivadoZhHtmlTest::translate);throw new AssertionError("Deep graph accepted");}
        catch(IOException expected) {check(true,"input depth limit rejects deeply nested graph");}
        source.putProperty("deep-object",null);source.putProperty("large-array",new byte[200_001]);
        try {VivadoZhHtml.translatedCopy(source,VivadoZhHtmlTest::translate);throw new AssertionError("Large array accepted");}
        catch(IOException expected) {check(true,"input array-size limit rejects large arrays before allocation");}
        source.putProperty("large-array",null);List<Object> many=new ArrayList<>();
        for(int i=0;i<50_001;i++) many.add("same immutable reference");
        source.putProperty("many-references",many);
        try {VivadoZhHtml.translatedCopy(source,VivadoZhHtmlTest::translate);throw new AssertionError("Excessive references accepted");}
        catch(IOException expected) {check(true,"input object/reference limit enforced");}
    }
    public static void main(String[] args) throws Exception {
        SwingUtilities.invokeAndWait(() -> {
            try {fontAndStyles();linksAndStructure();boundaries();}
            catch(Exception failure) {throw new RuntimeException(failure);}
        });
        System.out.println("VivadoZhHtmlTest PASS: "+checks+" checks");
    }
}
