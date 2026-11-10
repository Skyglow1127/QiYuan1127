from pathlib import Path
import json
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.lib import colors
from reportlab.platypus import Paragraph,Table,TableStyle
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.utils import ImageReader
O=Path(__file__).resolve().parents[1]
pdfmetrics.registerFont(UnicodeCIDFont('STSong-Light'))
W,H=595.28,841.89
C=canvas.Canvas(str(O/'KR260_V2_打印与装配说明.pdf'),pagesize=(W,H))
C.setTitle('KR260 V2 防护盒 - 打印与装配说明')
C.setAuthor('Codex')
INK=colors.HexColor('#182e36');TEAL=colors.HexColor('#217a80');LIGHT=colors.HexColor('#eef4f3')
style=ParagraphStyle('text',fontName='STSong-Light',fontSize=10.5,leading=16,textColor=INK,wordWrap='CJK')
small=ParagraphStyle('small',parent=style,fontSize=9,leading=13)

def text(x,y,t,size=11,c=INK):
 C.setFillColor(c);C.setFont('STSong-Light',size);C.drawString(x,y,t)
def para(t,y,size=10.5):
 st=style if size==10.5 else small;p=Paragraph(t,st);w,h=p.wrap(W-76,1000);p.drawOn(C,38,y-h);return y-h-9

def header(n,title,sub):
 C.setFillColor(TEAL);C.rect(0,H-10,W,10,fill=1,stroke=0)
 text(38,H-48,title,23);text(38,H-73,sub,10)
 C.setStrokeColor(colors.HexColor('#d6e2e0'));C.line(38,43,W-38,43)
 text(38,27,'KR260 PROTECTIVE CASE / V2 / 2026-10-21',8,c=TEAL);text(W-65,27,str(n),9)

def img(path,x,y,w,h):
 C.drawImage(str(path),x,y,width=w,height=h,preserveAspectRatio=True,anchor='c',mask='auto')

def table(rows,y,widths):
 data=[[Paragraph(str(a),small) for a in row] for row in rows]
 t=Table(data,colWidths=widths);t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),LIGHT),('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),9),('RIGHTPADDING',(0,0),(-1,-1),9),('TOPPADDING',(0,0),(-1,-1),7),('BOTTOMPADDING',(0,0),(-1,-1),7),('LINEBELOW',(0,0),(-1,-1),.35,colors.HexColor('#d9e3e1'))]))
 tw,th=t.wrap(520,800);t.drawOn(C,38,y-th);return y-th-16

header(1,'KR260 V2 防护盒','原装散热器、无扩展板 / PETG / 220 × 220 mm 打印平台')
img(O/'images'/'case_preview_v2.png',38,430,519,297)
y=410
y=para('这一版增加完整侧壁、主体实心底板和带窄缝格栅的顶盖，降低直接触碰板上元件的机会。前后接口区域保留插拔窗口，右侧保留 microSD 取卡口。小于通风缝的物体、灰尘和液体仍可进入；本设计没有防水、防尘等级或跌落认证。',y)
y=table([['项目','设计值 / 行为'],['外形尺寸','164 × 151 × 56.4 mm（包含顶部小盖）'],['外壁 / 底板厚度','2.8 mm；原装脚垫承托面高 5.8 mm'],['散热格栅','缝宽 3.2 mm；顶盖开孔总面积约 3778 平方毫米'],['风扇上方余量','对 36 mm 原装整机高度包络，顶盖下方留 9.4 mm'],['开发板安装','保留原装脚垫，从上方放入；不增加板壳连接螺丝'],['外壳连接','四个 PETG 侧边卡扣；卡扣和底座配合需先打印试件'],['顶部小盖','可取下，接近 HAT、SLVS-EC 与 PC4 JTAG 接口']],y,[136,383])
y=para('底座导向挡块限制板子横向移动，但没有把板子压紧锁死。整盒按水平桌面使用；取放时托住底座。开盖做板载按键操作，安装大尺寸扩展板前需重新检查空间。',y,9)
C.showPage()

header(2,'打印文件与摆放方向','三个正式零件；两个小卡扣试件；一个板卡省料试装框。')
y=739
y=table([['STL 文件','数量 / 说明'],['00_board_fit_gauge','1 件：先试板卡与脚垫。150 × 127 × 17 mm。'],['04_clip_test + 05_socket_test','各 1 件：先检查卡扣插入、保持和释放。约 2.1 立方厘米 总模型体积。'],['01_base','1 件：正式底座。平底朝下，无需支撑。'],['02_hood_inverted','1 件：顶盖外表面贴床，罩壳内部开口朝上。'],['03_service_hatch_inverted','1 件：小盖外表面贴床，定位凸台朝上，无需支撑。']],y,[215,304])
img(O/'images'/'print_orientation_v2.png',38,y-230,519,230)
y-=240
y=para('建议起始参数：PETG、0.4 mm 喷嘴、0.20 mm 层高、4 道墙、5 层顶/底、20%-30% 填充。使用所用耗材厂商的温度配置，速度采用打印机常规 PETG 配置。底座及罩壳可加 5 mm brim，220 × 220 mm 有效区域足够。',y)
y=para('罩壳四个卡扣的内侧倒钩有约 2.25 mm 悬挑，应对倒钩下方刷局部支撑。不要给整片顶盖格栅加支撑。清理支撑后，检查卡扣根部没有裂纹，舌片可以轻轻向外弹开。卡扣试件横放打印，主要验证尺寸，不能替代完整罩壳的层间强度检查。',y)
y=para('STL 以毫米导出，保持 100% 比例；已摆好打印姿态。试装框通过后，再打印正式零件。参数或壁厚变更后需要重新检查插拔窗口和卡扣配合，不建议整体缩放。',y)
C.showPage()

header(3,'装配、取板与实际验证','先空壳试扣，再断电装板；用装配实测判断是否可用。')
y=740
for t in [
'1. 先打印试装框，保留四个原装脚垫。板卡应靠自重自然放入，四脚落在托台上，底面焊点和元件不接触塑料。V2 的角部挡块已为 Pmod 插头让位，请用 V2 试装框。',
'2. 打印卡扣及卡槽试件。将卡扣试件转回竖直姿态，倒钩朝向卡槽；验证可扣入、保持、轻拨释放。过紧或脆裂时调整尺寸后重试，不靠强压。',
'3. 正式外壳先不放板卡，确认四个卡扣能逐个闭合和释放。释放时从侧边舌片下的小缺口轻轻向外拨动卡扣，再抬起罩壳；不要把力传给电路板。',
'4. 断电，将 KR260 原装脚垫放在四个托台上。确认底座取卡缺口对准 microSD 一侧，罩壳较高的网口窗对准四个 RJ45 接口。向下罩住并扣合。',
'5. 分别试插实际使用的网线、USB、DP、电源，以及需要的 SFP+、Micro-USB 和 Pmod。确认插头能插到底、网线卡扣可操作，microSD 可推出和抽出。粗线头、直角头或大 Pmod 模块的包络需要实测。',
'6. 顶部橙色小盖可从前端指甲槽提起，露出 HAT / SLVS-EC / PC4 JTAG 维护区域；小盖为间隙配合，未锁紧。板载复位、固件按键及较大扩展板操作请取下罩壳。',
'7. 保持原装风扇工作，在同环境、同负载、同风扇转速下对比裸板与装壳温度，运行至温度稳定并检查是否降频。格栅会引入风阻，不能仅凭开孔面积认定散热不变。']:
 y=para(t,y)
text(38,y-8,'验证结果与依据',15);y-=29
y=para('已完成 CAD 有效实体、STL 封闭网格/法向/单连通、外壳零件之间无体积重叠检查。简化板卡和假设的直头插头通道未检出干涉。未做实物打印、实际卡扣插入仿真、强度/跌落、真实插头或热性能验证。',y,9)
y=para('PCB 140 × 119 mm、含接口总深度 132 mm、原装总高 36 mm 来自 AMD DS988 机械图。脚垫高 5.5 mm、脚垫中心 (±65, ±54.5) mm、元件和插头包络是设计估算；配合实板试装框使用。',y,9)
y=para('参考：<link href="https://docs.amd.com/r/en-US/ds988-kr260-starter-kit/Mechanical" color="#217a80">AMD DS988 Mechanical</link>；<link href="https://docs.amd.com/r/en-US/ug1092-kr260-starter-kit/Interfaces" color="#217a80">UG1092 Interfaces</link>；<link href="https://docs.amd.com/r/en-US/ug1092-kr260-starter-kit/Fan-and-Heat-Sink" color="#217a80">UG1092 Fan and Heat Sink</link>。资料链接与假设记录见 reference/SOURCES.txt。',y,9)
C.save();print('V2 PDF created')
