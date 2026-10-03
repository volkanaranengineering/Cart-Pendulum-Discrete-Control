"""Render the Turkish Markdown tutorial with embedded fonts; no network access."""
from pathlib import Path
import re
from xml.sax.saxutils import escape
from reportlab.pdfgen import canvas
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, PageBreak, Image, Table, TableStyle, Preformatted
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from PIL import Image as PILImage

ROOT = Path(__file__).resolve().parents[1]
DOC = ROOT / 'docs/TUTORIAL_TR.md'
OUT = ROOT / 'output/pdf/Ayrik_Araba_Sarkac_Ders_Notu_TR.pdf'

def fonts():
    candidates = [(Path('C:/Windows/Fonts'), 'arial.ttf', 'arialbd.ttf', 'consola.ttf'),
                  (Path('/usr/share/fonts/truetype/dejavu'), 'DejaVuSans.ttf', 'DejaVuSans-Bold.ttf', 'DejaVuSansMono.ttf')]
    for base, normal, bold, mono in candidates:
        if all((base / n).exists() for n in (normal, bold, mono)):
            for name, filename in [('Body', normal), ('BodyBold', bold), ('Mono', mono)]:
                pdfmetrics.registerFont(TTFont(name, str(base / filename)))
            pdfmetrics.registerFontFamily('Body', normal='Body', bold='BodyBold', italic='Body', boldItalic='BodyBold')
            return
    raise RuntimeError('Install Arial or DejaVu Sans/Mono fonts, or configure font paths in this script.')

def inline(s):
    s = escape(s)
    s = re.sub(r'\*\*(.*?)\*\*', r'<b>\1</b>', s)
    s = re.sub(r'`(.*?)`', r'<font name="Mono">\1</font>', s)
    return s

def footer(c, doc):
    c.setStrokeColor(colors.HexColor('#b8c7d2'));c.line(42, 40, 553, 40)
    c.setFont('Body', 8);c.setFillColor(colors.HexColor('#486273'))
    c.drawString(42, 27, 'ARABA-SARKAÇ | AYRIK KONTROL | 03.10.2026')
    c.drawRightString(553, 27, str(doc.page))

def main():
    fonts();OUT.parent.mkdir(parents=True, exist_ok=True)
    style = ParagraphStyle('Body', fontName='Body', fontSize=10.1, leading=14.4, spaceAfter=9, textColor=colors.HexColor('#243746'))
    heading = ParagraphStyle('Heading', parent=style, fontName='BodyBold', fontSize=17, leading=21, spaceAfter=15, textColor=colors.HexColor('#103d5b'))
    title = ParagraphStyle('Title', parent=heading, fontSize=26, leading=31, spaceAfter=16)
    caption = ParagraphStyle('Caption', parent=style, fontSize=8.4, leading=11, textColor=colors.HexColor('#506878'))
    cell = ParagraphStyle('Cell', parent=style, fontSize=8.4, leading=11, spaceAfter=0)
    code = ParagraphStyle('Code', fontName='Mono', fontSize=8.4, leading=11.2, backColor=colors.HexColor('#eef3f7'), borderPadding=8, spaceBefore=4, spaceAfter=12)
    lines=DOC.read_text(encoding='utf-8').splitlines();story=[];i=0
    while i<len(lines):
        line=lines[i].strip()
        if not line:i+=1;continue
        if line=='---':story.append(PageBreak());i+=1;continue
        if line.startswith('```'):
            block=[];i+=1
            while i<len(lines) and not lines[i].startswith('```'):block.append(lines[i]);i+=1
            story.append(Preformatted('\n'.join(block),code));i+=1;continue
        if line.startswith('|'):
            rows=[]
            while i<len(lines) and lines[i].startswith('|'):
                parts=[p.strip() for p in lines[i].strip().strip('|').split('|')]
                if not all(re.fullmatch(r'[-: ]+',p) for p in parts):rows.append([Paragraph(inline(p),cell) for p in parts])
                i+=1
            widths=[511/len(rows[0])]*len(rows[0]);table=Table(rows,colWidths=widths,repeatRows=1,hAlign='LEFT')
            table.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#dceaf3')),('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor('#f3f6f8')]),('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),7),('RIGHTPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6),('LINEBELOW',(0,0),(-1,0),.7,colors.HexColor('#7595ab'))]))
            story.extend([table,Spacer(1,12)]);continue
        match=re.fullmatch(r'!\[(.*?)\]\((.*?)\)',line)
        if match:
            p=(DOC.parent/match.group(2)).resolve();w,h=PILImage.open(p).size
            maxh=290 if 'animation_preview' in str(p) else 235
            factor=min(511/w,maxh/h)
            story.extend([Image(str(p),width=w*factor,height=h*factor),Spacer(1,4),Paragraph(inline(match.group(1)),caption)]);i+=1;continue
        if line.startswith('# '):story.append(Paragraph(inline(line[2:]),title));i+=1;continue
        if line.startswith('## '):story.append(Paragraph(inline(line[3:]),heading));i+=1;continue
        para=[line];i+=1
        while i<len(lines) and lines[i].strip() and not lines[i].startswith(('#','```','|','![','---')):para.append(lines[i].strip());i+=1
        story.append(Paragraph(inline(' '.join(para)),style))
    doc=SimpleDocTemplate(str(OUT),pagesize=(595.28,841.89),leftMargin=42,rightMargin=42,topMargin=42,bottomMargin=55,title='Ayrık Zamanlı Araba-Sarkaç Kontrolü',author='Volkan Aran; OpenAI Codex katkısıyla')
    doc.build(story,onFirstPage=footer,onLaterPages=footer)
    print(OUT)

if __name__=='__main__':main()
