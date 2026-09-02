#!/usr/bin/env python3
"""Render THESIS_HANDOFF_BRIEF.md -> .pdf with full-Unicode DejaVu fonts."""
import os
import markdown
from xhtml2pdf import pisa


def find_font(name):
    cands = []
    try:
        import matplotlib
        cands.append(os.path.join(matplotlib.get_data_path(), "fonts", "ttf", name))
    except Exception:
        pass
    cands.append("/usr/share/fonts/truetype/dejavu/" + name)
    for c in cands:
        if os.path.isfile(c):
            return c
    return None


sans = find_font("DejaVuSans.ttf")
mono = find_font("DejaVuSansMono.ttf")
print("sans:", sans)
print("mono:", mono)

md = open("THESIS_HANDOFF_BRIEF.md", encoding="utf-8").read()
for a, b in [("✅", "[OK]"), ("❌", "[X]"), ("⛔", "[DO NOT CITE]"),
             ("\U0001f7e1", "[partial]")]:
    md = md.replace(a, b)
body = markdown.markdown(md, extensions=["tables", "fenced_code"])

faces, fb, fm = "", "Helvetica", "Courier"
if sans:
    faces += "@font-face{font-family:'Doc';src:url('%s');}" % sans
    fb = "'Doc'"
if mono:
    faces += "@font-face{font-family:'Mono';src:url('%s');}" % mono
    fm = "'Mono'"

css = ("%s body{font-family:%s;font-size:8.5pt;line-height:1.3;} "
       "h1{font-size:16pt;} h2{font-size:12.5pt;margin-top:10pt;} h3{font-size:10pt;} "
       "table{border-collapse:collapse;width:100%%;} "
       "th,td{border:0.5pt solid #888;padding:2pt;font-size:7.5pt;} "
       "code,pre{font-family:%s;font-size:7.5pt;background:#f2f2f2;}") % (faces, fb, fm)
html = ("<html><head><meta charset='utf-8'><style>%s</style></head><body>%s</body></html>"
        % (css, body))

with open("THESIS_HANDOFF_BRIEF.pdf", "wb") as f:
    st = pisa.CreatePDF(html, dest=f, encoding="utf-8")
print("font_body:", fb, "font_mono:", fm, "err:", st.err)
