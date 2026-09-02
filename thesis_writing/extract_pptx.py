import re, sys, zipfile
path = sys.argv[1]
z = zipfile.ZipFile(path)
def order(n): return int(re.search(r'(\d+)', n).group(1))
slides = sorted([n for n in z.namelist() if re.match(r'ppt/slides/slide\d+\.xml$', n)], key=order)
notes = {order(n): n for n in z.namelist() if re.match(r'ppt/notesSlides/notesSlide\d+\.xml$', n)}
print(f"{len(slides)} slides\n")
for i, n in enumerate(slides, 1):
    xml = z.read(n).decode('utf-8', 'ignore')
    texts = [t.strip() for t in re.findall(r'<a:t>(.*?)</a:t>', xml, re.S) if t.strip()]
    print(f"## Slide {i}")
    for t in texts:
        print("   -", t)
    if i in notes:
        nt = [t.strip() for t in re.findall(r'<a:t>(.*?)</a:t>', z.read(notes[i]).decode('utf-8','ignore'), re.S) if t.strip()]
        if nt:
            print("   [notes]", " ".join(nt)[:400])
    print()
