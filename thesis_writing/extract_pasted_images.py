#!/usr/bin/env python3
"""Extract user-pasted images (base64) from the Claude Code session transcript(s)
so they can be embedded in the deck. Writes them to Downloads/shots/extracted/."""
import base64
import glob
import json
import os

PROJ = "/mnt/c/Users/user/.claude/projects/C--Users-user-thesis-ws-src"
OUT = "/mnt/c/Users/user/Downloads/shots/extracted"
os.makedirs(OUT, exist_ok=True)
EXT = {"image/png": "png", "image/jpeg": "jpg", "image/webp": "webp", "image/gif": "gif"}


def walk(obj, acc):
    if isinstance(obj, dict):
        if obj.get("type") == "image" and isinstance(obj.get("source"), dict):
            s = obj["source"]
            if s.get("type") == "base64" and s.get("data"):
                acc.append((s.get("media_type", "image/png"), s["data"]))
        for v in obj.values():
            walk(v, acc)
    elif isinstance(obj, list):
        for v in obj:
            walk(v, acc)


records = []
for jf in sorted(glob.glob(os.path.join(PROJ, "*.jsonl")), key=os.path.getmtime):
    with open(jf, encoding="utf-8", errors="ignore") as f:
        for ln, line in enumerate(f, 1):
            line = line.strip()
            if not line or '"image"' not in line:
                continue
            try:
                rec = json.loads(line)
            except Exception:
                continue
            acc = []
            walk(rec, acc)
            role = (rec.get("message", {}) or {}).get("role") or rec.get("role") or rec.get("type")
            ts = rec.get("timestamp", "")
            for mt, data in acc:
                records.append((os.path.basename(jf), ln, role, ts, mt, data))

print(f"found {len(records)} image block(s)")
for i, (jf, ln, role, ts, mt, data) in enumerate(records):
    p = os.path.join(OUT, f"img_{i:02d}.{EXT.get(mt, 'png')}")
    try:
        raw = base64.b64decode(data)
        open(p, "wb").write(raw)
        print(f"[{i:02d}] {jf} L{ln} role={role} {mt} {len(raw)//1024}KB ts={ts}")
    except Exception as e:
        print(f"[{i:02d}] decode failed: {e}")
print("-> ", OUT)
