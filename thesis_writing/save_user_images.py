import base64, json, os
JSONL = "/mnt/c/Users/user/.claude/projects/C--Users-user-thesis-ws-src/f3f37122-08e2-410f-9227-faaaf2ee69aa.jsonl"
OUT = "/mnt/c/Users/user/Downloads/_media_tmp"
os.makedirs(OUT, exist_ok=True)
EXT = {"image/png": "png", "image/jpeg": "jpg", "image/webp": "webp", "image/gif": "gif"}


def is_img(b):
    return (isinstance(b, dict) and b.get("type") == "image"
            and isinstance(b.get("source"), dict)
            and b["source"].get("type") == "base64" and b["source"].get("data"))


imgs = []
for line in open(JSONL, encoding="utf-8", errors="ignore"):
    line = line.strip()
    if not line or '"image"' not in line:
        continue
    try:
        rec = json.loads(line)
    except Exception:
        continue
    msg = rec.get("message") or {}
    if msg.get("role") != "user":
        continue
    content = msg.get("content")
    if not isinstance(content, list):
        continue
    ts = rec.get("timestamp", "")
    # DIRECT image blocks = the user's pastes; tool_result-nested images (my Reads) are skipped
    for b in content:
        if is_img(b):
            imgs.append((ts, b["source"].get("media_type", "image/png"), b["source"]["data"]))

print(f"{len(imgs)} user-pasted images")
for i, (ts, mt, data) in enumerate(imgs, 1):
    raw = base64.b64decode(data)
    name = f"screenshot_{i:02d}.{EXT.get(mt, 'png')}"
    open(os.path.join(OUT, name), "wb").write(raw)
    print(f"  {name}  ({len(raw)//1024} KB)  {ts}")
print("->", OUT)
