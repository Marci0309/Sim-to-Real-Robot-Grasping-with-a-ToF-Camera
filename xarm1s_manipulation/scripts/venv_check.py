#!/usr/bin/env python3
"""Report which thesis Python dependencies are importable in the running
interpreter (run with ~/thesis_ws/.venv/bin/python to check the venv)."""
import importlib
import sys

print("python:", sys.version.split()[0], "->", sys.executable)
for m in ["numpy", "cv2", "matplotlib", "pymeshlab", "scipy", "yaml", "PIL"]:
    try:
        mod = importlib.import_module(m)
        print(f"OK    {m:12} {getattr(mod, '__version__', '?')}")
    except Exception as e:  # noqa: BLE001
        print(f"MISS  {m:12} {type(e).__name__}: {e}")
