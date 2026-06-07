#!/usr/bin/env python3
"""
Phase 4B -- generate the per-object ArUco GROUND-TRUTH markers.

Four single ArUco markers (one per tabletop object). Detected in the SHARP RGB
image (CAM_B, same camera as the calibration board), each marker gives its
object's TRUE position for the sim-to-real comparison:
    detect -> solvePnP (known marker size) -> marker pose in RGB frame
    -> table frame (via board_4b/table_extrinsics.json)
    -> object centre (via the known marker->object offset in markers.json)

DICT_4X4_50 (robust + distinct from the DICT_5X5_100 calibration board). Marker
IDs 0-3 map to the four objects. ~80 mm markers, printed 4-up on A4.

Run (WSL):  python3 make_object_markers.py
Outputs (../markers_4b/):
  object_markers_A4.pdf   <- PRINT AT 100% / actual size on A4
  object_markers_A4.png
  markers.json            <- dictionary, size, id->object, offsets (ingest reads this)
"""
import json
import os

import cv2
import numpy as np

DICT_NAME    = "DICT_4X4_50"
ID_TO_OBJECT = {0: "coke_can", 1: "banana", 2: "golf_ball", 3: "blue_box"}
MARKER_LEN_M = 0.080          # 80 mm
DPI          = 300
PAGE_W_MM, PAGE_H_MM = 210.0, 297.0    # A4 portrait
PX_PER_MM = DPI / 25.4

HERE    = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.normpath(os.path.join(HERE, "..", "markers_4b"))


def mm(v):
    return int(round(v * PX_PER_MM))


def get_dict():
    code = getattr(cv2.aruco, DICT_NAME)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(code)
    return cv2.aruco.Dictionary_get(code)


def draw_marker(d, mid, px):
    if hasattr(cv2.aruco, "generateImageMarker"):        # OpenCV >= 4.7
        return cv2.aruco.generateImageMarker(d, mid, px)
    return cv2.aruco.drawMarker(d, mid, px)              # OpenCV 4.5/4.6


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    d = get_dict()
    page = np.full((mm(PAGE_H_MM), mm(PAGE_W_MM)), 255, np.uint8)
    mpx = mm(MARKER_LEN_M * 1000)

    cols_mm = [60, 150]          # 2x2 grid centres on A4
    rows_mm = [75, 180]
    for k, mid in enumerate(sorted(ID_TO_OBJECT)):
        cx, cy = cols_mm[k % 2], rows_mm[k // 2]
        x0, y0 = mm(cx) - mpx // 2, mm(cy) - mpx // 2
        page[y0:y0 + mpx, x0:x0 + mpx] = draw_marker(d, mid, mpx)
        cv2.putText(page, "ID=%d   %s" % (mid, ID_TO_OBJECT[mid]),
                    (x0, y0 + mpx + mm(9)), cv2.FONT_HERSHEY_SIMPLEX,
                    2.3, 0, 5, cv2.LINE_AA)

    # title + 100 mm scale-verification bar
    cv2.putText(page, "Phase 4B object ground-truth markers  -  DICT_4X4_50  -  PRINT AT 100% ON A4",
                (mm(14), mm(22)), cv2.FONT_HERSHEY_SIMPLEX, 1.6, 0, 4, cv2.LINE_AA)
    bx, by = mm(20), mm(272)
    cv2.line(page, (bx, by), (bx + mm(100), by), 0, mm(1.0))
    cv2.line(page, (bx, by - mm(3)), (bx, by + mm(3)), 0, mm(1.0))
    cv2.line(page, (bx + mm(100), by - mm(3)), (bx + mm(100), by + mm(3)), 0, mm(1.0))
    cv2.putText(page, "100 mm  (verify print; each marker edge should measure 80 mm)",
                (bx + mm(104), by + mm(3)), cv2.FONT_HERSHEY_SIMPLEX, 1.5, 0, 4, cv2.LINE_AA)

    base = os.path.join(OUT_DIR, "object_markers_A4")
    cv2.imwrite(base + ".png", page)
    try:
        from PIL import Image
        Image.fromarray(page).save(base + ".pdf", "PDF", resolution=float(DPI))
        pdf_ok = True
    except Exception as e:
        pdf_ok = False
        print("PDF save failed (%r) -- print the .png at %d dpi" % (e, DPI))

    markers = {
        "dictionary": DICT_NAME,
        "marker_length_m": MARKER_LEN_M,
        "id_to_object": {str(k): v for k, v in ID_TO_OBJECT.items()},
        "marker_to_object_offset_m": {v: [0.0, 0.0] for v in ID_TO_OBJECT.values()},
        "offset_note": ("Vector from MARKER centre to OBJECT centre, expressed in the "
                        "MARKER's own frame (x = marker right, y = marker up). Set per "
                        "object once placement is fixed; the step-7 ingest rotates it by "
                        "the detected marker orientation. Default [0,0] = object centred "
                        "on the marker (use only if the marker stays visible)."),
    }
    json.dump(markers, open(os.path.join(OUT_DIR, "markers.json"), "w"), indent=2)

    print("OpenCV", cv2.__version__)
    print("marker px =", mpx, "(80 mm @ %d dpi)" % DPI)
    print("wrote:", OUT_DIR)
    print("  object_markers_A4.pdf" if pdf_ok else "  (pdf failed)")
    print("  object_markers_A4.png / markers.json")
    print("  IDs:", ID_TO_OBJECT)


if __name__ == "__main__":
    main()
