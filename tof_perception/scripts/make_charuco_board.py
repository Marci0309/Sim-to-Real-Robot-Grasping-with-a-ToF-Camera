#!/usr/bin/env python3
"""
Phase 4B / step 3 -- generate a print-ready ChArUco board for the camera->table
extrinsic calibration.

Outputs (into ../board_4b/):
  charuco_A1_7x5_100mm.pdf   <- PRINT THIS at 100% / "actual size" on A1
  charuco_A1_7x5_100mm.png   <- full-res raster (same content)
  charuco_preview.png        <- small preview for screen
  board_params.json          <- exact params; the solver reads this

The board is sized to FILL the OAK-D ToF field of view at 0.68 m
(~92 x 69 cm visible) with a safe margin: 700 x 500 mm board area.

Targets OpenCV's OLD aruco API (4.5.x, what this project has) but also
falls back to the new (>=4.7) API if run elsewhere.

After printing:
  1. Measure the OUTER edge of the black-square pattern. It should be 700 mm
     wide x 500 mm tall. Printers scale ~1-3%, so measure for real.
  2. true square_length (m) = measured_board_width_mm / 1000 / 7
     Put that number in board_params.json -> "square_length_m" before solving.
"""
import json
import os

import cv2
import numpy as np

# ---- board definition --------------------------------------------------------
DICT_NAME    = "DICT_5X5_100"
SQUARES_X    = 7
SQUARES_Y    = 5
SQUARE_LEN_M = 0.100          # 100 mm
MARKER_LEN_M = 0.075          # 75 mm  (0.75 * square)
DPI          = 300            # print resolution

# ---- A1 page (landscape) -----------------------------------------------------
PAGE_W_MM = 841.0
PAGE_H_MM = 594.0

PX_PER_M  = DPI / 0.0254      # 11811.0 px per metre at 300 dpi
PX_PER_MM = PX_PER_M / 1000.0

HERE    = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.normpath(os.path.join(HERE, "..", "board_4b"))


def mm(v):   # mm -> px
    return int(round(v * PX_PER_MM))


def get_dictionary():
    code = getattr(cv2.aruco, DICT_NAME)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(code)
    return cv2.aruco.Dictionary_get(code)


def draw_board(board_w_px, board_h_px):
    d = get_dictionary()
    if hasattr(cv2.aruco, "CharucoBoard_create"):          # old API (4.5.x/4.6)
        board = cv2.aruco.CharucoBoard_create(
            SQUARES_X, SQUARES_Y, SQUARE_LEN_M, MARKER_LEN_M, d)
        img = board.draw((board_w_px, board_h_px), None, 0, 1)
    else:                                                  # new API (>=4.7)
        board = cv2.aruco.CharucoBoard(
            (SQUARES_X, SQUARES_Y), SQUARE_LEN_M, MARKER_LEN_M, d)
        img = board.generateImage((board_w_px, board_h_px), None, 0, 1)
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    board_w_px = mm(SQUARES_X * SQUARE_LEN_M * 1000)   # 700 mm
    board_h_px = mm(SQUARES_Y * SQUARE_LEN_M * 1000)   # 500 mm
    page_w_px  = mm(PAGE_W_MM)
    page_h_px  = mm(PAGE_H_MM)

    page = np.full((page_h_px, page_w_px), 255, np.uint8)
    board = draw_board(board_w_px, board_h_px)

    x0 = (page_w_px - board_w_px) // 2
    y0 = (page_h_px - board_h_px) // 2
    page[y0:y0 + board_h_px, x0:x0 + board_w_px] = board

    # --- corner crop ticks: mark the exact 700x500 rectangle for measuring -----
    t = mm(8)        # tick length
    g = mm(3)        # gap from board
    for (cx, cy, dx, dy) in [(x0, y0, -1, -1), (x0 + board_w_px, y0, 1, -1),
                             (x0, y0 + board_h_px, -1, 1),
                             (x0 + board_w_px, y0 + board_h_px, 1, 1)]:
        cv2.line(page, (cx + dx * g, cy), (cx + dx * (g + t), cy), 0, mm(0.4))
        cv2.line(page, (cx, cy + dy * g), (cx, cy + dy * (g + t)), 0, mm(0.4))

    # --- 100 mm scale-verification bar (bottom margin) ------------------------
    bar = mm(100)
    bx = x0
    by = y0 + board_h_px + mm(22)
    cv2.line(page, (bx, by), (bx + bar, by), 0, mm(1.2))
    cv2.line(page, (bx, by - mm(4)), (bx, by + mm(4)), 0, mm(1.2))
    cv2.line(page, (bx + bar, by - mm(4)), (bx + bar, by + mm(4)), 0, mm(1.2))
    cv2.putText(page, "100 mm  (measure after printing -- must read 100 mm)",
                (bx + bar + mm(6), by + mm(4)),
                cv2.FONT_HERSHEY_SIMPLEX, mm(0.9) / 10.0, 0, mm(0.5))

    # --- caption (top margin) -------------------------------------------------
    lines = [
        "ChArUco extrinsics board  --  Phase 4B step 3  (camera->table)",
        "%s   %dx%d squares   square=%dmm   marker=%dmm   board=%dx%dmm"
        % (DICT_NAME, SQUARES_X, SQUARES_Y, int(SQUARE_LEN_M * 1000),
           int(MARKER_LEN_M * 1000), int(SQUARES_X * SQUARE_LEN_M * 1000),
           int(SQUARES_Y * SQUARE_LEN_M * 1000)),
        "PRINT AT 100% / ACTUAL SIZE ON A1 -- DO NOT 'fit to page'.  "
        "Then measure outer edge = 700mm and set square_length = measured/7.",
    ]
    ty = mm(16)
    for ln in lines:
        cv2.putText(page, ln, (x0, ty), cv2.FONT_HERSHEY_SIMPLEX,
                    mm(0.85) / 10.0, 0, mm(0.45))
        ty += mm(12)

    # --- save -----------------------------------------------------------------
    base = os.path.join(OUT_DIR, "charuco_A1_7x5_100mm")
    cv2.imwrite(base + ".png", page)

    prev = cv2.resize(page, (page_w_px // 8, page_h_px // 8),
                      interpolation=cv2.INTER_AREA)
    cv2.imwrite(os.path.join(OUT_DIR, "charuco_preview.png"), prev)

    try:
        from PIL import Image
        Image.fromarray(page).save(base + ".pdf", "PDF", resolution=float(DPI))
        pdf_ok = True
    except Exception as e:                                  # pragma: no cover
        pdf_ok = False
        print("PDF save failed (%r) -- print the .png at %d dpi instead" % (e, DPI))

    params = {
        "dictionary": DICT_NAME,
        "squares_x": SQUARES_X, "squares_y": SQUARES_Y,
        "square_length_m": SQUARE_LEN_M, "marker_length_m": MARKER_LEN_M,
        "board_width_mm": int(SQUARES_X * SQUARE_LEN_M * 1000),
        "board_height_mm": int(SQUARES_Y * SQUARE_LEN_M * 1000),
        "dpi": DPI, "page": "A1 landscape (841x594 mm)",
        "interior_corners": (SQUARES_X - 1) * (SQUARES_Y - 1),
        "after_print": ("Measure outer edge of square pattern. "
                        "true square_length_m = measured_board_width_mm/1000/7. "
                        "Update square_length_m here before running the solver."),
    }
    with open(os.path.join(OUT_DIR, "board_params.json"), "w") as fh:
        json.dump(params, fh, indent=2)

    print("OpenCV:", cv2.__version__)
    print("page  : %d x %d px  (%.0f x %.0f mm)" %
          (page_w_px, page_h_px, PAGE_W_MM, PAGE_H_MM))
    print("board : %d x %d px  (700 x 500 mm)" % (board_w_px, board_h_px))
    print("wrote :", OUT_DIR)
    print("  charuco_A1_7x5_100mm.pdf" if pdf_ok else "  (pdf failed)")
    print("  charuco_A1_7x5_100mm.png / charuco_preview.png / board_params.json")


if __name__ == "__main__":
    main()
