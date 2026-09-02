# Phase 4C — Bench Run Recipe (closed-loop visual servo)

The closed-loop grasp **works**. blue_box DONE: **11/15 = 73 %** (Wilson CI 48–89 %), **9/9 = 100 % in
the near zone (r ≤ 0.20 m)**. This is the copy-paste recipe to finish **banana / golf / coke**. Windows PowerShell.

## CRITICAL gotchas (these bit us — don't re-trip)
1. **Marker must be FLAT + RIGID.** A floppy/curled gripper marker reads as a *white blob* → `gripper marker
   not detected` on every step → `UNMEASURED-openloop` → open-loop miss. The whole fix is a **stiff, flat**
   marker (`DICT_4X4_50` **id 4**) on the gripper, facing straight up, centred over the jaws. **Glance at it
   every few trials** — if it droops, everything silently fails. (Not motion: `--servo-settle` did NOT help; it
   was physical. A rigid marker also extends the working zone — banana converged at r = 0.21 vs the box's
   floppy-marker limit of ~0.20.)
2. **PowerShell is case-insensitive:** `$s` and `$S` are the SAME variable. Never use `$s` for a scene name when
   `$S` is a path. The setup below uses `$SRC` + `$nm` (no collision). This is why earlier helper functions
   silently failed.
2b. **`gc` is a built-in alias for `Get-Content`, and ALIASES beat functions** — so a `function Gc {}` is
   shadowed: `gc 1` runs `Get-Content 1` and errors `Cannot find path ...\1`. The coke runner is therefore
   named **`Gck`** (below). (`Gb`/`Gg` are safe — no built-in alias.) If you must use `Gc`, first
   `Remove-Item Alias:gc -Force`.
3. **Near zone only:** place objects at table **x ≈ 0.34–0.40 (r ≤ ~0.20 m from the base)**; watch the `tilt`
   line ≈ **88** and the `[servo]` `err` converging **< 5 mm**. Far reach (r > 0.22) → the loop gain collapses
   (the marker-mount limit).
4. **EXCLUDE `UNMEASURED` / 0-meas rows when scoring** — those are equipment-down (marker not seen), NOT grasp
   attempts. A valid trial has `meas > 0` in the `[servo]` note.
5. **Summarize only the CLEAN file** `evaluation/results/real_manip_4c_formal.csv`. The main
   `real_manip_results.csv` mislabels placements as 1,1 and `summarize_phase4c.read_real` counts every row.

## Setup (paste once per fresh terminal; sanity-check with `Home`)
```powershell
$SRC = "\\wsl.localhost\Ubuntu-22.04\home\szokedencsim\thesis_ws\src"
function Home  { py -3.11 "$SRC\xarm1s_manipulation\scripts\phase4c_manual_grasp.py" --home }
function Open  { py -3.11 "$SRC\xarm1s_manipulation\scripts\phase4c_manual_grasp.py" --gripper-mm 55 }
function Cap($name)            { py -3.14 "$SRC\tof_perception\scripts\capture_scene_4b.py" --name $name --table-distance 0.68 --out-dir "$SRC\tof_perception\captures_4c" }
function Perc($name,$obj)      { wsl.exe -d Ubuntu-22.04 -- bash -lc "cd ~/thesis_ws/src/tof_perception/scripts && python3 perceive_scene_4c.py --name $name --object $obj" }
function Grasp($obj,$name,$cm) { py -3.11 "$SRC\xarm1s_manipulation\scripts\real_grasp_executor.py" --object $obj --scene $name --servo --servo-iters 8 --servo-settle 2.0 --close-mm $cm --pick-only }
function Gb($n){ $nm="ba$n"; Home; Cap $nm; Perc $nm banana;    Grasp banana    $nm 15; Open }
function Gg($n){ $nm="g$n";  Home; Cap $nm; Perc $nm golf_ball; Grasp golf_ball $nm 18; Open }
function Gck($n){ $nm="c$n"; Home; Cap $nm; Perc $nm coke_can;  Grasp coke_can  $nm 30; Open }  # NOT Gc (gc=Get-Content alias)
```

## Run (move object to a new near-zone spot each placement; watch the FIRST `[servo]` line for real `err`)
- **banana** — length **∥ table X**, ±10 mm tol (easy): `Gb 1` … `Gb 15` (5 placements × 3). *banana trial 1
  already succeeded: `8meas 26→3.9mm`, lift=1 "perfect!"* → 14 more.
- **golf_ball** — any orientation; round → closed-loop fixes the AIM, the residual risk is the smooth ball
  slipping out of flat jaws (an honest hardware result if so): `Gg 1` … `Gg 15`. (If it grips high/low on the
  ball, add `--grasp-z 0.81` to the Grasp fn.)
- **coke_can** — lying **on its side, length ∥ table Y**; 52 mm body in a 55 mm jaw = a **SIZE WALL** (≈0 %
  expected): `Gck 1` … `Gck 6` to *confirm* the limit, then document — don't chase.
- `--pick-only` holds the object after the lift; `Open` releases it; re-place for the next trial.

## Score each object → Results 5.5
After an object's run: extract its **valid (`meas > 0`)** rows from `real_manip_results.csv`, in order, re-label
`placement_id` 1–5 / `repeat` 1–3, **append** to `evaluation/results/real_manip_4c_formal.csv`, then:
```bash
cd ~/thesis_ws/src/evaluation/scripts
python3 summarize_phase4c.py --csv ~/thesis_ws/src/evaluation/results/real_manip_4c_formal.csv --out-dir ~/thesis_ws/src/evaluation/results
```
→ per-object lift rate (k/N) + Wilson 95 % CI + the sim-feasible / 4B-predictor comparison = the Results 5.5 table+figure.

## Status
- **blue_box: 11/15 = 73 %** (100 % in-zone) — in `real_manip_4c_formal.csv`.
- **banana: trial 1 ✓** (`8meas 26→3.9mm`); 14 more.
- **golf, coke: not started.** Predicted: banana good, golf marginal (round-slip), coke ≈ 0 % (size wall).
- New flag this session: `--servo-settle` (default 1.2 s). All Phase-4C code is **uncommitted**.
- Full thesis writeup: `src/thesis_writing/THESIS_HANDOFF_BRIEF.{md,pdf}`. Design: `PHASE4C_VISUAL_SERVO_PLAN.md`.
