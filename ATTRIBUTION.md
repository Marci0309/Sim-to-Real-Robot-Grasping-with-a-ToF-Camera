# Attribution and Third-Party Licenses

The six first-party thesis packages (`tof_perception`, `evaluation`,
`noise_injection`, `xarm1s_manipulation`, `xarm1s_description`,
`xarm1s_gazebo`) are MIT-licensed — see `LICENSE`. This file records the
third-party **code** and **3D assets** bundled in the repository, with their
licenses and any obligations.

## Third-party code

| Component | Path | License | Source | Obligation |
|---|---|---|---|---|
| `gazebo_ros2_control` (vendored) | `_thirdparty/gazebo_ros2_control/gazebo_ros2_control` | BSD-3-Clause + Apache-2.0 (dual) | github.com/ros-controls/gazebo_ros2_control (Humble branch, v0.4.12) | Retain `LICENSE` + source copyright headers; note modifications if any |
| `gazebo_ros2_control_demos` (vendored) | `_thirdparty/gazebo_ros2_control/gazebo_ros2_control_demos` | Apache-2.0 | same repo | Retain notices |

The dual-license text is preserved verbatim in
`_thirdparty/gazebo_ros2_control/LICENSE`. No NOTICE file ships with the
upstream, so the Apache-2.0 NOTICE-duplication clause does not apply. **No
copyleft (GPL/LGPL/AGPL) anywhere in the repo.** If any vendored file was
modified locally, state it here (none recorded to date).

## 3D assets

| Asset | Path | Status | Source | License | Action |
|---|---|---|---|---|---|
| Coke can mesh (`coke_slim_250ml.obj` + `.mtl`) | `xarm1s_gazebo/models/tabletop_objects/coke_can/meshes/` | **OK — attributed** | "Soda Can 250ml" by **sebastian204**, Sketchfab — https://sketchfab.com/3d-models/soda-can-250ml-557bd388b7b340a88053016f39c3d514 | **CC-BY-4.0** (commercial use allowed) | Credit retained below + verbatim in `sketchfab_raw/coke_slim_250ml/license.txt`. Centred + recoloured solid red (52×134 mm). The old un-attributed `777.obj/.jpg` (330 ml, UNVERIFIED) was **removed** 2026-06-04. |
| Banana mesh (`banana_solid.obj` + `.mtl`) | `xarm1s_gazebo/models/tabletop_objects/banana/meshes/` | **OK — attributed** | "Banana" by **Lassi Kaukonen** (@thesidekick), Sketchfab — https://sketchfab.com/3d-models/banana-dda3a1f707a94c52bed79578e120937c | **CC-BY-4.0** (commercial use allowed) | Credit retained below + in `sketchfab_raw/banana/license.txt`. `banana_solid.obj` is decimated + recoloured solid yellow from this mesh. (Author to do a final visual check that this is the exact model downloaded.) |
| Golf ball (deployed) | `xarm1s_gazebo/models/tabletop_objects/golf_ball/model.sdf` | OK | own work | MIT | none — primitive `<sphere>`, no external mesh |
| Golf ball mesh (downloaded, **unused**) | `sketchfab_raw/golf_ball/` | unused | Sketchfab | UNVERIFIED | not deployed; safe to delete |
| xArm 1S STL meshes (`base.stl`, `link2-6.stl`, gripper STLs) | `xarm1s_description/meshes/` | **needs provenance** | GrabCAD community model (proposal ref [3]) | per-uploader terms | capture GrabCAD URL + author + reuse terms; confirm academic reuse is permitted |

### Required attribution (CC-BY — copy into the thesis acknowledgements / asset appendix)
- **Coke can:** This work is based on "Soda Can 250ml"
  (https://sketchfab.com/3d-models/soda-can-250ml-557bd388b7b340a88053016f39c3d514)
  by sebastian204 (https://sketchfab.com/sebastian204) licensed under CC-BY-4.0
  (http://creativecommons.org/licenses/by/4.0/).
- **Banana:** This work is based on "Banana"
  (https://sketchfab.com/3d-models/banana-dda3a1f707a94c52bed79578e120937c)
  by Lassi Kaukonen (https://sketchfab.com/thesidekick) licensed under CC-BY-4.0
  (http://creativecommons.org/licenses/by/4.0/).

### Action items (license gaps to close before submission)
1. ✅ **Coke can** — replaced with a CC-BY-4.0 model (sebastian204) and credited above. Done (2026-06-04).
2. ✅ **Banana** — identified as CC-BY-4.0 (Lassi Kaukonen, @thesidekick) and credited above. Done (2026-06-04); author to confirm it is the exact model used.
3. **xArm URDF/STLs (GrabCAD)** — capture URL/author + confirm reuse terms.
4. Optionally delete the unused `sketchfab_raw/golf_ball/` download.

*(Automated licence audit 2026-05-30; coke resolved + this section updated 2026-06-04.)*
