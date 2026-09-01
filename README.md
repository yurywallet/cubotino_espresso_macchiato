# cubotino_espresso_macchiato

This is a personal, independently-maintained customization of **[CUBOTino_base_version](https://github.com/AndreaFavero71/CUBOTino_base_version)** by Andrea Favero — all credit for the original robot design and code goes to him. 
This repo is not kept in sync with the upstream project; it carries local changes only, summarized below.

For the original project, official builds, videos, and full documentation, see: **https://github.com/AndreaFavero71/CUBOTino_base_version**

## What's different in this fork

All changes are confined to `PC_files/` (`Cubotino_GUI.py`, `Cubotino_webcam.py`, `Cubotino_moves.py`) - no robot firmware, mechanical design, or solver internals were touched.

**Webcam scan window - rebuilt UI/UX**
- Replaced the old bare OpenCV window with a modern sidebar layout: live camera view, word-wrapped instructions, a 3x3 "captured colors" preview, an unfolded U/R/F/D/L/B face-progress map, and a themed button stack, all sized to a constant window height so the window no longer stretches/resizes between states.
- Hover highlighting on buttons and face-map cells, a real Windows hand cursor over clickable areas, and a Calibrate-colors action visually separated (its own section/divider) from the scan flow.
- Per-face accept/reject gate after each capture (Accept / Retry), a post-scan review screen that can rescan any single face, and - within that - click-to-cycle correction of an individual misread facelet's color, without rescanning the whole face.
- Camera feed upscales (aspect-preserving) to fill the sidebar's height instead of leaving dead gray space underneath it.

**Color detection - rewritten for reliability**
- Facelet-square detection ported from a proven small-square/neighbor-matching technique (QBR-style), with grid-clustering and one-missing-cell tolerance so a manufacturer's logo on a center facelet no longer breaks detection.
- Every detected color is classified (Lab/CIEDE2000 distance) to the nearest of the cube's 6 canonical colors and always displayed as that color's fixed reference value - a bad calibration sample can no longer leak through to the screen or solver as a 7th, muddy color (see `CLAUDE.md` for the standing rule this codified).
- Center facelets are sampled from multiple patches (median, not dead-center average) to avoid manufacturer-logo skew, and are additionally force-set to their structurally-known color as a final safety net.
- Full color calibration flow (`calibrate_cube_colors()`), with its scan/calibration order now matching the physical scan order below.

**Scan order changed to a natural physical sequence**
- Faces are now scanned White(U) -> Green(F) -> Yellow(D) -> Orange(L) -> Red(R) -> Blue(B) instead of raw Kociemba order. This required an actual reordering step (`reorder_to_kociemba_position()`) before handing facelets to the solver, rather than a relabel, since the solver's facelet indices are fixed to URFDLB - verified against the `twophase` cubie engine to confirm solving correctness was preserved.

**Scramble mode (new capability)**
- A "scramble" checkbox: generate a Random target cube, and have the robot scramble a *solved* physical cube into that exact target, instead of only ever solving a mixed one. Implemented via `invert_solution()` in `Cubotino_moves.py` (reverses move order and inverts each turn - the group-theory identity for "undo a solve") - verified by simulating the actual moves.
- The on-screen cube sketch now always shows real colors in scramble mode too (previously it hid the target behind a gray placeholder).

**Main GUI - consistent theming and small fixes**
- Applied the same button/font/color language as the webcam window across the whole app; buttons now render with a visible raised bevel (previously flat, easy to mistake for labels) and Tk's native press animation.
- Fixed a Windows-specific bug where secondary buttons rendered near-black after their `state` was toggled at runtime.
- Fixed clicking/scrolling on a center facelet's U/R/F/D/L/B letter label not recoloring the facelet underneath it.
- "Read & solve" relabels to "Scramble" while scramble mode is checked; the "clean cube" button relabeled to "Solved"; "Quit" relabeled to "Close"; the color-picking palette's label no longer overlaps the circles below it.

**Robot control robustness**
- Fixed the scramble-mode bug where the *displayed* moves were correctly inverted but the string actually transmitted to the robot was not - the robot would receive the original (un-inverted) solve, not the scramble.
- A **Reset** button appears in place of the usual disabled state after Stop, sending the robot back to its home position and clearing the GUI back to a clean state in one action.
- Hardened the serial-reading background thread against a silent crash (an uncaught exception in progress-bar/animation indexing used to kill it permanently, mid-run, with no further symptom than a frozen progress bar).

**Known limitation, deliberately not shipped**
- A "Resume" action (continue a Stopped run from its exact interruption point, instead of starting over) was prototyped and then pulled back out: it depends on the GUI's on-screen cube-state tracking (`cube_status`, updated via `animate_cube_sketch()`), which end-to-end simulation showed does **not** reliably reconstruct the correct final cube state even across a complete, uninterrupted run - a pre-existing issue in that animation model, not introduced here. Sending robot moves computed from it would risk being physically wrong, so only the safe Reset action is exposed until that's root-caused (or resume is rebuilt on a trustworthy state source, e.g. a fresh webcam rescan).

---

# CUBOTino base version

This repo contains the files to build CUBOTino_base_version: a Small, Simple, 3D Printed, Inexpensive Rubik's Cube Solver Robot.<br /><br />

This robot solves the Rubik's cube in less than 90 seconds: Not fast, but again ... it's rather simple and inexpensive.<br />
You can get an impression at https://youtu.be/ZVbVmCKwYnQ.<br />
Further robot info at: https://www.instructables.com/CUBOTino-a-Small-Simple-3D-Printed-Inexpensive-Rub/.<br />


![title image](/images/title_pic.jpg)



# How to make the robot:
All the needed info are collected in a pdf file in the /doc folder: [document here](doc/How_to_make_a_very_small_Rubik_cube_solver_robot_20230118.pdf).<br /><br />
Very high level notes:<br />
- 3D print without support.<br />
- Set the servo to their mid position.<br />
- Verify the servos have 180deg range.<br />
- For the rotating base (cube_holder) the servo must have at least 190deg rotation; If not check the instruction on how to proceed.<br />
- If you use macOS or Ubuntu, update RubikTwoPhase solver to latest version (v1.1.1, released on 16th Nov. 2022).<br /><br />


# How to present the cube to the webcam:
Video tutorial explaining how to present the cube to the webcam: https://youtu.be/udr6tryxA_Y.<br />
![title image](/images/title2_pic.png)



