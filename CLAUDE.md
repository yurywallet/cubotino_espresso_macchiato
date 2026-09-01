# Project rules for CUBOTino (this fork)

This is `yurywallet/cubotino_espresso_macchiato`, a personal fork of
[CUBOTino_base_version](https://github.com/AndreaFavero71/CUBOTino_base_version) (a small 3D-printed Rubik's
cube solver robot). See `README.md`'s "What's different in this fork" section for what's already changed
here versus upstream.

## Layout

- `PC_files/` — the PC-side app: `Cubotino_GUI.py` (Tkinter main window), `Cubotino_webcam.py` (webcam cube
  scanning/color detection), `Cubotino_moves.py` (Kociemba solution → robot move translation). This is where
  all work in this fork has happened so far.
- `ESP32_files/` — the robot's own MicroPython firmware (servo control, move execution). Treat changes here
  as higher-risk than PC-side changes: a bug here runs on physical motors against a real cube, not just on
  screen. Don't modify it without being explicitly asked to, and don't guess at its protocol/behavior — read
  it, or ask, rather than assuming PC-side logic generalizes to firmware.
- `doc/`, `images/`, `movies/`, `stl/` — upstream build documentation/media, not modified here.

## Running it

Python 3.12, dependencies in `PC_files/requirements.txt` (`pip install -r PC_files/requirements.txt`).
Entry point: `python PC_files/Cubotino_GUI.py` (add `-d`/`--debug` for verbose prints). `tkinter` ships with
standard Python installs on Windows; on Linux it may need a separate OS package (e.g. `python3-tk`).

## Camera access — hard rule

**Only view or log the webcam while the user has opened it themselves via the GUI.** Never launch the camera
independently (not even briefly "to check it's working") — this includes calling into `Cubotino_webcam.py`
functions that open a `cv2.VideoCapture`, or running `Cubotino_GUI.py` in a way that triggers a webcam read.
If you need to verify webcam-facing behavior, do it with synthetic frames/images instead (see how detection
logic has been verified with rendered test frames rather than live capture), or ask the user to test live and
report back.

## Cube color / solving-correctness facts (do not "fix" these — they're structural)

The cube has exactly **6 colors: white, red, green, yellow, orange, blue** — the same 6 names/order the
solver (`twophase`/kociemba) expects, in **URFDLB** facelet order (54 facelets, 9 per face, always U,R,F,D,L,B).

- **Never display or return a raw/tinted camera color as-is.** Classify every captured/displayed facelet
  color to the nearest of the 6 via Lab-space distance (`classify_bgr_to_6()` in `Cubotino_webcam.py`), and
  use *that* function's output for anything shown to the user or fed to the solver.
- A calibration/reference sample may decide *which* of the 6 colors is the closest match, but the color
  painted on screen must always be the fixed, bright reference for that name (`DEFAULT_REF_COLORS`), never
  the raw calibrated/captured value verbatim — a bad calibration sample must not visibly leak through as a
  7th, muddy, unrecognizable color. Any new preview/decoration surface (GUI canvas, cv2 overlay, etc.) must
  route through `classify_bgr_to_6()` rather than painting raw captured values directly.
- The physical **scan order** is U(white), F(green), D(yellow), L(orange), R(red), B(blue) — but the
  solver's facelet array is fixed to URFDLB order. Don't relabel/reorder without going through
  `reorder_to_kociemba_position()` (`Cubotino_webcam.py`) — a naive relabel silently produces a wrong cube
  status that still "looks" plausible. Same caution applies to `Cubotino_GUI.py`'s own `t = ("U","R","F","D","L","B")`
  ordering and anything indexed against it (e.g. `cube_status`, `SOLVED_DEFSTR`).
- Center facelets are a **structural fact**, not a classification guess: U's center is always white, R's
  always red, F's green, D's yellow, L's orange, B's blue — on a real cube this never varies. Code should
  force this (see `CENTER_COLOR` in `Cubotino_webcam.py`) rather than trust a distance-based read for centers.
- Never change the underlying color-to-letter solving logic (what determines the string handed to
  `sv.solve()`) as a side effect of a UI/display request. UI/cosmetic changes (sketch colors, gray/obscured
  previews, etc.) must not alter what gets solved.

## Verification standard

This project involves a physical robot — a wrong move sequence isn't just a bug, it's the robot acting on a
real cube (or, worse, mechanically fighting itself). Before presenting a change to solving/move-generation
logic as done:
- Prefer verifying against the bundled `twophase`/`RubikTwoPhase` cubie engine (simulate the actual moves on
  a cubie representation) over reasoning about notation by hand.
- If a change touches something that only manifests live (webcam detection quality, GUI layout/hover states,
  actual serial/robot behavior), say plainly that it's unverified and ask the user to test — don't claim
  success for something that was only compile-checked or reasoned about.
- A past example worth remembering: a "Resume after Stop" feature was prototyped, then pulled back out
  *because* an end-to-end simulation (not just code review) revealed the GUI's on-screen cube-state tracking
  doesn't reliably reflect the robot's true physical state. Don't skip this kind of check for anything that
  would send computed moves to real hardware.

## Style notes from this fork's history

- Comments explain *why*, not *what* — this codebase's existing comment style is heavier than usual; match
  the surrounding file's density rather than stripping comments down.
- Keep changes scoped to what's asked; this fork intentionally does not track upstream, so there's no need to
  generalize changes for hypothetical merge-back.
