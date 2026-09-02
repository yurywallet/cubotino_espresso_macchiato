#!/usr/bin/env python
# coding: utf-8

"""
Shared design tokens for CUBOTino espresso macchiato - the single source of truth for colors, fonts, and a
couple of shared shape constants, imported by both Cubotino_GUI.py (native Tkinter widgets, which want hex
strings) and Cubotino_webcam.py (a cv2-drawn canvas, which wants BGR tuples). Each file derives its own
values from what's defined here instead of keeping two independently hand-maintained copies in sync -
previously done by hand across this project's redesign, which works only as long as nobody ever forgets to
update both files identically.

"Colors may vary" between different UI elements is expected and fine (a danger/stop button is red, a primary
action is accent-colored, a neutral one is BTN_SECONDARY_BG, etc.) - what this file centralizes is the STYLE
SYSTEM itself: the token names, the actual color values behind them, and the couple of shared shape constants
below - not a rule that every widget everywhere must use one single color.

Typography is Tkinter-only: cv2 (Cubotino_webcam.py) can only draw text with its own built-in Hershey fonts
(e.g. cv2.FONT_HERSHEY_SIMPLEX), not an arbitrary system font family - there's no cross-technology way to make
a cv2-drawn canvas and a native Tk widget render literally the same font, so FONT_FAMILY/FONT below only
apply on the Tkinter side.
"""

# ---------------------------------------------------------------------------------------------------------
# palette
# ---------------------------------------------------------------------------------------------------------
BG = '#F7F8FA'                 # content-panel background (cards, sections, the webcam sidebar)
PAGE_BG = '#EAECEF'            # outer window/page background - one shade darker than BG, so a panel visibly
                                # separates from empty page space instead of blending into one flat tone
TEXT_PRIMARY = '#1A1D29'
TEXT_SECONDARY = '#6B7280'

ACCENT = '#4F46E5'             # primary / call-to-action color (indigo)
ACCENT_ACTIVE = '#4338CA'      # hover/press shade of ACCENT

DANGER = '#DC2626'             # destructive / stop actions (red)
DANGER_ACTIVE = '#B91C1C'

BTN_SECONDARY_BG = '#DBE0E6'   # neutral/secondary button fill - kept clearly darker than BG so it still
                                # reads as a button rather than blending into the panel behind it
BTN_SECONDARY_ACTIVE = '#C7CED6'

DIVIDER = '#E5E7EB'
DISABLED_BG = '#F1F2F4'
DISABLED_FG = '#C4C7CD'

# ---------------------------------------------------------------------------------------------------------
# typography (Tkinter only - see module docstring)
# ---------------------------------------------------------------------------------------------------------
FONT_FAMILY = 'Segoe UI'
FONT = (FONT_FAMILY, 11)
FONT_BOLD = (FONT_FAMILY, 12, 'bold')
FONT_TITLE = (FONT_FAMILY, 13, 'bold')

# ---------------------------------------------------------------------------------------------------------
# shape
# ---------------------------------------------------------------------------------------------------------
BTN_RADIUS = 8   # base corner radius for rounded buttons/panels. The webcam sidebar's compact cv2-drawn
                 # buttons use this value directly; Cubotino_GUI.py's larger native buttons use a
                 # proportionally bigger radius at their own call sites (they're a noticeably bigger physical
                 # button, not the same element) rather than this exact pixel count.


def hex_to_rgb(hex_color):
    """'#RRGGBB' -> (R, G, B), each 0-255."""
    hex_color = hex_color.lstrip('#')
    return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))


def hex_to_bgr(hex_color):
    """'#RRGGBB' -> (B, G, R), each 0-255 - the channel order cv2 expects everywhere (cv2.rectangle fill
    colors, etc.)."""
    r, g, b = hex_to_rgb(hex_color)
    return (b, g, r)
