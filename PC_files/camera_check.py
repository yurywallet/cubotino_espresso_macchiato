#!/usr/bin/env python
# coding: utf-8

"""
#############################################################################################################
# Standalone camera check for CUBOTino espresso macchiato.
#
# It opens nothing else: no solver, no GUI, no cube detection. For each webcam number it reports whether the
# device can be opened, whether it actually returns frames, and at which resolution - the three things that
# "Cannot open camera" and "the camera is open but returns no frames" cannot tell apart on their own.
#
# Nothing is displayed or saved: only the frame size is printed, never the image.
#
# Usage:
#   python camera_check.py              # probes webcam numbers 0 to 5
#   python camera_check.py 0 1          # probes only the given webcam numbers
#############################################################################################################
"""

import sys
import time

import cv2

WARMUP_S = 3.0                    # seconds waited for an opened camera to return its first frame


def backends():
    """The capture backends worth trying on this platform, as (name, cv2 constant) pairs."""
    if sys.platform.startswith('win'):
        return [('CAP_DSHOW', cv2.CAP_DSHOW), ('CAP_MSMF', cv2.CAP_MSMF), ('CAP_ANY', cv2.CAP_ANY)]
    elif sys.platform == 'darwin':
        return [('CAP_AVFOUNDATION', cv2.CAP_AVFOUNDATION), ('CAP_ANY', cv2.CAP_ANY)]
    else:
        return [('CAP_V4L2', cv2.CAP_V4L2), ('CAP_ANY', cv2.CAP_ANY)]


def first_frame(cap, timeout=WARMUP_S):
    """(width, height) of the first frame arriving within the timeout, or None."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        ret, frame = cap.read()
        if ret and frame is not None:
            return frame.shape[1], frame.shape[0]
        time.sleep(0.1)
    return None


def probe(cam_num, backend_name, backend):
    """Reports one webcam number on one backend."""
    cap = None
    try:
        cap = cv2.VideoCapture(cam_num, backend)
        if not cap.isOpened():
            print(f'  {backend_name:16s} not opened')
            return
        size = first_frame(cap)
        if size is None:
            asked = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            print(f'  {backend_name:16s} OPENED but no frames (device reports {asked[0]}x{asked[1]})')
        else:
            print(f'  {backend_name:16s} WORKS, frames at {size[0]}x{size[1]}')
    except Exception as ex:
        print(f'  {backend_name:16s} error: {ex}')
    finally:
        if cap is not None:
            cap.release()


def probe_resolution(cam_num, backend_name, backend, req_w, req_h):
    """Repeats the app's own open sequence: open, ask for a resolution, then try to read."""
    cap = None
    try:
        cap = cv2.VideoCapture(cam_num, backend)
        if not cap.isOpened():
            print(f'  {backend_name:16s} {req_w}x{req_h}: not opened')
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, req_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, req_h)
        size = first_frame(cap)
        if size is None:
            print(f'  {backend_name:16s} {req_w}x{req_h}: NO FRAMES once this resolution is requested')
        else:
            print(f'  {backend_name:16s} {req_w}x{req_h}: works, frames actually at {size[0]}x{size[1]}')
    except Exception as ex:
        print(f'  {backend_name:16s} {req_w}x{req_h}: error: {ex}')
    finally:
        if cap is not None:
            cap.release()


def main():
    print(f'platform: {sys.platform}   OpenCV: {cv2.__version__}\n')
    numbers = [int(a) for a in sys.argv[1:]] or list(range(6))

    print('--- opening each webcam number, no resolution requested ---\n')
    for cam_num in numbers:
        print(f'webcam number {cam_num}:')
        for name, backend in backends():
            probe(cam_num, name, backend)
        print()

    # the app does not just open the camera, it asks it for a resolution first; a camera that works when
    # taken as it comes can stop delivering frames once asked for a size it does not actually offer
    name, backend = backends()[0]
    print('--- repeating the app\'s own sequence: open, request a resolution, read ---\n')
    for cam_num in numbers:
        print(f'webcam number {cam_num}:')
        for req_w, req_h in ((640, 360), (640, 480), (1280, 720)):
            probe_resolution(cam_num, name, backend, req_w, req_h)
        print()

    print('Use a webcam number reported as WORKS on the GUI settings page.')


if __name__ == '__main__':
    main()
