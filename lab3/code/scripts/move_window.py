#!/usr/bin/env python3
"""Move/resize an X11 window by its title: move_window.py <title> <x> <y> [w h]  (ctypes + libX11, nothing to install)."""
import ctypes, subprocess, sys
title, x, y = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
info = subprocess.run(['xwininfo', '-name', title], capture_output=True, text=True).stdout
wid = int(info.split('Window id: ')[1].split()[0], 16)
X = ctypes.cdll.LoadLibrary('libX11.so.6')
X.XOpenDisplay.restype = ctypes.c_void_p
dpy = ctypes.c_void_p(X.XOpenDisplay(None))
if len(sys.argv) > 5:
    X.XMoveResizeWindow(dpy, ctypes.c_ulong(wid), x, y, int(sys.argv[4]), int(sys.argv[5]))
else:
    X.XMoveWindow(dpy, ctypes.c_ulong(wid), x, y)
X.XFlush(dpy)
X.XCloseDisplay(dpy)
