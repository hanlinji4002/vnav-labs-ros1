#!/usr/bin/env python3
"""按一个键（XTest，无需安装）：xkey.py <keysym 十六进制，如 ff1b = Escape>"""
import ctypes, sys, time
X = ctypes.cdll.LoadLibrary('libX11.so.6')
T = ctypes.cdll.LoadLibrary('libXtst.so.6')
X.XOpenDisplay.restype = ctypes.c_void_p
d = ctypes.c_void_p(X.XOpenDisplay(None))
kc = X.XKeysymToKeycode(d, ctypes.c_ulong(int(sys.argv[1], 16)))
T.XTestFakeKeyEvent(d, kc, 1, 0); X.XFlush(d); time.sleep(0.05)
T.XTestFakeKeyEvent(d, kc, 0, 0); X.XFlush(d)
X.XCloseDisplay(d)
