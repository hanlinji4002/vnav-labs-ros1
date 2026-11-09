#!/usr/bin/env python3
"""Capture one real mapped X11 window without changing its rendering.

Example (WSLg, no Xvfb):
  python3 -B capture_x11_window.py --window-name tesse_vnav_scenes \
    --output run/scene.mp4 --duration 12 --fps 60 \
    --ready-file run/capture.ready.json --backing-path /mnt/c

XGetImage -> original-size RGB PPM + per-frame monotonic JSONL -> FFmpeg
image2 nanosecond-mtime VFR (supported by Focal FFmpeg 4.2). A missed deadline is skipped, never filled by interpolated
images or compressed into a fixed-FPS timeline. Encoding occurs AFTER capture.
The final repeated image is an explicit end-boundary sentinel, not a new sample.
FFprobe duration must match the actual capture span before publishing output.

Raw frames remain ONLY beside output in <output>.capture (up to 3.111 GB RGB
for 30s/60fps/960x600). Do not transfer them to the low-free-space Mac. Both the
destination filesystem and --backing-path (WSL host physical drive) are checked.
SIGINT/SIGTERM during capture requests a graceful early finish and encoding;
a signal during encoding aborts only this helper's own encoder process.
"""
import argparse
import ctypes as C
import ctypes.util
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time


MAX_DURATION = 30.0
MAX_FPS = 60.0
MAX_WIDTH, MAX_HEIGHT = 960, 600
FRAME_NAME = re.compile(r"frames/frame_[0-9]{6}\.ppm\Z")


def parse_window_id(text):
    try:
        result = int(str(text), 16 if str(text).lower().startswith("0x") else 10)
    except (ValueError, TypeError):
        raise ValueError("window ID must be decimal or 0x hexadecimal")
    if not 1 <= result <= 0xffffffff:
        raise ValueError("window ID must be nonzero unsigned 32-bit")
    return result


def validate_request(window_id, duration, fps):
    if isinstance(window_id, bool) or not isinstance(window_id, int) or not 1 <= window_id <= 0xffffffff:
        raise ValueError("invalid window ID")
    if not math.isfinite(duration) or not 0 < duration <= MAX_DURATION:
        raise ValueError("duration must be finite and within (0, 30] seconds")
    if not math.isfinite(fps) or not 1 <= fps <= MAX_FPS:
        raise ValueError("fps must be finite and within [1, 60]")


def _pixel_layout(data, width, height, bytes_per_line, bits_per_pixel, byte_order,
                  red_mask, green_mask, blue_mask):
    if width <= 0 or height <= 0 or bits_per_pixel not in (24, 32) or byte_order not in (0, 1):
        raise ValueError("unsupported pixel dimensions/depth/byte order")
    size = bits_per_pixel // 8
    if bytes_per_line < width * size or len(data) < bytes_per_line * height:
        raise ValueError("truncated image or invalid row stride")
    masks = (red_mask, green_mask, blue_mask)
    if any(not isinstance(m, int) or m <= 0 or m >= 1 << bits_per_pixel for m in masks):
        raise ValueError("invalid RGB mask")
    if (red_mask & green_mask) or (red_mask & blue_mask) or (green_mask & blue_mask):
        raise ValueError("overlapping RGB masks")
    channels = []
    for mask in masks:
        shift = (mask & -mask).bit_length() - 1
        maximum = mask >> shift
        if maximum & (maximum + 1):
            raise ValueError("noncontiguous RGB mask")
        channels.append((mask, shift, maximum))
    return size, channels


def unpack_rgb(data, width, height, bytes_per_line, bits_per_pixel, byte_order,
               red_mask, green_mask, blue_mask):
    """Pure, dependency-free true-color pixel/stride decoder (no X11 calls)."""
    size, channels = _pixel_layout(data, width, height, bytes_per_line,
                                   bits_per_pixel, byte_order, red_mask, green_mask, blue_mask)
    result = bytearray(width * height * 3)
    target = 0
    for y in range(height):
        for x in range(width):
            offset = y * bytes_per_line + x * size
            value = int.from_bytes(data[offset:offset + size], "little" if byte_order == 0 else "big")
            for mask, shift, maximum in channels:
                result[target] = (((value & mask) >> shift) * 255 + maximum // 2) // maximum
                target += 1
    return bytes(result)


def _runtime_rgb(data, width, height, stride, bpp, order, red, green, blue):
    _pixel_layout(data, width, height, stride, bpp, order, red, green, blue)
    # Fast C implementation for the real WSLg BGRX/BGR layout; no resampling.
    if order == 0 and (red, green, blue) == (0xff0000, 0xff00, 0xff):
        try:
            from PIL import Image
            return Image.frombytes("RGB", (width, height), data, "raw",
                                   "BGRX" if bpp == 32 else "BGR", stride, 1).tobytes()
        except ImportError:
            pass
    return unpack_rgb(data, width, height, stride, bpp, order, red, green, blue)


def build_concat_text(frame_names, timestamps_ns, end_ns):
    """Optional diagnostic concat list for newer FFmpeg; runtime uses image2.

Focal 4.2's concat demuxer lacks per-file option directives. The runtime instead
uses its supported image2 -ts_from_file 2, retaining nanosecond frame intervals.
"""
    if not frame_names or len(frame_names) != len(timestamps_ns):
        raise ValueError("frames and timestamps must have equal nonzero lengths")
    if any(not FRAME_NAME.fullmatch(name) for name in frame_names) or len(set(frame_names)) != len(frame_names):
        raise ValueError("unsafe or duplicate frame filename")
    if any(isinstance(t, bool) or not isinstance(t, int) for t in timestamps_ns + [end_ns]):
        raise ValueError("timestamps must be integer monotonic nanoseconds")
    if any(b <= a for a, b in zip(timestamps_ns, timestamps_ns[1:])) or end_ns <= timestamps_ns[-1]:
        raise ValueError("timestamps/end must increase strictly")
    lines = ["ffconcat version 1.0"]
    for name, start, end in zip(frame_names, timestamps_ns, timestamps_ns[1:] + [end_ns]):
        lines.extend(["file '" + name + "'", "option framerate 1000",
                      "duration %.9f" % ((end - start) / 1e9)])
    lines.extend(["file '" + frame_names[-1] + "'", "option framerate 1000"])
    return "\n".join(lines) + "\n"


def frame_mtimes_ns(timestamps_ns, end_ns):
    """Map monotonic samples and the end sentinel to positive epoch file times."""
    if not timestamps_ns or any(isinstance(t, bool) or not isinstance(t, int) for t in timestamps_ns + [end_ns]):
        raise ValueError("need integer monotonic timestamps")
    if any(b <= a for a, b in zip(timestamps_ns, timestamps_ns[1:])) or end_ns <= timestamps_ns[-1]:
        raise ValueError("timestamps and end must increase")
    return [1_000_000_000 + t - timestamps_ns[0] for t in timestamps_ns + [end_ns]]


def duration_matches(capture_span_s, video_duration_s, tolerance_s=0.10):
    try:
        return (all(math.isfinite(v) for v in (capture_span_s, video_duration_s, tolerance_s))
                and capture_span_s > 0 and video_duration_s > 0 and tolerance_s >= 0
                and abs(capture_span_s - video_duration_s) <= tolerance_s)
    except (ValueError, TypeError):
        return False


def build_encode_command(frames_dir, pending):
    """Encode unchanged monotonic image timestamps; never resample frames."""
    command = ["ffmpeg", "-nostdin", "-n", "-hide_banner", "-loglevel", "warning",
               "-f", "image2", "-framerate", "1000", "-ts_from_file", "2", "-start_number", "0",
               "-i", str(Path(frames_dir) / "frame_%06d.ppm"),
               "-an", "-vf", "setpts=PTS-STARTPTS", "-vsync", "vfr", "-c:v", "libx264", "-preset", "ultrafast",
               "-crf", "18", "-pix_fmt", "yuv420p", "-enc_time_base", "1/1000"]
    if Path(pending).suffix.lower() == ".mp4":
        # Focal 4.2 otherwise writes an edit-list end equal to the final
        # sentinel's PTS, marking that sample discard during default decoding.
        # Keep all samples and their PTS; the strict post-encode checks remain.
        command.extend(["-video_track_timescale", "1000", "-use_editlist", "0", "-movflags", "+faststart"])
    command.append(str(pending))
    return command


class _ImageFunctions(C.Structure):
    _fields_ = [(name, C.c_void_p) for name in
                ("create_image", "destroy_image", "get_pixel", "put_pixel", "sub_image", "add_pixel")]


class _XImage(C.Structure):
    _fields_ = [("width", C.c_int), ("height", C.c_int), ("xoffset", C.c_int),
                ("format", C.c_int), ("data", C.c_void_p), ("byte_order", C.c_int),
                ("bitmap_unit", C.c_int), ("bitmap_bit_order", C.c_int), ("bitmap_pad", C.c_int),
                ("depth", C.c_int), ("bytes_per_line", C.c_int), ("bits_per_pixel", C.c_int),
                ("red_mask", C.c_ulong), ("green_mask", C.c_ulong), ("blue_mask", C.c_ulong),
                ("obdata", C.c_void_p), ("functions", _ImageFunctions)]


class _WindowAttributes(C.Structure):
    _fields_ = [("x", C.c_int), ("y", C.c_int), ("width", C.c_int), ("height", C.c_int),
                ("border_width", C.c_int), ("depth", C.c_int), ("visual", C.c_void_p),
                ("root", C.c_ulong), ("visual_class", C.c_int), ("bit_gravity", C.c_int),
                ("win_gravity", C.c_int), ("backing_store", C.c_int), ("backing_planes", C.c_ulong),
                ("backing_pixel", C.c_ulong), ("save_under", C.c_int), ("colormap", C.c_ulong),
                ("map_installed", C.c_int), ("map_state", C.c_int), ("all_event_masks", C.c_long),
                ("your_event_mask", C.c_long), ("do_not_propagate_mask", C.c_long),
                ("override_redirect", C.c_int), ("screen", C.c_void_p)]


class _XError(C.Structure):
    _fields_ = [("type", C.c_int), ("display", C.c_void_p), ("resourceid", C.c_ulong),
                ("serial", C.c_ulong), ("error_code", C.c_ubyte),
                ("request_code", C.c_ubyte), ("minor_code", C.c_ubyte)]


class X11Window:
    def __init__(self, display=None):
        library = C.util.find_library("X11")
        if not library:
            raise RuntimeError("libX11 unavailable")
        self.x = C.CDLL(library)
        signatures = {
            "XOpenDisplay": ([C.c_char_p], C.c_void_p),
            "XCloseDisplay": ([C.c_void_p], C.c_int),
            "XSync": ([C.c_void_p, C.c_int], C.c_int),
            "XSetErrorHandler": ([C.c_void_p], C.c_void_p),
            "XDefaultRootWindow": ([C.c_void_p], C.c_ulong),
            "XGetWindowAttributes": ([C.c_void_p, C.c_ulong, C.POINTER(_WindowAttributes)], C.c_int),
            "XGetImage": ([C.c_void_p, C.c_ulong, C.c_int, C.c_int, C.c_uint, C.c_uint, C.c_ulong, C.c_int], C.POINTER(_XImage)),
            "XFetchName": ([C.c_void_p, C.c_ulong, C.POINTER(C.c_void_p)], C.c_int),
            "XQueryTree": ([C.c_void_p, C.c_ulong, C.POINTER(C.c_ulong), C.POINTER(C.c_ulong), C.POINTER(C.POINTER(C.c_ulong)), C.POINTER(C.c_uint)], C.c_int),
            "XFree": ([C.c_void_p], C.c_int),
        }
        for name, (args, result) in signatures.items():
            fn = getattr(self.x, name)
            fn.argtypes, fn.restype = args, result
        self.display = self.x.XOpenDisplay(display.encode() if display else None)
        if not self.display:
            raise RuntimeError("cannot open the real X11 DISPLAY; no replacement display will be started")
        self.errors = []
        callback_type = C.CFUNCTYPE(C.c_int, C.c_void_p, C.POINTER(_XError))
        def on_error(unused, error):
            e = error.contents
            self.errors.append((e.error_code, e.request_code, e.resourceid))
            return 0
        self.callback = callback_type(on_error)
        self.old_handler = self.x.XSetErrorHandler(C.cast(self.callback, C.c_void_p))
        self.root = self.x.XDefaultRootWindow(self.display)

    def _sync(self):
        self.x.XSync(self.display, 0)
        if self.errors:
            errors, self.errors = self.errors, []
            raise RuntimeError("X11 errors: " + repr(errors))

    def attributes(self, window):
        result = _WindowAttributes()
        ok = self.x.XGetWindowAttributes(self.display, window, C.byref(result))
        self._sync()
        if not ok:
            raise RuntimeError("cannot read window attributes")
        return result

    def title(self, window):
        value = C.c_void_p()
        ok = self.x.XFetchName(self.display, window, C.byref(value))
        self._sync()
        try:
            return C.string_at(value.value).decode("utf-8", errors="replace") if ok and value.value else ""
        finally:
            if value.value:
                self.x.XFree(value)

    def find(self, name):
        if not name or len(name) > 256:
            raise ValueError("window name must contain 1..256 characters")
        todo, visited, matches = [self.root], set(), []
        while todo:
            window = todo.pop()
            if window in visited:
                continue
            visited.add(window)
            if len(visited) > 4096:
                raise RuntimeError("window tree exceeds bounded search")
            if window != self.root and name in self.title(window):
                attrs = self.attributes(window)
                if attrs.map_state == 2:
                    matches.append((window, self.title(window)))
            root, parent, count = C.c_ulong(), C.c_ulong(), C.c_uint()
            children = C.POINTER(C.c_ulong)()
            ok = self.x.XQueryTree(self.display, window, C.byref(root), C.byref(parent), C.byref(children), C.byref(count))
            self._sync()
            try:
                if ok:
                    todo.extend(children[i] for i in range(count.value))
            finally:
                if children:
                    self.x.XFree(C.cast(children, C.c_void_p))
        if len(matches) != 1:
            raise RuntimeError("need exactly one mapped matching X11 window, found: " + repr(matches))
        return matches[0][0]

    def grab(self, window, width, height):
        attrs = self.attributes(window)
        if window == self.root or window == attrs.root:
            raise ValueError("root-window capture is intentionally forbidden")
        if attrs.map_state != 2 or (attrs.width, attrs.height) != (width, height):
            raise RuntimeError("window unmapped or resized; capture cannot continue unchanged")
        started = time.monotonic_ns()
        image = self.x.XGetImage(self.display, window, 0, 0, width, height, C.c_ulong(-1).value, 2)
        ended = time.monotonic_ns()
        try:
            self._sync()
            if not image or not image.contents.data:
                raise RuntimeError("XGetImage returned no real window pixels")
            obj = image.contents
            if obj.width != width or obj.height != height or obj.xoffset != 0 or obj.format != 2:
                raise RuntimeError("unexpected XImage dimensions/layout")
            if obj.bytes_per_line > MAX_WIDTH * 8 or obj.bytes_per_line <= 0:
                raise RuntimeError("unbounded image row stride")
            raw = C.string_at(obj.data, obj.bytes_per_line * height)
            rgb = _runtime_rgb(raw, width, height, obj.bytes_per_line, obj.bits_per_pixel,
                               obj.byte_order, obj.red_mask, obj.green_mask, obj.blue_mask)
            return rgb, started, ended
        finally:
            if image:
                destroy = image.contents.functions.destroy_image
                if destroy:
                    C.CFUNCTYPE(C.c_int, C.POINTER(_XImage))(destroy)(image)

    def close(self):
        if self.display:
            self.x.XSync(self.display, 0)
            self.x.XSetErrorHandler(self.old_handler)
            self.x.XCloseDisplay(self.display)
            self.display = None


def _write_json_new(path, data):
    with path.open("x", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")


def _finish_owned(process):
    if process is not None and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)


def _disk_checks(destination, backing, needed):
    paths = [("capture_destination", destination), ("physical_backing_drive", backing)]
    result = []
    for label, path in paths:
        if not path.is_dir():
            raise ValueError(label + " is not exposed: " + str(path))
        free = shutil.disk_usage(path).free
        result.append({"kind": label, "path": str(path), "free_bytes": free, "required_bytes": needed})
        if free < needed:
            raise RuntimeError("insufficient space on " + label + ": " + str(path))
    return result


def execute(args):
    validate_request(args.window_id or 1, args.duration, args.fps)
    if not math.isfinite(args.encode_timeout) or not 1 <= args.encode_timeout <= 180:
        raise ValueError("encode timeout must be within 1..180 seconds")
    parent = args.output.expanduser().absolute().parent.resolve(strict=True)
    output = parent / args.output.name
    if output.suffix.lower() not in (".mp4", ".mkv") or os.path.lexists(output):
        raise ValueError("output must be a new .mp4 or .mkv file")
    ready = args.ready_file.expanduser().absolute() if args.ready_file else None
    if ready and (os.path.lexists(ready) or not ready.parent.is_dir()):
        raise ValueError("ready-file must be new and its parent must exist")
    if ready and ready.resolve() == output.resolve():
        raise ValueError("ready-file cannot be the video output")
    for command in ("ffmpeg", "ffprobe"):
        if not shutil.which(command):
            raise RuntimeError(command + " is unavailable")
    x, encoder = None, None
    artifacts = Path(str(output) + ".capture")
    artifacts_owned = False
    report = {"status": "not_started", "video_path": str(output), "duration_verified": False,
              "requested_duration_s": args.duration, "requested_fps": args.fps,
              "rendering_changes": False, "frame_interpolation": False,
              "raw_frames_retained_in_wsl_only": True}
    phase, stopped = {"name": "capture"}, {"signal": None}
    def request_stop(number, unused):
        if phase["name"] != "capture":
            raise KeyboardInterrupt("signal during post-capture encoding")
        stopped["signal"] = number
    previous = {s: signal.signal(s, request_stop) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        x = X11Window(args.display)
        window = args.window_id if args.window_id else x.find(args.window_name)
        validate_request(window, args.duration, args.fps)
        attrs = x.attributes(window)
        if window == x.root or attrs.map_state != 2:
            raise ValueError("need a mapped, non-root client window")
        width, height = attrs.width, attrs.height
        if not (1 <= width <= MAX_WIDTH and 1 <= height <= MAX_HEIGHT) or width % 2 or height % 2:
            raise ValueError("unchanged window size must be even and at most 960x600; no resizing is performed")
        frame_limit = math.ceil(args.duration * args.fps)
        raw_limit = frame_limit * (width * height * 3 + 64)
        report["space_checks"] = _disk_checks(parent, args.backing_path, 2 * raw_limit + 256 * 1024 * 1024)
        report.update(window_id=hex(window), window_title=x.title(window), width=width, height=height,
                      maximum_ppm_bytes=raw_limit, capture_artifacts=str(artifacts))
        artifacts.mkdir(mode=0o700, exist_ok=False)
        artifacts_owned = True
        frames_dir = artifacts / "frames"
        frames_dir.mkdir(mode=0o700)
        names, stamps = [], []
        started = time.monotonic_ns()
        deadline = started + round(args.duration * 1e9)
        scheduled_index, missed = 0, 0
        with (artifacts / "frames.jsonl").open("x", encoding="utf-8") as telemetry:
            while len(names) < frame_limit and not stopped["signal"]:
                target = started + round(scheduled_index * 1e9 / args.fps)
                now = time.monotonic_ns()
                if now >= deadline:
                    break
                if target > now:
                    time.sleep(min((target - now) / 1e9, (deadline - now) / 1e9))
                if time.monotonic_ns() >= deadline or stopped["signal"]:
                    break
                rgb, read_start, read_end = x.grab(window, width, height)
                stamp = (read_start + read_end) // 2
                name = "frames/frame_%06d.ppm" % len(names)
                write_start = time.monotonic_ns()
                with (artifacts / name).open("xb") as f:
                    f.write(("P6\n%d %d\n255\n" % (width, height)).encode("ascii"))
                    f.write(rgb)
                write_end = time.monotonic_ns()
                names.append(name)
                stamps.append(stamp)
                item = {"frame": len(names) - 1, "file": name, "scheduled_index": scheduled_index,
                        "monotonic_ns": stamp, "xgetimage_start_ns": read_start, "xgetimage_end_ns": read_end,
                        "elapsed_from_first_frame_s": (stamp - stamps[0]) / 1e9,
                        "write_duration_s": (write_end - write_start) / 1e9,
                        "rgb_sha256": hashlib.sha256(rgb).hexdigest()}
                telemetry.write(json.dumps(item, separators=(",", ":")) + "\n")
                telemetry.flush()
                if len(names) == 1 and ready:
                    _write_json_new(ready, {"ready": True, "video_path": str(output), "window_id": hex(window),
                                           "width": width, "height": height, "first_frame_monotonic_ns": stamp,
                                           "first_frame_file": str(artifacts / name)})
                if len(names) % max(1, round(args.fps)) == 0:
                    _disk_checks(parent, args.backing_path, raw_limit + 128 * 1024 * 1024)
                next_index = max(scheduled_index + 1, math.floor((time.monotonic_ns() - started) * args.fps / 1e9) + 1)
                missed += next_index - scheduled_index - 1
                scheduled_index = next_index
            if not stopped["signal"] and time.monotonic_ns() < deadline:
                time.sleep((deadline - time.monotonic_ns()) / 1e9)
            ended = time.monotonic_ns()
        x.close()
        x = None
        if not names:
            raise RuntimeError("no real window frame captured")
        concat = build_concat_text(names, stamps, ended)
        with (artifacts / "frames.ffconcat").open("x", encoding="ascii") as f:
            f.write(concat)
        # Focal's concat demuxer cannot set the image input time base per file.
        # image2 -ts_from_file 2 reads exact POSIX nanosecond mtimes in 4.2.
        # Only OUR newly-created PPM files are timestamped; no system/ROS clock
        # or renderer state is touched. One explicit end-boundary image follows.
        sentinel_name = "frames/frame_%06d.ppm" % len(names)
        with (artifacts / sentinel_name).open("xb") as f:
            f.write(("P6\n%d %d\n255\n" % (width, height)).encode("ascii"))
            f.write(rgb)
        for name, mtime in zip(names + [sentinel_name], frame_mtimes_ns(stamps, ended)):
            path = artifacts / name
            os.utime(path, ns=(mtime, mtime))
            if abs(path.stat().st_mtime_ns - mtime) > 1_000_000:
                raise RuntimeError("filesystem cannot retain frame timestamp precision within 1 ms")
        capture_span = (ended - stamps[0]) / 1e9
        report.update(status="captured_encoding", frame_count=len(names), missed_schedule_slots=missed,
                      first_frame_monotonic_ns=stamps[0], last_frame_monotonic_ns=stamps[-1],
                      capture_end_monotonic_ns=ended, capture_span_s=capture_span,
                      first_to_last_sample_span_s=(stamps[-1] - stamps[0]) / 1e9,
                      capture_average_fps=len(names) / capture_span,
                      stop_signal=stopped["signal"], end_boundary_duplicate_frames=1,
                      encoding_timing="image2 -ts_from_file 2; new PPM mtimes map exactly from monotonic samples; concat list is diagnostic only")
        phase["name"] = "encoding"
        pending = artifacts / ("encoded_pending" + output.suffix.lower())
        command = build_encode_command(frames_dir, pending)
        report["ffmpeg_command"] = command
        with (artifacts / "ffmpeg.log").open("xb") as log:
            encoder = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            try:
                result = encoder.wait(timeout=args.encode_timeout)
            finally:
                _finish_owned(encoder)
        if result:
            raise RuntimeError("owned FFmpeg failed; inspect ffmpeg.log")
        probe = subprocess.run(["ffprobe", "-v", "error", "-show_frames", "-show_entries",
                                "frame=best_effort_timestamp_time:format=duration:stream=codec_type,start_time,duration,nb_frames,avg_frame_rate,r_frame_rate,time_base",
                                "-of", "json", str(pending)], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               timeout=10, check=True)
        info = json.loads(probe.stdout)
        _write_json_new(artifacts / "ffprobe.json", info)
        video_duration = float(info["format"]["duration"])
        videos = [s for s in info.get("streams", []) if s.get("codec_type") == "video"]
        if len(videos) != 1:
            raise RuntimeError("need exactly one encoded video stream")
        report.update(ffprobe_duration_s=video_duration, duration_error_s=video_duration - capture_span,
                      duration_tolerance_s=0.10,
                      duration_verified=duration_matches(capture_span, video_duration))
        decoded_times = [float(frame["best_effort_timestamp_time"]) for frame in info.get("frames", [])]
        expected_times = [(stamp - stamps[0]) / 1e9 for stamp in stamps + [ended]]
        frame_error = max((abs(a - b) for a, b in zip(decoded_times, expected_times)), default=float("inf"))
        frame_verified = len(decoded_times) == len(expected_times) and frame_error <= 0.010
        report.update(decoded_frame_count=len(decoded_times), frame_timestamps_verified=frame_verified,
                      maximum_frame_timestamp_error_s=frame_error if math.isfinite(frame_error) else None,
                      frame_timestamp_tolerance_s=0.010)
        if not report["duration_verified"]:
            raise RuntimeError("video duration does not match monotonic capture span; refusing publication")
        if not frame_verified:
            raise RuntimeError("decoded frame count/timestamps differ from real samples; refusing publication")
        # Same filesystem, atomic publication without replacing any existing target.
        os.link(pending, output)
        pending.unlink()
        report.update(status="complete", video_bytes=output.stat().st_size)
        return report
    except BaseException as error:
        report.update(status="failed", error=type(error).__name__ + ": " + str(error))
        raise
    finally:
        _finish_owned(encoder)
        if x:
            x.close()
        for number, handler in previous.items():
            signal.signal(number, handler)
        if artifacts_owned and not (artifacts / "capture_report.json").exists():
            _write_json_new(artifacts / "capture_report.json", report)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--window-id", type=parse_window_id)
    selection.add_argument("--window-name", help="unique mapped X11 window whose title contains this text")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--fps", type=float, default=60)
    parser.add_argument("--ready-file", type=Path)
    parser.add_argument("--backing-path", type=Path, default=Path("/mnt/c"),
                        help="exposed physical host drive backing the WSL VHD; checked as well as output filesystem")
    parser.add_argument("--display", help="existing real WSLg/X11 display; default is DISPLAY, never starts a server")
    parser.add_argument("--encode-timeout", type=float, default=30)
    args = parser.parse_args()
    try:
        report = execute(args)
    except (Exception, KeyboardInterrupt) as error:
        print(json.dumps({"status": "failed", "duration_verified": False,
                          "error": type(error).__name__ + ": " + str(error)}, ensure_ascii=False), flush=True)
        return 2
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
