#!/usr/bin/env python3
"""Annotate real X11 window samples with the same run's gate stopwatch.

The unchanged native capture helper owns sampling, timestamps, VFR encoding,
and strict decode/PTS checks. Only a small HUD rectangle is replaced in each
sample. No simulator motion, clock, frame order or frame interval is changed.
The pre-annotation RGB hash and exact timer snapshot used are retained per
sample, separately from the helper's annotated-frame RGB hash.
"""
import argparse
import collections
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def timer_view(state, now, stale_s=.5):
    """Validate an observed snapshot; never extrapolate elapsed simulation time."""
    fallback = {"display_status": "WAIT", "elapsed_sim_s": None,
                "elapsed_wall_estimate_s": None, "gates_passed": None,
                "gates_total": None, "collision_messages_first_to_last": None, "snapshot_age_s": None,
                "valid_snapshot": False, "reason": "timer state unavailable"}
    if not isinstance(state, dict):
        return fallback
    stamp = state.get("observer_monotonic_s")
    if not finite(stamp) or not finite(now) or not finite(stale_s) or stale_s <= 0:
        return dict(fallback, display_status="INVALID", reason="invalid monotonic clock")
    age = now-stamp
    if age < -.01:
        return dict(fallback, display_status="INVALID", snapshot_age_s=age,
                    reason="timer snapshot is from a different/future monotonic clock")
    if age > stale_s:
        return dict(fallback, display_status="STALE", snapshot_age_s=age,
                    reason="timer state no longer fresh")
    if state.get("valid_data") is not True:
        return dict(fallback, display_status="DATA ERROR", snapshot_age_s=age,
                    reason="timer rejected source data")
    status = state.get("status")
    elapsed = state.get("elapsed_sim_s")
    passed, total = state.get("gates_passed"), state.get("gates_total")
    if (status not in ("WAITING", "RUNNING", "FINISHED") or
            not finite(elapsed) or elapsed < 0 or
            isinstance(passed, bool) or not isinstance(passed, int) or
            isinstance(total, bool) or not isinstance(total, int) or
            total < 2 or not 0 <= passed <= total):
        return dict(fallback, display_status="INVALID", snapshot_age_s=age,
                    reason="invalid stopwatch values")
    if ((status == "WAITING" and (passed != 0 or elapsed != 0)) or
            (status == "RUNNING" and not 1 <= passed < total) or
            (status == "FINISHED" and passed != total)):
        return dict(fallback, display_status="INVALID", snapshot_age_s=age,
                    reason="stopwatch status disagrees with gate sequence")
    if status != "WAITING":
        start = state.get("start_sim_s")
        if not finite(start):
            return dict(fallback, display_status="INVALID", snapshot_age_s=age,
                        reason="missing actual first-gate time")
    if status == "FINISHED":
        finish = state.get("finish_sim_s")
        if not finite(finish) or finish < start or abs((finish-start)-elapsed) > 1e-6:
            return dict(fallback, display_status="INVALID", snapshot_age_s=age,
                        reason="final time is not last-gate minus first-gate time")
    wall = state.get("elapsed_wall_estimate_s")
    if wall is not None and (not finite(wall) or wall < 0):
        return dict(fallback, display_status="INVALID", snapshot_age_s=age,
                    reason="invalid wall-time estimate")
    hits = state.get("true_collision_messages_first_to_last")
    if isinstance(hits, bool) or not isinstance(hits, int) or hits < 0:
        return dict(fallback, display_status="INVALID", snapshot_age_s=age,
                    reason="missing or invalid first-to-last collision message count")
    if status == "WAITING":
        hits = 0  # Pre-start floor contacts never become race-interval HITS.
    return {"display_status": {"WAITING": "READY", "RUNNING": "RUNNING", "FINISHED": "FINISH"}[status],
            "elapsed_sim_s": float(elapsed), "elapsed_wall_estimate_s": wall,
            "gates_passed": passed, "gates_total": total,
            "collision_messages_first_to_last": hits,
            "snapshot_age_s": age, "valid_snapshot": True, "reason": None}


def read_timer(path):
    try:
        raw = Path(path).read_bytes()
        if len(raw) > 2*1024*1024:
            raise ValueError("unbounded timer snapshot")
        return json.loads(raw), raw, None
    except (OSError, ValueError, UnicodeError) as error:
        return None, None, type(error).__name__+": "+str(error)


def _font(size):
    from PIL import ImageFont
    for name in ("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
                 "/System/Library/Fonts/Menlo.ttc"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


class Hud:
    def __init__(self, banner=None):
        self.big, self.small, self.medium = _font(30), _font(12), _font(17)
        # Optional disclosure line drawn under the timer box (e.g. modified physics).
        self.banner = banner

    def draw(self, rgb, width, height, view):
        from PIL import Image, ImageDraw
        if len(rgb) != width*height*3 or width < 300 or height < 150:
            raise ValueError("unexpected true-color frame dimensions for timer HUD")
        image = Image.frombytes("RGB", (width, height), rgb)
        draw = ImageDraw.Draw(image)
        x, y, w, h = width-294, 10, 284, 132
        color = ((255, 106, 92) if view["collision_messages_first_to_last"] else (64, 233, 156)) if view["valid_snapshot"] else (255, 190, 70)
        draw.rectangle((x, y, x+w, y+h), fill=(14, 21, 34), outline=color, width=2)
        total = view["gates_total"]
        title = "GATE 1 -> GATE {}".format(total) if total is not None else "GATE TIMER"
        draw.text((x+11, y+7), title, font=self.small, fill=(223, 230, 240))
        draw.text((x+11, y+25), view["display_status"], font=self.medium, fill=color)
        seconds = "--.---" if view["elapsed_sim_s"] is None else "{:.3f}".format(view["elapsed_sim_s"])
        draw.text((x+11, y+46), seconds+" s", font=self.big, fill=color)
        passed = view["gates_passed"]
        gates = "--/--" if passed is None else "{}/{}".format(passed, total)
        draw.text((x+11, y+86), "SIM TIME  |  GATES "+gates, font=self.small, fill=(223, 230, 240))
        wall = view["elapsed_wall_estimate_s"]
        text = "REAL --.---s" if wall is None else "REAL {:.3f}s".format(wall)
        hits = view["collision_messages_first_to_last"]
        text += " | HITS "+("--" if hits is None else str(hits))
        draw.text((x+11, y+106), text, font=self.small, fill=(181, 192, 209))
        bottom = y+h+1
        if self.banner:
            bh = 22
            draw.rectangle((x, y+h+4, x+w, y+h+4+bh), fill=(96, 34, 34), outline=(255, 150, 90), width=2)
            draw.text((x+11, y+h+8), self.banner, font=self.small, fill=(255, 220, 190))
            bottom = y+h+4+bh+1
        return image.tobytes(), [x, y, x+w+1, bottom]


class TimerAnnotation:
    def __init__(self, state_path, artifacts, stale_s=.5, banner=None):
        self.state_path, self.artifacts = Path(state_path).resolve(), Path(artifacts)
        self.stale_s = stale_s
        self.hud = Hud(banner)
        self.banner = banner
        self.frames, self.counts, self.last_state = 0, collections.Counter(), None
        self.last_raw = None
        self.stream = None

    def sample(self, rgb, width, height, read_start_ns, read_end_ns):
        state, raw, error = read_timer(self.state_path)
        observed_ns = time.monotonic_ns()
        view = timer_view(state, observed_ns/1e9, self.stale_s)
        annotated, rectangle = self.hud.draw(rgb, width, height, view)
        if self.stream is None:
            self.stream = (self.artifacts/"timer_annotations.jsonl").open("x", encoding="utf-8")
        row = {"frame": self.frames, "monotonic_ns": (read_start_ns+read_end_ns)//2,
               "annotation_observed_monotonic_ns": observed_ns, "view": view,
               "timer_read_error": error, "timer_snapshot": state,
               "timer_snapshot_sha256": hashlib.sha256(raw).hexdigest() if raw is not None else None,
               "source_window_rgb_sha256": hashlib.sha256(rgb).hexdigest(),
               "annotated_rgb_sha256": hashlib.sha256(annotated).hexdigest(),
               "annotation_rectangle_half_open_px": rectangle}
        self.stream.write(json.dumps(row, separators=(",", ":"))+"\n")
        self.stream.flush()
        self.frames += 1
        self.counts[view["display_status"]] += 1
        if raw is not None:
            self.last_state, self.last_raw = state, raw
        return annotated

    def report(self):
        snapshot = None
        if self.last_raw is not None:
            snapshot = self.artifacts/"timer_annotation_last_state.json"
            with snapshot.open("xb") as handle:
                handle.write(self.last_raw)
        return {"kind": "same-run live telemetry HUD on real Unity X11 samples",
                "timer_state_path": str(self.state_path), "time_basis": "simulation seconds; first gate to last gate",
                "pixel_edits": "Only the disclosed top-right timer rectangle; no motion reconstruction",
                "hud_banner": self.banner,
                "raw_ppm_frames_are_annotated": True, "underlying_simulator_rendering_changed": False,
                "frame_interpolation": False, "capture_timestamps_changed": False,
                "display_sim_time_extrapolated": False, "stale_threshold_s": self.stale_s,
                "hits_label_definition": "true collision messages between first and last gate, not distinct impacts; pre-start contacts excluded only from this interval label and remain in source logs",
                "sampled_frames": self.frames, "frame_status_counts": dict(self.counts),
                "annotation_log": str(self.artifacts/"timer_annotations.jsonl"),
                "last_timer_snapshot": str(snapshot) if snapshot is not None else None,
                "last_timer_snapshot_sha256": hashlib.sha256(self.last_raw).hexdigest() if self.last_raw is not None else None,
                "finish_displayed": self.counts["FINISH"] > 0,
                "note": "State is observed after the X11 read; callback/publishing delay remains. A final value is frozen by actual gate events, not video PTS or scheduled trajectory time."}

    def close(self):
        if self.stream is not None:
            self.stream.close()


def load_helper(path):
    spec = importlib.util.spec_from_file_location("lab4_original_capture_for_timer", str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main(argv=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--timer-state", type=Path, required=True)
    parser.add_argument("--timer-stale-s", type=float, default=.5)
    parser.add_argument("--hud-banner", default=None,
                        help="disclosure text drawn under the timer box, e.g. modified simulator physics")
    parser.add_argument("--base-helper", type=Path,
                        default=Path(__file__).resolve().parent.parent/"runtime/capture_x11_window.py")
    args, remaining = parser.parse_known_args(argv)
    if not finite(args.timer_stale_s) or not .1 <= args.timer_stale_s <= 2:
        parser.error("timer stale threshold must be 0.1..2 seconds")
    # Determine only the destination for the annotation sidecars; the unchanged
    # helper still validates all its own arguments and creates its own folder.
    destination = argparse.ArgumentParser(add_help=False)
    destination.add_argument("--output", type=Path, required=True)
    output_args, unused = destination.parse_known_args(remaining)
    artifacts = Path(str(output_args.output.expanduser().absolute())+".capture")
    helper = load_helper(args.base_helper)
    if args.hud_banner is not None and not (3 <= len(args.hud_banner) <= 40):
        raise ValueError("HUD banner must be 3..40 characters")
    annotation = TimerAnnotation(args.timer_state, artifacts, args.timer_stale_s, args.hud_banner)
    original_grab, original_write = helper.X11Window.grab, helper._write_json_new
    def grab(window, identifier, width, height):
        rgb, started, ended = original_grab(window, identifier, width, height)
        return annotation.sample(rgb, width, height, started, ended), started, ended
    def write(path, value):
        if Path(path).name == "capture_report.json":
            value["timer_annotation"] = annotation.report()
            value["capture_wrapper_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
            value["base_capture_helper_sha256"] = hashlib.sha256(args.base_helper.read_bytes()).hexdigest()
        return original_write(path, value)
    helper.X11Window.grab, helper._write_json_new = grab, write
    previous = sys.argv
    try:
        sys.argv = [str(args.base_helper)]+remaining
        return helper.main()
    finally:
        annotation.close()
        sys.argv = previous


if __name__ == "__main__":
    raise SystemExit(main())
