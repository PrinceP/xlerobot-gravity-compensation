#!/usr/bin/env python3
"""Record the head + both wrist cameras into one demo video (runs on the Orin).

    python tools/record_cameras.py --out demo/run1            # Ctrl+C to stop
    python tools/record_cameras.py --out demo/run1 --raw      # also keep one mp4 per camera
    python tools/record_cameras.py --out demo/board --overlay-board --title "..."   # + live board tilt

Layout (1440x540): head camera large on the left, left/right wrist cameras stacked on the
right, with a label and wall-clock time burned in. A frame timestamp CSV is written next
to the video (unix time per frame), so frames line up with `gravcomp run --log` CSVs.

Uses only the cameras, never the motor buses, so it runs next to `gravcomp run`.
"""
import argparse
import csv
import signal
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import importlib.util

def _load_level_board():
    """tools/level_board.py, loaded by path (it sits next to this file)."""
    spec = importlib.util.spec_from_file_location("level_board", Path(__file__).with_name("level_board.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

# Stable USB paths on this Orin (from robotics-antigravity-plugin xlerobot_real_plan.py).
CAMS = {
    "head": "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0",
    "left_wrist": "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4:1.0-video-index0",
    "right_wrist": "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.2:1.0-video-index0",
}


class Cam(threading.Thread):
    def __init__(self, name, dev, w, h):
        super().__init__(daemon=True)
        self.name, self.dev, self.w, self.h = name, dev, w, h
        self.frame = None
        self.running = True

    def run(self):
        while self.running:
            cap = cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
            if not cap.isOpened():
                print(f"[{self.name}] cannot open {self.dev}, retrying", flush=True)
                time.sleep(2)
                continue
            print(f"[{self.name}] open {self.dev}", flush=True)
            fails = 0
            while self.running and fails < 30:
                ok, f = cap.read()
                if ok:
                    self.frame, fails = f, 0
                else:
                    fails += 1
                    time.sleep(0.03)
            cap.release()


class BoardOverlay(threading.Thread):
    """Polls both arms' impedance state (UDP, this machine) and keeps the latest board tilt."""

    def __init__(self, host):
        super().__init__(daemon=True)
        self.lb = _load_level_board()
        self.arms = self.lb.Arms(host)
        for s in self.arms.sock.values():
            s.settimeout(0.3)
        self.m = self.lb.mounts()
        self.latest = None
        self.running = True

    def run(self):
        while self.running:
            try:
                self.latest = self.lb.measure(self.arms, self.m)
            except (SystemExit, OSError, ValueError, KeyError):
                self.latest = None
            time.sleep(0.1)


def draw_board(frame, r):
    """Board tilt and per-arm error, top-right of the head view."""
    if r is None:
        lines = [("board: waiting for gravcomp state", (80, 80, 255))]
    else:
        tilt = max(abs(r["side_deg"]), r["left"]["normal_deg"], r["right"]["normal_deg"])
        col = (80, 220, 80) if tilt < 1.0 else (0, 165, 255) if tilt < 3.0 else (80, 80, 255)
        lines = [(f"board tilt  side {r['side_deg']:+.1f} deg   normals L {r['left']['normal_deg']:.1f}  R {r['right']['normal_deg']:.1f} deg", col)]
        for a in ("left", "right"):
            x = r[a]
            state = "returning" if x["assist"] > 0.5 else "compliant"
            lines.append((f"{a:5s} err {x['err_mm']:5.1f} mm   {state}", (255, 255, 255)))
    y = 44
    for text, col in lines:
        cv2.rectangle(frame, (8, y - 18), (8 + len(text) * 10 + 8, y + 8), (0, 0, 0), -1)
        cv2.putText(frame, text, (14, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1, cv2.LINE_AA)
        y += 30


def fit(img, w, h, label):
    if img is None:
        img = np.zeros((h, w, 3), np.uint8)
        cv2.putText(img, "no signal", (w // 2 - 70, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (80, 80, 255), 2)
    else:
        img = cv2.resize(img, (w, h))
    cv2.rectangle(img, (0, 0), (len(label) * 13 + 16, 30), (0, 0, 0), -1)
    cv2.putText(img, label, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=f"demo/{datetime.now():%Y%m%d-%H%M%S}")
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--title", default="XLeRobot - gravity compensation")
    ap.add_argument("--raw", action="store_true", help="also write one mp4 per camera")
    ap.add_argument("--overlay-board", action="store_true", help="burn in board tilt from `gravcomp run --listen` (both arms)")
    ap.add_argument("--state-host", default="127.0.0.1")
    ap.add_argument("--width", type=int, default=640, help="head camera")
    ap.add_argument("--height", type=int, default=480)
    # The three cameras are USB 2.0 and share one 480 Mbit/s link (the Orin Nano dev kit's
    # Type-A ports sit behind one hub). At 640x480 each, the third camera gets no frames;
    # head 640x480 + wrists 320x240 streams all three (tested 2026-11-25).
    ap.add_argument("--wrist-width", type=int, default=320)
    ap.add_argument("--wrist-height", type=int, default=240)
    for k, v in CAMS.items():
        ap.add_argument(f"--{k.replace('_', '-')}", default=v)
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    devs = {"head": args.head, "left_wrist": args.left_wrist, "right_wrist": args.right_wrist}
    size = {k: (args.width, args.height) if k == "head" else (args.wrist_width, args.wrist_height) for k in devs}
    cams = {k: Cam(k, d, *size[k]) for k, d in devs.items()}
    for c in cams.values():
        c.start()

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    tiled = cv2.VideoWriter(str(out / "demo.mp4"), fourcc, args.fps, (1440, 540))
    raw = {k: cv2.VideoWriter(str(out / f"{k}.mp4"), fourcc, args.fps, size[k]) for k in cams} if args.raw else {}
    ts = open(out / "frames.csv", "w", newline="")
    tw = csv.writer(ts)
    overlay = None
    if args.overlay_board:
        overlay = BoardOverlay(args.state_host)
        overlay.start()
    tw.writerow(["frame", "t_unix", "side_tilt_deg", "left_normal_deg", "right_normal_deg", "left_err_mm", "right_err_mm"])

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    time.sleep(1.0)  # let cameras open
    print(f"recording to {out}/ (Ctrl+C to stop)", flush=True)

    period = 1.0 / args.fps
    nxt = time.time()
    n = 0
    while not stop.is_set():
        now = time.time()
        head = fit(cams["head"].frame, 960, 540, "head")
        lw = fit(cams["left_wrist"].frame, 480, 270, "left wrist")
        rw = fit(cams["right_wrist"].frame, 480, 270, "right wrist")
        frame = np.hstack([head, np.vstack([lw, rw])])
        r = overlay.latest if overlay else None
        if overlay:
            draw_board(frame, r)
        stamp = datetime.fromtimestamp(now).strftime("%H:%M:%S.%f")[:-3]
        cv2.putText(frame, f"{args.title}   {stamp}", (12, 530), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        tiled.write(frame)
        for k, w in raw.items():
            f = cams[k].frame
            if f is not None:
                w.write(cv2.resize(f, size[k]))
        if r:
            tw.writerow([n, f"{now:.4f}", f"{r['side_deg']:.3f}", f"{r['left']['normal_deg']:.3f}", f"{r['right']['normal_deg']:.3f}",
                         f"{r['left']['err_mm']:.2f}", f"{r['right']['err_mm']:.2f}"])
        else:
            tw.writerow([n, f"{now:.4f}", "", "", "", "", ""])
        n += 1
        nxt += period
        time.sleep(max(0.0, nxt - time.time()))

    if overlay:
        overlay.running = False
    for c in cams.values():
        c.running = False
    for c in cams.values():
        c.join(timeout=3)
    tiled.release()
    for w in raw.values():
        w.release()
    ts.close()
    print(f"wrote {n} frames ({n / args.fps:.1f} s) to {out}/demo.mp4", flush=True)


if __name__ == "__main__":
    main()
