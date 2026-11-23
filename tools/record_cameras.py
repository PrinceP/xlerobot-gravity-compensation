#!/usr/bin/env python3
"""Record the head + both wrist cameras into one demo video (runs on the Orin).

    python tools/record_cameras.py --out demo/run1            # Ctrl+C to stop
    python tools/record_cameras.py --out demo/run1 --raw      # also keep one mp4 per camera

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
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    for k, v in CAMS.items():
        ap.add_argument(f"--{k.replace('_', '-')}", default=v)
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    devs = {"head": args.head, "left_wrist": args.left_wrist, "right_wrist": args.right_wrist}
    cams = {k: Cam(k, d, args.width, args.height) for k, d in devs.items()}
    for c in cams.values():
        c.start()

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    tiled = cv2.VideoWriter(str(out / "demo.mp4"), fourcc, args.fps, (1440, 540))
    raw = {k: cv2.VideoWriter(str(out / f"{k}.mp4"), fourcc, args.fps, (args.width, args.height)) for k in cams} if args.raw else {}
    ts = open(out / "frames.csv", "w", newline="")
    tw = csv.writer(ts)
    tw.writerow(["frame", "t_unix"])

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
        stamp = datetime.fromtimestamp(now).strftime("%H:%M:%S.%f")[:-3]
        cv2.putText(frame, f"{args.title}   {stamp}", (12, 530), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        tiled.write(frame)
        for k, w in raw.items():
            f = cams[k].frame
            if f is not None:
                w.write(cv2.resize(f, (args.width, args.height)))
        tw.writerow([n, f"{now:.4f}"])
        n += 1
        nxt += period
        time.sleep(max(0.0, nxt - time.time()))

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
