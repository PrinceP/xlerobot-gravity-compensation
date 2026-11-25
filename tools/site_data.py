#!/usr/bin/env python3
"""Build the GitHub Pages data and video from a board-demo recording. Stdlib + ffmpeg.

    python tools/site_data.py demo/<stamp>-board [--trim S] [--poster S]

Writes, under docs/:
  data/board-demo.json   charts: board tilt, per-arm error, shoulder PWM split (10 Hz,
                         time = seconds into the video, so a chart click can seek it)
  media/board-demo.mp4   the tiled 3-camera video re-encoded to H.264 (browsers don't
                         play OpenCV's MPEG-4 Part 2), plus a poster frame
"""
import csv
import json
import re
import subprocess
import sys
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOCS = ROOT / "docs"


def rows(p):
    return list(csv.DictReader(open(p)))


def num(x, nd=3):
    try:
        return round(float(x), nd)
    except (TypeError, ValueError):
        return None


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("recording")
    ap.add_argument("--poster", type=float, help="poster frame, seconds into the (trimmed) video")
    ap.add_argument("--trim", type=float, default=0.0, help="cut this many seconds off the start of the video")
    args = ap.parse_args()
    src = Path(args.recording).resolve()
    poster_s, trim = args.poster, args.trim
    # Which camera files actually have frames (an empty one is a few hundred bytes).
    cameras = {c: (src / f"{c}.mp4").exists() and (src / f"{c}.mp4").stat().st_size > 100_000
               for c in ("head", "left_wrist", "right_wrist")}
    cfg = tomllib.loads((ROOT / "config/xlerobot.toml").read_text())
    frames = rows(src / "frames.csv")
    t0 = float(frames[0]["t_unix"]) + trim  # time 0 of the published (trimmed) video

    tilt = {"t": [], "side": [], "left": [], "right": []}
    for x in frames[::2]:  # 20 fps -> 10 Hz
        if not x["side_tilt_deg"] or float(x["t_unix"]) < t0:
            continue
        tilt["t"].append(num(float(x["t_unix"]) - t0, 2))
        tilt["side"].append(num(abs(float(x["side_tilt_deg"])), 2))
        tilt["left"].append(num(x["left_normal_deg"], 2))
        tilt["right"].append(num(x["right_normal_deg"], 2))

    arms = {}
    for arm in ("left", "right"):
        r = rows(src / f"{arm}.csv")
        ov = cfg["arms"][arm].get("overrides", {}).get("shoulder_lift", {})
        sl = next(j for j in cfg["joints"] if j["name"] == "shoulder_lift")
        k = ov.get("pwm_per_nm", sl["pwm_per_nm"])
        s = sl["pwm_sign"]
        a = {"t": [], "err": [], "rot": [], "pwm": [], "grav": []}
        for x in r[::25]:  # 250 Hz -> 10 Hz
            if not x["imp_mode"] or float(x["t_unix"]) < t0:
                continue
            a["t"].append(num(float(x["t_unix"]) - t0, 2))
            a["err"].append(num(x["pos_err_mm"], 1))
            a["rot"].append(num(x["rot_err_deg"], 2))
            # PWM sent to shoulder_lift vs. the gravity feedforward part of it.
            a["pwm"].append(int(x["shoulder_lift_pwm"]))
            a["grav"].append(round(s * k * float(x["shoulder_lift_tau"])))
        cyc = sorted(float(x["cycle_ms"]) for x in r)
        a["cycle_ms"] = {"p50": num(cyc[len(cyc) // 2], 2), "p99": num(cyc[int(0.99 * len(cyc))], 2), "max": num(cyc[-1], 2)}
        a["samples"] = len(r)
        arms[arm] = a

    worst = [max(a, b, c) for a, b, c in zip(tilt["side"], tilt["left"], tilt["right"])]
    # Hand interaction window: first/last moment either arm is pushed (> 9 mm) or the board
    # is tilted (> 2 deg), after the hold has settled for 3 s.
    t_settle = tilt["t"][0] + 3
    marks = [t for t, w in zip(tilt["t"], worst) if t > t_settle and w > 2.0]
    for a in arms.values():
        marks += [t for t, e in zip(a["t"], a["err"]) if t > t_settle and e is not None and e > 9.0]
    win = [round(min(marks)), round(max(marks)) + 1] if marks else None
    after = {}
    for name, a in arms.items():
        tail = [e for t, e in zip(a["t"], a["err"]) if win and t > win[1] + 3 and e is not None]
        after[name] = num(sorted(tail)[len(tail) // 2], 1) if tail else None
    tail_tilt = [w for t, w in zip(tilt["t"], worst) if win and t > win[1] + 3]
    summary = {
        "video_s": num(float(frames[-1]["t_unix"]) - t0, 1),
        "trimmed_s": trim,
        "hold_start_s": tilt["t"][0],
        "within_1deg_pct": round(100 * sum(w < 1 for w in worst) / len(worst)),
        "tilt_median_deg": num(sorted(worst)[len(worst) // 2], 2),
        "side_median_deg": num(sorted(tilt["side"])[len(tilt["side"]) // 2], 2),
        "err_median_mm": {a: num(sorted(arms[a]["err"])[len(arms[a]["err"]) // 2], 1) for a in arms},
        "err_max_mm": {a: max(arms[a]["err"]) for a in arms},
        "recording": src.name,
        "interaction_s": win,
        "after_err_mm": after,
        "after_tilt_deg": num(sorted(tail_tilt)[len(tail_tilt) // 2], 2) if tail_tilt else None,
        "cameras": cameras,
    }
    out = {"summary": summary, "tilt": tilt, "arms": arms}
    (DOCS / "data").mkdir(parents=True, exist_ok=True)
    (DOCS / "data/board-demo.json").write_text(json.dumps(out, separators=(",", ":")))
    print("data:", DOCS / "data/board-demo.json", json.dumps(summary))

    media = DOCS / "media"
    media.mkdir(parents=True, exist_ok=True)
    # File names carry the recording stamp: GitHub Pages lets browsers cache media for 10 min,
    # and a re-recorded demo under the same name was served stale (the old "no signal" video).
    video = f"board-demo-{src.name}.mp4"
    poster = f"board-demo-{src.name}-poster.jpg"
    for old in media.glob("board-demo*"):
        old.unlink()
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", str(trim), "-i", str(src / "demo.mp4"), "-c:v", "libx264",
                    "-preset", "slow", "-crf", "27", "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-an",
                    str(media / video)], check=True)
    # Poster: a frame from the hold (default: middle of the hand interaction).
    if poster_s is None:
        poster_s = (win[0] + win[1]) / 2 if win else summary["video_s"] / 2
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", str(poster_s), "-i", str(media / video),
                    "-frames:v", "1", "-q:v", "4", str(media / poster)], check=True)
    # Point the page at the new names.
    page = DOCS / "index.html"
    html = page.read_text()
    html = re.sub(r'media/board-demo[^"]*\.mp4', f"media/{video}", html)
    html = re.sub(r'media/board-demo[^"]*-poster\.jpg', f"media/{poster}", html)
    html = re.sub(r'data/board-demo\.json(\?v=[^"]*)?', f"data/board-demo.json?v={src.name}", html)
    page.write_text(html)
    print("video:", media / video)


if __name__ == "__main__":
    main()
