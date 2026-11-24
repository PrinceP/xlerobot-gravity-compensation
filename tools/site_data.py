#!/usr/bin/env python3
"""Build the GitHub Pages data and video from a board-demo recording. Stdlib + ffmpeg.

    python tools/site_data.py demo/20261124-194403-board

Writes, under docs/:
  data/board-demo.json   charts: board tilt, per-arm error, shoulder PWM split (10 Hz,
                         time = seconds into the video, so a chart click can seek it)
  media/board-demo.mp4   the tiled 3-camera video re-encoded to H.264 (browsers don't
                         play OpenCV's MPEG-4 Part 2), plus a poster frame
"""
import csv
import json
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
    src = Path(sys.argv[1]).resolve()
    cfg = tomllib.loads((ROOT / "config/xlerobot.toml").read_text())
    frames = rows(src / "frames.csv")
    t0 = float(frames[0]["t_unix"])  # video time 0

    tilt = {"t": [], "side": [], "left": [], "right": []}
    for x in frames[::2]:  # 20 fps -> 10 Hz
        if not x["side_tilt_deg"]:
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
            if not x["imp_mode"]:
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
    summary = {
        "video_s": num(float(frames[-1]["t_unix"]) - t0, 1),
        "hold_start_s": tilt["t"][0],
        "within_1deg_pct": round(100 * sum(w < 1 for w in worst) / len(worst)),
        "tilt_median_deg": num(sorted(worst)[len(worst) // 2], 2),
        "side_median_deg": num(sorted(tilt["side"])[len(tilt["side"]) // 2], 2),
        "err_median_mm": {a: num(sorted(arms[a]["err"])[len(arms[a]["err"]) // 2], 1) for a in arms},
        "err_max_mm": {a: max(arms[a]["err"]) for a in arms},
        "recording": src.name,
    }
    out = {"summary": summary, "tilt": tilt, "arms": arms}
    (DOCS / "data").mkdir(parents=True, exist_ok=True)
    (DOCS / "data/board-demo.json").write_text(json.dumps(out, separators=(",", ":")))
    print("data:", DOCS / "data/board-demo.json", json.dumps(summary))

    media = DOCS / "media"
    media.mkdir(parents=True, exist_ok=True)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(src / "demo.mp4"), "-c:v", "libx264",
                    "-preset", "slow", "-crf", "27", "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-an",
                    str(media / "board-demo.mp4")], check=True)
    # Poster: a frame from the middle of the pushes.
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", "45", "-i", str(media / "board-demo.mp4"),
                    "-frames:v", "1", "-q:v", "4", str(media / "board-demo-poster.jpg")], check=True)
    print("video:", media / "board-demo.mp4")


if __name__ == "__main__":
    main()
