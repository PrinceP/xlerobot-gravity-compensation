#!/usr/bin/env python3
"""Send impedance goals to `gravcomp run --listen` and read the arm state back. Stdlib only.

    python tools/send_goal.py --arm left state                  # print tool pose / joints (no change)
    python tools/send_goal.py --arm left hold                   # soft spring at the present tool pose
    python tools/send_goal.py --arm left nudge 0 0 0.02         # goal 2 cm above the present pose
    python tools/send_goal.py --arm left pose 0.0 -0.25 0.20    # absolute goal, arm base frame (m)
    python tools/send_goal.py --arm left joints 0 0 0 0 0       # joint goal, LeRobot degrees
    python tools/send_goal.py --arm left circle --radius 0.03 --period 4 --duration 12
    python tools/send_goal.py --arm left off                    # back to gravity only

Pose goals are in the arm's base frame (URDF `Base` / `Base_2`); use `state` to see where the
tool is. Goals sent once use --ttl 0 (stay until replaced); streamed goals (circle) use a short
ttl, so the arm goes back to gravity-only if this script stops. A policy does the same thing:
send {"mode":"joints","q_deg":[...]} or {"mode":"pose","pos":[...],"quat":[w,x,y,z]} each step
with a short ttl, and read the state stream for observations.
"""
import argparse
import json
import math
import socket
import time
from pathlib import Path

PORTS = {"left": 5701, "right": 5702}


def default_host():
    """ORIN_HOST from the environment, else config.local.mk / config.mk next to the repo root."""
    import os
    import re
    if os.environ.get("ORIN_HOST"):
        return os.environ["ORIN_HOST"]
    root = Path(__file__).resolve().parent.parent
    for name in ("config.local.mk", "config.mk"):
        f = root / name
        if f.exists():
            m = re.search(r"^ORIN_HOST\s*\??=\s*(\S+)", f.read_text(), re.M)
            if m:
                return m.group(1)
    return "orin.local"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=default_host())
    ap.add_argument("--arm", default="left", choices=PORTS)
    ap.add_argument("--ttl", type=float, default=0.0, help="goal lifetime for one-shot goals (0 = until replaced)")
    ap.add_argument("--stiffness", type=float, default=1.0)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("state")
    sub.add_parser("hold")
    sub.add_parser("off")
    j = sub.add_parser("joints"); j.add_argument("q", type=float, nargs=5)
    p = sub.add_parser("pose"); p.add_argument("xyz", type=float, nargs=3); p.add_argument("--quat", type=float, nargs=4)
    n = sub.add_parser("nudge"); n.add_argument("dxyz", type=float, nargs=3)
    c = sub.add_parser("circle")
    c.add_argument("--radius", type=float, default=0.03)
    c.add_argument("--period", type=float, default=4.0)
    c.add_argument("--duration", type=float, default=12.0)
    c.add_argument("--plane", default="yz", choices=["xy", "yz", "xz"])
    a = ap.parse_args()

    addr = (a.host, PORTS[a.arm])
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(1.0)

    def send(msg):
        msg.setdefault("stiffness", a.stiffness)
        s.sendto(json.dumps(msg).encode(), addr)

    def state():
        send({"mode": "ping"})
        deadline = time.time() + 1.0
        while time.time() < deadline:
            try:
                return json.loads(s.recv(4096))
            except socket.timeout:
                break
        raise SystemExit(f"no state from {addr}: is `gravcomp run --listen` running for the {a.arm} arm?")

    if a.cmd == "state":
        st = state()
        print(json.dumps(st, indent=1))
    elif a.cmd in ("hold", "off"):
        send({"mode": a.cmd, "ttl": a.ttl})
    elif a.cmd == "joints":
        send({"mode": "joints", "q_deg": a.q, "ttl": a.ttl})
    elif a.cmd == "pose":
        m = {"mode": "pose", "pos": a.xyz, "ttl": a.ttl}
        if a.quat:
            m["quat"] = a.quat
        send(m)
    elif a.cmd == "nudge":
        st = state()
        pos = [st["pos"][i] + a.dxyz[i] for i in range(3)]
        send({"mode": "pose", "pos": pos, "quat": st["quat"], "ttl": a.ttl})
        print("goal", [round(v, 4) for v in pos])
    elif a.cmd == "circle":
        st = state()
        c0, quat = st["pos"], st["quat"]
        i, k = {"xy": (0, 1), "yz": (1, 2), "xz": (0, 2)}[a.plane]
        t0 = time.time()
        while (t := time.time() - t0) < a.duration:
            ph = 2 * math.pi * t / a.period
            pos = list(c0)
            pos[i] += a.radius * math.sin(ph)
            pos[k] += a.radius * (1 - math.cos(ph))  # starts at the present pose
            send({"mode": "pose", "pos": pos, "quat": quat, "ttl": 0.3})
            time.sleep(0.02)
        send({"mode": "pose", "pos": c0, "quat": quat, "ttl": a.ttl})
    if a.cmd != "state":
        time.sleep(0.1)
        try:
            st = state()
            print(f"{a.arm}: mode {st['mode']}, tool {[round(v, 3) for v in st['pos']]}, err {st['err_m'] * 1e3:.1f} mm")
        except SystemExit as e:
            print(e)


if __name__ == "__main__":
    main()
