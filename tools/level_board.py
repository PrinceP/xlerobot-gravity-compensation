#!/usr/bin/env python3
"""Two-arm board hold: report how level the board is, and level it. Stdlib only.

    python tools/level_board.py               # tilt report, once
    python tools/level_board.py --watch       # tilt report at 5 Hz (Ctrl+C to stop)
    python tools/level_board.py --apply       # send levelled pose goals to both arms
    python tools/level_board.py --wait 300 --apply --watch   # demo: wait for the hold, level, keep reporting

Needs `gravcomp run --arm both --listen --hold` (see `make board`). Each gripper clamps the
board edge with its jaws closing vertically, so the board lies in the gripper's forward
(local -y) / sideways (local z) plane, and its normal is the gripper's local +x axis.

Level means: both board normals point straight down, and both gripper tips are at the same
height. --apply keeps each gripper's x/y position and yaw, sets both tips to their mean
height, and rotates each gripper by the smallest rotation that makes its normal vertical.
The arms move there at the controller's goal rate limits (0.15 m/s, 1 rad/s).
"""
import argparse
import json
import math
import socket
import time
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PORTS = {"left": 5701, "right": 5702}
MOUNT_JOINT = {"left": "arm_base_joint_2", "right": "arm_base_joint"}


def mounts():
    """World position and yaw of each arm's base frame, from the URDF."""
    out = {}
    root = ET.parse(ROOT / "urdf/xlerobot.urdf").getroot()
    for arm, jn in MOUNT_JOINT.items():
        o = root.find(f"./joint[@name='{jn}']/origin")
        xyz = [float(v) for v in o.get("xyz").split()]
        rpy = [float(v) for v in o.get("rpy").split()]
        assert abs(rpy[0]) < 1e-6 and abs(rpy[1]) < 1e-6, "mount must be level (yaw only)"
        out[arm] = (xyz, rpy[2])
    return out


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [aw*bw - ax*bx - ay*by - az*bz, aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx, aw*bz + ax*by - ay*bx + az*bw]


def rot(q, v):
    w, x, y, z = q
    R = [[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
         [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
         [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]]
    return [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]


def from_to(a, b):
    """Smallest rotation (quaternion) taking unit vector a to unit vector b."""
    c = [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
    d = sum(a[i]*b[i] for i in range(3))
    q = [1 + d, *c]
    n = math.sqrt(sum(v*v for v in q))
    return [v / n for v in q]


def to_world(arm, p, m):
    (ox, oy, oz), yaw = m[arm]
    c, s = math.cos(yaw), math.sin(yaw)
    return [ox + c*p[0] - s*p[1], oy + s*p[0] + c*p[1], oz + p[2]]


class Arms:
    def __init__(self, host):
        self.host = host
        self.sock = {a: socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for a in PORTS}
        for s in self.sock.values():
            s.settimeout(1.0)

    def state(self, arm):
        s = self.sock[arm]
        s.sendto(b'{"mode":"ping"}', (self.host, PORTS[arm]))
        try:
            while True:
                st = json.loads(s.recv(4096))
                if st.get("arm") == arm:
                    return st
        except socket.timeout:
            raise SystemExit(f"no state from the {arm} arm: is `gravcomp run --arm both --listen` running?")

    def send(self, arm, msg):
        self.sock[arm].sendto(json.dumps(msg).encode(), (self.host, PORTS[arm]))


def measure(arms, m):
    """Board tilt from both arms' states: dict of numbers (degrees, mm) plus the raw states."""
    st = {a: arms.state(a) for a in PORTS}
    w = {a: to_world(a, st[a]["pos"], m) for a in PORTS}
    dy = w["left"][1] - w["right"][1]
    dz = w["left"][2] - w["right"][2]
    out = {"side_deg": math.degrees(math.atan2(dz, abs(dy))), "dz_mm": dz * 1e3,
           "spacing_mm": math.dist(w["left"], w["right"]) * 1e3, "st": st, "w": w}
    for a in PORTS:
        n = rot(st[a]["quat"], [1, 0, 0])            # board normal (gripper local +x), base frame
        f = rot(st[a]["quat"], [0, -1, 0])           # gripper forward
        out[a] = {"normal_deg": math.degrees(math.acos(max(-1, min(1, -n[2])))),
                  "pitch_deg": math.degrees(math.asin(max(-1, min(1, f[2])))),
                  "err_mm": st[a]["err_m"] * 1e3, "assist": st[a].get("assist", float("nan")),
                  "mode": st[a]["mode"]}
    return out


def report(arms, m):
    r = measure(arms, m)
    parts = [f"side-to-side tilt {r['side_deg']:+5.2f} deg (left tip {r['dz_mm']:+5.1f} mm vs right), tip spacing {r['spacing_mm']:5.1f} mm"]
    for a in PORTS:
        x = r[a]
        parts.append(f"{a}: normal {x['normal_deg']:4.1f} deg off vertical (fwd pitch {x['pitch_deg']:+5.1f}), "
                     f"err {x['err_mm']:4.1f} mm, assist {x['assist']:.2f}, mode {x['mode']}")
    print(" | ".join(parts))
    return r["st"], r["w"]


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
    ap.add_argument("--watch", action="store_true")
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--wait", type=float, default=0.0, help="seconds to keep retrying until both arms answer")
    a = ap.parse_args()
    m = mounts()
    arms = Arms(a.host)
    deadline = time.time() + a.wait
    while True:
        try:
            st, w = report(arms, m)
            break
        except SystemExit:
            if time.time() > deadline:
                raise
            time.sleep(1.0)
    if a.apply and a.wait:
        time.sleep(2.0)  # let the hold settle after its fade-in before levelling
    if a.apply:
        z_mean = (w["left"][2] + w["right"][2]) / 2
        for arm in PORTS:
            p = list(st[arm]["pos"])
            p[2] = z_mean - m[arm][0][2]          # base z = world z - mount z
            q = st[arm]["quat"]
            n = rot(q, [1, 0, 0])
            q_new = qmul(from_to(n, [0, 0, -1]), q)
            arms.send(arm, {"mode": "pose", "pos": p, "quat": q_new, "ttl": 0})
        print("levelling goals sent; settling...")
        for _ in range(6):
            time.sleep(0.5)
            report(arms, m)
    if a.watch:
        try:
            while True:
                time.sleep(0.2)
                report(arms, m)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
