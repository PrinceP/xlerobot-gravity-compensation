#!/usr/bin/env python3
"""Cross-check gravcomp's Rust gravity model, tool pose and Jacobian against Pinocchio (the solver NVIDIA's tutorial uses).

    pip install pin numpy
    python tools/check_gravity_pinocchio.py [--bin target/release/gravcomp] [--n 200]

Loads the same URDF in Pinocchio, locks every joint except one arm's six, and compares
computeGeneralizedGravity with `gravcomp gravity` at random poses. The wrist-camera extra
mass in the config is added to Pinocchio the same way, so the numbers must agree to ~1e-9.
"""
import argparse
import json
import subprocess
import sys
import tomllib
from pathlib import Path

import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parent.parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default=str(ROOT / "target/release/gravcomp"))
    ap.add_argument("--config", default=str(ROOT / "config/xlerobot.toml"))
    ap.add_argument("--n", type=int, default=200)
    args = ap.parse_args()
    cfg = tomllib.loads(Path(args.config).read_text())

    full = pin.buildModelFromUrdf(str(ROOT / cfg["urdf"]))
    worst = 0.0
    worst_kin = 0.0
    for arm in ("left", "right"):
        a = cfg["arms"][arm]
        names = [j["urdf"] + a["urdf_suffix"] for j in cfg["joints"]] + [cfg["gripper"]["urdf"] + a["urdf_suffix"]]
        lock = [full.getJointId(n) for n in full.names[1:] if n not in names]
        model = pin.buildReducedModel(full, lock, pin.neutral(full))
        for e in cfg.get("extra_mass", []):
            fid = model.getFrameId(e["link"] + a["urdf_suffix"])
            f = model.frames[fid]
            extra = pin.Inertia.FromSphere(e["mass_kg"], 1e-4)
            extra = pin.SE3(np.eye(3), np.array(e["xyz"])).act(extra)
            model.inertias[f.parentJoint] += f.placement.act(extra)
        data = model.createData()
        idx = [model.idx_qs[model.getJointId(n)] for n in names]
        rng = np.random.default_rng(0)
        for _ in range(args.n):
            q_arm = rng.uniform(-1.8, 1.8, size=len(names))
            q = pin.neutral(model)
            q[idx] = q_arm
            g_pin = pin.computeGeneralizedGravity(model, data, q)
            g_pin = np.array([g_pin[model.idx_vs[model.getJointId(n)]] for n in names])
            out = subprocess.run(
                [args.bin, "--config", args.config, "gravity", "--arm", arm, "--q", ",".join(f"{x:.17g}" for x in q_arm)],
                check=True, capture_output=True, text=True).stdout
            g_rs = np.array(json.loads(out)["tau"])
            worst = max(worst, float(np.max(np.abs(g_rs - g_pin))))
            # Tool pose and Jacobian (impedance control).
            link = cfg["impedance"]["ee_link"] + a["urdf_suffix"]
            fid = model.getFrameId(link)
            pin.computeJointJacobians(model, data, q)
            pin.updateFramePlacements(model, data)
            J = pin.getFrameJacobian(model, data, fid, pin.LOCAL_WORLD_ALIGNED)
            J = np.stack([J[:, model.idx_vs[model.getJointId(n)]] for n in names], axis=1)
            k = json.loads(subprocess.run(
                [args.bin, "--config", args.config, "kin", "--arm", arm, "--q", ",".join(f"{x:.17g}" for x in q_arm)],
                check=True, capture_output=True, text=True).stdout)
            worst_kin = max(worst_kin, float(np.max(np.abs(np.array(k["jac"]) - J))),
                            float(np.max(np.abs(np.array(k["pos"]) - data.oMf[fid].translation))))
        print(f"{arm}: {args.n} poses, max |tau_rust - tau_pinocchio| = {worst:.3e} N·m, "
              f"max |tool pos / Jacobian diff| = {worst_kin:.3e}")
    ok = worst < 1e-6 and worst_kin < 1e-6
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
