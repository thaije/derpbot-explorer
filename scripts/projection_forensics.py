#!/usr/bin/env python3.12
"""Projection forensics (zero-sim) — t_fd80a2f9 Phase 1.

For every sighting in every recorded stream: recompute the bbox-center→world
projection analytically from (bbox, depth, robot pose) with the SAME camera
model as agent/depth_projector.py, decompose the sighting→GT error into

  e_perp  — perpendicular distance of GT from the bbox-center bearing ray
            (bearing/intrinsics error: grows with range if fx/FOV is wrong)
  e_along — range error along the ray (depth error: depth sampled behind /
            in front of the object — bbox-center-patch depth overshoot)

and correlate with range, elapsed sim-time (odom/SLAM drift test per operator
addendum), bbox aspect/size, and robot-object bearing. Also test the
bbox-center-patch hypothesis directly: 5×5 patch at the bbox centre reads the
background depth when the object does not fill the patch (small objects).

Frames (verified t_ea3a57f5): sightings x,y and robot pose are MAP frame
(map==odom==spawn frame); GT is WORLD frame → map = R(-spawn_yaw)(world-spawn).

Outputs JSON (results/projection_forensics.json) for Phase 2/3 reuse and the
markdown report. No sim, no ROS needed — pure math on recorded streams.
"""
from __future__ import annotations

import glob
import json
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STREAMS = os.path.join(REPO, "results", "streams")
SUBMISSIONS = os.path.expanduser(
    "~/Projects/robot-sandbox/results/submissions")

# camera model — MIRRORS agent/depth_projector.py V1 FALLBACK (640×480, 90°).
# streams (t_4aa1e7ff): each stream's header may carry a per-run camera
# object {"fx", "fy", "cx", "cy", "width", "height", "source"}; when present
# it overrides the constants below (v2.0.0: 1920×1080, fx≈960). Streams
# without a "camera" header key are pre-v2 and replay on these v1 constants,
# keeping the whole v1 corpus valid.
H_FOV_DEG = 90.0
W_PX, H_PX = 640, 480
FX = FY = (W_PX / 2) / math.tan(math.radians(H_FOV_DEG / 2))  # ≈320
CX, CY = W_PX / 2, H_PX / 2
V1_CAMERA = {"fx": FX, "fy": FY, "cx": CX, "cy": CY,
             "width": W_PX, "height": H_PX, "source": "v1_no_header"}
SAMPLE_HALF = 2            # 5×5 patch, median (as live code)
DEPTH_MIN, DEPTH_MAX = 0.30, 5.00
CAM_Z = 0.20               # camera_joint z above base_link/base_footprint

ATTR_RANGE = 3.0           # bootstrap: nearest same-class GT ≤3 m
MATCH_WINDOW = 1.5         # scorer TP window


def ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def world_to_map(wx, wy, spawn):
    sx, sy = float(spawn.get("x", 0.0)), float(spawn.get("y", 0.0))
    yaw = float(spawn.get("yaw", 0.0))
    dx, dy = wx - sx, wy - sy
    c, s = math.cos(-yaw), math.sin(-yaw)
    return c * dx - s * dy, s * dx + c * dy


def load_stream(path):
    header, out = {}, []
    with open(path) as fh:
        for raw in fh:
            raw = raw.strip()
            if not raw:
                continue
            rec = json.loads(raw)
            if rec.get("type") == "header":
                header = rec
            elif rec.get("type") == "sighting":
                out.append(rec)
    return header, out


def inventory():
    runs = []
    for f in sorted(glob.glob(os.path.join(STREAMS, "*.jsonl"))):
        base = os.path.basename(f)[len("stream_"):-len(".jsonl")]
        base = base.split("_2026")[0].strip("_")
        is_v6 = base.startswith("easy_")
        agname = ("derpbot-explorer-v6-fastconfirm" if is_v6
                  else "derpbot-explorer-stream-corpus")
        rest = base[len("easy_"):] if is_v6 else base
        cand = os.path.join(SUBMISSIONS, agname, f"easy_{rest}.json")
        if os.path.exists(cand):
            runs.append({"stream": f, "gt": cand, "agent": agname,
                         "key": rest, "batch": is_v6})
    return runs


def stream_camera(header: dict) -> dict:
    """Per-run camera model from the stream header (v2, t_4aa1e7ff).

    Returns the header "camera" object when present, else the v1 640×480
    constants — pre-v2 streams replay byte-identical to their live run.
    Missing keys are synthesised from the header's OWN width/height
    (cx=width/2, cy=height/2, fy=fx), never from the v1 constants — a v2
    header must not silently inherit v1 geometry.
    """
    cam = header.get("camera")
    if isinstance(cam, dict) and cam.get("fx") and cam.get("width"):
        fx = float(cam["fx"])
        w = int(cam["width"])
        h = int(cam["height"])
        return {
            "fx": fx,
            "fy": float(cam.get("fy", fx)),
            "cx": float(cam.get("cx", w / 2.0)),
            "cy": float(cam.get("cy", h / 2.0)),
            "width": w,
            "height": h,
            "source": cam.get("source", "header"),
        }
    return V1_CAMERA


def replay_projection(s, header, cam=None):
    """Analytic re-derivation of depth_projector.project() from the stream row.

    Returns (u, v, depth_used, px, py, bearing_ray_map) — all map frame.
    NOTE: the recorded per-sighting x,y IS this projection (depth from the live
    5×5 patch); we recompute the full ray geometry instead of only its 2D end.
    `cam` = per-run camera model (stream_camera(header)); defaults to v1.
    """
    if cam is None:
        cam = stream_camera(header)
    fx, fy, cx, cy = cam["fx"], cam["fy"], cam["cx"], cam["cy"]
    cx_, cy_, w, h = s["bbox"]
    u, v = cx_, cy_
    depth = s.get("depth")
    rrx, rry, ryaw = s["rx"], s["ry"], s["ryaw"]
    if depth is None:
        return None
    x_cam = (u - cx) / fx * depth
    y_cam = (v - cy) / fy * depth
    # camera→world: +Z fwd = robot +x; camera +X right → world -y (code maps)
    # code builds PointStamped(z_cam, -x_cam, -y_cam) then TF2 rotates by ryaw.
    dx_r, dy_r = depth, -x_cam                   # robot/body frame pre-yaw
    c, sn = math.cos(ryaw), math.sin(ryaw)
    dx_m = c * dx_r - sn * dy_r
    dy_m = sn * dx_r + c * dy_r
    px, py = rrx + dx_m, rry + dy_m
    return {"u": u, "v": v, "depth": depth, "px": px, "py": py}


def main():
    rows = []          # per-sighting forensics rows
    runs = inventory()
    print(f"streams with GT: {len(runs)}")

    for run in runs:
        header, sightings = load_stream(run["stream"])
        spawn = header["spawn_pose"]
        raw = json.loads(open(run["gt"]).read())
        gt = (raw.get("raw_metrics", raw).get("ground_truth_objects")
              or raw.get("ground_truth_objects"))
        gtm = {k: {"type": e["type"], "mission": bool(e.get("mission_target")),
                   "mx": world_to_map(float(e["x"]), float(e["y"]), spawn)[0],
                   "my": world_to_map(float(e["x"]), float(e["y"]), spawn)[1]}
               for k, e in gt.items()}

        for s in sightings:
            rr = replay_projection(s, header, stream_camera(header))
            if rr is None:
                continue
            cls = s["class_name"]
            # bootstrap attribution: nearest same-class GT ≤3 m from ROBOT pose
            # (robot near object ⇒ likely the one in FOV; use robot, not the
            #  buggy projection, to avoid attributing to the defect itself)
            near = []
            for k, e in gtm.items():
                d_rob = math.hypot(e["mx"] - s["rx"], e["my"] - s["ry"])
                d_proj = math.hypot(e["mx"] - rr["px"], e["my"] - rr["py"])
                if e["type"] == cls:
                    near.append((d_rob, d_proj, k))
            near_same = [x for x in near if x[0] <= ATTR_RANGE]
            near_same.sort()
            row = {
                "agent": run["agent"], "batch6": run["batch"],
                "key": run["key"], "cls": cls,
                "conf": s["confidence"], "t": s["t"],
                "bbox": s["bbox"], "depth": rr["depth"],
                "proj_err": None, "gt_key": None, "type_match": False,
                "d_rob_gt": None, "bearing_gt_deg": None,
                "bearing_px_deg": None, "e_perp": None,
                "e_along": None, "range_gt": None,
            }
            if near_same:
                d_rob, d_proj, k = near_same[0]
                e = gtm[k]
                # range along the robot→GT ray vs along the bbox bearing ray
                rg = math.hypot(e["mx"] - s["rx"], e["my"] - s["ry"])
                p_range = math.hypot(rr["px"] - s["rx"], rr["py"] - s["ry"])
                bear_gt = math.atan2(e["my"] - s["ry"], e["mx"] - s["rx"])
                bear_px = math.atan2(rr["py"] - s["ry"], rr["px"] - s["rx"])
                # decompose: component of (proj - gt) along the gt bearing ray
                vec = (rr["px"] - e["mx"], rr["py"] - e["my"])
                ux, uy = math.cos(bear_gt), math.sin(bear_gt)
                e_along = vec[0] * ux + vec[1] * uy
                e_perp = vec[0] * (-uy) + vec[1] * ux
                row.update({
                    "gt_key": k, "type_match": True,
                    "d_rob_gt": round(d_rob, 3),
                    "proj_err": round(math.hypot(*vec), 3),
                    "bearing_gt_deg": round(math.degrees(ang_diff(bear_gt, s["ryaw"])), 1),
                    "bearing_px_deg": round(math.degrees(ang_diff(bear_px, s["ryaw"])), 1),
                    "bearing_offset_deg": round(math.degrees(bear_px - bear_gt), 2),
                    "e_along": round(e_along, 3),
                    "e_perp": round(e_perp, 3),
                    "range_gt": round(rg, 3),
                    "range_proj": round(p_range, 3),
                    # bbox physical size implied at GT range (px per pixel sanity)
                    "obj_h_px": s["bbox"][3],
                })
            rows.append(row)

    # ── aggregate ────────────────────────────────────────────────────────────
    at = [r for r in rows if r["type_match"]]
    at_inwin = [r for r in at if r["proj_err"] <= MATCH_WINDOW]
    print(f"sightings: {len(rows)}  attributable: {len(at)}  "
          f"in-window({MATCH_WINDOW}m): {len(at_inwin)}")

    import statistics as st
    def corr(xs, ys):
        n = len(xs)
        if n < 3:
            return None
        mx, my = sum(xs) / n, sum(ys) / n
        sx = math.sqrt(sum((x - mx) ** 2 for x in xs))
        sy = math.sqrt(sum((y - my) ** 2 for y in ys))
        if sx == 0 or sy == 0:
            return None
        return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / (sx * sy)

    # hypothesis regressions on attributable rows
    hyp = {}
    errs = [r["proj_err"] for r in at]
    hyp["corr_err_vs_range"] = corr([r["range_gt"] for r in at], errs)
    hyp["corr_err_vs_time"] = corr([r["t"] for r in at], errs)
    hyp["corr_err_vs_bear"] = corr([abs(r["bearing_gt_deg"]) for r in at], errs)
    hyp["corr_eperp_vs_range"] = corr([r["range_gt"] for r in at],
                                      [abs(r["e_perp"]) for r in at])
    hyp["corr_ealong_vs_range"] = corr([r["range_gt"] for r in at],
                                       [r["e_along"] for r in at])
    hyp["corr_bearoff_vs_range"] = corr([r["range_gt"] for r in at],
                                        [r["bearing_offset_deg"] for r in at])
    hyp["corr_bearoff_vs_bear"] = corr([r["bearing_gt_deg"] for r in at],
                                       [r["bearing_offset_deg"] for r in at])
    hyp["corr_bevox_vs_bear"] = hyp["corr_bearoff_vs_bear"]
    # bbox-center-depth signature: e_along>0 (overshoot behind) dominant?
    overs = [r["e_along"] for r in at]
    hyp["e_along_mean"] = st.mean(overs)
    hyp["e_along_median"] = st.median(overs)
    hyp["e_along_p05"] = sorted(overs)[int(0.05 * (len(overs) - 1))]
    hyp["e_along_p95"] = sorted(overs)[int(0.95 * (len(overs) - 1))]
    hyp["e_along_pos_frac"] = sum(1 for e in overs if e > 0.15) / len(overs)
    hyp["e_perp_mean"] = st.mean(abs(r["e_perp"]) for r in at)
    hyp["e_perp_median"] = st.median(abs(r["e_perp"]) for r in at)
    hyp["proj_err_mean"] = st.mean(errs)
    hyp["proj_err_median"] = st.median(errs)
    hyp["proj_err_p95"] = sorted(errs)[int(0.95 * (len(errs) - 1))]
    # drift test: early vs late half (median split on t)
    ts = sorted(r["t"] for r in at)
    tmid = st.median(ts)
    early = [r["proj_err"] for r in at if r["t"] <= tmid]
    late = [r["proj_err"] for r in at if r["t"] > tmid]
    hyp["err_mean_early_half"] = st.mean(early) if early else None
    hyp["err_mean_late_half"] = st.mean(late) if late else None

    print(json.dumps(hyp, indent=1, default=str))

    json.dump({"rows": rows, "hypotheses": hyp,
               "n_sightings": len(rows), "n_attributable": len(at)},
              open(os.path.join(REPO, "results", "projection_forensics.json"), "w"),
              indent=1, default=str)
    print("wrote results/projection_forensics.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())