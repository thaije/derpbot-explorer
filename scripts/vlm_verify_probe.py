#!/usr/bin/env python3
"""v13-cv Step-0 probe: glm-5.3-flash on glimpse (+ matched negative) keyframes.

Zero-sim; fires the EXACT Phase-3 ladder prompt style (single-parse structured
output, mission-target prompts derived from the mission classes with the
detector's underscore->space rule) per frame. abort bar: hallucination >50%%
on the no-mission-target negatives -> stop, report, no sim.

Usage:  .venv/bin/python scripts/vlm_verify_probe.py \
            [--manifest <probe manifest json>] [--out <out.jsonl>] [--limit N]
Manifest format: {"frames": [{"id", "path", "kind": "glimpse"|"negA"|"negB",
                              "run", "label", "cls", "t", ...}]}
Resumable: skips ids already present in --out.
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import re
import sys
import time
import urllib.request

MODEL = "glm-5.3-flash:cloud"
OLLAMA_URL = "http://127.0.0.1:11434/api/chat"
MISSION_PROMPTS = {"fire_extinguisher": "fire extinguisher",
                   "first_aid_kit": "first aid kit", "person": "person"}


def build_prompt(cls_lookup=MISSION_PROMPTS):
    classes = ", ".join(cls_lookup[t] for t in sorted(cls_lookup))
    return (
        "Look at this image from a robot's camera. Does the image clearly show "
        f"any of the following objects: {classes}?\n\n"
        'Answer with ONLY a JSON object, no other text:\n'
        '{"detections": [{"class": "<object name>", "bbox": [x1, y1, x2, y2], '
        '"confidence": <number between 0.0 and 1.0>}]}\n'
        "- bbox is pixel coordinates [x1, y1, x2, y2] in the image.\n"
        '- "class" must be one of the listed object names.\n'
        "- Include an object ONLY if it is clearly visible in this image. "
        'If none are visible, return {"detections": []}.'
    )


def parse_strict(text: str):
    """Strict single-parse: first JSON object in the reply, else ValueError."""
    text = text.strip()
    m = re.search(r"\{.*\}", text, re.S)
    if not m:
        raise ValueError(f"no JSON object in reply: {text[:120]!r}")
    obj = json.loads(m.group(0))
    dets = obj.get("detections", [])
    if not isinstance(dets, list):
        raise ValueError("detections not a list")
    out = []
    for d in dets:
        if not isinstance(d, dict) or "class" not in d or "bbox" not in d:
            raise ValueError(f"malformed det: {str(d)[:120]}")
        conf = float(d.get("confidence", 0.0))
        bb = [float(x) for x in d["bbox"]]
        if len(bb) != 4:
            raise ValueError(f"bbox len != 4: {bb}")
        out.append({"cls": d["class"], "conf": conf, "bbox": bb})
    return out


def call_model(image_path: str, prompt: str, timeout: float = 300.0):
    b64 = base64.b64encode(open(image_path, "rb").read()).decode()
    body = json.dumps({
        "model": MODEL,
        "messages": [{"role": "user", "content": prompt, "images": [b64]}],
        "stream": False,
    }).encode()
    req = urllib.request.Request(OLLAMA_URL, data=body,
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            d = json.loads(r.read())
        return {"dets_raw": d.get("message", {}).get("content", ""),
                "wall_s": time.time() - t0, "err": None}
    except Exception as exc:  # noqa: BLE001
        return {"dets_raw": "", "wall_s": time.time() - t0, "err": str(exc)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--retries", type=int, default=2)
    args = ap.parse_args()

    man = json.load(open(args.manifest))
    frames = man["frames"]
    done = set()
    if os.path.exists(args.out):
        for line in open(args.out):
            try:
                done.add(json.loads(line)["id"])
            except Exception:
                pass
    todo = [f for f in frames if f["id"] not in done]
    if args.limit:
        todo = todo[: args.limit]
    print(f"{len(done)} already done, {len(todo)} to run", flush=True)

    prompt = build_prompt()
    with open(args.out, "a", buffering=1) as out:
        for f in todo:
            rec = None
            for attempt in range(1 + args.retries):
                r = call_model(f["path"], prompt)
                try:
                    dets = parse_strict(r["dets_raw"])
                    rec = {"id": f["id"], "kind": f["kind"], "run": f.get("run"),
                           "label": f.get("label"), "cls": f.get("cls"),
                           "t": f.get("t"), "dets": dets,
                           "wall_s": round(r["wall_s"], 1), "err": None,
                           "attempts": attempt + 1}
                    break
                except (ValueError, json.JSONDecodeError) as exc:
                    rec = {"id": f["id"], "kind": f["kind"], "run": f.get("run"),
                           "label": f.get("label"), "cls": f.get("cls"),
                           "t": f.get("t"), "dets": [], "dets_raw": r["dets_raw"][:400],
                           "wall_s": round(r["wall_s"], 1),
                           "err": f"parse: {exc}", "attempts": attempt + 1}
            # 429 / quota: mark incomplete and move on (cost protocol)
            if rec.get("err") and "429" in str(rec["err"]):
                rec["err"] = "429-quota: marked incomplete, no retry"
                print(f"  429 on {f['id']} — skipping ladder for this candidate", flush=True)
            out.write(json.dumps(rec) + "\n")
            flat = [f"{d['cls']}@{d['conf']:.2f}" for d in rec.get("dets", [])]
            print(f"{f['id']}: {flat or '(none)'} err={rec.get('err')} "
                  f"{rec['wall_s']}s try{rec['attempts']}", flush=True)


if __name__ == "__main__":
    main()