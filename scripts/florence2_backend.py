"""Florence-2 BATCH backend for eval_recog (t_bbe085d2).

Florence-2 (microsoft/Florence-2-large) does not load under the main venv's
transformers 5.3 (the remote-code processor and config both predate the 5.x
tokenizer backend), so it runs in an isolated interpreter pinned to
transformers 4.49 (/tmp/f2venv) with the main venv's torch/cv2 on PYTHONPATH
(florence2_batch.py in the task workspace). PIPE-stdin workers hang at model
load (measured nondeterministically; file-stdin runs are reliable), so the
backend is BATCH: the eval driver dumps its manifest slice to a spec file,
invokes the worker once, and serves frames from the collected JSONL. Latency
is the worker's per-frame wall; conf is the constant 1.0 (Florence-2 emits no
calibrated scores — only the loosest operating point is meaningful).
"""
import json
import os
import subprocess
import tempfile


class Florence2Backend:
    name = "florence2"

    def __init__(self, device: str, size: str = "large"):
        self.size = size
        self.name = f"florence2_{size}"
        self.f2py = os.environ.get("FLORENCE2_PY", "/tmp/f2venv/bin/python")
        self.worker = os.environ.get(
            "FLORENCE2_BATCH",
            "/home/plip/.hermes/kanban/boards/derpbot-explorer/"
            "workspaces/t_bbe085d2/florence2_batch.py")
        self.device = device
        self._tmpdir = tempfile.mkdtemp(prefix="florence2_eval_")
        self._spec = self._tmpdir + "/in.json"
        self._out = self._tmpdir + "/out.jsonl"
        self._queue = []
        self.wall_ms = None

    def init(self, prompts, cls_names=None):
        self.prompts = list(prompts)
        self._cls_of_prompt = {
            pr: (cls_names[i] if cls_names and i < len(cls_names) else pr)
            for i, pr in enumerate(self.prompts)}

    def _drain(self):
        """Run a worker pass over the queued frames and extend the result
        store. Frame-echo protocol keeps frames on disk; nothing but the
        spec path crosses a pipe."""
        rows, self._queue = self._queue, []
        json.dump({"frames": rows}, open(self._spec, "w"))
        if os.path.exists(self._out):
            os.unlink(self._out)
        env = dict(os.environ)
        env["PYTHONPATH"] = os.environ.get(
            "FLORENCE2_PYTHONPATH",
            "/tmp/f2venv/lib/python3.12/site-packages:"
            "/home/plip/Projects/derpbot-explorer/.venv/lib/python3.12/"
            "site-packages")
        env.setdefault("HF_HOME", os.path.expanduser("~/.cache/huggingface"))
        rc = subprocess.call(
            [self.f2py, self.worker, self._spec, self._out,
             self.size, self.device],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if rc != 0:
            raise RuntimeError(f"florence2 batch worker exited rc={rc}")
        self._results = {}
        for line in open(self._out):
            r = json.loads(line)
            self._results[r["frame_id"]] = r

    def infer(self, bgr) -> list:
        raise RuntimeError(
            "Florence2Backend is a batch backend: use infer_batch(frames) "
            "(it cannot serve one-frame pipe calls)")

    def infer_batch(self, frames) -> dict:
        """frames: list of (frame_id, bgr). Returns {frame_id: [det dicts]}.
        Runs one worker pass over everything queued so far."""
        self._queue = [
            {"frame_id": fid,
             "image": self._dump_image(fid, bgr),
             "prompts": self.prompts}
            for fid, bgr in frames]
        self._drain()
        out = {}
        for fid, _ in frames:
            r = self._results.get(fid)
            if r is None:
                out[fid] = [{"error": "missing from worker output"}]
                continue
            if r.get("err"):
                out[fid] = [{"error": r["err"]}]
                continue
            dets = []
            for d in r["dets"]:
                cls = self._cls_of_prompt.get(d["cls"].strip())
                if cls is None:
                    continue
                x1, y1, x2, y2 = d["bbox"]
                dets.append({"class": cls, "conf": float(d["conf"]),
                             "bbox": (float(x1), float(y1),
                                      float(x2), float(y2))})
            out[fid] = dets
            self.wall_ms = r.get("wall_ms")
        return out

    def _dump_image(self, fid, bgr) -> str:
        import cv2
        img_path = f"{self._tmpdir}/{fid.replace('/', '__')}.jpg"
        ok = cv2.imwrite(img_path, bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        if not ok:
            raise RuntimeError(f"image dump failed for {fid}")
        return img_path