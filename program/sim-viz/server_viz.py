#!/usr/bin/env python3
"""Inference server for the model-driven sim copy (VISUALISATION variant).

Same as server.py but:
  * serves sim-model-viz/ instead of sim-model/
  * listens on its own port (default 8766) so it can run next to server.py
  * POST /predict?viz=1 additionally returns {"viz": "<base64 jpeg>"} — the exact
    frame the model saw with the segmentation masks + fitted polylines painted on
    top. The sim shows that thumbnail in the corner so you can see the seg working.

Hosts sim-model-viz/ on http://127.0.0.1:8766 and exposes
POST /predict[?viz=1]  raw JPEG (Content-Type: image/jpeg) or {jpeg: "<base64>"}
               -> {outer, inner, centre: [[u,v],...], viz?: "<base64 jpeg>"}
GET  /health

Backend: ONNX Runtime (model_fp32_480.onnx, ~57 ms/frame on the 4750U).
Falls back to the .pt automatically if ONNX is missing.

Run:  python server_viz.py     (miniconda python recommended: has onnxruntime)
Stop: Ctrl+C
"""
import base64, json, os, sys, threading
from importlib.metadata import PackageNotFoundError, distribution
# The sim must never wait on the model. Keep this process from hogging the CPU:
# single-threaded BLAS/OpenMP (set before numpy loads), no thread spin-waiting,
# and below-normal priority so the browser always wins a contested core.
for _v in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
import numpy as np
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote

import cv2
cv2.setNumThreads(1)
if os.name == "nt":
    import ctypes
    ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(), 0x4000)  # BELOW_NORMAL

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim-model-viz")
_HERE = os.path.dirname(os.path.abspath(__file__))
# Weights live next to this script when packaged, else fall back to the Downloads
# testbed copy so the viz server can run from anywhere (e.g. the project dir).
_ONNX_CANDIDATES = [
    os.path.join(_HERE, "model_fp32_480.onnx"),
    r"D:\00 DOWNLOADS\ksti-model-test\model_fp32_480.onnx",
]
ONNX = next((p for p in _ONNX_CANDIDATES if os.path.isfile(p)), _ONNX_CANDIDATES[0])
PT = os.path.join(_HERE, "best.pt")   # shipped copy (packaged zip)
if not os.path.isfile(PT):
    PT = r"D:\00 DOWNLOADS\best (1).pt"                                     # original location on Salim's box
PORT = int(os.environ.get("KSTI_VIZ_PORT", "8766"))                         # own port: run alongside server.py
IMSZ = 480 if os.path.isfile(ONNX) else 640
WEIGHTS = ONNX if os.path.isfile(ONNX) else PT
CONF = 0.25

print("loading", WEIGHTS, f"(imgsz {IMSZ}) ...")
USE_ONNX = WEIGHTS.endswith(".onnx")
if USE_ONNX:
    # Prefer the DirectML distribution when another site-packages has CPU ORT.
    BACKEND = os.environ.get("KSTI_BACKEND", "dml").lower()
    if BACKEND == "dml":
        try:
            _dml_site = str(distribution("onnxruntime-directml").locate_file(""))
            sys.path.insert(0, _dml_site)
        except PackageNotFoundError:
            pass
    import onnxruntime as ort
    _so = ort.SessionOptions()
    _so.intra_op_num_threads = 4
    _so.inter_op_num_threads = 1
    _so.add_session_config_entry("session.intra_op.allow_spinning", "0")
    _so.add_session_config_entry("session.inter_op.allow_spinning", "0")
    # Set KSTI_BACKEND=cpu to opt out of DirectML.
    if BACKEND == "dml":
        if "DmlExecutionProvider" not in ort.get_available_providers():
            raise RuntimeError("DirectML is unavailable; install onnxruntime-directml or set KSTI_BACKEND=cpu")
        _so.enable_mem_pattern = False
        _so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        _prov = ["DmlExecutionProvider", "CPUExecutionProvider"]
    else:
        _prov = ["CPUExecutionProvider"]
    SESS = ort.InferenceSession(WEIGHTS, _so, providers=_prov)
    print("onnxruntime providers:", SESS.get_providers())
    INAME = SESS.get_inputs()[0].name
    MODEL = None
else:
    from ultralytics import YOLO                      # torch only when there is no ONNX
    MODEL = YOLO(WEIGHTS, task="segment")
LOCK = threading.Lock()


def _letterbox(img, sz):
    r = sz / max(img.shape[:2])
    im = cv2.resize(img, (int(img.shape[1] * r), int(img.shape[0] * r)))
    c = np.full((sz, sz, 3), 114, np.uint8)
    c[:im.shape[0], :im.shape[1]] = im
    x = np.ascontiguousarray(c[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0)
    return x, r


def _nms(xy1, xy2, cid, cf, iou=0.7):
    order = cf.argsort()[::-1]
    keep = []
    for i in order:
        ok = True
        for j in keep:
            if cid[i] != cid[j]:
                continue
            xx1 = max(xy1[i, 0], xy1[j, 0]); yy1 = max(xy1[i, 1], xy1[j, 1])
            xx2 = min(xy2[i, 0], xy2[j, 0]); yy2 = min(xy2[i, 1], xy2[j, 1])
            inter = max(0, xx2 - xx1) * max(0, yy2 - yy1)
            a = (xy2[i, 0] - xy1[i, 0]) * (xy2[i, 1] - xy1[i, 1])
            b = (xy2[j, 0] - xy1[j, 0]) * (xy2[j, 1] - xy1[j, 1])
            if inter / max(a + b - inter, 1e-9) > iou:
                ok = False; break
        if ok:
            keep.append(i)
    return keep


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


# BGR colours for the debug overlay: one per segmentation class.
CLASS_COL = {"outer": (60, 220, 90), "inner": (255, 150, 40), "centre": (60, 160, 255)}


def _mask_viz(img, insts, out):
    """Composite the literal YOLO-seg output over the input frame and return a
    base64 JPEG. For every kept detection we draw its segmentation mask as a
    semi-transparent filled polygon plus its contour outline (the mask polygon),
    coloured per class. The fitted centre polylines the lane controller consumes
    are drawn thin and white on top. Debug view only."""
    H, W = img.shape[:2]
    disp = img.copy()
    fill = np.zeros_like(disp)
    outlines = []
    for name, comp in insts:
        col = CLASS_COL.get(name)
        if col is None or comp is None or not comp.any():
            continue
        m8 = comp.astype(np.uint8) * 255
        cnts, _ = cv2.findContours(m8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            continue
        cv2.drawContours(fill, cnts, -1, col, -1)          # filled mask polygon
        outlines.append((cnts, col))
    if fill.any():
        disp = cv2.addWeighted(disp, 1.0, fill, 0.45, 0)   # semi-transparent fill
    for cnts, col in outlines:
        cv2.drawContours(disp, cnts, -1, col, 2, cv2.LINE_AA)   # mask polygon outline
    for name, pts in out.items():
        if name == "viz" or len(pts) < 2:
            continue
        arr = np.array(pts, np.int32).reshape(-1, 1, 2)
        cv2.polylines(disp, [arr], False, (255, 255, 255), 1, cv2.LINE_AA)
    tw = 320
    th = max(1, int(round(H * tw / W)))
    disp = cv2.resize(disp, (tw, th), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", disp, [int(cv2.IMWRITE_JPEG_QUALITY), 72])
    return base64.b64encode(buf.tobytes()).decode() if ok else ""



def _predict_onnx(img, want_viz=False):
    sz = IMSZ
    x, r = _letterbox(img, sz)
    out0, protos = SESS.run(None, {INAME: x})
    ps = protos.shape[2]
    pred = out0[0].T                          # (N, 4+3+32)
    scores = pred[:, 4:7]
    cid = scores.argmax(1); cf = scores.max(1)
    keep = cf > CONF
    boxes, cid, cf, mc = pred[keep, :4], cid[keep], cf[keep], pred[keep, 7:]
    H, W = img.shape[:2]
    out = {"outer": [], "inner": [], "centre": []}
    key = {0: "outer", 1: "inner", 2: "centre"}
    if len(boxes) == 0:
        return _viz_out(img, out, [], want_viz)
    xy1 = boxes[:, :2] - boxes[:, 2:] / 2
    xy2 = boxes[:, :2] + boxes[:, 2:] / 2
    sel = _nms(xy1, xy2, cid, cf)
    if not sel:
        return _viz_out(img, out, [], want_viz)
    # mask = sigmoid(coefficients @ protos), sigmoid AFTER the linear combination
    masks_all = _sigmoid(mc[sel] @ protos[0].reshape(32, -1)).reshape(-1, ps, ps)
    vh, vw = int(round(H * r)), int(round(W * r))         # un-padded letterbox region
    # All mask work happens at letterbox resolution (e.g. 480x270), then the
    # polyline is scaled to frame pixels: about 20x fewer pixels than full-res.
    best = {}                                             # per class: largest mask wins
    insts = []                                            # every kept detection's raw mask (debug view)
    for k, i in enumerate(sel):
        name = key.get(int(cid[i]))
        if name is None:
            continue
        x1i, y1i = max(0, int(xy1[i, 0])), max(0, int(xy1[i, 1]))
        x2i, y2i = min(vw, int(xy2[i, 0])), min(vh, int(xy2[i, 1]))
        if x2i <= x1i or y2i <= y1i:
            continue
        full = cv2.resize(masks_all[k], (sz, sz), interpolation=cv2.INTER_LINEAR)
        m = np.zeros((vh, vw), np.uint8)
        m[y1i:y2i, x1i:x2i] = full[y1i:y2i, x1i:x2i] > 0.5
        insts.append((name, m.copy()))                    # the literal YOLO-seg mask for this instance
        n, lab, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
        if n < 2:
            continue
        big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        area = int(stats[big, cv2.CC_STAT_AREA])
        if area < 6 or area <= best.get(name, (0,))[0]:
            continue
        ys, xs = np.nonzero(lab == big)
        cnt = np.bincount(ys, minlength=vh)
        sx = np.bincount(ys, weights=xs, minlength=vh)
        rows = np.nonzero(cnt)[0]
        best[name] = (area, rows, sx[rows] / cnt[rows])
    for name, (_, rows, mx) in best.items():
        u = (mx + 0.5) / r - 0.5                          # letterbox px -> frame px
        v = (rows + 0.5) / r - 0.5
        out[name] = [[round(float(a), 1), int(round(b))] for a, b in zip(u, v)]
    return _viz_out(img, out, insts, want_viz)


def _viz_out(img, out, insts, want_viz):
    """Attach a base64 'viz' thumbnail when the caller asked for one. insts is a list
    of (class, mask@letterbox) for every kept detection. Empty insts still returns the
    raw frame so the corner feed stays alive when the model detects nothing."""
    if not want_viz:
        return out
    H, W = img.shape[:2]
    instsF = [(name, cv2.resize(m, (W, H), interpolation=cv2.INTER_NEAREST) > 0)
              for name, m in insts]
    out["viz"] = _mask_viz(img, instsF, out)
    return out


def predict_polylines(jpeg_bytes, want_viz=False):
    img = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
    with LOCK:
        if USE_ONNX:
            return _predict_onnx(img, want_viz)
        res = MODEL.predict(img, imgsz=IMSZ, conf=CONF, verbose=False)[0]
    out = {"outer": [], "inner": [], "centre": []}
    key = {0: "outer", 1: "inner", 2: "centre"}
    insts = []                                    # every kept detection's raw mask (frame res), debug view
    if res.masks is None:
        return _viz_out_pt(img, out, insts, want_viz)
    masks = res.masks.data.cpu().numpy()
    cls = res.boxes.cls.cpu().numpy().astype(int)
    h, w = img.shape[:2]
    for mi in range(len(masks)):
        name = key.get(int(cls[mi]) if mi < len(cls) else -1)
        if name is None:
            continue
        m = (masks[mi] > 0.5).astype(np.uint8)
        if m.shape != (h, w):
            m = cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST)
        if m.sum() < 12:
            continue
        insts.append((name, m.astype(bool)))      # the literal YOLO-seg mask for this instance
        n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
        if n < 2:
            continue
        big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        comp = (lab == big)
        ys, xs = np.nonzero(comp)
        order = np.argsort(ys)
        ys, xs = ys[order], xs[order]
        rows, rep = [], []
        for y in np.unique(ys[::2]):
            sel = ys == y
            rows.append(int(y)); rep.append(float(xs[sel].mean()))
        out[name] = list(zip([round(v, 1) for v in rep], rows))
    return _viz_out_pt(img, out, insts, want_viz)


def _viz_out_pt(img, out, insts, want_viz):
    if want_viz:
        out["viz"] = _mask_viz(img, insts, out)
    return out


class H(BaseHTTPRequestHandler):
    def log_message(self, fmt, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = unquote(self.path.split("?")[0])
        if path == "/health":
            return self._send(200, b'{"ok":true}')
        if path.endswith("/"):
            path += "index.html"                 # bare http://127.0.0.1:8765/ opens the sim
        fp = os.path.normpath(os.path.join(ROOT, path.lstrip("/")))
        if not fp.startswith(ROOT) or not os.path.isfile(fp):
            return self._send(404, b"not found", "text/plain")
        ext = os.path.splitext(fp)[1].lower()
        ct = {".html": "text/html", ".js": "text/javascript", ".json": "application/json",
              ".jpg": "image/jpeg", ".png": "image/png"}.get(ext, "application/octet-stream")
        with open(fp, "rb") as f:
            self._send(200, f.read(), ct)

    def do_POST(self):
        path, _, query = self.path.partition("?")
        if path != "/predict":
            return self._send(404, b"not found", "text/plain")
        want_viz = "viz=1" in query                            # ?viz=1 -> include the mask overlay thumbnail
        try:
            n = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(n)
            if self.headers.get("Content-Type", "").startswith("image/"):
                jpeg = raw                                    # binary JPEG body (sim)
            else:
                jpeg = base64.b64decode(json.loads(raw)["jpeg"])  # {jpeg: b64} (scripts)
            polylines = predict_polylines(jpeg, want_viz)
            self._send(200, json.dumps(polylines).encode())
        except Exception as e:
            self._send(500, json.dumps({"error": str(e)}).encode())


if __name__ == "__main__":
    srv = ThreadingHTTPServer(("127.0.0.1", PORT), H)
    print(f"serving sim-model-viz/ + /predict (?viz=1 for the mask overlay) on http://127.0.0.1:{PORT}  (Ctrl+C to stop)")
    srv.serve_forever()
