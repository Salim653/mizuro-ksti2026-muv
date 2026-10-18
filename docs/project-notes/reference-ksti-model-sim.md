---
name: reference-ksti-model-sim
description: KSTI model-driven sim testbed at D:\00 DOWNLOADS\ksti-model-test (YOLO11n-seg lane model steering a copy of arena-3d via a local inference server); sim-timing rule he set; laptop GPU (DirectML/ncnn Vulkan) measured as no help
metadata:
  type: project
---

🧪 **Testbed (built 2026-10-16/17):** `D:\00 DOWNLOADS\ksti-model-test\` — his trained lane model (`D:\00 DOWNLOADS\best (1).pt`, YOLO11n-seg, classes outer / inner / centre) exported to ONNX at 480 px, served by `server.py` on 127.0.0.1:8765, which also hosts `sim-model/` — a **copy** of the arena-3d sim where the model drives (pure pursuit on the fitted lane). The original arena-3d sim stays untouched; edit only the copy. Open it via the server address, not by double-clicking the html (fetch to the model fails from file://). Headless lap/perception/frame-timing checker scripts sit next to the server.

⏱️ **Rule he set:** the model is only a driver — it must never dictate sim timing. Sim runs on wall-clock time; the driver holds its last command between results; if inference is late the car is allowed to go astray.
**Why:** he saw the car slow down whenever the model ran and called it wrong — a real car doesn't wait for perception.
**How to apply:** any sim/HIL change keeps physics on wall time and perception asynchronous; judge the model by laps completed under real latency, never by pausing the world for it.

🖥️ **GPU on the ThinkPad doesn't help (measured 2026-10-17):** ncnn Vulkan on the Vega iGPU is ~2× slower than CPU ONNX Runtime; DirectML looks faster in a tight loop but end-to-end round trip is worse than CPU (shared power budget / memory with the CPU, and the iGPU also renders the sim). CPU ONNX Runtime stays the default; a DirectML switch exists but is off. Sim speed matters more: 2× sim speed doubles effective perception latency and laps failed, 1× completes laps.
**How to apply:** don't re-propose laptop GPU acceleration; the real speedup path is TensorRT on the team's Jetson Orin Nano.

Related: [[project-ksti-2026-mav]], [[reference-ksti-arena3d-dataset]]
