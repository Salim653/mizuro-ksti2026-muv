#!/usr/bin/env python3
"""Export model ultralytics (.pt) -> ONNX buat runtime robot.

    .venv/bin/python tools/export_model.py yolo11n-seg.pt

PERINGATAN VENV: export ultralytics sering meng-upgrade numpy/opencv
di venv target. Di repo ini pernah terjadi: numpy 1.26.4 -> 2.2.6 dan
opencv-python 4.8.0 -> 5.0.0.93, yang membuat cv2 TIDAK BISA di-import
(ABI numpy). Kalau terpaksa pakai .venv repo, cek `import cv2` setelah
export dan kalau rusak:

    .venv/bin/pip install "numpy==1.26.4" "opencv-python==4.8.0.76"

Runtime robot pakai cv2.dnn (bukan onnxruntime) karena /usr/bin/python3
di Jetson tidak punya onnxruntime — cv2 ada di kedua interpreter.
"""
from __future__ import annotations

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Export .pt -> ONNX (runtime pakai cv2.dnn)")
    parser.add_argument("model", type=Path, help="file .pt ultralytics")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--no-simplify", action="store_true",
                        help="lewati onnxslim (default: simplify)")
    parser.add_argument("--output", type=Path, default=None,
                        help="path .onnx output (default: sebelah model)")
    args = parser.parse_args()

    if not args.model.is_file():
        raise SystemExit(f"model tidak ada: {args.model}")

    from ultralytics import YOLO
    print(f"export {args.model} (imgsz={args.imgsz})...")
    out = YOLO(str(args.model)).export(
        format="onnx", imgsz=args.imgsz, simplify=not args.no_simplify)
    out_path = Path(out)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        out_path.replace(args.output)
        out_path = args.output
    print(f"ONNX: {out_path} ({out_path.stat().st_size / 1e6:.1f} MB)")
    print("cek cv2 masih hidup: python -c 'import cv2'")
    print("pakai: set model_path di robot.json ke file ini")


if __name__ == "__main__":
    main()
