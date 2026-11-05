# program/robot/models — file model (gitignored)

Direktori tempat menyimpan model ML. **Isinya tidak masuk git** —
`.gitignore` root sudah memuat `*.pt` dan `*.onnx`, jadi file model
besar tidak pernah ke-commit tidak sengaja. Yang masuk git hanya
README ini.

## Isi sekarang

| File | Asal | Kegunaan |
|---|---|---|
| `yolo11n.pt` | ultralytics COCO pretrained | validasi pipeline deteksi (bukan model arena) |
| `yolo11n-seg.pt` | ultralytics COCO pretrained | validasi pipeline seg + sumber export ONNX |

Keduanya **tidak tahu kelas checkpoint** — cuma buat membuktikan
pipeline jalan. Model yang berguna di kompetisi belum ada (butuh
dataset arena + spec Technical Meeting).

## Alur pakai

```
dataset (runs/dataset-*/)  ->  train  ->  best.pt  ->  export  ->  .onnx
                                                          |
                                                model_path di robot.json
```

```sh
# Train (di laptop/Jetson, pakai .venv repo):
.venv/bin/yolo segment train data=arena.yaml model=models/yolo11n-seg.pt epochs=100

# Export ke ONNX (runtime pakai cv2.dnn, bukan ultralytics):
.venv/bin/python tools/export_model.py runs/segment/train/weights/best.pt \
    --output models/checkpoint.onnx

# Aktifkan di robot.json (path relatif terhadap program/robot/):
#   "model_path": "models/checkpoint.onnx",
#   "checkpoint_mode": "model"
```

## Konvensi

- Simpan `.pt` (sumber/ultralytics) dan `.onnx` (runtime) di sini.
- `model_path` di config relatif terhadap `program/robot/` (cwd saat
  `main.py` dijalankan), jadi `models/checkpoint.onnx` sudah benar.
- Beri nama yang jelas: `checkpoint.onnx`, `checkpoint-v2.onnx` —
  jangan timpa model yang sudah terpakai tanpa catat di CHANGELOG.
- Kalau butuh model COCO buat bench `test-yolo*/`, script tersebut
  mengharapkan copy di folder masing-masing (lihat AGENTS.md).
