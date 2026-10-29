import cv2
from ultralytics import YOLO

model = YOLO("yolo11n.pt")

pipeline = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
    "nvvidconv ! video/x-raw,format=BGRx ! "
    "videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
)
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Running... tekan Ctrl+C untuk berhenti.")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        small = cv2.resize(frame, (320, 240))
        sx = frame.shape[1] / small.shape[1]
        sy = frame.shape[0] / small.shape[0]

        results = model(small, verbose=False)

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                x1, x2 = int(x1 * sx), int(x2 * sx)
                y1, y2 = int(y1 * sy), int(y2 * sy)
                conf = round(box.conf[0].item(), 2)
                label = f"{model.names[int(box.cls[0].item())]} {conf}"
                print(label, (x1, y1, x2, y2))
finally:
    cap.release()