import time
import cv2
import serial
from ultralytics import YOLO

# ===== Pengaturan =====
SERIAL_PORT = "/dev/ttyCH341USB0"
BAUD        = 115200
SPEED       = 200      # kecepatan maju (0-255), mulai kecil
CONF        = 0.5      # batas keyakinan deteksi
BOTTLE_ID   = 39       # kelas "bottle" di COCO
CONFIRM     = 2        # botol harus terlihat N frame berturut-turut -> maju
LOST        = 3        # botol hilang N frame berturut-turut -> berhenti

# ===== Serial ke ESP32 =====
ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0)
time.sleep(2)  # ESP32 reset saat port dibuka

def send(l, r):
    ser.write(f"M {l} {r}\n".encode())
    ser.reset_input_buffer()  # buang balasan "OK" agar buffer tidak menumpuk

# ===== Model & kamera =====
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
    ser.close()
    exit()

print("Running... tekan Ctrl+C untuk berhenti.")

seen = 0      # hitungan frame botol terlihat
missed = 0    # hitungan frame botol tidak terlihat
moving = False
t0, n = time.time(), 0

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            send(0, 0)
            break

        small = cv2.resize(frame, (320, 240))
        results = model(small, verbose=False, device=0,
                        classes=[BOTTLE_ID], conf=CONF)

        found = len(results[0].boxes) > 0

        if found:
            seen += 1
            missed = 0
            if seen >= CONFIRM:
                moving = True
        else:
            missed += 1
            seen = 0
            if missed >= LOST:
                moving = False

        # Kirim perintah di SETIAP frame (failsafe ESP32 butuh perintah rutin)
        if moving:
            send(SPEED, SPEED)
        else:
            send(0, 0)

        n += 1
        if n % 30 == 0:
            print(f"FPS: {30 / (time.time() - t0):.1f} | "
                  f"bottle: {found} | motor: {'MAJU' if moving else 'STOP'}")
            t0 = time.time()

except KeyboardInterrupt:
    print("\nBerhenti.")
finally:
    try:
        send(0, 0)
        ser.write(b"M 0 0\n")
    finally:
        ser.close()
        cap.release()