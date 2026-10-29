import serial, time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
time.sleep(2)  # ESP32 reset saat port dibuka

print(ser.readline().decode(errors='ignore').strip())  # harus "READY"

def send(l, r):
    ser.write(f"M {l} {r}\n".encode())
    print(ser.readline().decode(errors='ignore').strip())

try:
    send(150, 150)       # maju
    time.sleep(2)
    send(0, 0)           # berhenti
    time.sleep(1)
    send(-150, -150)     # mundur
    time.sleep(2)
    send(0, 0)
finally:
    ser.write(b"M 0 0\n")
    ser.close()