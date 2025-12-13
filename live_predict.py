import serial
import numpy as np
import joblib
from collections import deque

# Load trained model & scaler
model = joblib.load("fall_model.pkl")
scaler = joblib.load("scaler.pkl")


ser = serial.Serial('/dev/cu.usbserial-0001', 115200)  # Mac

WINDOW_SIZE = 200
buffer = deque(maxlen=WINDOW_SIZE)

print("Listening to ESP32...")

while True:
    line = ser.readline().decode().strip()
    values = list(map(float, line.split(",")))

    buffer.append(values)

    if len(buffer) == WINDOW_SIZE:
        data = np.array(buffer)

        # Feature extraction (MUST match training)
        features = [
            data[:, 0].mean(), data[:, 1].mean(), data[:, 2].mean(),
            data[:, 0].std(),  data[:, 1].std(),  data[:, 2].std(),
            data[:, 0].max(),  data[:, 1].max(),  data[:, 2].max(),
            data[:, 0].min(),  data[:, 1].min(),  data[:, 2].min(),
            data[:, 3].mean(), data[:, 4].mean(), data[:, 5].mean()
        ]

        features = scaler.transform([features])
        prediction = model.predict(features)

        if prediction[0] == 1:
            print("🚨 FALL DETECTED 🚨")
        else:
            print("Normal activity")
