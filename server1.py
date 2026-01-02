
import firebase_admin
from firebase_admin import credentials, messaging
from flask import Flask, request, jsonify
import joblib
import numpy as np

cred = credentials.Certificate("serviceAccountKey.json")
firebase_admin.initialize_app(cred)

app = Flask(__name__)

model = joblib.load("fall_model (1).pkl")
scaler = joblib.load("scaler (1).pkl")

THRESHOLD = 0.35   # tuned for windowed model

FCM_TOKENS = set()


@app.route("/register_token", methods=["POST"])
def register_token():
    token = request.json.get("token")

    if not token:
        return jsonify({"error": "No token"}), 400

    FCM_TOKENS.add(token)
    print("📲 Token registered")
    print("📦 Total tokens:", len(FCM_TOKENS))

    return jsonify({"status": "ok"})


def send_push_notification(title, body, source="UNKNOWN"):
    if not FCM_TOKENS:
        print("⚠️ No FCM tokens registered")
        return

    for token in FCM_TOKENS:
        message = messaging.Message(
            notification=messaging.Notification(
                title=title,
                body=body,
            ),
            token=token,
        )
        response = messaging.send(message)
        print(f"✅ Push sent [{source}]: {response}")


@app.route("/predict", methods=["POST"])
def predict():
    data = request.json

    acc_mean = data.get("acc_mean")
    acc_std = data.get("acc_std")
    gyro_mean = data.get("gyro_mean")
    gyro_std = data.get("gyro_std")

    if None in (acc_mean, acc_std, gyro_mean, gyro_std):
        return jsonify({"error": "Invalid input"}), 400

    X = np.array([[acc_mean, acc_std, gyro_mean, gyro_std]])
    X_scaled = scaler.transform(X)

    prob = model.predict_proba(X_scaled)[0][1]
    prediction = "FALL" if prob > THRESHOLD else "NO_FALL"

    print(
        f"Acc={acc_mean:.2f} Gyro={gyro_mean:.2f} "
        f"→ {prediction} ({prob:.2f})"
    )

    if prediction == "FALL":
        send_push_notification(
            title="🚨 Fall Detected",
            body="Possible fall detected. Please check immediately."
        )

    return jsonify({
        "prediction": prediction,
        "probability": float(prob)
    })


@app.route("/geofence", methods=["POST"])
def geofence():
    data = request.json

    status = data.get("status")
    distance = data.get("distance")
    lat = data.get("lat")
    lon = data.get("lon")

    if status not in ("OUT", "IN"):
        return jsonify({"error": "Invalid status"}), 400

    print(
        f"📍 GEOFENCE {status} | "
        f"Distance={distance:.2f}m "
        f"Lat={lat:.6f} Lon={lon:.6f}"
    )

    if status == "OUT":
        send_push_notification(
            title="📍 Geofence Alert",
            body=f"Patient exited safe zone ({distance:.1f} m away)."
        )

    if status == "IN":
        send_push_notification(
            title="✅ Geofence Update",
            body="Patient returned inside safe zone."
        )

    return jsonify({"status": "ok"})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
