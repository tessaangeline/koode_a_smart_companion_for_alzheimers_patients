from flask import Flask, request, jsonify
import joblib
import numpy as np

app = Flask(__name__)

model = joblib.load("fall_model.pkl")
scaler = joblib.load("scaler.pkl")


@app.route("/predict", methods=["POST"])
def predict():
    print("📥 REQUEST RECEIVED")

    features = request.json["features"]
    features = scaler.transform([features])
    prediction = model.predict(features)

    if prediction[0] == 1:
        print("🚨 FALL DETECTED")
    else:
        print("✅ NORMAL ACTIVITY")

    return jsonify({"result": int(prediction[0])})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
