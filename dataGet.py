import serial
import joblib
import numpy as np
import pandas as pd

# RUN MODEL PREDICTION
svm_model = joblib.load("svm_model2.pkl")
knn_model = joblib.load("knn_model2.pkl")

window_size = 400
sliding_window = []

def predict_gesture(data):
    global sliding_window
    global window_size

    if len(sliding_window) == 0:
        print("Start Recording")

    sliding_window.append(data)

    if len(sliding_window) == window_size: 
        print("Finish Recording")
        
        data = np.array(sliding_window)
        data = (data - data.min(axis=0)) / (data.max(axis=0) - data.min(axis=0))

        input_features = data.flatten()[:2004].reshape(1, -1)
        svm_prediction = svm_model.predict(input_features)
        knn_prediction = knn_model.predict(input_features)
        print(f"SVM Predicted Gesture: {svm_prediction[0]}")
        print(f"KNN Predicted Gesture: {knn_prediction[0]}")

        sliding_window = []
        

# GET DATA FROM IMU

# Open the serial port
port = serial.Serial('COM8', 115200, timeout=1)

# Function to read from serial port
def read_ser(num_char=1):
    string = port.read(num_char)
    return string.decode()

buffer = ""

def read_line(): 
    global buffer
    while True: 
        char = read_ser(1)
        if char == "\n": 
            line = buffer.strip()
            buffer = ""
            if line.startswith("\x1b") or line.startswith("acce_x"):
                continue
            else:
                values = list(map(float, line.split()))
                if len(values) == 7: 
                    predict_gesture(values[:-1])
        else: 
            buffer += char
            
if __name__ == "__main__":    
    read_line()
