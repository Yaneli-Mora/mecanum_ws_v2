
import cv2
import numpy as np

cap = cv2.VideoCapture(2)
ret, frame = cap.read()
cap.release()

if not ret:
    print("Failed to capture frame")
    exit()

hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
h, w = frame.shape[:2]

region = hsv[h//2-25:h//2+25, w//2-25:w//2+25]
print(f"Center region HSV min: {region.min(axis=(0,1))}")
print(f"Center region HSV max: {region.max(axis=(0,1))}")
print(f"Center region HSV mean: {region.mean(axis=(0,1)).astype(int)}")

