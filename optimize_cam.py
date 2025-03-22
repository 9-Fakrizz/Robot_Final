import cv2
import numpy as np
import serial
import time
import matplotlib.pyplot as plt

# Set up serial communication with ESP32
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Allow time for ESP32 to initialize

# Open the camera
cap = cv2.VideoCapture(2)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to HSV for better color filtering
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define black color range (tune values if necessary)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([130, 200, 80])  # Adjust based on lighting conditions

    # Create a mask to filter only black regions
    mask = cv2.inRange(hsv, lower_black, upper_black)

    # Apply Morphological Transformations to remove small noise
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Calculate black percentage
    total_pixels = mask.size  # Total pixels in the frame
    black_pixels = np.sum(mask == 255)  # Count white pixels in mask (representing black)
    black_percentage = (black_pixels / total_pixels) * 100  # Convert to percentage

    print(f"Black Percentage: {black_percentage:.2f}%")

    # Show results
    cv2.putText(frame, f"Black %: {black_percentage:.2f}%", (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Threshold", mask)
    cv2.imshow("Line Detection", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
ser.close()
cv2.destroyAllWindows()
