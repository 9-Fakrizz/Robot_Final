import cv2
import numpy as np
import serial
import time

# Initialize serial communication (adjust port and baud rate)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Change 'COM3' to your port

# Frame properties
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CENTER_X = FRAME_WIDTH // 2  # X center of the frame
TOLERANCE = 50  # Allowable deviation for centering
SIZE_THRESHOLD = 18000  # Threshold for the object size to determine if the robot should move closer

def is_object_centered_and_size_correct(frame):
    global ser
    
    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define HSV range for object detection (adjust values as needed)
    lower_bound = np.array([30, 40, 40])  
    upper_bound = np.array([90, 255, 255])  
    
    # Create a mask
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    # Apply morphological operations to remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)  # Get the largest object
        if cv2.contourArea(largest_contour) > 500:  # Filter small noise
            x, y, w, h = cv2.boundingRect(largest_contour)
            centroid_x = x + w // 2
            centroid_y = y + h // 2
            
            # Draw bounding box and centroid
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (centroid_x, centroid_y), 5, (0, 0, 255), -1)
            cv2.putText(frame, "Object Detected", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Calculate object size (area)
            object_size = w * h  # Area of the bounding box
            cv2.putText(frame, f"Size: {object_size} px", (x, y + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            
            # Control logic to move robot
            if object_size < SIZE_THRESHOLD:
                ser.write(b'backward\n')  # Move closer if object size is smaller than threshold
                print("Object is too small, moving closer.")
            
            if centroid_x < CENTER_X - TOLERANCE:
                ser.write(b'CCW\n')  # Turn left
                print("Turning CCW")
            elif centroid_x > CENTER_X + TOLERANCE:
                ser.write(b'CW\n')  # Turn right
                print("Turning CW")
            elif object_size > SIZE_THRESHOLD:
                ser.write(b'stop\n')  # Stop if the object is centered
                print("Object centered, stopping.")
                # Return True if the object is centered and its size is above the threshold
                # return True
        else:
            ser.write(b'backward\n')  # Stop if object too small
            print("Object too small, moving forward to find it.")
    else:
        ser.write(b'backward\n')  # Stop if no object detected
        print("No object detected, moving forward.")
    
    # Return False if conditions are not met
    return False

def main():
    # Start video capture
    cap = cv2.VideoCapture(2)
    cap.set(3, FRAME_WIDTH)
    cap.set(4, FRAME_HEIGHT)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Check if the object is centered and of the correct size
        if is_object_centered_and_size_correct(frame):
            print("Object is centered and size is correct.")
            break  # Exit the loop if the object is correctly centered and sized
        
        # Display the processed frame
        cv2.imshow("Object Detection", frame)
        
        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release resources
    cap.release()
    cv2.destroyAllWindows()
    ser.close()

if __name__ == "__main__":
    main()
