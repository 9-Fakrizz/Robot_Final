import cv2
import numpy as np
import serial
import time
import sys

checkpoint_1 = True
last_command = ""
allow_twice_turn = True
allow_mix_turn_point = True
color_line_state = False

# Validate command-line argument
if len(sys.argv) != 2 or sys.argv[1] not in ["red", "green", "blue", "black"]:
    print("Usage: python select.py [red | green | blue | black]")
    sys.exit(1)

color_choice = sys.argv[1]  # Get color from command-line argument

def get_color_mask(hsv, color):
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([130, 200, 80])  # Adjust based on lighting

    mask_black = cv2.inRange(hsv, lower_black, upper_black)
    mask_color = np.zeros_like(mask_black)  # Default to black only

    if color == "red":
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask_color = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), cv2.inRange(hsv, lower_red2, upper_red2))

    elif color == "green":
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        mask_color = cv2.inRange(hsv, lower_green, upper_green)

    elif color == "blue":
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([130, 255, 255])
        mask_color = cv2.inRange(hsv, lower_blue, upper_blue)

    return cv2.bitwise_or(mask_black, mask_color)


# Set up serial communication with ESP32
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Allow time for ESP32 to initialize
cap = cv2.VideoCapture(2)

def detection_percentage(region):
    return np.sum(region == 255) / region.size * 100

ignore_turns_until = 0  # Timestamp to ignore SCCW and SCW

while True:  
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to HSV for better color filtering
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = get_color_mask(hsv, color_choice)

    # Apply Morphological Transformations to remove small noise
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    thresh = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

     # Calculate black percentage
    total_pixels = mask.size  # Total pixels in the frame
    black_pixels = np.sum(mask == 255)  # Count white pixels in mask (representing black)
    black_percentage = (black_pixels / total_pixels) * 100  # Convert to percentage

    # print(f"Black Percentage: {black_percentage:.2f}%")

    # # Show results
    cv2.putText(frame, f"Black %: {black_percentage:.2f}%", (90, 250),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("threshold", thresh)

    # Get frame dimensions
    h, w = thresh.shape
    num_boxes = 7  # Total sensor regions
    x_center = w // 2  # Center position
    y_pos = int(h * 0.8)  # Lower in the frame for better detection
    box_width = w // (num_boxes + 3)  # Ensure non-overlapping width
    box_height = h // 10  # Height remains fixed
    spacing = box_width + 10  # Ensure proper spacing

    # Define sensor positions, ensuring even spacing
    positions = {
        "left2": (x_center - 3 * spacing, y_pos),
        "left1": (x_center - 2 * spacing, y_pos),
        "center_left": (x_center - spacing, y_pos),
        "center": (x_center, y_pos),  # Perfectly centered
        "center_right": (x_center + spacing, y_pos),
        "right1": (x_center + 2 * spacing, y_pos),
        "right2": (x_center + 3 * spacing, y_pos)
    }

    # Extract sensor regions and calculate percentages
    percentages = {key: detection_percentage(thresh[y:y+box_height, x-box_width//2:x+box_width//2])
                   for key, (x, y) in positions.items()}

    left_percentage = (percentages["left2"] + percentages["left1"])//2
    right_percentage = (percentages["right1"] + percentages["right2"])//2
    center_detected = percentages["center"] > 30
    center_left_detected = percentages["center_left"] > 30
    center_right_detected = percentages["center_right"] > 30


    # Convert to HSV for better color filtering
    hsv2 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask2 = get_color_mask(hsv2, "black")

    # Apply Morphological Transformations to remove small noise
    kernel = np.ones((7, 7), np.uint8)
    mask2 = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, kernel)
    thresh2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, kernel)

    # Calculate black percentage
    total_pixels2 = mask2.size  # Total pixels in the frame
    justblack_pixels = np.sum(mask2 == 255)  # Count white pixels in mask (representing black)
    justblack_percentage = (justblack_pixels / total_pixels2) * 100  # Convert to percentage


    current_time = time.time()  # Get the current time
    command = ""

    # SCCW and SCW should be ignored if within cooldown period
    if current_time > ignore_turns_until:
        if center_left_detected and left_percentage > 60 and not center_right_detected:
            command = "SCCW"
            ignore_turns_until = current_time + 5  # Ignore SCCW and SCW for 2 sec
        elif center_right_detected and right_percentage > 60 and not center_left_detected:
            command = "SCW"
            ignore_turns_until = current_time + 5  # Ignore SCCW and SCW for 2 sec

    # Other commands remain active
    if not command:  # If no SCCW/SCW is triggered (or they are ignored)
        if black_percentage >= 18 and color_line_state == False:
            if (black_percentage - justblack_percentage) == 0 :
                print("black line point!")
                if checkpoint_1 == False:
                    for i in range(2):
                        command = "backward"
                        ser.write((command + "\n").encode())
                        time.sleep(1)
                    checkpoint_1 = True
                    print("Check Point 1 Complete !!!!")
                    
                # command = "stop"
            elif (black_percentage - justblack_percentage) > 10 and allow_mix_turn_point:
                print("mix turn point!")
                # command = "backward"
                # ser.write((command + "\n").encode())
                if left_percentage > right_percentage :
                    command = "SCCW"
                    print("mix turn point! SCCW")
                    allow_mix_turn_point = False
                    color_line_state = True
                elif right_percentage > left_percentage:
                    command = "SCW"
                    print("mix turn point! SCW")
                    allow_mix_turn_point = False
                    color_line_state = True
            
        elif justblack_percentage <= 1 and black_percentage >= 35:
            print("stop color point!")
            command = "stop"
        elif left_percentage > right_percentage and last_command != "CW":
            command = "CCW"
        elif right_percentage > left_percentage and last_command != "CCW":
            command = "CW"
        elif center_detected:
            command = "backward"

        # if last_command == "SCCW" and allow_twice_turn:
        #     command = "SCCW"
        #     print("*2 turn")
        #     allow_twice_turn = False
        # elif last_command == "SCW" and allow_twice_turn:
        #     command = "SCW"
        #     print("*2 turn")
        #     allow_twice_turn = False
        # elif last_command == "backward" and allow_twice_turn == False:
        #     allow_twice_turn = True

    # Send command to ESP32
    if command:
        last_command = command
        ser.write((command + "\n").encode())

    # Display detection results
    detected_text = f"Detected: {command} | Left: {left_percentage:.1f}% | Right: {right_percentage:.1f}%"
    cv2.putText(frame, detected_text, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # Draw sensor boxes with corrected alignment
    for key, (x, y) in positions.items():
        color = (0, 0, 255) if percentages[key] > 30 else (255, 255, 255)
        cv2.rectangle(frame, (x - box_width // 2, y), (x + box_width // 2, y + box_height), color, 2)

    # Show the result

    cv2.imshow("Line Following Detection", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
# ser.close()
cv2.destroyAllWindows()
    
