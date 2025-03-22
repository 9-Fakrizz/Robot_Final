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
reach_point = False

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
cap = cv2.VideoCapture(0)

def detection_percentage(region):
    return np.sum(region == 255) / region.size * 100

ignore_turns_until = 0  # Timestamp to ignore SCCW and SCW

def followline():
    global checkpoint_1, last_command, allow_mix_turn_point, color_line_state
    global reach_point
    ignore_turns_until = 0  # Timestamp to ignore SCCW and SCW

    while reach_point != True:
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

        # Show results
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

        # Detection of the stop color (black line point)
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
                elif (black_percentage - justblack_percentage) > 5 and allow_mix_turn_point:
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
                
            elif justblack_percentage <= 1 and black_percentage >= 30:
                print("stop color point!")
                command = "stop"
                ser.write((command + "\n").encode())
                reach_point = True
                
            elif left_percentage > right_percentage and last_command != "CW":
                command = "CCW"
            elif right_percentage > left_percentage and last_command != "CCW":
                command = "CW"
            elif center_detected:
                command = "backward"

        # Send command to ESP32
        if command:
            last_command = command
            ser.write((command + "\n").encode())


        # Display detection results
        detected_text = f"Detected: {command} | Left: {left_percentage:.1f}% | Right: {right_percentage:.1f}%"
        cv2.putText(frame, detected_text, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Show the result
        cv2.imshow("Line Following Detection", frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    # ser.close()
    return

# Frame properties
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CENTER_X = FRAME_WIDTH // 2  # X center of the frame
TOLERANCE = 50  # Allowable deviation for centering
SIZE_THRESHOLD = 20000  # Threshold for the object size to determine if the robot should move closer

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
                return True
        else:
            ser.write(b'backward\n')  # Stop if object too small
            print("Object too small, moving forward to find it.")
    else:
        ser.write(b'backward\n')  # Stop if no object detected
        print("No object detected, moving forward.")
    
    # Return False if conditions are not met
    return False

def find_object():
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
    # ser.close()

def send_command(command):
    """Send the command to serial and wait 1 second."""
    print(f"Sending command: {command}")
    ser.write((command + "\n").encode())  # Send command to serial
    time.sleep(4)  # Delay of 1 second

def grap():
    """Function for the 'grap' sequence of commands."""
    commands = [
        "stop",
        "0,220,220,-60",
        "0,260,220,-80",
        "0,300,220,-80",
        "0,340,220,-80",
        "0,340,180,-80",
        "0,340,140,-80",
        "0,340,120,-80",
        "0,340,115,-80",
        "s2,0",
        "0,300,220,-80",
        "0,220,220,-60"
    ]
    
    for command in commands:
        send_command(command)

def release():
    """Function for the 'release' sequence of commands."""
    commands = [
        "0,220,220,-60",
        "0,260,220,-80",
        "0,360,160,-80",
        "0,340,130,-80",
        "s2,180",
        "0,300,220,-80",
        "0,220,220,-60"
    ]
    
    for command in commands:
        send_command(command)

def turnback():
    """Function for the 'release' sequence of commands."""
    commands = [
        "SCCW",
        "stop",
        "SCCW",
        "stop",
        "SCCW",
        "stop"
    ]
    
    for command in commands:
        send_command(command)

def testIK():
    """Function for the 'release' sequence of commands."""
    commands = [
        "0,220,220,-60",
        "0,0,550,90",
        "0,550,0,0",
        "0,100,220,-90",
        "stop"
    ]
    
    for command in commands:
        send_command(command)

if __name__ == "__main__":
    # followline()
    print("reach the color point")
    # find_object()
    # print("found the opject !")
    # # Send the grap commands
    # print("Starting grap function")
    # grap()
    # # release()

    # print("turnback")
    # time.sleep(2)
    # turnback()
    
    # reach_point = False
    # followline()

    testIK()
    # Send the release commands
    # print("Starting release function")
    # release()