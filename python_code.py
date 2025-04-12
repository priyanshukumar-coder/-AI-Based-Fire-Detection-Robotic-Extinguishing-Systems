import cv2
import torch
import serial
import time
import numpy as np
from PIL import Image

# Initialize serial connection to Arduino
arduino = serial.Serial('COM8', 9600, timeout=1)  # Change COM port as needed
time.sleep(2)  # Allow time for connection to establish

# Load the PyTorch fire detection model
model = torch.hub.load('ultralytics/yolov5', 'custom', path='dj.pt')  # Your custom .pt model
model.conf = 0.5  # Confidence threshold

# Initialize camera
cap = cv2.VideoCapture(0)  # Use 0 for default camera

# Camera parameters (adjust based on your setup)
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FOV_HORIZONTAL = 60  # degrees
FOV_VERTICAL = 45    # degrees

# Arm parameters (adjust based on your mechanical arm)
ARM_RANGE_X = 1000   # mm
ARM_RANGE_Y = 1000   # mm
ARM_RANGE_Z = 500    # mm

# Buzzer control variables
last_fire_detection_time = 0
buzzer_timeout = 1.0  # seconds to keep buzzer on after last detection

def pixel_to_arm_coordinates(x_pixel, y_pixel, confidence):
    """
    Convert pixel coordinates to arm coordinates
    """
    # Normalize pixel coordinates to [-1, 1] range
    x_normalized = (x_pixel - CAMERA_WIDTH/2) / (CAMERA_WIDTH/2)
    y_normalized = -(y_pixel - CAMERA_HEIGHT/2) / (CAMERA_HEIGHT/2)
    
    # Convert to angles (simplified approximation)
    x_angle = x_normalized * (FOV_HORIZONTAL/2)
    y_angle = y_normalized * (FOV_VERTICAL/2)
    
    # Convert angles to arm coordinates (this is a simplified mapping)
    # In a real system, you'd need proper kinematic calculations
    x_arm = int(ARM_RANGE_X/2 + x_normalized * ARM_RANGE_X/2)
    y_arm = int(ARM_RANGE_Y/2 + y_normalized * ARM_RANGE_Y/2)
    
    # Z coordinate could be based on flame size or fixed
    z_arm = int(ARM_RANGE_Z * (1 - confidence))  # Closer for more confident detections
    
    return x_arm, y_arm, z_arm

def send_to_arduino(x, y, z, fire_detected):
    """
    Send coordinates and buzzer command to Arduino in format "X1234Y5678Z900B1"
    B1 = buzzer on, B0 = buzzer off
    """
    buzzer_cmd = 1 if fire_detected else 0
    command = f"X{x}Y{y}Z{z}B{buzzer_cmd}\n"
    arduino.write(command.encode())
    response = arduino.readline().decode().strip()
    print(f"Sent: {command.strip()} | Received: {response}")
    return response == "ACK"

def process_frame(frame):
    """
    Process a single frame for fire detection
    """
    # Convert to RGB (YOLOv5 expects RGB)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Perform inference
    results = model(rgb_frame)
    
    # Parse results
    detections = results.pandas().xyxy[0]
    fire_detections = detections[detections['name'] == 'fire']
    
    return fire_detections

def main():
    try:
        fire_detected = False
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame")
                break
                
            # Detect fire
            fire_detections = process_frame(frame)
            current_time = time.time()
            
            # Reset fire detection status if timeout has passed
            if fire_detected and (current_time - last_fire_detection_time > buzzer_timeout):
                fire_detected = False
                send_to_arduino(0, 0, 0, False)  # Send buzzer off command
            
            # Process detections
            for _, detection in fire_detections.iterrows():
                x1, y1, x2, y2 = int(detection['xmin']), int(detection['ymin']), int(detection['xmax']), int(detection['ymax'])
                confidence = detection['confidence']
                
                # Draw bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, f"Fire: {confidence:.2f}", (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                # Calculate center of detection
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                
                # Convert to arm coordinates
                x_arm, y_arm, z_arm = pixel_to_arm_coordinates(center_x, center_y, confidence)
                
                # Update fire detection status
                fire_detected = True
                last_fire_detection_time = current_time
                
                # Send to Arduino with buzzer command
                if send_to_arduino(x_arm, y_arm, z_arm, True):
                    print(f"Moving arm to: X={x_arm}, Y={y_arm}, Z={z_arm} | Buzzer ON")
                
                # Draw center point
                cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
            
            # Display frame
            cv2.imshow('Fire Detection', frame)
            
            # Exit on 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("Program stopped by user")
    finally:
        # Ensure buzzer is turned off when exiting
        send_to_arduino(0, 0, 0, False)
        cap.release()
        cv2.destroyAllWindows()
        arduino.close()
        print("Resources released")

if __name__ == "__main__":
    main()