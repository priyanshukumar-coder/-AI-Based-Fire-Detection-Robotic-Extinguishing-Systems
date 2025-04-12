import cv2
import torch

# Load the model
model = torch.hub.load('ultralytics/yolov5', 'custom', path='dj.pt', force_reload=True)

# Set device
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model.to(device)

# Open video stream (0 for webcam, or path to video file)
cap = cv2.VideoCapture(0)  # or 'video.mp4'

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Run detection
    results = model(frame)

    # Render results
    annotated_frame = results.render()[0]

    # Show the frame
    cv2.imshow('Flame Detection', annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
