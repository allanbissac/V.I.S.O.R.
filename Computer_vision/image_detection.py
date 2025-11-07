import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import torch

# Force YOLOv8 to use CPU
device = 'cpu'

# Load the YOLOv8 nano model
model = YOLO('yolov8n.pt')
model.to(device)

# Initialize RealSense
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipe.start(cfg)

try:
    while True:
        frames = pipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        # YOLOv8 expects BGR images, input as list for batch
        results = model.predict(source=[color_image], device=device, verbose=False)

        # Plot results on image
        annotated = results[0].plot()

        # Display
        cv2.imshow('YOLOv8 RealSense CPU', annotated)

        if cv2.waitKey(1) == ord('q'):
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()
