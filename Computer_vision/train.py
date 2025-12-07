from ultralytics import YOLO

# Load a pretrained YOLO model (choose one)
model = YOLO("yolov8s.pt")   # small, good balance
# model = YOLO("yolov8n.pt") # fastest
# model = YOLO("yolov8m.pt") # higher accuracy

# Train the model
model.train(
    data="/home/student10/Robotics_Systems_Design_Project/rgbd_vision/BlockClassifier.v4-objectclassification_v3.yolov8/data.yaml",  # path to your data.yaml
    epochs=50,                     # increase for more accuracy
    imgsz=640,                      # image size
    batch=16,                       # adjust based on GPU
    device="cpu",                       # device=0 for GPU, device='cpu' for CPU
    workers=4,                      # dataloader workers
)

# Evaluate on the test set
model.val()