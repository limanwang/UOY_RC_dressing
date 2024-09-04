from ultralytics import YOLO

# Load a model
# model = YOLO("yolov8x-pose-p6.pt")  # load an official model
model = YOLO("yolov8n-pose.pt")  # load an official model
# model = YOLO("path/to/best.pt")  # load a custom model

# Predict with the model
# results = model("rgb_hand3.png")  # predict on an image

results = model(["rgb_hand3.png"])  
for result in results:
    boxes = result.boxes  # Boxes object for bounding box outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    obb = result.obb  # Oriented boxes object for OBB outputs
    result.show()  # display to screen
    result.save(filename="rgb_hand3_pose.jpg")  # save to disk
