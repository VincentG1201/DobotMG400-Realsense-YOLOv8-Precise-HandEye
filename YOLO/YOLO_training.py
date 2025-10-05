from ultralytics import YOLO

model = YOLO('yolov8n-obb.pt')

model.train(data="test1.yaml", epochs=50)

model.val()