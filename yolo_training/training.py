from ultralytics import YOLO

# 加载预训练 YOLOv8 模型
model = YOLO('yolov8n.pt')  # 你可以尝试不同的模型，如 yolov8s.pt（较大的模型）

# 开始训练
model.train(
    data='data.yaml',  # 替换为你的 data.yaml 文件路径
    epochs=150,                 # 训练轮数，可以根据需要调整
    imgsz=(640, 480),           # 保持原始图像尺寸
    batch=16,                   # 批次大小，视显存情况可调整为 32
    workers=8,                  # 加速数据加载的线程数
    device=0                    # 使用 GPU 进行训练（NVIDIA 4080），默认为 device 0
)
