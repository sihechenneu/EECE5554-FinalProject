from pathlib import Path
from typing import List, Tuple, Optional

import numpy as np

_WORKSPACE_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_WEIGHTS = Path(__file__).resolve().parent / "best.pt"


def _ensure_yolo():
    try:
        from ultralytics import YOLO
        return YOLO
    except ImportError:
        raise ImportError("ultralytics is required for detector. Install with: pip install ultralytics")


def run_detection(
    image: np.ndarray,
    weights: Optional[str] = None,
    conf: float = 0.35,
    iou: float = 0.45,
    imgsz: int = 640,
    device: Optional[str] = None,
    model: Optional[object] = None,
) -> Tuple[List[dict], Optional[object]]:
    """
    Run YOLO detection on a single BGR image.

    Args:
        image: BGR numpy array (H, W, 3), uint8.
        weights: Path to .pt weights file. Defaults to project best.pt.
        conf: Confidence threshold.
        iou: IOU threshold for NMS.
        imgsz: Input size for the model.
        device: Device string (e.g. 'cuda:0', 'cpu') or None for auto.
        model: Optional pre-loaded YOLO model; if given, weights is ignored and this is reused.

    Returns:
        (detections, model): detections is a list of dicts with keys:
            'class_id', 'class_name', 'confidence', 'bbox_xyxy' (x1,y1,x2,y2).
        model is the YOLO model instance.
    """
    if model is not None:
        pass  # reuse provided model
    else:
        YOLO = _ensure_yolo()
        weights = weights or str(DEFAULT_WEIGHTS)
        model = YOLO(weights)
    results = model.predict(
        image,
        conf=conf,
        iou=iou,
        imgsz=imgsz,
        device=device or None,
        verbose=False,
    )
    detections = []
    r = results[0]
    if r.boxes is not None and len(r.boxes):
        for box in r.boxes:
            cls_id = int(box.cls[0])
            detections.append({
                "class_id": cls_id,
                "class_name": model.names[cls_id],
                "confidence": float(box.conf[0]),
                "bbox_xyxy": box.xyxy[0].tolist(),
            })
    return detections, model
