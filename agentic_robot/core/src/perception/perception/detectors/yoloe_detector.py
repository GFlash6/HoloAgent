"""
YOLO-E 2D Detector for G1 Perception.

Reference: dimos/dimos/perception/detection/detectors/yoloe.py
Supports:
- LRPC (Label-Relevant Prompt-Free Classification) mode: Zero-shot open-vocabulary
- PROMPT mode: Text/Visual prompt-based detection
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any, Optional
import threading
import os

import numpy as np
from numpy.typing import NDArray

if TYPE_CHECKING:
    from ultralytics import YOLOE
    from ultralytics.engine.results import Results


class YoloePromptMode(Enum):
    """YOLO-E prompt modes for open-vocabulary detection."""

    # Label-Relevant Prompt-Free Classification (default, zero-shot)
    LRPC = "lrpc"
    PROMPT = "prompt"  # Text/Visual prompt mode


@dataclass
class Detection2DResult:
    """2D detection result."""

    class_id: int
    class_name: str
    confidence: float
    bbox: tuple[int, int, int, int]  # x_min, y_min, x_max, y_max
    track_id: Optional[int] = None
    mask: Optional[NDArray[np.uint8]] = None

    def bbox_area(self) -> float:
        """Calculate bounding box area."""
        x_min, y_min, x_max, y_max = self.bbox
        return float((x_max - x_min) * (y_max - y_min))

    def center(self) -> tuple[float, float]:
        """Get bounding box center (cx, cy)."""
        x_min, y_min, x_max, y_max = self.bbox
        return ((x_min + x_max) / 2, (y_min + y_max) / 2)


@dataclass
class ImageDetections2D:
    """Container for 2D detections on an image."""

    detections: list[Detection2DResult] = field(default_factory=list)
    image_width: int = 0
    image_height: int = 0
    timestamp: Optional[float] = None

    def filter_by_area_ratio(self, max_ratio: float) -> "ImageDetections2D":
        """Filter detections by bounding box area ratio."""
        if max_ratio is None:
            return self

        image_area = self.image_width * self.image_height
        if image_area <= 0:
            return self

        filtered = [
            det for det in self.detections
            if (det.bbox_area() / image_area) <= max_ratio
        ]
        return ImageDetections2D(
            detections=filtered,
            image_width=self.image_width,
            image_height=self.image_height,
            timestamp=self.timestamp
        )

    def filter_by_class_ids(
            self,
            exclude_ids: set[int]) -> "ImageDetections2D":
        """Filter out detections by class IDs."""
        if not exclude_ids:
            return self

        filtered = [
            det for det in self.detections
            if det.class_id not in exclude_ids
        ]
        return ImageDetections2D(
            detections=filtered,
            image_width=self.image_width,
            image_height=self.image_height,
            timestamp=self.timestamp
        )


class Yoloe2DDetector:
    """
    YOLO-E 2D Detector with open-vocabulary support.

    Supports two modes:
    - LRPC: Zero-shot open-vocabulary detection using class-agnostic classifier
    - PROMPT: Text or visual prompt-based detection
    """

    def __init__(
        self,
        model_dir: str = "models_yoloe",
        model_name_lrpc: str = "yoloe-11l-seg-pf.pt",
        model_name_prompt: str = "yoloe-11l-seg.pt",
        device: str = "auto",
        prompt_mode: YoloePromptMode = YoloePromptMode.LRPC,
        exclude_class_ids: Optional[list[int]] = None,
        max_area_ratio: Optional[float] = 0.3,
        text_prompts: Optional[list[str]] = None,
        conf_threshold: float = 0.6,
        iou_threshold: float = 0.6,
    ) -> None:
        """
        Initialize YOLO-E detector.

        Args:
            model_dir: Directory to store/load models.
            model_name_lrpc: Model filename for LRPC mode.
            model_name_prompt: Model filename for PROMPT mode.
            device: Device to run inference ('cuda', 'cpu', 'auto').
            prompt_mode: Detection mode (LRPC or PROMPT).
            exclude_class_ids: Class IDs to filter out.
            max_area_ratio: Maximum bbox area ratio (0-1).
            text_prompts: Text prompts for PROMPT mode.
            conf_threshold: Confidence threshold.
            iou_threshold: IOU threshold for NMS.
        """
        self.model_dir = model_dir
        self.model_name_lrpc = model_name_lrpc
        self.model_name_prompt = model_name_prompt
        self.prompt_mode = prompt_mode
        self.exclude_class_ids = set(
            exclude_class_ids) if exclude_class_ids else set()
        self.max_area_ratio = max_area_ratio
        self.text_prompts = text_prompts or []
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold

        # Visual prompts storage
        self._visual_prompts: Optional[dict[str, NDArray[Any]]] = None

        # Thread safety
        self._lock = threading.Lock()

        # Determine device
        self.device = self._auto_select_device(device)

        # Load model
        self._load_model()

    def _auto_select_device(self, device: str) -> str:
        """Auto-select device based on availability."""
        if device != "auto":
            return device
        try:
            import torch
            if torch.cuda.is_available():
                return "cuda"
        except ImportError:
            pass
        return "cpu"

    def _get_model_name(self) -> str:
        """Get model name based on prompt mode."""
        if self.prompt_mode == YoloePromptMode.LRPC:
            return self.model_name_lrpc
        return self.model_name_prompt

    def _get_model_path(self) -> str:
        """Get full model path, downloading if necessary."""
        model_name = self._get_model_name()
        model_file = os.path.join(self.model_dir, model_name)

        # Check if model exists
        if os.path.exists(model_file):
            return model_file

        # Create model directory if needed
        os.makedirs(self.model_dir, exist_ok=True)

        # Download model using ultralytics
        from ultralytics import YOLOE
        print(
            f"[Yoloe2DDetector] Downloading model {model_name} to {self.model_dir}...")
        YOLOE(model_name)  # This downloads the model
        print(f"[Yoloe2DDetector] Model downloaded successfully.")

        return model_file

    def _load_model(self) -> None:
        """Load YOLO-E model."""
        model_path = self._get_model_path()

        try:
            from ultralytics import YOLOE
            self.model: "YOLOE" = YOLOE(model_path)

            # Initialize prompts
            if self.prompt_mode == YoloePromptMode.PROMPT:
                if self.text_prompts:
                    self.set_text_prompts(self.text_prompts)
                else:
                    self.set_text_prompts(["nothing"])

            print(f"[Yoloe2DDetector] Model loaded from {model_path}")
        except Exception as e:
            raise RuntimeError(f"Failed to load YOLO-E model: {e}")

    def set_prompts(
        self,
        text: Optional[list[str]] = None,
        bboxes: Optional[NDArray[np.float64]] = None,
    ) -> None:
        """
        Set prompts for detection.

        Provide either text or bboxes, not both.
        """
        if text is not None and bboxes is not None:
            raise ValueError("Provide either text or bboxes, not both.")
        if text is None and bboxes is None:
            raise ValueError("Must provide either text or bboxes.")

        with self._lock:
            self.model.predictor = None
            if text is not None:
                self.model.set_classes(
                    text,
                    self.model.get_text_pe(text)
                )
                self._visual_prompts = None
            else:
                cls = np.arange(len(bboxes), dtype=np.int16)
                self._visual_prompts = {"bboxes": bboxes, "cls": cls}

    def set_text_prompts(self, class_names: list[str]) -> None:
        """Set text prompts for open-vocabulary detection."""
        self.set_prompts(text=class_names)

    def set_visual_prompts(self, bboxes: NDArray[np.float64]) -> None:
        """Set visual prompts for detection."""
        self.set_prompts(bboxes=bboxes)

    def process_image(
        self,
        image: np.ndarray,
        image_width: int,
        image_height: int,
        conf: Optional[float] = None,
        iou: Optional[float] = None,
    ) -> ImageDetections2D:
        """
        Process an image and return detection results.

        Args:
            image: Input image in BGR format.
            image_width: Image width.
            image_height: Image height.
            conf: Confidence threshold (uses default if None).
            iou: IOU threshold (uses default if None).

        Returns:
            ImageDetections2D containing detected objects.
        """
        if conf is None:
            conf = self.conf_threshold
        if iou is None:
            iou = self.iou_threshold

        track_kwargs: dict[str, Any] = {
            "source": image,
            "device": self.device,
            "conf": conf,
            "iou": iou,
            "persist": True,
            "verbose": False,
        }

        with self._lock:
            if self._visual_prompts is not None:
                track_kwargs["visual_prompts"] = self._visual_prompts

            results: list["Results"] = self.model.track(**track_kwargs)

        detections = self._extract_detections(
            results, image_width, image_height)
        detections = self._apply_filters(detections)

        return detections

    def _extract_detections(
        self,
        results: list["Results"],
        image_width: int,
        image_height: int
    ) -> ImageDetections2D:
        """Extract detections from YOLO-E results."""
        detections: list[Detection2DResult] = []

        if not results or len(results) == 0:
            return ImageDetections2D(
                detections=[],
                image_width=image_width,
                image_height=image_height)

        result = results[0]

        if result.boxes is None:
            return ImageDetections2D(
                detections=[],
                image_width=image_width,
                image_height=image_height)

        boxes = result.boxes
        num_detections = len(boxes)

        for i in range(num_detections):
            try:
                box = boxes[i]

                xyxy = box.xyxy[0].cpu().numpy()
                x_min, y_min, x_max, y_max = map(int, xyxy)

                class_id = int(box.cls[0].cpu().numpy())
                confidence = float(box.conf[0].cpu().numpy())

                class_name = result.names[class_id] if hasattr(
                    result, 'names') else str(class_id)

                track_id = None
                if hasattr(box, 'track_id') and box.track_id is not None:
                    track_id = int(box.track_id[0].cpu().numpy())

                mask = None
                if result.masks is not None and i < len(result.masks):
                    mask_data = result.masks.data[i].cpu().numpy()
                    mask = (mask_data > 0.5).astype(np.uint8)

                detections.append(Detection2DResult(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=confidence,
                    bbox=(x_min, y_min, x_max, y_max),
                    track_id=track_id,
                    mask=mask,
                ))
            except Exception:
                continue

        return ImageDetections2D(
            detections=detections,
            image_width=image_width,
            image_height=image_height
        )

    def _apply_filters(
            self,
            detections: ImageDetections2D) -> ImageDetections2D:
        """Apply filtering to detections."""
        if self.exclude_class_ids:
            detections = detections.filter_by_class_ids(self.exclude_class_ids)

        if self.max_area_ratio is not None:
            detections = detections.filter_by_area_ratio(self.max_area_ratio)

        return detections

    def stop(self) -> None:
        """Clean up resources."""
        if hasattr(
                self.model,
                "predictor") and self.model.predictor is not None:
            predictor = self.model.predictor
            if hasattr(predictor, "trackers") and predictor.trackers:
                for tracker in predictor.trackers:
                    if hasattr(
                            tracker, "tracker") and hasattr(
                            tracker.tracker, "gmc"):
                        gmc = tracker.tracker.gmc
                        if hasattr(
                                gmc, "executor") and gmc.executor is not None:
                            gmc.executor.shutdown(wait=True)
            self.model.predictor = None
