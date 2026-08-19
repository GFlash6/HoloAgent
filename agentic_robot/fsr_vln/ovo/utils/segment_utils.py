from __future__ import annotations
from typing import Dict, Any, Tuple, List

import torchvision.transforms.functional as F
from copy import deepcopy
import numpy as np
import heapq
import torch
import os
import base64
import io
import json
from pathlib import Path
from urllib import request


def mask2segmap(masks: np.ndarray, image: np.ndarray,
                sort: bool = True) -> Tuple[np.ndarray, np.ndarray]:
    if sort:
        masks = heapq.nlargest(
            len(masks),
            masks,
            key=lambda x: x['stability_score'])

    seg_map = -np.ones(image.shape[:2], dtype=np.int32)
    binary_maps = []
    for i, mask in enumerate(masks):
        binary_maps.append(mask['segmentation'])
        seg_map_mask = mask['segmentation'].copy()
        if sort:
            mask_overlap = np.logical_and(seg_map > -1, seg_map_mask)
            # previous masks have higher stability score, if there is overlap,
            # current mask is removed
            seg_map_mask[mask_overlap] = False
        # If seg map mask .sum() == 0 it won't assign anything.
        seg_map[seg_map_mask] = i
    # TODO: If masks is empty np.stack will raise an error
    binary_maps = np.stack(binary_maps)
    return seg_map, binary_maps


def segmap2segimg(
        binary_map: torch.Tensor,
        image: torch.Tensor,
        also_bbox: bool,
        bbox_margin: int = 50,
        out_l: int = 224) -> torch.Tensor:
    seg_imgs = []

    bboxes_xyxy = batched_mask_to_box(binary_map)
    bboxes_xyhw = batched_box_xyxy_to_xywh(bboxes_xyxy)
    # Filter out invalid bboxes (h or w <= 0)
    for i in range(binary_map.shape[0]):
        x, y, w, h = bboxes_xyhw[i]
        # Skip invalid bboxes
        if h <= 0 or w <= 0:
            # Create a default black image for invalid bbox
            if also_bbox:
                default_img = torch.zeros(
                    (6, out_l, out_l), dtype=image.dtype, device=image.device)
            else:
                default_img = torch.zeros(
                    (3, out_l, out_l), dtype=image.dtype, device=image.device)
            seg_imgs.append(default_img)
        else:
            padded_img = seg_img_from_image(
                binary_map[i],
                bboxes_xyhw[i],
                image,
                also_bbox,
                bbox_margin,
                out_l)
            seg_imgs.append(padded_img)

    if len(seg_imgs) > 0:
        seg_imgs = torch.stack(seg_imgs, axis=0)  # b,3,H,W
    else:
        # If no valid images, return empty tensor with correct shape
        if also_bbox:
            seg_imgs = torch.zeros(
                (0, 6, out_l, out_l), dtype=image.dtype, device=image.device)
        else:
            seg_imgs = torch.zeros(
                (0, 3, out_l, out_l), dtype=image.dtype, device=image.device)

    return seg_imgs


def batched_mask_to_box(masks: torch.Tensor) -> torch.Tensor:
    """
    From SAM.utils.amg code Calculates boxes in XYXY format around masks.

    Return [0,0,0,0] for an empty mask. For input shape C1xC2x...xHxW, the
    output shape is C1xC2x...x4.
    """
    # torch.max below raises an error on empty inputs, just skip in this case
    if torch.numel(masks) == 0:
        return torch.zeros(*masks.shape[:-2], 4, device=masks.device)

    # Normalize shape to CxHxW
    shape = masks.shape
    h, w = shape[-2:]
    if len(shape) > 2:
        masks = masks.flatten(0, -3)
    else:
        masks = masks.unsqueeze(0)

    # Get top and bottom edges
    in_height, _ = torch.max(masks, dim=-1)
    in_height_coords = in_height *\
        torch.arange(h, device=in_height.device)[None, :]
    bottom_edges, _ = torch.max(in_height_coords, dim=-1)
    in_height_coords = in_height_coords + h * (~in_height)
    top_edges, _ = torch.min(in_height_coords, dim=-1)

    # Get left and right edges
    in_width, _ = torch.max(masks, dim=-2)
    in_width_coords = in_width *\
        torch.arange(w, device=in_width.device)[None, :]
    right_edges, _ = torch.max(in_width_coords, dim=-1)
    in_width_coords = in_width_coords + w * (~in_width)
    left_edges, _ = torch.min(in_width_coords, dim=-1)

    # If the mask is empty the right edge will be to the left of the left edge.
    # Replace these boxes with [0, 0, 0, 0]
    empty_filter = (right_edges < left_edges) | (bottom_edges < top_edges)
    out = torch.stack(
        [left_edges, top_edges, right_edges, bottom_edges], dim=-1)
    out = out * (~empty_filter).unsqueeze(-1)

    # Return to original shape
    if len(shape) > 2:
        out = out.reshape(*shape[:-2], 4)
    else:
        out = out[0]

    return out


def batched_box_xyxy_to_xywh(box_xyxy: torch.Tensor) -> torch.Tensor:
    """From SAM.utils.amg code."""
    box_xywh = box_xyxy  # deepcopy(box_xyxy)
    box_xywh[:, 2] = box_xywh[:, 2] - box_xywh[:, 0]
    box_xywh[:, 3] = box_xywh[:, 3] - box_xywh[:, 1]
    return box_xywh


def segmap2bboximg(binary_map: torch.Tensor,
                   image: torch.Tensor,
                   bbox_margin: int = 50,
                   out_l: int = 224) -> Tuple[torch.Tensor,
                                              torch.Tensor]:
    seg_imgs = []
    bmaps = []
    if len(binary_map) > 0:
        bboxes_xyxy = batched_mask_to_box(binary_map)
        bboxes_xyhw = batched_box_xyxy_to_xywh(bboxes_xyxy)
        # Filter out invalid bboxes (h or w <= 0)
        for i in range(binary_map.shape[0]):
            x, y, w, h = bboxes_xyhw[i]
            # Skip invalid bboxes
            if h <= 0 or w <= 0:
                # Create default black images for invalid bbox
                default_img = torch.zeros(
                    (3, out_l, out_l), dtype=image.dtype, device=image.device)
                default_mask = torch.zeros(
                    (1, out_l, out_l), dtype=binary_map.dtype, device=binary_map.device)
                seg_imgs.append(default_img)
                bmaps.append(default_mask)
            else:
                padded_img, bmap = bbox_img_from_image(
                    binary_map[i], bboxes_xyhw[i], image, bbox_margin, out_l)
                seg_imgs.append(padded_img)
                bmaps.append(bmap)

        if len(seg_imgs) > 0:
            seg_imgs = torch.stack(seg_imgs, axis=0)  # b,3,H,W
            bmaps = torch.stack(bmaps, axis=0)
        else:
            # If no valid images, return empty tensors with correct shape
            seg_imgs = torch.zeros(
                (0, 3, out_l, out_l), dtype=image.dtype, device=image.device)
            bmaps = torch.zeros(
                (0, 1, out_l, out_l), dtype=binary_map.dtype, device=binary_map.device)

    return seg_imgs, bmaps


def bbox_img_from_image(mask: torch.Tensor,
                        bbox: torch.Tensor,
                        image: torch.Tensor,
                        bbox_margin: int = 50,
                        size: int = 224) -> Tuple[torch.Tensor,
                                                  torch.Tensor]:
    x, y, w, h = bbox
    # 检查 bbox 是否有效（高度和宽度必须大于0）
    if h <= 0 or w <= 0:
        # 返回默认的黑色图像
        default_img = torch.zeros(
            (3, size, size), dtype=image.dtype, device=image.device)
        default_mask = torch.zeros(
            (1, size, size), dtype=mask.dtype, device=mask.device)
        return default_img, default_mask

    bbox_img_raw = get_bbox_img(bbox, image, bbox_margin)
    bbox_mask_raw = get_bbox_img(bbox, mask[None], bbox_margin)

    # 检查提取的图像尺寸是否有效
    if bbox_img_raw.shape[1] <= 0 or bbox_img_raw.shape[2] <= 0:
        default_img = torch.zeros(
            (3, size, size), dtype=image.dtype, device=image.device)
        default_mask = torch.zeros(
            (1, size, size), dtype=mask.dtype, device=mask.device)
        return default_img, default_mask

    bbox_img = F.resize(bbox_img_raw, (size, size), antialias=True)
    bmap = F.resize(bbox_mask_raw, (size, size), antialias=True)
    return bbox_img, bmap


def seg_img_from_image(
        mask: torch.Tensor,
        bbox: torch.Tensor,
        image: torch.Tensor,
        also_bbox: bool,
        bbox_margin: int = 50,
        size: int = 224) -> torch.Tensor:
    x, y, w, h = bbox
    # 检查 bbox 是否有效（高度和宽度必须大于0）
    if h <= 0 or w <= 0:
        # 返回一个默认的黑色图像
        if also_bbox:
            default_img = torch.zeros(
                (6, size, size), dtype=image.dtype, device=image.device)
        else:
            default_img = torch.zeros(
                (3, size, size), dtype=image.dtype, device=image.device)
        return default_img

    seg_img = get_seg_img(mask, bbox, image)
    if also_bbox:
        bbox_img = F.resize(
            get_bbox_img(
                bbox,
                image,
                bbox_margin),
            (size,
             size),
            antialias=True)
        padded_img = torch.concatenate(
            [F.resize(seg_img, (size, size)), bbox_img], axis=0)
    else:
        padded_img = F.resize(pad_img(seg_img), (size, size), antialias=True)
    return padded_img


def get_seg_img(
        mask: torch.Tensor,
        bbox: torch.Tensor,
        image: torch.Tensor) -> torch.Tensor:
    x, y, w, h = bbox
    seg_img = torch.zeros((3, h, w), dtype=image.dtype, device=image.device)
    seg_img[:, mask[y:y + h, x:x + w]] = image[..., y:y +
                                               h, x:x + w][:, mask[y:y + h, x:x + w]].clone()
    return seg_img


def get_bbox_img(bbox: Tuple[int,
                             int,
                             int,
                             int],
                 image: torch.Tensor,
                 bbox_margin: int) -> torch.Tensor:
    x, y, w, h = increase_bbox_by_margin(bbox, bbox_margin)
    bbox_img = image[..., y:y + h, x:x + w].clone()
    return bbox_img


def pad_img(img: torch.Tensor) -> torch.Tensor:
    c, h, w = img.shape
    biggest_side = max(w, h)
    pad = torch.zeros((c, biggest_side, biggest_side),
                      dtype=img.dtype, device=img.device)
    if h > w:
        pad[..., (h - w) // 2:(h - w) // 2 + w] = img
    else:
        pad[:, (w - h) // 2:(w - h) // 2 + h, :] = img
    return pad


def increase_bbox_by_margin(
        bbox: Tuple[int, int, int, int], margin: int) -> Tuple[int, int, int, int]:
    """
    # Functino from https://github.com/hovsg/HOV-
    SG/blob/main/hovsg/utils/sam_utils.py Increases the size of a bounding box
    by the given margin.

    :param bbox: The bounding box coordinates in XYWH format as a tuple of (x, y, w, h).
    :param margin: The margin to increase the bounding box size by in pixels.
    :return: The increased bounding box coordinates as a tuple of (x, y, w, h).
    """
    x, y, w, h = bbox
    x -= margin
    y -= margin
    w += margin * 2
    h += margin * 2
    # Check if x is negative
    if x < 0:
        w += x
        x = 0

    # Check if y is negative
    if y < 0:
        h += y
        y = 0
    return (x, y, w, h)


def masks_update(*args, **kwargs) -> Tuple[np.ndarray]:
    # remove redundant masks based on the scores and overlap rate between masks
    masks_new = ()
    for masks_lvl in (args):
        seg_pred = torch.from_numpy(
            np.stack([m['segmentation'] for m in masks_lvl], axis=0))
        iou_pred = torch.from_numpy(
            np.stack([m['predicted_iou'] for m in masks_lvl], axis=0))
        stability = torch.from_numpy(
            np.stack([m['stability_score'] for m in masks_lvl], axis=0))

        scores = stability * iou_pred
        keep_mask_nms = mask_nms(seg_pred, scores, **kwargs)
        masks_lvl = filter(keep_mask_nms, masks_lvl)

        masks_new += (masks_lvl,)
    return masks_new


def filter(keep: torch.Tensor, masks_result) -> List[np.ndarray]:
    keep = keep.int().cpu().numpy()
    result_keep = []
    for i, m in enumerate(masks_result):
        if i in keep:
            result_keep.append(m)
    return result_keep


def mask_nms(
        masks: torch.Tensor,
        scores: torch.Tensor,
        iou_thr: float = 0.7,
        score_thr: float = 0.1,
        inner_thr: float = 0.2,
        **kwargs) -> torch.Tensor:
    """
    Perform mask non-maximum suppression (NMS) on a set of masks based on their
    scores.

    Args:
        masks (torch.Tensor): has shape (num_masks, H, W)
        scores (torch.Tensor): The scores of the masks, has shape (num_masks,)
        iou_thr (float, optional): The threshold for IoU.
        score_thr (float, optional): The threshold for the mask scores.
        inner_thr (float, optional): The threshold for the overlap rate.
        **kwargs: Additional keyword arguments.
    Returns:
        selected_idx (torch.Tensor): A tensor representing the selected indices of the masks after NMS.
    """
    if masks.shape[0] == 0:
        return torch.empty(0, dtype=torch.long, device=masks.device)

    scores, idx = scores.sort(0, descending=True)
    num_masks = idx.shape[0]

    masks_ord = masks[idx.view(-1), :]
    masks_area = torch.sum(masks_ord, dim=(1, 2), dtype=torch.float)

    iou_matrix = torch.zeros((num_masks,) *
                             2, dtype=torch.float, device=masks.device)
    inner_iou_matrix = torch.zeros(
        (num_masks,) * 2, dtype=torch.float, device=masks.device)
    for i in range(num_masks):
        for j in range(i, num_masks):
            intersection = torch.sum(
                torch.logical_and(
                    masks_ord[i],
                    masks_ord[j]),
                dtype=torch.float)
            union = torch.sum(
                torch.logical_or(
                    masks_ord[i],
                    masks_ord[j]),
                dtype=torch.float)
            iou = intersection / union
            iou_matrix[i, j] = iou
            # select mask pairs that may have a severe internal relationship
            if intersection /\
                    masks_area[i] < 0.5 and intersection / masks_area[j] >= 0.85:
                inner_iou = 1 -\
                    (intersection / masks_area[j]) * \
                    (intersection / masks_area[i])
                inner_iou_matrix[i, j] = inner_iou
            if intersection /\
                    masks_area[i] >= 0.85 and intersection / masks_area[j] < 0.5:
                inner_iou = 1 -\
                    (intersection / masks_area[j]) * \
                    (intersection / masks_area[i])
                inner_iou_matrix[j, i] = inner_iou

    iou_matrix.triu_(diagonal=1)
    iou_max, _ = iou_matrix.max(dim=0)
    inner_iou_matrix_u = torch.triu(inner_iou_matrix, diagonal=1)
    inner_iou_max_u, _ = inner_iou_matrix_u.max(dim=0)
    inner_iou_matrix_l = torch.tril(inner_iou_matrix, diagonal=1)
    inner_iou_max_l, _ = inner_iou_matrix_l.max(dim=0)

    keep = iou_max <= iou_thr
    keep_conf = scores > score_thr
    keep_inner_u = inner_iou_max_u <= 1 - inner_thr
    keep_inner_l = inner_iou_max_l <= 1 - inner_thr

    # If there are no masks with scores above threshold, the top 3 masks are
    # selected
    if keep_conf.sum() == 0:
        index = scores.topk(min(3, num_masks)).indices
        keep_conf[index] = True
    if keep_inner_u.sum() == 0:
        index = scores.topk(min(3, num_masks)).indices
        keep_inner_u[index] = True
    if keep_inner_l.sum() == 0:
        index = scores.topk(min(3, num_masks)).indices
        keep_inner_l[index] = True
    keep *= keep_conf
    keep *= keep_inner_u
    keep *= keep_inner_l

    selected_idx = idx[keep]
    return selected_idx


def box_xyxy_to_xywh(box_xyxy: torch.Tensor) -> torch.Tensor:
    """From SAM.utils.amg code."""
    box_xywh = deepcopy(box_xyxy)
    box_xywh[2] = box_xywh[2] - box_xywh[0]
    box_xywh[3] = box_xywh[3] - box_xywh[1]
    return box_xywh


def resolve_sam3_checkpoint(checkpoint_path: str | Path) -> Path:
    checkpoint = Path(checkpoint_path).expanduser()
    if checkpoint.is_dir():
        checkpoint = checkpoint / "sam3.pt"
    if not checkpoint.is_file():
        raise FileNotFoundError(f"SAM3 checkpoint not found: {checkpoint}")
    return checkpoint


class Sam3TextMaskGenerator:
    """Expose SAM3 text-prompt masks through the SAM automatic-mask API."""

    def __init__(self, config: Dict[str, Any], device: str) -> None:
        prompts = config.get("sam_prompts", [])
        if not isinstance(prompts, list) or not all(
                isinstance(prompt, str) and prompt.strip() for prompt in prompts):
            raise ValueError("SAM3 requires a non-empty sam_prompts string list")
        if not prompts:
            raise ValueError("SAM3 requires at least one text prompt")

        checkpoint = resolve_sam3_checkpoint(
            Path(os.path.expandvars(config["sam_ckpt_path"])))
        from sam3.model_builder import build_sam3_image_model
        from sam3.model.sam3_image_processor import Sam3Processor

        self.model = build_sam3_image_model(
            checkpoint_path=str(checkpoint),
            load_from_HF=False,
            device=device,
            eval_mode=True)
        self.model.to(device)
        self.processor = Sam3Processor(
            self.model,
            device=device,
            confidence_threshold=config.get("confidence_threshold", 0.5))
        self.prompts = prompts
        self.min_mask_region_area = config.get("min_mask_region_area", 100)

    def generate(self, image: np.ndarray) -> List[Dict[str, Any]]:
        from PIL import Image

        state = self.processor.set_image(Image.fromarray(image))
        masks = []
        for prompt in self.prompts:
            state = self.processor.set_text_prompt(prompt, state)
            for segmentation, score, box in zip(
                    state["masks"][:, 0], state["scores"], state["boxes"]):
                segmentation = segmentation.cpu().numpy().astype(bool)
                area = int(segmentation.sum())
                if area < self.min_mask_region_area:
                    continue
                score = float(score.cpu())
                x0, y0, x1, y1 = box.cpu().tolist()
                masks.append({
                    "segmentation": segmentation,
                    "area": area,
                    "bbox": [x0, y0, x1 - x0, y1 - y0],
                    "predicted_iou": score,
                    "stability_score": score,
                    "prompt": prompt,
                })
        return masks

    def to(self, device: str) -> None:
        self.model.to(device)
        self.processor.device = device

    def cpu(self) -> None:
        self.to("cpu")

    def cuda(self) -> None:
        self.to("cuda")


class Sam3ServiceMaskGenerator:
    """Use a separately hosted real SAM3 model through its HTTP API."""

    is_remote = True

    def __init__(self, config: Dict[str, Any]) -> None:
        self.url = config.get("service_url", "http://127.0.0.1:8114").rstrip("/")
        prompts = config.get("sam_prompts", [])
        if not isinstance(prompts, list) or not prompts or not all(
                isinstance(prompt, str) and prompt.strip() for prompt in prompts):
            raise ValueError("SAM3 service requires a non-empty sam_prompts string list")
        self.prompts = prompts
        self.score_threshold = float(config.get("confidence_threshold", 0.05))
        self.min_mask_region_area = int(config.get("min_mask_region_area", 100))
        self.max_masks = int(config.get("max_masks", 50))
        self.timeout = float(config.get("service_timeout", 120.0))
        if not 0.0 <= self.score_threshold <= 1.0:
            raise ValueError("confidence_threshold must be between 0 and 1")
        if self.max_masks <= 0 or self.timeout <= 0:
            raise ValueError("max_masks and service_timeout must be positive")

    @staticmethod
    def _encode_image(image: np.ndarray) -> str:
        from PIL import Image

        if image.dtype != np.uint8:
            image = np.clip(image, 0, 255).astype(np.uint8)
        buffer = io.BytesIO()
        Image.fromarray(image).convert("RGB").save(buffer, format="PNG")
        return base64.b64encode(buffer.getvalue()).decode("ascii")

    def generate(self, image: np.ndarray) -> List[Dict[str, Any]]:
        encoded = self._encode_image(image)
        masks = []
        for prompt in self.prompts:
            payload = json.dumps({
                "image_base64": encoded,
                "text_prompt": prompt,
            }).encode("utf-8")
            http_request = request.Request(
                f"{self.url}/segment",
                data=payload,
                headers={"Content-Type": "application/json"},
                method="POST")
            try:
                with request.urlopen(http_request, timeout=self.timeout) as response:
                    result = json.load(response)
            except Exception as exc:
                raise RuntimeError(
                    f"SAM3 service request failed at {self.url}: {exc}") from exc

            results = result.get("results")
            if not isinstance(results, list):
                raise RuntimeError("SAM3 service returned a malformed response")
            for item in results:
                score = float(item["score"])
                if score < self.score_threshold:
                    continue
                shape = tuple(int(value) for value in item["shape"])
                if shape != image.shape[:2]:
                    raise RuntimeError(
                        f"SAM3 mask shape {shape} does not match image {image.shape[:2]}")
                raw = base64.b64decode(item["mask_base64"], validate=True)
                segmentation = np.frombuffer(raw, dtype=np.uint8)
                if segmentation.size != shape[0] * shape[1]:
                    raise RuntimeError("SAM3 service returned an invalid mask payload")
                segmentation = segmentation.reshape(shape).astype(bool)
                area = int(segmentation.sum())
                if area < self.min_mask_region_area:
                    continue
                x0, y0, x1, y1 = (float(value) for value in item["box"])
                masks.append({
                    "segmentation": segmentation,
                    "area": area,
                    "bbox": [x0, y0, x1 - x0, y1 - y0],
                    "predicted_iou": score,
                    "stability_score": score,
                    "prompt": prompt,
                })
        masks.sort(key=lambda item: item["predicted_iou"], reverse=True)
        return masks[:self.max_masks]


def load_sam(config: Dict[str, Any],
             device: str = "cuda") -> SamAutomaticMaskGenerator:
    """Load SAM, SAM2, or text-prompted SAM3."""
    sam_version = config.get("sam_version", "2.1")

    if sam_version == "3-service":
        return Sam3ServiceMaskGenerator(config)
    if sam_version == "3":
        return Sam3TextMaskGenerator(config, device)

    model_cards = {
        "vit_b": "vit_b_01ec64.pth",
        "vit_h": "vit_h_4b8939.pth",
        "hiera_l": "hiera_large.pt",
        "hiera_t": "hiera_tiny.pt"}
    sam_encoder = config.get("sam_encoder", "hiera_l")
    checkpoint_path = os.path.join(
        config["sam_ckpt_path"],
        f"sam{sam_version}_{model_cards[sam_encoder]}")

    if sam_version == "":
        from segment_anything import SamAutomaticMaskGenerator, sam_model_registry

        sam = sam_model_registry[sam_encoder](
            checkpoint=checkpoint_path).to(device).eval()
        sam_config = {
            "points_per_side": config.get("points_per_side", 32),
            "pred_iou_thresh": config.get("nms_iou_th", 0.8),
            "stability_score_thresh": config.get("stability_score_th", 0.85),
            "min_mask_region_area": config.get("min_mask_region_area", 100),
        }
    else:
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
        from sam2.build_sam import build_sam2
        from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator as SamAutomaticMaskGenerator

        model_cfg = os.path.join(
            "configs",
            f"sam{sam_version}",
            f"sam{sam_version}_{sam_encoder}.yaml")
        sam = build_sam2(
            model_cfg,
            checkpoint_path,
            device=device,
            mode="eval",
            apply_postprocessing=False)
        sam_config = {
            "points_per_side": config.get("points_per_side", 32),
            "pred_iou_thresh": config.get("nms_iou_th", 0.8),
            "stability_score_thresh": config.get("stability_score_th", 0.95),
            "min_mask_region_area": config.get("min_mask_region_area", 0),
            "use_m2m": config.get("use_m2m", False),
        }

    mask_generator = SamAutomaticMaskGenerator(
        model=sam,
        **sam_config
    )
    return mask_generator
