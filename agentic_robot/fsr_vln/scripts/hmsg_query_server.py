#!/usr/bin/env python3
"""Serve real HMSG object queries and align results to the live Nav2 map."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException
from omegaconf import OmegaConf
from pydantic import BaseModel


WORKSPACE = Path(__file__).resolve().parents[4]
FSR_ROOT = Path(__file__).resolve().parents[1]
for path in (WORKSPACE, FSR_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from api import FsrVlnClient
from tools.shared_memory_utils import MultiImageReader


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array([
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ], dtype=np.float64)


def quaternion_inverse(quaternion: np.ndarray) -> np.ndarray:
    norm_squared = float(np.dot(quaternion, quaternion))
    if not math.isfinite(norm_squared) or norm_squared < 1e-12:
        raise ValueError("Invalid zero quaternion")
    result = quaternion.astype(np.float64, copy=True)
    result[1:] *= -1
    return result / norm_squared


def quaternion_rotate(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    pure = np.concatenate(([0.0], np.asarray(vector, dtype=np.float64)))
    return quaternion_multiply(
        quaternion_multiply(quaternion, pure), quaternion_inverse(quaternion))[1:]


def sim_point_to_map(
        point_sim: np.ndarray, robot_sim: np.ndarray, robot_map: np.ndarray) -> np.ndarray:
    relative = quaternion_rotate(
        quaternion_inverse(robot_sim[3:]), point_sim - robot_sim[:3])
    return robot_map[:3] + quaternion_rotate(robot_map[3:], relative)


class QueryRequest(BaseModel):
    object_query: str
    robot_map_pose_wxyz: list[float]


class AnchorUpdateRequest(BaseModel):
    object_id: str
    center_map: list[float]
    score: float
    observation_count: int
    source_timestamp_ms: int


class AnchorStore:
    """Small atomic JSON store for refined map-frame object anchors."""

    def __init__(self, path: Path, min_observations: int = 3) -> None:
        self.path = path
        self.min_observations = min_observations
        self._lock = threading.Lock()
        if path.exists():
            document = json.loads(path.read_text(encoding="utf-8"))
            self._anchors = dict(document.get("anchors", {}))
        else:
            self._anchors = {}

    def get(self, object_id: str) -> dict | None:
        with self._lock:
            anchor = self._anchors.get(object_id)
            return None if anchor is None else dict(anchor)

    def update(self, request: AnchorUpdateRequest) -> dict:
        center = np.asarray(request.center_map, dtype=np.float64)
        if center.shape != (3,) or not np.isfinite(center).all():
            raise ValueError("center_map must contain 3 finite numbers")
        if not math.isfinite(request.score):
            raise ValueError("score must be finite")
        if request.observation_count < self.min_observations:
            raise ValueError(
                f"observation_count must be at least {self.min_observations}")
        if request.source_timestamp_ms <= 0:
            raise ValueError("source_timestamp_ms must be positive")
        record = {
            "center_map": center.tolist(),
            "score": request.score,
            "observation_count": request.observation_count,
            "source_timestamp_ms": request.source_timestamp_ms,
            "updated_at_ms": int(time.time() * 1000),
        }
        with self._lock:
            self._anchors[request.object_id] = record
            document = {"version": 1, "anchors": self._anchors}
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix(self.path.suffix + ".tmp")
            temporary.write_text(
                json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8")
            os.replace(temporary, self.path)
        return dict(record)


def create_app(args) -> FastAPI:
    evidence_path = args.scene_root / "runtime_evidence.json"
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    cfg = OmegaConf.create({
        "main": {
            "device": "cuda",
            "use_gpt": False,
            "dataset": "g1",
            "scene_id": args.scene_root.name,
            "dataset_path": str(args.scene_root),
            "graph_path": str(args.graph_path),
            "save_path": str(args.scene_root),
            "depth_cut": 4,
        },
        "models": {"clip": {
            "type": "SigLIP-384",
            "checkpoint": str(args.siglip_snapshot),
        }},
    })
    client = FsrVlnClient(
        cfg, use_hmsg_graph=True, designated_room_names=[args.room_name])
    anchor_store = AnchorStore(args.anchor_path, args.min_persist_observations)
    reader = MultiImageReader()
    app = FastAPI(title="HoloAgent live HMSG query", version="1.0")

    @app.get("/health")
    def health():
        return {
            "status": "ready",
            "graph": str(args.graph_path),
            "floors": len(client.graph.floors),
            "rooms": len(client.graph.rooms),
            "objects": len(client.graph.objects),
            "source_timestamp_ms": evidence["source_timestamp_ms"],
        }

    @app.post("/query")
    def query(request: QueryRequest):
        object_query = request.object_query.strip()
        robot_map = np.asarray(request.robot_map_pose_wxyz, dtype=np.float64)
        if not object_query:
            raise HTTPException(status_code=400, detail="object_query is empty")
        if robot_map.shape != (7,) or not np.isfinite(robot_map).all():
            raise HTTPException(status_code=400, detail="robot_map_pose_wxyz must be 7 finite numbers")

        poses = reader.read_single_image("head_pose")
        timestamp_ms = reader.last_timestamps.get("head_pose", 0)
        if poses is None or poses.shape != (2, 7) or not np.isfinite(poses).all():
            raise HTTPException(status_code=503, detail="live Isaac robot pose is unavailable")
        age_seconds = time.time() - timestamp_ms / 1000.0
        if age_seconds < 0 or age_seconds > args.max_pose_age:
            raise HTTPException(
                status_code=503, detail=f"live Isaac robot pose is stale ({age_seconds:.3f}s)")

        result = client.query(
            f"navigate to {object_query}",
            top_k=1,
            use_gpt=False,
            parsed_query=("1", args.room_name, object_query))
        if not result.targets:
            raise HTTPException(status_code=404, detail="HMSG found no object")
        scores = result.raw_metrics.get("object_scores", [])
        score = float(scores[0]) if len(scores) else float("nan")
        if not math.isfinite(score) or score < args.min_score:
            raise HTTPException(
                status_code=404,
                detail=f"HMSG similarity {score:.4f} is below {args.min_score:.4f}")

        target = result.targets[0]
        center_sim = np.asarray(target.center_map, dtype=np.float64)
        anchor = anchor_store.get(target.object_id)
        center_map = (
            np.asarray(anchor["center_map"], dtype=np.float64)
            if anchor is not None
            else sim_point_to_map(center_sim, poses[1], robot_map)
        )
        if not np.isfinite(center_map).all():
            raise HTTPException(status_code=500, detail="non-finite map target")
        return {
            "object_query": object_query,
            "object_id": target.object_id,
            "score": score,
            "center_sim": center_sim.tolist(),
            "center_map": center_map.tolist(),
            "center_source": "persistent_refinement" if anchor is not None else "hmsg",
            "anchor_updated_at_ms": None if anchor is None else anchor["updated_at_ms"],
            "live_pose_timestamp_ms": timestamp_ms,
            "graph_source_timestamp_ms": evidence["source_timestamp_ms"],
        }

    @app.post("/anchors/update")
    def update_anchor(request: AnchorUpdateRequest):
        try:
            record = anchor_store.update(request)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"status": "UPDATED", "object_id": request.object_id, **record}

    return app


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-root", type=Path, required=True)
    parser.add_argument("--graph-path", type=Path, required=True)
    parser.add_argument("--siglip-snapshot", type=Path, required=True)
    parser.add_argument("--room-name", default="warehouse")
    parser.add_argument("--min-score", type=float, default=0.1)
    parser.add_argument("--max-pose-age", type=float, default=1.0)
    parser.add_argument("--anchor-path", type=Path)
    parser.add_argument("--min-persist-observations", type=int, default=3)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8120)
    args = parser.parse_args()
    if args.anchor_path is None:
        args.anchor_path = args.scene_root / "semantic_anchors.json"
    if args.min_persist_observations < 1:
        parser.error("--min-persist-observations must be positive")
    if not args.graph_path.is_dir() or not args.siglip_snapshot.is_dir():
        parser.error("--graph-path and --siglip-snapshot must exist")
    uvicorn.run(create_app(args), host=args.host, port=args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
