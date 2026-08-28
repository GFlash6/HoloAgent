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
from holoagent_bridge.sim_map_transform import (
    align_sim_to_map,
    load_sim_to_map,
    sim_point_to_map,
)


class QueryRequest(BaseModel):
    object_query: str
    robot_map_pose_wxyz: list[float] | None = None


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
    sim_to_map = load_sim_to_map(args.sim_to_map)
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
            "sim_to_map": str(args.sim_to_map),
        }

    def query_result(request: QueryRequest, apply_threshold: bool):
        object_query = request.object_query.strip()
        if not object_query:
            raise HTTPException(status_code=400, detail="object_query is empty")

        result = client.query(
            f"navigate to {object_query}",
            top_k=2,
            use_gpt=False,
            parsed_query=("1", args.room_name, object_query))
        if not result.targets:
            raise HTTPException(status_code=404, detail="HMSG found no object")
        scores = result.raw_metrics.get("object_scores", [])
        score = float(scores[0]) if len(scores) else float("nan")
        second_score = float(scores[1]) if len(scores) > 1 else None
        passes_threshold = math.isfinite(score) and score >= args.min_score
        if apply_threshold and not passes_threshold:
            raise HTTPException(
                status_code=404,
                detail=f"HMSG similarity {score:.4f} is below {args.min_score:.4f}")

        target = result.targets[0]
        source_object = next(
            (obj for obj in client.graph.objects
             if str(obj.object_id) == target.object_id),
            None,
        )
        observation_count = max(
            1, len(getattr(source_object, "view_ids", [])))
        center_sim = np.asarray(target.center_map, dtype=np.float64)
        anchor = anchor_store.get(target.object_id)
        center_map = (
            np.asarray(anchor["center_map"], dtype=np.float64)
            if anchor is not None
            else sim_point_to_map(center_sim, sim_to_map)
        )
        if not np.isfinite(center_map).all():
            raise HTTPException(status_code=500, detail="non-finite map target")
        return {
            "status": "FOUND",
            "object_query": object_query,
            "object_id": target.object_id,
            "score": score,
            "second_score": second_score,
            "passes_backend_threshold": passes_threshold,
            "backend_min_score": args.min_score,
            "candidates": [
                {"object_id": candidate.object_id, "score": float(candidate_score)}
                for candidate, candidate_score in zip(result.targets, scores)
                if math.isfinite(float(candidate_score))
            ],
            "center_sim": center_sim.tolist(),
            "center_map": center_map.tolist(),
            "center_source": "persistent_refinement" if anchor is not None else "hmsg",
            "anchor_updated_at_ms": None if anchor is None else anchor["updated_at_ms"],
            "alignment_source": str(args.sim_to_map),
            "graph_source_timestamp_ms": evidence["source_timestamp_ms"],
            "source_timestamp_ms": evidence["source_timestamp_ms"],
            "observation_count": observation_count,
            "frame_id": "map",
        }

    @app.post("/query")
    def query(request: QueryRequest):
        return query_result(request, apply_threshold=True)

    @app.post("/evidence")
    def raw_evidence(request: QueryRequest):
        return query_result(request, apply_threshold=False)

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
    parser.add_argument("--sim-to-map", type=Path)
    parser.add_argument("--anchor-path", type=Path)
    parser.add_argument("--min-persist-observations", type=int, default=3)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8120)
    args = parser.parse_args()
    if args.anchor_path is None:
        args.anchor_path = args.scene_root / "semantic_anchors.json"
    if args.sim_to_map is None:
        args.sim_to_map = args.scene_root / "sim_to_map.json"
    if args.min_persist_observations < 1:
        parser.error("--min-persist-observations must be positive")
    if (not args.graph_path.is_dir() or not args.siglip_snapshot.is_dir()
            or not args.sim_to_map.is_file()):
        parser.error("--graph-path, --siglip-snapshot and --sim-to-map must exist")
    uvicorn.run(create_app(args), host=args.host, port=args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
