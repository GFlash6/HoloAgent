"""Reusable fsr_vln query API for external callers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
from omegaconf import DictConfig

from memory.hmsg.graph.graph import Graph


_DEFAULT_SWITCH_AXIS = np.array(
    [[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]],
    dtype=np.float64,
)


@dataclass
class QueryTarget:
    """Single retrieved target."""

    object_id: str
    room_id: Optional[str]
    room_name: Optional[str]
    center_scenegraph: List[float]
    center_map: List[float]


@dataclass
class QueryResult:
    """Structured query result."""

    floor_id: Optional[str]
    query: str
    floor_query: Optional[str]
    room_query: Optional[str]
    object_query: Optional[str]
    targets: List[QueryTarget]
    raw_metrics: Dict[str, Any]


class FsrVlnClient:
    """Thin wrapper around the internal Graph API for external projects."""

    def __init__(
        self,
        cfg: DictConfig,
        *,
        room_name_generate_method: str = "view_embedding",
        default_room_types: Optional[Sequence[str]] = None,
        designated_room_names: Optional[Sequence[str]] = None,
        use_hmsg_graph: bool = False,
        switch_axis_transform: Optional[np.ndarray] = None,
    ) -> None:
        self.cfg = cfg
        self.graph = Graph(cfg)
        self._switch_axis_transform = (
            np.array(switch_axis_transform, dtype=np.float64)
            if switch_axis_transform is not None
            else _DEFAULT_SWITCH_AXIS.copy()
        )
        self._to_map_transform = np.linalg.inv(self._switch_axis_transform)

        graph_path = self.cfg.main.graph_path
        if use_hmsg_graph:
            self.graph.load_hmsg_graph(graph_path)
        else:
            self.graph.load_graph(graph_path)

        if designated_room_names is not None:
            self.graph.set_room_names(room_names=list(designated_room_names))
        elif default_room_types is not None:
            self.graph.generate_room_names(
                generate_method=room_name_generate_method,
                default_room_types=list(default_room_types),
            )

    def query(
        self,
        query_instruction: str,
        *,
        top_k: int = 1,
        use_gpt: bool = False,
        use_icra_api: bool = False,
        parsed_query: Optional[Tuple[Optional[str],
                                     Optional[str], Optional[str]]] = None,
    ) -> QueryResult:
        if use_icra_api:
            floor, rooms, objects, res_dict = self.graph.query_hierarchy_protected_icra(
                query_instruction,
                top_k=top_k,
                use_gpt=use_gpt,
            )
        else:
            floor, rooms, objects, res_dict = self.graph.query_hierarchy_protected(
                query_instruction,
                top_k=top_k,
                use_gpt=use_gpt,
                parsed_query=parsed_query,
            )

        targets: List[QueryTarget] = []
        for index, obj in enumerate(objects):
            obj_center = np.asarray(obj.pcd.get_center(), dtype=np.float64)
            obj_center_h = np.hstack((obj_center, 1.0))
            obj_center_in_map = (self._to_map_transform @ obj_center_h)[:3]

            room = rooms[index] if index < len(rooms) else None
            targets.append(
                QueryTarget(
                    object_id=str(obj.object_id),
                    room_id=None if room is None else str(room.room_id),
                    room_name=None if room is None else getattr(
                        room, "name", None),
                    center_scenegraph=obj_center.tolist(),
                    center_map=obj_center_in_map.tolist(),
                )
            )

        return QueryResult(
            floor_id=None if floor is None else str(floor.floor_id),
            query=query_instruction,
            floor_query=res_dict.get("floor_query"),
            room_query=res_dict.get("room_query"),
            object_query=res_dict.get("object_query"),
            targets=targets,
            raw_metrics=res_dict,
        )


def query_goal_centers(
    cfg: DictConfig,
    query_instruction: str,
    *,
    top_k: int = 1,
    use_gpt: bool = False,
    use_icra_api: bool = False,
    room_name_generate_method: str = "view_embedding",
    default_room_types: Optional[Sequence[str]] = None,
    designated_room_names: Optional[Sequence[str]] = None,
    use_hmsg_graph: bool = False,
    switch_axis_transform: Optional[np.ndarray] = None,
    parsed_query: Optional[Tuple[Optional[str],
                                 Optional[str], Optional[str]]] = None,
) -> QueryResult:
    """Convenience helper for one-shot querying."""

    client = FsrVlnClient(
        cfg,
        room_name_generate_method=room_name_generate_method,
        default_room_types=default_room_types,
        designated_room_names=designated_room_names,
        use_hmsg_graph=use_hmsg_graph,
        switch_axis_transform=switch_axis_transform,
    )
    return client.query(
        query_instruction,
        top_k=top_k,
        use_gpt=use_gpt,
        use_icra_api=use_icra_api,
        parsed_query=parsed_query,
    )
