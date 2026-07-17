"""
G1 Perception Modules.

Contains 2D and 3D detection modules.
"""

from perception.modules.detection_2d import Detection2DModule
from perception.modules.detection_3d import Detection3DModule, Detection3DResult

__all__ = [
    'Detection2DModule',
    'Detection3DModule',
    'Detection3DResult',
]
