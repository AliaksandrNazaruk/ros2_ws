from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Tuple


OriginMode = Literal[
    # Directly use Symovo offsets as YAML origin.
    "origin_offsets",
    # Negate Symovo offsets when writing YAML origin.
    "origin_neg_offsets",
]

PoseTransformMode = Literal[
    # Pose already in Nav2 map frame.
    "pose_identity",
    # Translate by subtracting Symovo offsets.
    "pose_subtract_offsets",
    # Translate by adding Symovo offsets.
    "pose_add_offsets",
    # Translate by subtracting offsets, then flip Y using map height.
    "pose_subtract_offsets_flip_y",
    # Translate by adding offsets, then flip Y using map height.
    "pose_add_offsets_flip_y",
]


@dataclass(frozen=True)
class SymovoMapGeometry:
    """
    Pure geometry helper (no ROS deps).

    NOTE: Symovo offset semantics are deployment-dependent. This module is intentionally
    parameterized by explicit modes so we can lock behavior to a tested runtime contract.
    """

    width_px: int
    height_px: int
    resolution: float  # m/pixel
    offset_x: float  # meters
    offset_y: float  # meters

    @property
    def width_m(self) -> float:
        return float(self.width_px) * float(self.resolution)

    @property
    def height_m(self) -> float:
        return float(self.height_px) * float(self.resolution)

    def yaml_origin(self, *, origin_mode: OriginMode) -> Tuple[float, float, float]:
        if origin_mode == "origin_offsets":
            return float(self.offset_x), float(self.offset_y), 0.0
        if origin_mode == "origin_neg_offsets":
            return -float(self.offset_x), -float(self.offset_y), 0.0
        raise ValueError(f"Unsupported origin_mode: {origin_mode}")

    def transform_pose(self, *, x: float, y: float, theta: float, mode: PoseTransformMode) -> Tuple[float, float, float]:
        """
        Convert Symovo pose (meters, radians) into Nav2 map frame pose.

        For flip_y variants, we mirror Y around the map height in meters:
          y' = map_height_m - y
        """

        x_out = float(x)
        y_out = float(y)

        if mode == "pose_identity":
            return x_out, y_out, float(theta)

        if mode == "pose_subtract_offsets":
            return x_out - self.offset_x, y_out - self.offset_y, float(theta)

        if mode == "pose_add_offsets":
            return x_out + self.offset_x, y_out + self.offset_y, float(theta)

        if mode == "pose_subtract_offsets_flip_y":
            x_t = x_out - self.offset_x
            y_t = y_out - self.offset_y
            return x_t, self.height_m - y_t, float(theta)

        if mode == "pose_add_offsets_flip_y":
            x_t = x_out + self.offset_x
            y_t = y_out + self.offset_y
            return x_t, self.height_m - y_t, float(theta)

        raise ValueError(f"Unsupported pose transform mode: {mode}")

