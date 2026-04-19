#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import deque
import json
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

MAGIC = b"LDR1"


def read_exact(sock: socket.socket, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = size
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise EOFError("socket closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def candidate_ips() -> list[str]:
    ips: set[str] = set()

    try:
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.connect(("8.8.8.8", 80))
        ips.add(udp.getsockname()[0])
        udp.close()
    except OSError:
        pass

    try:
        infos = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET, socket.SOCK_STREAM)
        ips.update(info[4][0] for info in infos if not info[4][0].startswith("127."))
    except OSError:
        pass

    return sorted(ips)


@dataclass
class FramePacket:
    header: dict
    rgb: np.ndarray
    depth_mm: np.ndarray


@dataclass
class MapStats:
    voxel_count: int
    path_length_m: float
    span_m: tuple[float, float, float]
    height_range_m: tuple[float, float]


@dataclass
class OrbitViewState:
    yaw_deg: float = -36.0
    pitch_deg: float = 24.0
    distance_m: float = 3.0
    target: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    initialized: bool = False
    user_interacted: bool = False
    dirty: bool = True

    def reset(self) -> None:
        self.yaw_deg = -36.0
        self.pitch_deg = 24.0
        self.distance_m = 3.0
        self.target = np.zeros(3, dtype=np.float64)
        self.initialized = False
        self.user_interacted = False
        self.dirty = True

    def fit_to_scene(self, scene_min: np.ndarray, scene_max: np.ndarray, force: bool = False) -> None:
        if self.user_interacted and not force:
            return

        scene_min = scene_min.astype(np.float64)
        scene_max = scene_max.astype(np.float64)
        center = (scene_min + scene_max) * 0.5
        span = np.maximum(scene_max - scene_min, 0.25)
        diagonal = float(np.linalg.norm(span))

        self.target = center
        self.distance_m = max(diagonal * 1.35, 1.8)
        self.initialized = True
        self.dirty = True

    def orbit(self, dx: float, dy: float) -> None:
        self.yaw_deg += dx * 0.35
        self.pitch_deg = float(np.clip(self.pitch_deg - dy * 0.25, -80.0, 80.0))
        self.user_interacted = True
        self.dirty = True

    def zoom(self, wheel_steps: float) -> None:
        zoom_factor = 0.88 ** wheel_steps
        self.distance_m = float(np.clip(self.distance_m * zoom_factor, 0.35, 80.0))
        self.user_interacted = True
        self.dirty = True

    def pan(self, dx: float, dy: float) -> None:
        _, right, up, _ = self.camera_frame()
        pan_scale = self.distance_m * 0.0018
        self.target -= right * dx * pan_scale
        self.target += up * dy * pan_scale
        self.user_interacted = True
        self.dirty = True

    def camera_frame(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        yaw = np.deg2rad(self.yaw_deg)
        pitch = np.deg2rad(self.pitch_deg)

        offset = np.array(
            [
                np.sin(yaw) * np.cos(pitch),
                np.sin(pitch),
                np.cos(yaw) * np.cos(pitch),
            ],
            dtype=np.float64,
        )
        camera_position = self.target + self.distance_m * offset
        forward = self.target - camera_position
        forward /= max(np.linalg.norm(forward), 1e-6)

        world_up = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        right = np.cross(forward, world_up)
        if np.linalg.norm(right) < 1e-6:
            right = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        else:
            right /= np.linalg.norm(right)

        up = np.cross(right, forward)
        up /= max(np.linalg.norm(up), 1e-6)
        return camera_position, right, up, forward


@dataclass
class MapPanelRect:
    x: int
    y: int
    width: int
    height: int

    def contains(self, px: int, py: int) -> bool:
        return self.x <= px < self.x + self.width and self.y <= py < self.y + self.height


@dataclass
class DisplayLayoutState:
    mode: str = "landscape"
    preview_rotation_steps: int = 0
    dirty: bool = True

    def toggle(self) -> None:
        self.mode = "landscape" if self.mode == "portrait" else "portrait"
        self.dirty = True

    def rotate_preview(self) -> None:
        self.preview_rotation_steps = (self.preview_rotation_steps + 1) % 4
        self.dirty = True


class MouseMapController:
    def __init__(self, view_state: OrbitViewState) -> None:
        self.view_state = view_state
        self.panel_rect = MapPanelRect(0, 0, 0, 0)
        self.left_dragging = False
        self.right_dragging = False
        self.last_mouse = (0, 0)

    def set_panel_rect(self, x: int, y: int, width: int, height: int) -> None:
        self.panel_rect = MapPanelRect(x, y, width, height)

    def handle(self, event: int, x: int, y: int, flags: int) -> None:
        inside = self.panel_rect.contains(x, y)

        if event == cv2.EVENT_LBUTTONDBLCLK and inside:
            self.view_state.reset()
            return

        if event == cv2.EVENT_LBUTTONDOWN and inside:
            self.left_dragging = True
            self.last_mouse = (x, y)
            return

        if event == cv2.EVENT_RBUTTONDOWN and inside:
            self.right_dragging = True
            self.last_mouse = (x, y)
            return

        if event == cv2.EVENT_MOUSEMOVE and (self.left_dragging or self.right_dragging):
            dx = x - self.last_mouse[0]
            dy = y - self.last_mouse[1]
            self.last_mouse = (x, y)
            if self.left_dragging:
                self.view_state.orbit(dx, dy)
            elif self.right_dragging:
                self.view_state.pan(dx, dy)
            return

        if event == cv2.EVENT_LBUTTONUP:
            self.left_dragging = False
            return

        if event == cv2.EVENT_RBUTTONUP:
            self.right_dragging = False
            return

        if event == cv2.EVENT_MOUSEWHEEL and inside:
            try:
                wheel_delta = cv2.getMouseWheelDelta(flags)
            except AttributeError:
                wheel_delta = 120 if flags > 0 else -120
            self.view_state.zoom(wheel_delta / 120.0)

class SharedState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.latest_frame: FramePacket | None = None
        self.connection_text = "waiting for phone"
        self.total_frames = 0
        self.last_error = ""

    def update_frame(self, frame: FramePacket, peer: str) -> None:
        with self.lock:
            self.latest_frame = frame
            self.total_frames += 1
            self.connection_text = f"connected: {peer}"
            self.last_error = ""

    def set_error(self, message: str) -> None:
        with self.lock:
            self.last_error = message

    def snapshot(self) -> tuple[FramePacket | None, str, int, str]:
        with self.lock:
            return self.latest_frame, self.connection_text, self.total_frames, self.last_error


class TopDownMap:
    def __init__(
        self,
        voxel_size: float = 0.05,
        max_depth_m: float = 5.0,
        sample_step: int = 4,
        truncation_m: float = 0.18,
        max_weight: float = 24.0,
        surface_epsilon: float = 0.22,
        min_surface_weight: float = 1.1,
    ) -> None:
        self.voxel_size = voxel_size
        self.max_depth_m = max_depth_m
        self.sample_step = sample_step
        self.truncation_m = max(truncation_m, voxel_size * 2.0)
        self.max_weight = max_weight
        self.surface_epsilon = surface_epsilon
        self.min_surface_weight = min_surface_weight
        self.integration_offsets = np.linspace(-self.truncation_m, self.truncation_m, 7, dtype=np.float32)
        self.voxels: dict[tuple[int, int, int], np.ndarray] = {}
        self.trajectory: list[np.ndarray] = []
        self.current_pose: np.ndarray | None = None
        self.last_integrated_pose: np.ndarray | None = None
        self._surface_points = np.empty((0, 3), dtype=np.float32)
        self._surface_colors = np.empty((0, 3), dtype=np.uint8)
        self._stats = MapStats(0, 0.0, (0.0, 0.0, 0.0), (0.0, 0.0))
        self._snapshot_dirty = True

    def clear(self) -> None:
        self.voxels.clear()
        self.trajectory.clear()
        self.current_pose = None
        self.last_integrated_pose = None
        self._surface_points = np.empty((0, 3), dtype=np.float32)
        self._surface_colors = np.empty((0, 3), dtype=np.uint8)
        self._stats = MapStats(0, 0.0, (0.0, 0.0, 0.0), (0.0, 0.0))
        self._snapshot_dirty = True

    def integrate(self, packet: FramePacket) -> None:
        header = packet.header
        pose = np.array(header["pose"], dtype=np.float32).reshape(4, 4)
        position = pose[:3, 3].copy()

        self.current_pose = pose
        if not self.trajectory or np.linalg.norm(self.trajectory[-1] - position) > 0.02:
            self.trajectory.append(position)
            self._snapshot_dirty = True

        if self.last_integrated_pose is not None:
            translation_delta = np.linalg.norm(position - self.last_integrated_pose[:3, 3])
            rotation_delta = np.linalg.norm(pose[:3, :3] - self.last_integrated_pose[:3, :3])
            if translation_delta < 0.03 and rotation_delta < 0.05:
                return

        self.last_integrated_pose = pose

        depth_m = packet.depth_mm.astype(np.float32) / 1000.0
        step = self.sample_step
        sampled_depth = depth_m[::step, ::step]
        valid = (sampled_depth > 0.15) & (sampled_depth < self.max_depth_m)
        if not np.any(valid):
            return

        height, width = depth_m.shape
        ys, xs = np.mgrid[0:height:step, 0:width:step]
        u = xs[valid].astype(np.float32)
        v = ys[valid].astype(np.float32)
        z = sampled_depth[valid]

        fx = float(header["fx"])
        fy = float(header["fy"])
        cx = float(header["cx"])
        cy = float(header["cy"])

        x = (u - cx) / fx * z
        y = -(v - cy) / fy * z
        z_camera = -z
        surface_camera_points = np.stack([x, y, z_camera], axis=1)

        ray_norms = np.linalg.norm(surface_camera_points, axis=1, keepdims=True)
        ray_directions = surface_camera_points / np.maximum(ray_norms, 1e-6)

        rotation = pose[:3, :3]
        translation = pose[:3, 3]
        surface_world_points = surface_camera_points @ rotation.T + translation

        rgb = packet.rgb
        scale_x = rgb.shape[1] / width
        scale_y = rgb.shape[0] / height
        rgb_x = np.clip(np.round(u * scale_x).astype(np.int32), 0, rgb.shape[1] - 1)
        rgb_y = np.clip(np.round(v * scale_y).astype(np.int32), 0, rgb.shape[0] - 1)
        colors = rgb[rgb_y, rgb_x].astype(np.float32)

        offset_points = surface_camera_points[None, :, :] + ray_directions[None, :, :] * self.integration_offsets[:, None, None]
        offset_points_world = offset_points.reshape(-1, 3) @ rotation.T + translation
        voxel_keys = np.floor(offset_points_world / self.voxel_size).astype(np.int32)

        tsdf_values = np.repeat(np.clip(-self.integration_offsets / self.truncation_m, -1.0, 1.0), len(surface_camera_points)).astype(np.float32)
        observation_weights = (0.55 + 0.45 * (1.0 - np.abs(tsdf_values))).astype(np.float32)
        surface_weights = np.clip(1.0 - np.abs(tsdf_values) * 1.35, 0.0, 1.0).astype(np.float32)
        repeated_surface_points = np.tile(surface_world_points, (len(self.integration_offsets), 1)).astype(np.float32)
        repeated_colors = np.tile(colors, (len(self.integration_offsets), 1)).astype(np.float32)

        unique_keys, inverse = np.unique(voxel_keys, axis=0, return_inverse=True)
        unique_count = len(unique_keys)
        if unique_count == 0:
            return

        tsdf_sum = np.zeros(unique_count, dtype=np.float32)
        obs_weight_sum = np.zeros(unique_count, dtype=np.float32)
        surface_weight_sum = np.zeros(unique_count, dtype=np.float32)
        point_sum = np.zeros((unique_count, 3), dtype=np.float32)
        color_sum = np.zeros((unique_count, 3), dtype=np.float32)

        np.add.at(tsdf_sum, inverse, tsdf_values * observation_weights)
        np.add.at(obs_weight_sum, inverse, observation_weights)
        np.add.at(surface_weight_sum, inverse, surface_weights)
        for axis in range(3):
            np.add.at(point_sum[:, axis], inverse, repeated_surface_points[:, axis] * surface_weights)
            np.add.at(color_sum[:, axis], inverse, repeated_colors[:, axis] * surface_weights)

        safe_obs_weights = np.maximum(obs_weight_sum, 1e-6)
        average_tsdf = tsdf_sum / safe_obs_weights

        for idx, voxel in enumerate(unique_keys):
            key = (int(voxel[0]), int(voxel[1]), int(voxel[2]))
            obs_weight = float(obs_weight_sum[idx])
            avg_tsdf = float(average_tsdf[idx])
            surface_weight = float(surface_weight_sum[idx])

            if surface_weight > 1e-5:
                avg_point = point_sum[idx] / surface_weight
                avg_color = color_sum[idx] / surface_weight
            else:
                avg_point = (voxel.astype(np.float32) + 0.5) * self.voxel_size
                avg_color = np.array([180.0, 180.0, 180.0], dtype=np.float32)

            if key not in self.voxels:
                self.voxels[key] = np.array(
                    [
                        avg_tsdf,
                        min(obs_weight, self.max_weight),
                        min(surface_weight, self.max_weight),
                        avg_point[0],
                        avg_point[1],
                        avg_point[2],
                        avg_color[0],
                        avg_color[1],
                        avg_color[2],
                    ],
                    dtype=np.float32,
                )
                continue

            bucket = self.voxels[key]
            previous_weight = float(bucket[1])
            blended_weight = previous_weight + obs_weight
            bucket[0] = (bucket[0] * previous_weight + avg_tsdf * obs_weight) / max(blended_weight, 1e-6)
            bucket[1] = min(blended_weight, self.max_weight)

            if surface_weight > 1e-5:
                previous_surface_weight = float(bucket[2])
                blended_surface_weight = previous_surface_weight + surface_weight
                bucket[3:6] = (bucket[3:6] * previous_surface_weight + avg_point * surface_weight) / max(blended_surface_weight, 1e-6)
                bucket[6:9] = (bucket[6:9] * previous_surface_weight + avg_color * surface_weight) / max(blended_surface_weight, 1e-6)
                bucket[2] = min(blended_surface_weight, self.max_weight)

        self._snapshot_dirty = True

    def _rebuild_surface_snapshot(self) -> None:
        if not self.voxels:
            self._surface_points = np.empty((0, 3), dtype=np.float32)
            self._surface_colors = np.empty((0, 3), dtype=np.uint8)
            path_length = 0.0
            if len(self.trajectory) >= 2:
                trajectory = np.array(self.trajectory, dtype=np.float32)
                path_length = float(np.linalg.norm(np.diff(trajectory, axis=0), axis=1).sum())
            self._stats = MapStats(0, path_length, (0.0, 0.0, 0.0), (0.0, 0.0))
            self._snapshot_dirty = False
            return

        values = np.array(list(self.voxels.values()), dtype=np.float32)
        surface_mask = (
            (values[:, 1] >= self.min_surface_weight)
            & (values[:, 2] >= 0.45)
            & (np.abs(values[:, 0]) <= self.surface_epsilon)
        )

        if not np.any(surface_mask):
            surface_mask = (
                (values[:, 1] >= self.min_surface_weight * 0.8)
                & (values[:, 2] >= 0.25)
                & (np.abs(values[:, 0]) <= self.surface_epsilon * 1.8)
            )

        points = values[surface_mask, 3:6]
        colors = np.clip(values[surface_mask, 6:9], 0, 255).astype(np.uint8)
        self._surface_points = points.astype(np.float32, copy=False)
        self._surface_colors = colors

        if len(points):
            min_corner = points.min(axis=0)
            max_corner = points.max(axis=0)
            span = np.maximum(max_corner - min_corner, 0.0)
            span_tuple = (float(span[0]), float(span[1]), float(span[2]))
            height_range = (float(min_corner[1]), float(max_corner[1]))
        else:
            span_tuple = (0.0, 0.0, 0.0)
            height_range = (0.0, 0.0)

        path_length = 0.0
        if len(self.trajectory) >= 2:
            trajectory = np.array(self.trajectory, dtype=np.float32)
            path_length = float(np.linalg.norm(np.diff(trajectory, axis=0), axis=1).sum())

        self._stats = MapStats(
            voxel_count=len(points),
            path_length_m=path_length,
            span_m=span_tuple,
            height_range_m=height_range,
        )
        self._snapshot_dirty = False

    def point_snapshot(self) -> tuple[np.ndarray, np.ndarray]:
        if self._snapshot_dirty:
            self._rebuild_surface_snapshot()
        return self._surface_points, self._surface_colors

    def stats(self) -> MapStats:
        if self._snapshot_dirty:
            self._rebuild_surface_snapshot()
        return self._stats

    def save_ply(self, path: Path) -> None:
        points, colors = self.point_snapshot()
        with path.open("w", encoding="utf-8") as handle:
            handle.write("ply\n")
            handle.write("format ascii 1.0\n")
            handle.write(f"element vertex {len(points)}\n")
            handle.write("property float x\n")
            handle.write("property float y\n")
            handle.write("property float z\n")
            handle.write("property uchar red\n")
            handle.write("property uchar green\n")
            handle.write("property uchar blue\n")
            handle.write("end_header\n")
            for point, color in zip(points, colors, strict=False):
                handle.write(
                    f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                    f"{int(color[2])} {int(color[1])} {int(color[0])}\n"
                )

    def _sample_points(
        self,
        points: np.ndarray,
        colors: np.ndarray,
        max_points: int = 35000,
    ) -> tuple[np.ndarray, np.ndarray]:
        if len(points) <= max_points:
            return points, colors

        indices = np.linspace(0, len(points) - 1, max_points, dtype=np.int32)
        return points[indices], colors[indices]

    def render_topdown(self, width: int, height: int) -> np.ndarray:
        canvas = np.full((height, width, 3), (18, 18, 18), dtype=np.uint8)
        points, colors = self.point_snapshot()

        world_xz_parts: list[np.ndarray] = []
        if len(points):
            world_xz_parts.append(points[:, [0, 2]])
        if self.trajectory:
            world_xz_parts.append(np.array(self.trajectory, dtype=np.float32)[:, [0, 2]])

        if not world_xz_parts:
            cv2.putText(canvas, "Waiting for streamed points...", (24, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 220, 220), 2)
            return canvas

        world_xz = np.vstack(world_xz_parts)
        min_corner = world_xz.min(axis=0)
        max_corner = world_xz.max(axis=0)
        span = np.maximum(max_corner - min_corner, 0.25)
        padding = np.maximum(span * 0.12, 0.25)
        min_corner -= padding
        max_corner += padding
        span = np.maximum(max_corner - min_corner, 0.25)

        usable_width = max(10, width - 40)
        usable_height = max(10, height - 40)
        scale = min(usable_width / span[0], usable_height / span[1])

        def project(xz: np.ndarray) -> np.ndarray:
            px = 20 + (xz[:, 0] - min_corner[0]) * scale
            py = height - (20 + (xz[:, 1] - min_corner[1]) * scale)
            return np.stack([px, py], axis=1).astype(np.int32)

        if len(points):
            rendered_points = project(points[:, [0, 2]])
            for point, color in zip(rendered_points, colors, strict=False):
                cv2.circle(canvas, tuple(point), 1, tuple(int(c) for c in color), -1, lineType=cv2.LINE_AA)

        if len(self.trajectory) >= 2:
            trajectory = np.array(self.trajectory, dtype=np.float32)
            projected = project(trajectory[:, [0, 2]])
            cv2.polylines(canvas, [projected], False, (255, 220, 64), 2, lineType=cv2.LINE_AA)

        if self.current_pose is not None:
            position = self.current_pose[:3, 3]
            forward = self.current_pose[:3, :3] @ np.array([0.0, 0.0, -1.0], dtype=np.float32)
            right = self.current_pose[:3, :3] @ np.array([1.0, 0.0, 0.0], dtype=np.float32)

            origin = position[[0, 2]]
            tip = origin + forward[[0, 2]] * 0.25
            left = origin + forward[[0, 2]] * 0.14 + right[[0, 2]] * 0.08
            right_tip = origin + forward[[0, 2]] * 0.14 - right[[0, 2]] * 0.08
            frustum = project(np.vstack([tip, left, right_tip]))
            cv2.fillConvexPoly(canvas, frustum, (0, 140, 255), lineType=cv2.LINE_AA)
            cv2.circle(canvas, tuple(project(origin.reshape(1, 2))[0]), 4, (255, 255, 255), -1, lineType=cv2.LINE_AA)

        cv2.putText(canvas, f"voxels: {len(points)}", (20, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, lineType=cv2.LINE_AA)
        return canvas

    def render_showcase(self, width: int, height: int, view_state: OrbitViewState) -> tuple[np.ndarray, MapStats]:
        canvas = np.zeros((height, width, 3), dtype=np.uint8)
        canvas[:] = (16, 18, 24)
        gradient = np.linspace(0, 42, height, dtype=np.uint8).reshape(height, 1, 1)
        canvas[:] = np.clip(canvas + gradient, 0, 255)

        points, colors = self.point_snapshot()
        stats = self.stats()
        if not len(points):
            cv2.putText(canvas, "Move the phone slowly to grow the room map.", (28, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (245, 245, 245), 2, lineType=cv2.LINE_AA)
            cv2.putText(canvas, "Tip: keep walls, corners, and furniture in view.", (28, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (180, 188, 210), 1, lineType=cv2.LINE_AA)
            return canvas, stats

        max_points = max(6000, min(18000, (width * height) // 55))
        points, colors = self._sample_points(points, colors, max_points=max_points)

        scene_min = np.percentile(points, 2, axis=0).astype(np.float32)
        scene_max = np.percentile(points, 98, axis=0).astype(np.float32)
        if self.trajectory:
            trajectory_array = np.array(self.trajectory, dtype=np.float32)
            scene_min = np.minimum(scene_min, trajectory_array.min(axis=0))
            scene_max = np.maximum(scene_max, trajectory_array.max(axis=0))
        if self.current_pose is not None:
            position = self.current_pose[:3, 3]
            forward = self.current_pose[:3, :3] @ np.array([0.0, 0.0, -1.0], dtype=np.float32)
            scene_min = np.minimum(scene_min, np.minimum(position, position + forward * 0.35))
            scene_max = np.maximum(scene_max, np.maximum(position, position + forward * 0.35))
        view_state.fit_to_scene(scene_min, scene_max)
        camera_position, right, up, forward = view_state.camera_frame()
        focal = 0.88 * min(width, height)
        cx = width * 0.5
        cy = height * 0.57
        near_clip = 0.05

        def project(world_points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
            shifted = world_points.astype(np.float64) - camera_position
            camera_x = shifted @ right
            camera_y = shifted @ up
            camera_z = shifted @ forward
            valid = camera_z > near_clip
            if not np.any(valid):
                return np.empty((0, 2), dtype=np.int32), np.empty((0,), dtype=np.float64), valid

            px = cx + (camera_x[valid] / camera_z[valid]) * focal
            py = cy - (camera_y[valid] / camera_z[valid]) * focal
            projected = np.stack([px, py], axis=1).astype(np.int32)
            return projected, camera_z[valid], valid

        floor_y = float(np.percentile(points[:, 1], 8))
        grid_min_x = np.floor(scene_min[0] * 2.0) / 2.0
        grid_max_x = np.ceil(scene_max[0] * 2.0) / 2.0
        grid_min_z = np.floor(scene_min[2] * 2.0) / 2.0
        grid_max_z = np.ceil(scene_max[2] * 2.0) / 2.0

        for x_value in np.arange(grid_min_x, grid_max_x + 0.001, 0.5, dtype=np.float32):
            line_points = np.array(
                [[x_value, floor_y, grid_min_z], [x_value, floor_y, grid_max_z]],
                dtype=np.float32,
            )
            projected, _, valid = project(line_points)
            if len(projected) == 2 and np.all(valid):
                cv2.line(canvas, tuple(projected[0]), tuple(projected[1]), (42, 52, 74), 1, lineType=cv2.LINE_AA)

        for z_value in np.arange(grid_min_z, grid_max_z + 0.001, 0.5, dtype=np.float32):
            line_points = np.array(
                [[grid_min_x, floor_y, z_value], [grid_max_x, floor_y, z_value]],
                dtype=np.float32,
            )
            projected, _, valid = project(line_points)
            if len(projected) == 2 and np.all(valid):
                cv2.line(canvas, tuple(projected[0]), tuple(projected[1]), (42, 52, 74), 1, lineType=cv2.LINE_AA)

        projected_points, depth_order, visible = project(points)
        visible_points = points[visible]
        visible_colors = colors[visible]
        if not len(projected_points):
            cv2.putText(canvas, "Rotate or zoom to bring the map into view.", (28, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.78, (245, 245, 245), 2, lineType=cv2.LINE_AA)
            return canvas, stats

        near = float(depth_order.min())
        far = float(depth_order.max())
        depth_span = max(far - near, 1e-3)
        order = np.argsort(depth_order)[::-1]
        height_min = float(visible_points[:, 1].min())
        height_span = max(float(visible_points[:, 1].max()) - height_min, 0.05)
        sorted_points = projected_points[order]
        sorted_depth = depth_order[order]
        sorted_world = visible_points[order]
        sorted_colors = visible_colors[order].astype(np.float32)

        brightness = 0.62 + 0.38 * ((sorted_depth - near) / depth_span)
        height_mix = (sorted_world[:, 1] - height_min) / height_span
        tint = np.stack(
            [
                50.0 + 120.0 * height_mix,
                110.0 + 70.0 * height_mix,
                255.0 - 120.0 * height_mix,
            ],
            axis=1,
        ).astype(np.float32)
        shaded_colors = np.clip(sorted_colors * 0.65 + tint * 0.35, 0, 255)
        shaded_colors = np.clip(shaded_colors * brightness[:, None], 0, 255).astype(np.uint8)

        in_bounds = (
            (sorted_points[:, 0] >= 0)
            & (sorted_points[:, 0] < width)
            & (sorted_points[:, 1] >= 0)
            & (sorted_points[:, 1] < height)
        )
        draw_points = sorted_points[in_bounds]
        draw_colors = shaded_colors[in_bounds]
        if len(draw_points):
            canvas[draw_points[:, 1], draw_points[:, 0]] = draw_colors
            if len(draw_points) < 9000:
                neighbor_offsets = np.array([[1, 0], [0, 1], [1, 1]], dtype=np.int32)
                for offset in neighbor_offsets:
                    shifted = draw_points + offset
                    valid_shift = (
                        (shifted[:, 0] >= 0)
                        & (shifted[:, 0] < width)
                        & (shifted[:, 1] >= 0)
                        & (shifted[:, 1] < height)
                    )
                    shifted = shifted[valid_shift]
                    shifted_colors = draw_colors[valid_shift]
                    canvas[shifted[:, 1], shifted[:, 0]] = shifted_colors

        if len(self.trajectory) >= 2:
            trajectory = np.array(self.trajectory, dtype=np.float32)
            projected_path, _, _ = project(trajectory)
            if len(projected_path) >= 2:
                cv2.polylines(canvas, [projected_path], False, (18, 18, 18), 5, lineType=cv2.LINE_AA)
                cv2.polylines(canvas, [projected_path], False, (255, 214, 84), 2, lineType=cv2.LINE_AA)

        if self.current_pose is not None:
            position = self.current_pose[:3, 3]
            basis = self.current_pose[:3, :3]
            forward = basis @ np.array([0.0, 0.0, -1.0], dtype=np.float32)
            right = basis @ np.array([1.0, 0.0, 0.0], dtype=np.float32)
            up = basis @ np.array([0.0, 1.0, 0.0], dtype=np.float32)

            frustum_points = np.stack(
                [
                    position,
                    position + forward * 0.28 + right * 0.09 + up * 0.06,
                    position + forward * 0.28 - right * 0.09 + up * 0.06,
                    position + forward * 0.28 - right * 0.09 - up * 0.06,
                    position + forward * 0.28 + right * 0.09 - up * 0.06,
                ],
                axis=0,
            ).astype(np.float32)
            projected_frustum, _, _ = project(frustum_points)
            if len(projected_frustum) == 5:
                origin = tuple(projected_frustum[0])
                for corner in projected_frustum[1:]:
                    cv2.line(canvas, origin, tuple(corner), (244, 244, 244), 1, lineType=cv2.LINE_AA)
                cv2.polylines(canvas, [projected_frustum[1:]], True, (0, 162, 255), 2, lineType=cv2.LINE_AA)
                cv2.circle(canvas, origin, 5, (255, 255, 255), -1, lineType=cv2.LINE_AA)

        axis_origin = np.array([scene_min[0], floor_y, scene_min[2]], dtype=np.float32)
        axis_length = max(float(np.linalg.norm(scene_max - scene_min)) * 0.18, 0.35)
        axis_points = np.array(
            [
                axis_origin,
                axis_origin + np.array([axis_length, 0.0, 0.0], dtype=np.float32),
                axis_origin + np.array([0.0, axis_length, 0.0], dtype=np.float32),
                axis_origin + np.array([0.0, 0.0, axis_length], dtype=np.float32),
            ]
        )
        projected_axes, _, _ = project(axis_points)
        if len(projected_axes) == 4:
            origin = tuple(projected_axes[0])
            cv2.line(canvas, origin, tuple(projected_axes[1]), (82, 130, 255), 2, lineType=cv2.LINE_AA)
            cv2.line(canvas, origin, tuple(projected_axes[2]), (92, 220, 130), 2, lineType=cv2.LINE_AA)
            cv2.line(canvas, origin, tuple(projected_axes[3]), (255, 180, 82), 2, lineType=cv2.LINE_AA)
            cv2.putText(canvas, "X", tuple(projected_axes[1] + np.array([4, -4])), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (82, 130, 255), 1, lineType=cv2.LINE_AA)
            cv2.putText(canvas, "Y", tuple(projected_axes[2] + np.array([4, -4])), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (92, 220, 130), 1, lineType=cv2.LINE_AA)
            cv2.putText(canvas, "Z", tuple(projected_axes[3] + np.array([4, -4])), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 180, 82), 1, lineType=cv2.LINE_AA)

        inset_width = min(280, max(200, width // 3))
        inset_height = min(220, max(160, height // 3))
        inset = self.render_topdown(inset_width, inset_height)
        inset_x = width - inset_width - 20
        inset_y = 20
        cv2.rectangle(canvas, (inset_x - 2, inset_y - 2), (inset_x + inset_width + 2, inset_y + inset_height + 2), (255, 255, 255), 1, lineType=cv2.LINE_AA)
        canvas[inset_y:inset_y + inset_height, inset_x:inset_x + inset_width] = inset
        cv2.putText(canvas, "Floor Plan", (inset_x + 14, inset_y + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, lineType=cv2.LINE_AA)

        help_lines = [
            "Left drag: orbit",
            "Right drag: pan",
            "Wheel: zoom",
            "Double-click or f: refit",
        ]
        line_y = 36
        for line in help_lines:
            cv2.putText(canvas, line, (24, line_y), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (205, 212, 224), 1, lineType=cv2.LINE_AA)
            line_y += 24

        return canvas, stats


class VideoRecorder:
    def __init__(self) -> None:
        self.writer: cv2.VideoWriter | None = None
        self.output_path: Path | None = None

    @property
    def is_recording(self) -> bool:
        return self.writer is not None

    def start(self, output_path: Path, frame_size: tuple[int, int], fps: float = 20.0) -> None:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(output_path), fourcc, fps, frame_size)
        if not writer.isOpened():
            raise RuntimeError(f"failed to open video writer for {output_path}")

        self.writer = writer
        self.output_path = output_path

    def write(self, frame: np.ndarray) -> None:
        if self.writer is not None:
            self.writer.write(frame)

    def stop(self) -> Path | None:
        if self.writer is None:
            return None

        self.writer.release()
        self.writer = None
        output_path = self.output_path
        self.output_path = None
        return output_path


def fit_panel(image: np.ndarray, width: int, height: int) -> np.ndarray:
    canvas = np.full((height, width, 3), (12, 12, 12), dtype=np.uint8)
    if image.size == 0:
        return canvas

    source_height, source_width = image.shape[:2]
    scale = min(width / source_width, height / source_height)
    scaled_width = max(1, int(source_width * scale))
    scaled_height = max(1, int(source_height * scale))
    resized = cv2.resize(image, (scaled_width, scaled_height), interpolation=cv2.INTER_AREA)

    offset_x = (width - scaled_width) // 2
    offset_y = (height - scaled_height) // 2
    canvas[offset_y:offset_y + scaled_height, offset_x:offset_x + scaled_width] = resized
    return canvas


def colorize_depth(depth_mm: np.ndarray) -> np.ndarray:
    depth = depth_mm.astype(np.float32) / 1000.0
    valid = depth > 0
    clipped = np.clip(depth, 0.2, 5.0)
    normalized = ((clipped - 0.2) / (5.0 - 0.2) * 255.0).astype(np.uint8)
    colored = cv2.applyColorMap(255 - normalized, cv2.COLORMAP_TURBO)
    colored[~valid] = (0, 0, 0)
    return colored


def quarter_turn_image(image: np.ndarray, steps: int) -> np.ndarray:
    normalized = steps % 4
    if normalized == 0:
        return image
    if normalized == 1:
        return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    if normalized == 2:
        return cv2.rotate(image, cv2.ROTATE_180)
    return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)


def auto_rotation_steps(orientation_code: str) -> int:
    mapping = {
        "portrait": 1,
        "portraitUpsideDown": 3,
        "landscapeLeft": 2,
        "landscapeRight": 0,
        "unknown": 0,
    }
    return mapping.get(orientation_code, 0)


def draw_badge(
    canvas: np.ndarray,
    text: str,
    origin: tuple[int, int],
    background: tuple[int, int, int],
    foreground: tuple[int, int, int] = (255, 255, 255),
) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.58
    thickness = 1
    (text_width, text_height), baseline = cv2.getTextSize(text, font, scale, thickness)
    x, y = origin
    top_left = (x, y)
    bottom_right = (x + text_width + 18, y + text_height + baseline + 12)
    cv2.rectangle(canvas, top_left, bottom_right, background, -1, lineType=cv2.LINE_AA)
    cv2.putText(canvas, text, (x + 9, y + text_height + 4), font, scale, foreground, thickness, lineType=cv2.LINE_AA)


def fit_panel_to_rect(canvas: np.ndarray, image: np.ndarray, rect: MapPanelRect) -> None:
    fitted = fit_panel(image, rect.width, rect.height)
    canvas[rect.y:rect.y + rect.height, rect.x:rect.x + rect.width] = fitted


def build_dashboard_layout(width: int, height: int, mode: str) -> tuple[MapPanelRect, MapPanelRect, MapPanelRect]:
    header_h = 58
    footer_h = 46
    content_y = header_h
    content_h = height - header_h - footer_h

    if mode == "landscape":
        side_w = max(320, int(width * 0.33))
        map_w = width - side_w
        rgb_h = content_h // 2
        depth_h = content_h - rgb_h
        rgb_rect = MapPanelRect(0, content_y, side_w, rgb_h)
        depth_rect = MapPanelRect(0, content_y + rgb_h, side_w, depth_h)
        map_rect = MapPanelRect(side_w, content_y, map_w, content_h)
        return rgb_rect, depth_rect, map_rect

    map_h = max(360, int(content_h * 0.74))
    lower_h = content_h - map_h
    rgb_w = width // 2
    depth_w = width - rgb_w
    map_rect = MapPanelRect(0, content_y, width, map_h)
    rgb_rect = MapPanelRect(0, content_y + map_h, rgb_w, lower_h)
    depth_rect = MapPanelRect(rgb_w, content_y + map_h, depth_w, lower_h)
    return rgb_rect, depth_rect, map_rect


def compose_dashboard(
    frame: FramePacket,
    connection_text: str,
    total_frames: int,
    map_view: np.ndarray,
    map_stats: MapStats,
    viewer_fps: float,
    is_recording: bool,
    notice_text: str,
    layout_state: DisplayLayoutState,
    width: int,
    height: int,
) -> np.ndarray:
    canvas = np.full((height, width, 3), (12, 14, 18), dtype=np.uint8)
    cv2.rectangle(canvas, (0, 0), (width, 58), (10, 12, 18), -1, lineType=cv2.LINE_AA)
    cv2.rectangle(canvas, (0, height - 46), (width, height), (10, 12, 18), -1, lineType=cv2.LINE_AA)

    orientation_code = str(frame.header.get("displayOrientation", "unknown"))
    rotation_steps = auto_rotation_steps(orientation_code) + layout_state.preview_rotation_steps
    preview_rgb = quarter_turn_image(frame.rgb, rotation_steps)
    preview_depth = quarter_turn_image(colorize_depth(frame.depth_mm), rotation_steps)

    rgb_rect, depth_rect, map_rect = build_dashboard_layout(width, height, layout_state.mode)
    fit_panel_to_rect(canvas, preview_rgb, rgb_rect)
    fit_panel_to_rect(canvas, preview_depth, depth_rect)
    fit_panel_to_rect(canvas, map_view, map_rect)

    title_specs = [
        ("RGB", rgb_rect),
        ("Depth", depth_rect),
        ("3D Room View", map_rect),
    ]
    for title, rect in title_specs:
        cv2.putText(canvas, title, (rect.x + 18, rect.y + 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, lineType=cv2.LINE_AA)

    badge_x = width - 18
    badges: list[tuple[str, tuple[int, int, int]]] = [
        (f"{viewer_fps:0.1f} fps", (45, 92, 174)),
        (f"frame {frame.header['frameIndex']}", (62, 72, 86)),
        (f"{map_stats.voxel_count:,} voxels", (58, 62, 74)),
    ]
    if is_recording:
        badges.insert(0, ("REC", (26, 36, 210)))

    for text, color in badges:
        (text_width, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.58, 1)
        badge_width = text_width + 18
        badge_x -= badge_width
        draw_badge(canvas, text, (badge_x, 14), color)
        badge_x -= 8

    metrics = (
        f"{connection_text} | recv {total_frames} | path {map_stats.path_length_m:.1f} m | "
        f"span {map_stats.span_m[0]:.1f} x {map_stats.span_m[1]:.1f} x {map_stats.span_m[2]:.1f} m"
    )
    cv2.putText(canvas, metrics, (18, height - 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (235, 235, 235), 1, lineType=cv2.LINE_AA)

    help_text = f"{layout_state.mode}   mouse: orbit/pan/zoom   v layout   o rotate   f refit   q quit"
    (help_width, _), _ = cv2.getTextSize(help_text, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
    cv2.putText(canvas, help_text, (width - help_width - 18, height - 19), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (190, 198, 215), 1, lineType=cv2.LINE_AA)

    if notice_text:
        draw_badge(canvas, notice_text, (18, 14), (82, 52, 28))

    return canvas


def receive_loop(shared: SharedState, host: str, port: int) -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    server.settimeout(1.0)

    ip_list = candidate_ips()
    if ip_list:
        print("Enter one of these Mac IPs on the phone:")
        for ip in ip_list:
            print(f"  {ip}:{port}")
    else:
        print(f"Listening on {host}:{port}")

    while not shared.stop_event.is_set():
        try:
            connection, address = server.accept()
        except socket.timeout:
            continue

        peer = f"{address[0]}:{address[1]}"
        print(f"Accepted connection from {peer}")
        connection.settimeout(5.0)

        try:
            while not shared.stop_event.is_set():
                magic = read_exact(connection, 4)
                if magic != MAGIC:
                    raise RuntimeError("invalid packet magic")

                header_size = struct.unpack(">I", read_exact(connection, 4))[0]
                header = json.loads(read_exact(connection, header_size).decode("utf-8"))

                rgb_bytes = read_exact(connection, int(header["rgbSize"]))
                depth_bytes = read_exact(connection, int(header["depthSize"]))

                rgb = cv2.imdecode(np.frombuffer(rgb_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                if rgb is None:
                    raise RuntimeError("failed to decode JPEG frame")

                depth_shape = (int(header["depthHeight"]), int(header["depthWidth"]))
                depth_mm = np.frombuffer(depth_bytes, dtype="<u2").reshape(depth_shape)

                shared.update_frame(FramePacket(header=header, rgb=rgb, depth_mm=depth_mm), peer)
        except EOFError:
            print(f"Connection closed: {peer}")
        except (ConnectionError, OSError, RuntimeError, ValueError) as error:
            shared.set_error(str(error))
            print(f"Connection closed: {peer} ({error})")
        finally:
            connection.close()

    server.close()


def run_gui(shared: SharedState, width: int, height: int, output_dir: Path) -> None:
    cv2.namedWindow("LiDAR Stream Receiver", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("LiDAR Stream Receiver", width, height)

    map_model = TopDownMap()
    view_state = OrbitViewState()
    layout_state = DisplayLayoutState()
    mouse_controller = MouseMapController(view_state)
    _, _, initial_map_rect = build_dashboard_layout(width, height, layout_state.mode)
    mouse_controller.set_panel_rect(initial_map_rect.x, initial_map_rect.y, initial_map_rect.width, initial_map_rect.height)
    cv2.setMouseCallback(
        "LiDAR Stream Receiver",
        lambda event, x, y, flags, _param: mouse_controller.handle(event, x, y, flags),
    )

    recorder = VideoRecorder()
    last_frame_index = -1
    last_canvas = np.full((height, width, 3), (12, 12, 12), dtype=np.uint8)
    frame_times: deque[float] = deque(maxlen=90)
    notice_text = ""
    notice_until = 0.0
    last_notice_text = ""

    while not shared.stop_event.is_set():
        frame, connection_text, total_frames, last_error = shared.snapshot()
        frame_is_new = frame is not None and int(frame.header["frameIndex"]) != last_frame_index
        if frame_is_new:
            last_frame_index = int(frame.header["frameIndex"])
            frame_times.append(time.monotonic())
            map_model.integrate(frame)

        if notice_until < time.monotonic():
            notice_text = ""

        needs_redraw = (
            frame is not None
            and (
                frame_is_new
                or view_state.dirty
                or layout_state.dirty
                or notice_text != last_notice_text
            )
        )
        if needs_redraw and frame is not None:
            _, _, map_rect = build_dashboard_layout(width, height, layout_state.mode)
            mouse_controller.set_panel_rect(map_rect.x, map_rect.y, map_rect.width, map_rect.height)
            map_panel, map_stats = map_model.render_showcase(map_rect.width, map_rect.height, view_state)

            viewer_fps = 0.0
            if len(frame_times) >= 2:
                elapsed = frame_times[-1] - frame_times[0]
                if elapsed > 0:
                    viewer_fps = (len(frame_times) - 1) / elapsed

            last_canvas = compose_dashboard(
                frame,
                connection_text,
                total_frames,
                map_panel,
                map_stats,
                viewer_fps,
                recorder.is_recording,
                notice_text,
                layout_state,
                width,
                height,
            )
            last_notice_text = notice_text
            view_state.dirty = False
            layout_state.dirty = False
            recorder.write(last_canvas)

        if frame is None:
            last_canvas = np.full((height, width, 3), (12, 12, 12), dtype=np.uint8)
            cv2.putText(last_canvas, "Waiting for iPhone stream...", (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, lineType=cv2.LINE_AA)
            if last_error:
                cv2.putText(last_canvas, last_error, (40, 102), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (80, 180, 255), 2, lineType=cv2.LINE_AA)

        cv2.imshow("LiDAR Stream Receiver", last_canvas)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):
            shared.stop_event.set()
            break

        if key == ord("c"):
            map_model.clear()
            view_state.reset()
            notice_text = "Map cleared"
            notice_until = time.monotonic() + 2.5

        if key == ord("f"):
            view_state.reset()
            notice_text = "View refit"
            notice_until = time.monotonic() + 2.5

        if key == ord("v"):
            layout_state.toggle()
            if layout_state.mode == "portrait":
                cv2.resizeWindow("LiDAR Stream Receiver", min(width, 1200), min(max(height, 900), 1200))
            else:
                cv2.resizeWindow("LiDAR Stream Receiver", max(width, 1400), min(height, 900))
            notice_text = f"Layout: {layout_state.mode}"
            notice_until = time.monotonic() + 2.5

        if key == ord("o"):
            layout_state.rotate_preview()
            notice_text = f"Preview rotation: {layout_state.preview_rotation_steps * 90} deg"
            notice_until = time.monotonic() + 2.5

        if key == ord("p"):
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            ply_path = output_dir / f"lidar_stream_map-{timestamp}.ply"
            png_path = output_dir / f"lidar_stream_view-{timestamp}.png"
            map_model.save_ply(ply_path)
            cv2.imwrite(str(png_path), last_canvas)
            print(f"Saved {ply_path}")
            print(f"Saved {png_path}")
            notice_text = f"Saved {ply_path.name}"
            notice_until = time.monotonic() + 3.0

        if key == ord("r"):
            if recorder.is_recording:
                saved_path = recorder.stop()
                if saved_path is not None:
                    print(f"Saved {saved_path}")
                    notice_text = f"Saved {saved_path.name}"
                    notice_until = time.monotonic() + 3.0
            else:
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                video_path = output_dir / f"lidar_stream_demo-{timestamp}.mp4"
                recorder.start(video_path, (width, height))
                print(f"Recording {video_path}")
                notice_text = f"Recording {video_path.name}"
                notice_until = time.monotonic() + 3.0

    cv2.destroyAllWindows()
    recorder.stop()


def run_headless(shared: SharedState) -> None:
    last_count = 0
    last_report = time.monotonic()

    while not shared.stop_event.is_set():
        time.sleep(0.25)
        _, connection_text, total_frames, last_error = shared.snapshot()
        now = time.monotonic()
        if now - last_report < 1.0:
            continue

        delta = total_frames - last_count
        last_count = total_frames
        last_report = now
        print(f"{connection_text} | total frames: {total_frames} | last second: {delta} | error: {last_error or 'none'}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Receive RGB + depth + pose from the iPhone LiDAR demo and render a live triptych view on the Mac.")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host for the TCP listener.")
    parser.add_argument("--port", type=int, default=9000, help="TCP port the iPhone should connect to.")
    parser.add_argument("--width", type=int, default=1680, help="Viewer window width.")
    parser.add_argument("--height", type=int, default=720, help="Viewer window height.")
    parser.add_argument("--headless", action="store_true", help="Receive frames without opening the GUI.")
    parser.add_argument("--output-dir", default=str(Path.cwd()), help="Directory for saved screenshots and PLY exports.")
    args = parser.parse_args()

    shared = SharedState()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    receiver = threading.Thread(target=receive_loop, args=(shared, args.host, args.port), daemon=True)
    receiver.start()

    try:
        if args.headless:
            run_headless(shared)
        else:
            run_gui(shared, args.width, args.height, output_dir)
    except KeyboardInterrupt:
        pass
    finally:
        shared.stop_event.set()
        receiver.join(timeout=1.0)


if __name__ == "__main__":
    main()
