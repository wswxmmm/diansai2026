#!/usr/bin/env python3
"""Render the STL assembly coordinates and overlay recovered kinematic axes."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from analyze_stl import load_stl


PARTS = [
    ("robot_belt_arm basering.stl", "#777777"),
    ("robot_belt_arm socket.stl", "#8c8c8c"),
    ("robot_belt_arm rotategear.stl", "#d95f02"),
    ("robot_belt_arm main_body.stl", "#4d4d4d"),
    ("robot_belt_arm gear_body.stl", "#e6ab02"),
    ("robot_belt_arm lever.stl", "#a6761d"),
    ("robot_belt_arm lower_shank.stl", "#1b9e77"),
    ("robot_belt_arm pleuel.stl", "#66a61e"),
    ("robot_belt_arm triplate.stl", "#7570b3"),
    ("robot_arm upper_shank.stl", "#1f78b4"),
    ("robot_belt_arm pleuel_bend_lower.stl", "#e7298a"),
    ("robot_belt_arm pleuel_bend_upper.stl", "#e6ab02"),
    ("robot_belt_arm manipulator.stl", "#d73027"),
]

JOINTS_XZ = np.asarray(
    [
        [100.000, 62.500],
        [11.685, 143.744],
        [119.541, 196.348],
    ]
)


def decimate(triangles: np.ndarray, maximum: int) -> np.ndarray:
    if len(triangles) <= maximum:
        return triangles
    return triangles[:: max(1, len(triangles) // maximum)][:maximum]


def equal_3d(ax, points: np.ndarray) -> None:
    minimum = points.min(axis=0)
    maximum = points.max(axis=0)
    center = (minimum + maximum) / 2
    radius = (maximum - minimum).max() / 2
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("stl_root", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    meshes = []
    for name, color in PARTS:
        path = args.stl_root / name
        meshes.append((name[:-4] if name.endswith(".stl") else name, color, load_stl(path)))

    figure = plt.figure(figsize=(16, 6.7), constrained_layout=True)
    ax3d = figure.add_subplot(1, 3, 1, projection="3d")
    ax_side = figure.add_subplot(1, 3, 2)
    ax_top = figure.add_subplot(1, 3, 3)
    all_points = []

    for name, color, triangles in meshes:
        all_points.append(triangles.reshape((-1, 3)))
        tri3d = decimate(triangles, 2400)
        ax3d.add_collection3d(
            Poly3DCollection(tri3d, facecolor=color, edgecolor="none", alpha=0.46)
        )
        tri2d = decimate(triangles, 4500)
        ax_side.add_collection(
            PolyCollection(tri2d[:, :, [0, 2]], facecolor=color, edgecolor="none", alpha=0.42)
        )
        ax_top.add_collection(
            PolyCollection(tri2d[:, :, [0, 1]], facecolor=color, edgecolor="none", alpha=0.42)
        )

    points = np.concatenate(all_points, axis=0)
    equal_3d(ax3d, points)
    ax3d.view_init(elev=24, azim=-58)
    ax3d.set_xlabel("X (mm)")
    ax3d.set_ylabel("Y (mm)")
    ax3d.set_zlabel("Z (mm)")
    ax3d.set_title("Core STL assembly coordinates")

    ax_side.autoscale()
    ax_side.set_aspect("equal", adjustable="box")
    ax_side.plot(JOINTS_XZ[:, 0], JOINTS_XZ[:, 1], "k--", linewidth=2.0, zorder=20)
    ax_side.scatter(JOINTS_XZ[:, 0], JOINTS_XZ[:, 1], s=58, c="#ffff33", edgecolors="black", zorder=21)
    for index, point in enumerate(JOINTS_XZ, start=2):
        ax_side.annotate(f"J{index}", point, xytext=(6, 7), textcoords="offset points", weight="bold", zorder=22)
    ax_side.axvline(100.0, color="black", linestyle=":", linewidth=1.2)
    ax_side.set_xlabel("X (mm)")
    ax_side.set_ylabel("Z (mm)")
    ax_side.grid(alpha=0.2)
    ax_side.set_title("Side X-Z: recovered 120 + 120 mm chain")

    ax_top.autoscale()
    ax_top.set_aspect("equal", adjustable="box")
    ax_top.scatter([99.989], [-49.005], s=75, c="#ffff33", edgecolors="black", zorder=21)
    ax_top.annotate("J1 (vertical Z axis)", (99.989, -49.005), xytext=(7, 7), textcoords="offset points", weight="bold")
    ax_top.set_xlabel("X (mm)")
    ax_top.set_ylabel("Y (mm)")
    ax_top.grid(alpha=0.2)
    ax_top.set_title("Top X-Y: base yaw axis")

    figure.suptitle("Robot arm geometry recovered from STEP/STL", fontsize=15, weight="bold")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180)
    print(args.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
