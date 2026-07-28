#!/usr/bin/env python3
"""Print deterministic geometric statistics for binary or ASCII STL files."""

from __future__ import annotations

import argparse
import re
import struct
from pathlib import Path

import numpy as np


VERTEX_RE = re.compile(
    rb"vertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)"
)


def load_stl(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    if len(raw) >= 84:
        face_count = struct.unpack_from("<I", raw, 80)[0]
        if 84 + face_count * 50 == len(raw):
            dtype = np.dtype(
                [
                    ("normal", "<f4", (3,)),
                    ("vertices", "<f4", (3, 3)),
                    ("attribute", "<u2"),
                ]
            )
            return np.frombuffer(raw, dtype=dtype, offset=84, count=face_count)[
                "vertices"
            ].astype(np.float64)

    matches = VERTEX_RE.findall(raw)
    if not matches or len(matches) % 3:
        raise ValueError("not a valid binary or ASCII STL")
    values = np.asarray(matches, dtype=np.float64)
    return values.reshape((-1, 3, 3))


def mesh_metrics(triangles: np.ndarray) -> dict:
    flat = triangles.reshape((-1, 3))
    minimum = flat.min(axis=0)
    maximum = flat.max(axis=0)
    extents = maximum - minimum

    rounded = np.round(flat, decimals=5)
    vertices, inverse = np.unique(rounded, axis=0, return_inverse=True)
    faces = inverse.reshape((-1, 3))

    edges = np.concatenate(
        (faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]), axis=0
    )
    edges.sort(axis=1)
    _unique_edges, edge_counts = np.unique(edges, axis=0, return_counts=True)
    watertight = bool(edge_counts.size and np.all(edge_counts == 2))

    cross = np.cross(triangles[:, 1] - triangles[:, 0],
                     triangles[:, 2] - triangles[:, 0])
    surface_area = float(0.5 * np.linalg.norm(cross, axis=1).sum())
    signed_tetra_volume = np.einsum(
        "ij,ij->i", triangles[:, 0],
        np.cross(triangles[:, 1], triangles[:, 2])
    ) / 6.0
    signed_volume = float(signed_tetra_volume.sum())
    volume = abs(signed_volume) if watertight else float("nan")

    if abs(signed_volume) > 1e-9:
        centroid = (
            ((triangles[:, 0] + triangles[:, 1] + triangles[:, 2]) / 4.0)
            * signed_tetra_volume[:, None]
        ).sum(axis=0) / signed_volume
    else:
        centroid = vertices.mean(axis=0)

    centered = vertices - vertices.mean(axis=0)
    covariance = np.cov(centered, rowvar=False)
    eigenvalues = np.sort(np.linalg.eigvalsh(covariance))[::-1]

    return {
        "triangles": int(len(triangles)),
        "vertices": int(len(vertices)),
        "min": minimum,
        "max": maximum,
        "extents": extents,
        "bbox_center": (minimum + maximum) / 2.0,
        "centroid": centroid,
        "surface_area": surface_area,
        "volume": volume,
        "watertight": watertight,
        "pca": eigenvalues,
    }


def vector_text(values: np.ndarray) -> str:
    return "x".join(f"{value:.3f}" for value in values)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--match", default="*.stl")
    args = parser.parse_args()

    root = args.root.resolve()
    paths = sorted(root.rglob(args.match), key=lambda item: str(item).lower())
    print(
        "file\ttriangles\tvertices\tsize_xyz_mm\tmin_xyz\tmax_xyz\t"
        "centroid_xyz\tarea_mm2\tvolume_mm3\twatertight\tpca"
    )
    for path in paths:
        try:
            metrics = mesh_metrics(load_stl(path))
            volume = (
                f"{metrics['volume']:.3f}"
                if np.isfinite(metrics["volume"])
                else "NA"
            )
            print(
                f"{path.relative_to(root)}\t{metrics['triangles']}\t"
                f"{metrics['vertices']}\t{vector_text(metrics['extents'])}\t"
                f"{vector_text(metrics['min'])}\t{vector_text(metrics['max'])}\t"
                f"{vector_text(metrics['centroid'])}\t"
                f"{metrics['surface_area']:.3f}\t{volume}\t"
                f"{int(metrics['watertight'])}\t{vector_text(metrics['pca'])}"
            )
        except Exception as exc:  # Keep the inventory useful if one file is bad.
            print(f"{path.relative_to(root)}\tERROR\t{exc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
