#!/usr/bin/env python3
"""Extract repeated axis-aligned circles/cylinders from an AP203 STEP file.

The robot STEP exported by Shapr3D contains one flattened product, so its
geometric circle/cylinder definitions are more useful than product names for
recovering joint axes.
"""

from __future__ import annotations

import argparse
import collections
import re
from pathlib import Path

import numpy as np


ENTITY_RE = re.compile(r"#(\d+)\s*=\s*([^;]+);", re.DOTALL)
REF_RE = re.compile(r"#(\d+)")
NUMBER_RE = re.compile(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:E[-+]?\d+)?", re.I)
RADIUS_RE = re.compile(
    r",\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:E[-+]?\d+)?)"
    r"(?:\s*,|\s*\))",
    re.I,
)


def numbers(text: str) -> tuple[float, ...]:
    body = text[text.rfind("(") + 1:text.rfind(")")]
    return tuple(float(value) for value in NUMBER_RE.findall(body))


def axis_family(direction: np.ndarray, tolerance: float = 1e-5):
    direction = direction / np.linalg.norm(direction)
    index = int(np.argmax(np.abs(direction)))
    if abs(direction[index]) < 1.0 - tolerance:
        return None
    return "XYZ"[index], index


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("step", type=Path)
    parser.add_argument("--minimum-count", type=int, default=2)
    parser.add_argument("--limit", type=int, default=80)
    args = parser.parse_args()

    text = args.step.read_text(encoding="utf-8", errors="replace")
    entities = {int(key): value.strip() for key, value in ENTITY_RE.findall(text)}

    points: dict[int, np.ndarray] = {}
    directions: dict[int, np.ndarray] = {}
    placements: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    for key, value in entities.items():
        if value.startswith("CARTESIAN_POINT"):
            points[key] = np.asarray(numbers(value), dtype=float)
        elif value.startswith("DIRECTION"):
            directions[key] = np.asarray(numbers(value), dtype=float)

    for key, value in entities.items():
        if not value.startswith("AXIS2_PLACEMENT_3D"):
            continue
        refs = [int(ref) for ref in REF_RE.findall(value)]
        if len(refs) >= 2 and refs[0] in points and refs[1] in directions:
            placements[key] = points[refs[0]], directions[refs[1]]

    groups = collections.defaultdict(lambda: {"count": 0, "radii": [], "along": [], "kinds": collections.Counter()})
    kinds = ("CIRCLE", "CYLINDRICAL_SURFACE", "CONICAL_SURFACE", "TOROIDAL_SURFACE")
    for _key, value in entities.items():
        kind = next((candidate for candidate in kinds if value.startswith(candidate)), None)
        if kind is None:
            continue
        refs = [int(ref) for ref in REF_RE.findall(value)]
        if not refs or refs[0] not in placements:
            continue
        point, direction = placements[refs[0]]
        family = axis_family(direction)
        if family is None:
            continue
        axis_name, axis_index = family
        transverse = [index for index in range(3) if index != axis_index]
        line = tuple(round(float(point[index]), 3) for index in transverse)
        radius_match = RADIUS_RE.search(value)
        radius = float(radius_match.group(1)) if radius_match else float("nan")
        group = groups[(axis_name, line)]
        group["count"] += 1
        group["radii"].append(radius)
        group["along"].append(float(point[axis_index]))
        group["kinds"][kind] += 1

    print(f"entities={len(entities)} points={len(points)} placements={len(placements)} axis_groups={len(groups)}")
    print("axis\tline_transverse_mm\tcount\talong_span_mm\tradii_mm\tkinds")
    selected = [item for item in groups.items() if item[1]["count"] >= args.minimum_count]
    selected.sort(key=lambda item: (-item[1]["count"], item[0]))
    for (axis_name, line), group in selected[:args.limit]:
        radii = sorted({round(value, 3) for value in group["radii"] if np.isfinite(value)})
        along = group["along"]
        kind_text = ",".join(f"{key}:{value}" for key, value in sorted(group["kinds"].items()))
        print(
            f"{axis_name}\t{line}\t{group['count']}\t"
            f"{min(along):.3f}..{max(along):.3f}\t{radii}\t{kind_text}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
