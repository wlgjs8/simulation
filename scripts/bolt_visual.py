"""Render-only bolt geometry: a threaded shaft mesh in the bolt's own frame.

The rig's bolts are two smooth cylinders (shaft + head) that are both the collider and what the
wrist cameras see. On the robot every bolt -- the black socket-head cap screws and the gray
button-head screws alike -- is threaded along its whole shaft, and at pre-grasp range the thread
is one of the most visible features of the part. This module builds that thread as a mesh that
is only ever RENDERED: the scene keeps the original cylinder as the (hidden) collider, so contact,
mass and every frozen scene state are untouched and a board scored with the thread differs from
one without it by the pixels alone.

Frame: the bolt prim's local X is the bolt axis (build_scene.py / eval_closed_loop.py create the
cylinders with axis "X"); the shaft occupies 0 <= x <= L with the head underside at x = 0.

Profile: ISO 68-1 basic metric thread. H = sqrt(3)/2 P, major radius R = d/2, basic minor radius
r1 = R - 5/8 H, flat crest P/8 wide, flat root P/4 wide, 60 deg flanks. A right-hand helix: the
profile phase advances by one pitch per turn. The tip carries a 45 deg chamfer that starts just
below the minor radius, as a rolled screw does.
"""
from __future__ import annotations

import math

import numpy as np


def thread_radius(x, theta, d_major: float, pitch: float, length: float) -> np.ndarray:
    """Radius of the threaded surface at axial position x (m) and angle theta (rad)."""
    R = d_major / 2.0
    H = math.sqrt(3.0) / 2.0 * pitch
    r1 = R - 5.0 / 8.0 * H
    u = np.mod(np.asarray(x) / pitch - np.asarray(theta) / (2.0 * math.pi), 1.0)
    t = np.abs(np.where(u >= 0.5, u - 1.0, u))          # distance from the crest centre, [0, 0.5]
    crest, root = 1.0 / 16.0, 0.5 - 1.0 / 8.0             # half-widths of the flats, in pitches
    ramp = np.clip((t - crest) / (root - crest), 0.0, 1.0)
    r = R - ramp * (R - r1)
    chamfer_r = (r1 - 0.1 * (R - r1)) + (length - np.asarray(x))   # 45 deg, starts below the root
    return np.minimum(r, chamfer_r)


def threaded_shaft_mesh(d_major: float, length: float, pitch: float,
                        n_theta: int = 48, samples_per_pitch: int = 16):
    """(points (N,3), face_vertex_counts (F,), face_vertex_indices (sum,), normals (N,3)).

    A ring-by-ring quad grid over [0, length] x [0, 2 pi) closed with a fan at each end. Vertex
    normals are area-weighted face normals, so the renderer shades the flanks instead of a faceted
    cylinder.
    """
    n_x = int(math.ceil(length / pitch * samples_per_pitch)) + 1
    xs = np.linspace(0.0, length, n_x)
    th = np.linspace(0.0, 2.0 * math.pi, n_theta, endpoint=False)
    X, TH = np.meshgrid(xs, th, indexing="ij")                        # (n_x, n_theta)
    Rr = thread_radius(X, TH, d_major, pitch, length)
    ring = np.stack([X, Rr * np.cos(TH), Rr * np.sin(TH)], axis=-1).reshape(-1, 3)
    caps = np.array([[0.0, 0.0, 0.0], [length, 0.0, 0.0]])
    points = np.vstack([ring, caps])
    c0, c1 = len(ring), len(ring) + 1

    idx = np.arange(n_x * n_theta).reshape(n_x, n_theta)
    a, b = idx[:-1, :], np.roll(idx[:-1, :], -1, axis=1)
    c, e = np.roll(idx[1:, :], -1, axis=1), idx[1:, :]
    # right-handed winding a -> b -> c -> e: (theta step) x (axial step) points away from the axis
    quads = np.stack([a, b, c, e], axis=-1).reshape(-1, 4)
    first, last = idx[0], idx[-1]
    tri0 = np.stack([np.full(n_theta, c0), np.roll(first, -1), first], axis=-1)   # faces -x
    tri1 = np.stack([np.full(n_theta, c1), last, np.roll(last, -1)], axis=-1)     # faces +x
    counts = np.concatenate([np.full(len(quads), 4), np.full(2 * n_theta, 3)])
    indices = np.concatenate([quads.ravel(), tri0.ravel(), tri1.ravel()])

    normals = np.zeros_like(points)
    for poly in (quads, tri0, tri1):
        p0, p1, p2 = points[poly[:, 0]], points[poly[:, 1]], points[poly[:, 2]]
        fn = np.cross(p1 - p0, p2 - p0)
        for k in range(poly.shape[1]):
            np.add.at(normals, poly[:, k], fn)
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    return points, counts.astype(np.int32), indices.astype(np.int32), normals
