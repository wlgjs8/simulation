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


# ------------------------------------------------------------------------------------------------
# Heads. On the robot the black bolts are socket-head cap screws (cylindrical, knurled side, hex
# socket) and the gray bolts are button-head socket screws (a low, wide dome with a hex socket);
# the rig used one cylinder for both. A head is built as a surface of revolution walked along its
# meridian in the same orientation as the shaft (face normal = theta_hat x walk direction points
# out of the solid), segment by segment so creases stay sharp.

def hex_radius(theta, s: float):
    """Radius of a regular hexagon (flat-to-flat s) with a corner at theta = 0."""
    phi = np.mod(np.asarray(theta), math.pi / 3.0) - math.pi / 6.0
    return (s / 2.0) / np.cos(phi)


def _revolve(segments, n_theta: int):
    """segments: list of meridian polylines [(x, r), ...]; r is a float or callable(theta).

    Consecutive points of one segment share rings and are joined by quads; different segments do
    not share vertices (a crease). A point with r == 0 collapses to one vertex joined by a fan.
    """
    th = np.linspace(0.0, 2.0 * math.pi, n_theta, endpoint=False)
    pts, counts, idx = [], [], []
    base = 0

    def ring(x, r):
        rr = r(th) if callable(r) else np.full(n_theta, float(r))
        return np.stack([np.full(n_theta, float(x)), rr * np.cos(th), rr * np.sin(th)], axis=-1)

    for seg in segments:
        seg_rings = []
        for x, r in seg:
            if not callable(r) and float(r) == 0.0:
                pts.append(np.array([[float(x), 0.0, 0.0]]))
                seg_rings.append(("c", base)); base += 1
            else:
                pts.append(ring(x, r))
                seg_rings.append(("r", base)); base += n_theta
        for (ka, a0), (kb, b0) in zip(seg_rings[:-1], seg_rings[1:]):
            j = np.arange(n_theta); jn = np.roll(j, -1)
            if ka == "r" and kb == "r":
                q = np.stack([a0 + j, a0 + jn, b0 + jn, b0 + j], axis=-1)
                counts.append(np.full(n_theta, 4)); idx.append(q.ravel())
            elif ka == "c" and kb == "r":
                t = np.stack([np.full(n_theta, a0), b0 + jn, b0 + j], axis=-1)
                counts.append(np.full(n_theta, 3)); idx.append(t.ravel())
            elif ka == "r" and kb == "c":
                t = np.stack([a0 + j, a0 + jn, np.full(n_theta, b0)], axis=-1)
                counts.append(np.full(n_theta, 3)); idx.append(t.ravel())
    points = np.vstack(pts)
    counts = np.concatenate(counts).astype(np.int32)
    indices = np.concatenate(idx).astype(np.int32)
    normals = np.zeros_like(points)
    start = 0
    for c in counts:          # small meshes; clarity over speed
        f = indices[start:start + c]
        fn = np.cross(points[f[1]] - points[f[0]], points[f[2]] - points[f[0]])
        normals[f] += fn
        start += c
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    return points, counts, indices, normals


def button_profile(spec, n_arc: int = 16):
    """Meridian (x, r) of a button head: bearing face at x = 0, dome to a flat top at x = -k.

    A short cylindrical edge, then a circular arc (centre on the axis) through the edge top and the
    outer rim of the top flat. Returns (arc points walked top -> rim, top flat radius).
    """
    R, k = spec["head_d_m"] / 2.0, spec["head_k_m"]
    e, rt = spec["edge_h_m"], spec["top_flat_r_m"]
    # centre c on the axis (x = c): R^2 + (e + c)^2 = rt^2 + (k + c)^2
    c = (R ** 2 + e ** 2 - rt ** 2 - k ** 2) / (2.0 * (k - e))
    rs = math.hypot(R, -e - c)
    a0, a1 = math.atan2(R, -e - c), math.atan2(rt, -k - c)
    arc = [(c + rs * math.cos(a), rs * math.sin(a)) for a in np.linspace(a1, a0, n_arc)]
    return arc, rt


def button_head_mesh(spec, d_shaft: float, n_theta: int = 144):
    R, k = spec["head_d_m"] / 2.0, spec["head_k_m"]
    s, depth = spec["socket_s_m"], spec["socket_depth_m"]
    arc, rt = button_profile(spec)
    hexr = lambda t: hex_radius(t, s)
    segments = [
        [(-k + depth, 0.0), (-k + depth, hexr)],             # socket floor, faces -x
        [(-k + depth, hexr), (-k, hexr)],                     # socket wall, faces the axis
        [(-k, hexr), (-k, rt)],                               # top flat
        arc,                                                  # dome, top -> rim
        [(-spec["edge_h_m"], R), (0.0, R)],                   # cylindrical edge
        [(0.0, R), (0.0, d_shaft / 2.0)],                     # bearing face, faces +x
    ]
    return _revolve(segments, n_theta)


def button_head_collider(spec, n_theta: int = 32):
    """Closed convex dome without the socket, for a convexHull collider."""
    R, k = spec["head_d_m"] / 2.0, spec["head_k_m"]
    arc, rt = button_profile(spec, n_arc=8)
    segments = [[(-k, 0.0), (-k, rt)], arc, [(-spec["edge_h_m"], R), (0.0, R)], [(0.0, R), (0.0, 0.0)]]
    return _revolve(segments, n_theta)


def socket_cap_head_mesh(spec, d_shaft: float, n_theta: int = 288):
    R, k = spec["head_d_m"] / 2.0, spec["head_k_m"]
    s, depth = spec["socket_s_m"], spec["socket_depth_m"]
    ch = spec.get("chamfer_m", 0.0006)
    n_ridge, kd = spec.get("knurl_ridges", 0), spec.get("knurl_depth_m", 0.0)
    knurl = (lambda t: R - kd * 0.5 * (1.0 - np.cos(n_ridge * np.asarray(t)))) if n_ridge and kd else R
    hexr = lambda t: hex_radius(t, s)
    segments = [
        [(-k + depth, 0.0), (-k + depth, hexr)],
        [(-k + depth, hexr), (-k, hexr)],
        [(-k, hexr), (-k, R - ch)],                           # top flat
        [(-k, R - ch), (-k + ch, knurl)],                     # top chamfer
        [(-k + ch, knurl), (-ch, knurl)],                     # knurled side
        [(-ch, knurl), (0.0, R - ch)],                        # bearing-edge chamfer
        [(0.0, R - ch), (0.0, d_shaft / 2.0)],
    ]
    return _revolve(segments, n_theta)


def mesh_volume(points, counts, indices) -> float:
    """Signed volume of a closed, outward-wound polygon mesh (fan-triangulated)."""
    v, start = 0.0, 0
    for c in counts:
        f = indices[start:start + c]
        for i in range(1, c - 1):
            v += np.dot(points[f[0]], np.cross(points[f[i]], points[f[i + 1]])) / 6.0
        start += c
    return float(v)


def load_geometry(path):
    """Validated bolt geometry spec (config/bolts/*.json) plus derived masses and sha256."""
    import hashlib
    import json
    import pathlib
    p = pathlib.Path(path)
    raw = p.read_bytes()
    spec = json.loads(raw)
    if spec.get("schema") != "simulation.bolt_geometry.v1":
        raise ValueError(f"{p}: not a simulation.bolt_geometry.v1 spec")
    for color in ("gray", "black"):
        c = spec[color]
        if c["head"] not in ("socket_cap", "button"):
            raise ValueError(f"{p}: {color}.head must be socket_cap or button")
        for key in ("d_m", "length_m", "head_d_m", "head_k_m", "socket_s_m", "socket_depth_m"):
            if not (isinstance(c[key], (int, float)) and c[key] > 0):
                raise ValueError(f"{p}: {color}.{key} must be a positive number")
        if c["socket_depth_m"] >= c["head_k_m"] or c["socket_s_m"] / math.sqrt(3) * 2 >= c["head_d_m"]:
            raise ValueError(f"{p}: {color} socket does not fit in its head")
        if c["head"] == "button" and not (0 < c["top_flat_r_m"] < c["head_d_m"] / 2
                                          and 0 <= c["edge_h_m"] < c["head_k_m"]):
            raise ValueError(f"{p}: {color} button dome parameters out of range")
    ref = spec["mass_reference"]
    vol = {color: collider_volume(spec[color]) for color in ("gray", "black")}
    for color in ("gray", "black"):
        m = spec[color]["mass_kg"]
        spec[color]["mass_kg_effective"] = (ref["mass_kg"] * vol[color] / vol[ref["color"]]
                                            if m == "scale_by_collider_volume" else float(m))
        spec[color]["collider_volume_m3"] = vol[color]
    spec["sha256"] = hashlib.sha256(raw).hexdigest()
    spec["path"] = str(p.resolve())
    return spec


def collider_volume(c) -> float:
    shaft = math.pi * (c["d_m"] / 2) ** 2 * c["length_m"]
    if c["head"] == "button":
        pts, cnt, idx, _ = button_head_collider(c, n_theta=256)
        return shaft + mesh_volume(pts, cnt, idx)
    return shaft + math.pi * (c["head_d_m"] / 2) ** 2 * c["head_k_m"]


def aabb_proxies(c):
    """(axial centre, half length, radius) cylinders bounding shaft and head, for support checks."""
    return [(c["length_m"] / 2, c["length_m"] / 2, c["d_m"] / 2),
            (-c["head_k_m"] / 2, c["head_k_m"] / 2, c["head_d_m"] / 2)]
