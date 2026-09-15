"""The threaded shaft mesh is geometry the camera sees; pin its shape without Isaac."""
import math
import pathlib
import sys
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))
import bolt_visual  # noqa: E402

D, L, P = 0.012, 0.025, 0.00175   # M12 x 25, coarse pitch


class ThreadedShaftTest(unittest.TestCase):
    def setUp(self):
        self.pts, self.counts, self.idx, self.nrm = bolt_visual.threaded_shaft_mesh(D, L, P)
        self.ring = self.pts[:-2]

    def test_envelope_matches_the_collider(self):
        # the hidden collider is a cylinder of radius d/2 over 0..L: the visible thread must never
        # stand proud of it, or a rendered crest would sit where contact says there is nothing
        r = np.hypot(self.ring[:, 1], self.ring[:, 2])
        self.assertLessEqual(r.max(), D / 2 + 1e-9)
        self.assertAlmostEqual(self.ring[:, 0].min(), 0.0, places=9)
        self.assertAlmostEqual(self.ring[:, 0].max(), L, places=9)

    def test_iso_basic_depth(self):
        H = math.sqrt(3) / 2 * P
        r = np.hypot(self.ring[:, 1], self.ring[:, 2])
        body = self.ring[:, 0] < L - P            # away from the tip chamfer
        self.assertAlmostEqual(r[body].max(), D / 2, delta=1e-6)
        self.assertAlmostEqual(r[body].min(), D / 2 - 5 / 8 * H, delta=1e-6)

    def test_helix_advances_one_pitch_per_turn(self):
        x = np.array([0.010, 0.010 + P / 4])
        a = bolt_visual.thread_radius(x, 0.0, D, P, L)
        b = bolt_visual.thread_radius(x + P / 4, math.pi / 2, D, P, L)
        np.testing.assert_allclose(a, b, atol=1e-12)

    def test_tip_is_chamfered(self):
        r_tip = bolt_visual.thread_radius(np.full(8, L), np.linspace(0, 2 * math.pi, 8), D, P, L)
        H = math.sqrt(3) / 2 * P
        self.assertTrue(np.all(r_tip < D / 2 - 5 / 8 * H))

    def test_topology_and_normals(self):
        self.assertEqual(int(self.counts.sum()), len(self.idx))
        self.assertTrue(self.idx.min() >= 0 and self.idx.max() < len(self.pts))
        np.testing.assert_allclose(np.linalg.norm(self.nrm, axis=1), 1.0, atol=1e-6)
        # side normals point away from the axis
        radial = self.ring[:, 1:] / np.linalg.norm(self.ring[:, 1:], axis=1, keepdims=True)
        side = (self.ring[:, 0] > P) & (self.ring[:, 0] < L - 2 * P)
        self.assertGreater(np.mean(np.sum(self.nrm[:-2][side, 1:] * radial[side], axis=1) > 0), 0.99)


if __name__ == "__main__":
    unittest.main()


GEOM = pathlib.Path(__file__).resolve().parents[1] / "config/bolts/iso_heads_20260914.json"


class HeadTest(unittest.TestCase):
    def setUp(self):
        self.spec = bolt_visual.load_geometry(GEOM)

    def _envelope(self, mesh, c):
        pts = mesh[0]
        r = np.hypot(pts[:, 1], pts[:, 2])
        self.assertLessEqual(r.max(), c["head_d_m"] / 2 + 1e-9)
        self.assertAlmostEqual(pts[:, 0].min(), -c["head_k_m"], places=9)
        self.assertLessEqual(pts[:, 0].max(), 1e-12)

    def test_black_head_stays_inside_the_unchanged_collider(self):
        c = self.spec["black"]
        self._envelope(bolt_visual.socket_cap_head_mesh(c, c["d_m"]), c)
        self.assertEqual(c["mass_kg_effective"], 0.022)

    def test_gray_head_is_a_low_wide_dome(self):
        c = self.spec["gray"]
        self._envelope(bolt_visual.button_head_mesh(c, c["d_m"]), c)
        arc, rt = bolt_visual.button_profile(c)
        self.assertAlmostEqual(arc[0][0], -c["head_k_m"], places=12)
        self.assertAlmostEqual(arc[0][1], rt, places=12)
        self.assertAlmostEqual(arc[-1][1], c["head_d_m"] / 2, places=12)
        r = [p[1] for p in arc]
        self.assertTrue(all(b >= a - 1e-12 for a, b in zip(r, r[1:])))   # widens monotonically to the rim

    def test_socket_is_a_hexagon_of_the_declared_size(self):
        s = self.spec["gray"]["socket_s_m"]
        self.assertAlmostEqual(float(bolt_visual.hex_radius(math.pi / 6, s)), s / 2, places=12)      # flat
        self.assertAlmostEqual(float(bolt_visual.hex_radius(0.0, s)), s / math.sqrt(3), places=12)  # corner

    def test_collider_is_closed_and_outward(self):
        pts, cnt, idx, _ = bolt_visual.button_head_collider(self.spec["gray"], n_theta=256)
        dome = bolt_visual.mesh_volume(pts, cnt, idx)
        c = self.spec["gray"]
        cyl = math.pi * (c["head_d_m"] / 2) ** 2 * c["head_k_m"]
        self.assertGreater(dome, 0.0)
        self.assertLess(dome, cyl)                    # a dome is less than its bounding cylinder
        self.assertGreater(dome, 0.5 * cyl)

    def test_gray_is_lighter_by_its_shape(self):
        g, b = self.spec["gray"], self.spec["black"]
        self.assertAlmostEqual(g["mass_kg_effective"] / b["mass_kg_effective"],
                               g["collider_volume_m3"] / b["collider_volume_m3"], places=12)
        self.assertLess(g["mass_kg_effective"], b["mass_kg_effective"])

    def test_invalid_spec_is_refused(self):
        import json, tempfile
        bad = json.loads(GEOM.read_text())
        bad["gray"]["socket_depth_m"] = 0.01                  # deeper than the 6.6 mm head
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            json.dump(bad, f)
        with self.assertRaises(ValueError):
            bolt_visual.load_geometry(f.name)


MEASURED = pathlib.Path(__file__).resolve().parents[1] / "config/bolts/measured_20260915.json"


class MeasuredSpecTest(unittest.TestCase):
    """The caliper-measured spec must stay inside the ISO parts it was matched to."""

    def setUp(self):
        self.spec = bolt_visual.load_geometry(MEASURED)

    def test_gray_is_iso_7380_1_m12x20(self):
        g = self.spec["gray"]
        self.assertEqual((g["d_m"], g["length_m"]), (0.012, 0.020))
        self.assertTrue(0.02048 <= g["head_d_m"] <= 0.02100)
        R, k, e, rt = g["head_d_m"] / 2, g["head_k_m"], g["edge_h_m"], g["top_flat_r_m"]
        c = (R ** 2 + e ** 2 - rt ** 2 - k ** 2) / (2 * (k - e))
        self.assertTrue(0.01050 <= math.hypot(R, e + c) <= 0.01120)     # ISO r_f
        self.assertGreater(rt, g["socket_s_m"] / math.sqrt(3))          # the socket fits in the top flat

    def test_black_is_iso_4762_m12x25(self):
        b = self.spec["black"]
        self.assertEqual((b["d_m"], b["length_m"]), (0.012, 0.025))
        self.assertTrue(0.01773 <= b["head_d_m"] <= 0.01827)
        self.assertTrue(0.01157 <= b["head_k_m"] <= 0.01200)
        pts = bolt_visual.socket_cap_head_mesh(b, b["d_m"])[0]
        r = np.hypot(pts[:, 1], pts[:, 2])
        ch = b["chamfer_m"]
        side = (pts[:, 0] <= -ch + 1e-9) & (pts[:, 0] >= -b["head_k_m"] + ch - 1e-9) & (r > 0.008)
        # knurl valleys reach the measured 17.5 mm, crests the 18.0 mm collider
        self.assertAlmostEqual(2 * r[side].min(), 0.0175, places=5)
        self.assertAlmostEqual(2 * r[side].max(), 0.0180, places=5)
