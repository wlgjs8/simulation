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
