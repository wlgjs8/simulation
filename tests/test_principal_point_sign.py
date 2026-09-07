"""The principal-point knob's sign, pinned against USD's own projection matrix.

EVAL_PP_LEFT/RIGHT are quoted in the convention the collected intrinsics use: pixels from the
image centre, +x right and +y down. USD takes an APERTURE OFFSET, which moves the frustum
window rather than the principal point, so the two differ by a sign on both axes -- and the
first implementation had it backwards. This is the check that caught it.

Runs anywhere pxr is importable; no Isaac, no GPU.
"""
import unittest

from pxr import Gf

H_APERTURE, V_APERTURE, FOCAL = 17.8885, 13.4524, 11.0
W, H = 640, 480


def principal_point_px(ppx, ppy):
    """Where the optical axis lands, given the knob's requested (ppx, ppy) in pixels."""
    c = Gf.Camera()
    c.projection = Gf.Camera.Perspective
    c.focalLength = FOCAL
    c.horizontalAperture = H_APERTURE
    c.verticalAperture = V_APERTURE
    # the formula under test, verbatim from eval_closed_loop.py / render_wrist_cam.py
    c.horizontalApertureOffset = -ppx / W * H_APERTURE
    c.verticalApertureOffset = ppy / H * V_APERTURE
    q = Gf.Vec4d(0.0, 0.0, -1.0, 1.0) * c.frustum.ComputeProjectionMatrix()
    return W / 2 * (1 + q[0] / q[3]), H / 2 * (1 - q[1] / q[3])


class PrincipalPointSign(unittest.TestCase):
    def test_zero_is_centred(self):
        x, y = principal_point_px(0.0, 0.0)
        self.assertAlmostEqual(x, 320.0, places=3)
        self.assertAlmostEqual(y, 240.0, places=3)

    def test_positive_is_right_and_down(self):
        x, y = principal_point_px(+40.0, +40.0)
        self.assertAlmostEqual(x, 360.0, places=3)
        self.assertAlmostEqual(y, 280.0, places=3)

    def test_collection_units_land_where_quoted(self):
        # left (319.08, 229.52) and right (316.93, 238.20) on a 640x480 frame
        for (dx, dy), (want_x, want_y) in (((-0.92, -10.48), (319.08, 229.52)),
                                           ((-3.07, -1.80), (316.93, 238.20))):
            x, y = principal_point_px(dx, dy)
            self.assertAlmostEqual(x, want_x, places=2)
            self.assertAlmostEqual(y, want_y, places=2)


if __name__ == "__main__":
    unittest.main()
