"""
Offline test for apriltag_cluster.py (run on a PC, not the Limelight).

Renders a synthetic cluster sticker at known camera poses through the
calibrated lens (default.cal), hides different subsets of its tags, and
checks range, bearing, height, elevation, sticker pitch, and roll.

The Limelight's `from apriltag import apriltag` is emulated with
pupil_apriltags, which wraps the same AprilTag3 C library.

    pip install opencv-python numpy pupil-apriltags
    python test_apriltag_cluster.py
"""

import itertools
import math
import sys
import time
import types

import cv2
import numpy as np
from pupil_apriltags import Detector


# --- Emulate Limelight's built-in `apriltag` module -------------------------
class _LLApriltag:
    def __init__(self, family, threads=1, maxhamming=1, decimate=2.0,
                 blur=0.0, refine_edges=True, debug=False):
        self._d = Detector(families=family, nthreads=threads,
                           quad_decimate=decimate, quad_sigma=blur,
                           refine_edges=refine_edges)

    def detect(self, gray):
        return [{"id": d.tag_id, "hamming": d.hamming, "margin": d.decision_margin,
                 "center": d.center, "lb-rb-rt-lt": d.corners}
                for d in self._d.detect(gray)]


sys.modules["apriltag"] = types.SimpleNamespace(apriltag=_LLApriltag)
import apriltag_cluster as ac  # noqa: E402

W, H = 640, 480
PPI = 60  # texture pixels per inch
K = ac.camera_matrix(W)
D = ac.CAL_DIST

# Lens distortion map: for each distorted output pixel, where it comes from
# in the ideal pinhole image.
_grid = np.stack(np.meshgrid(np.arange(W), np.arange(H)), -1).reshape(-1, 1, 2)
_src = cv2.undistortPoints(_grid.astype(np.float32), K, D, P=K).reshape(H, W, 2)
MAP_X, MAP_Y = _src[..., 0].copy(), _src[..., 1].copy()


def official_tag(tag_id, px):
    # OpenCV's tag36h11 bitmap is the official AprilTag rotated 180 deg.
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    m = np.rot90(cv2.aruco.generateImageMarker(d, tag_id, 8), 2)
    return cv2.resize(m, (px, px), interpolation=cv2.INTER_NEAREST)


def render(base, visible, rvec, tvec):
    sw, sh = 19.0, 5.5
    tex = np.full((int(sh * PPI), int(sw * PPI)), 255, np.uint8)
    tpx = int(round(3.25 * PPI))
    for i, cx in enumerate(ac.TAG_CENTER_X):
        if i in visible:
            x0 = int(round((cx - ac.HALF + sw / 2) * PPI))
            y0 = int(round((-ac.HALF + sh / 2) * PPI))
            tex[y0:y0 + tpx, x0:x0 + tpx] = official_tag(base + i, tpx)
    S = np.array([[1 / PPI, 0, -sw / 2], [0, 1 / PPI, -sh / 2], [0, 0, 1]])
    R, _ = cv2.Rodrigues(np.asarray(rvec, float))
    Hm = K @ np.column_stack([R[:, 0], R[:, 1], tvec]) @ S
    img = cv2.warpPerspective(tex, Hm, (W, H), flags=cv2.INTER_AREA,
                              borderValue=110)
    img = cv2.remap(img, MAP_X, MAP_Y, cv2.INTER_LINEAR, borderValue=110)
    img = cv2.GaussianBlur(img, (3, 3), 0)
    img = np.clip(img + np.random.default_rng(0).normal(0, 3, img.shape), 0, 255)
    return cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)


def euler(roll, pitch, yaw):
    r = lambda v: cv2.Rodrigues(np.array(v, float))[0]
    R = r([0, math.radians(yaw), 0]) @ r([math.radians(pitch), 0, 0]) @ r([0, 0, math.radians(roll)])
    return cv2.Rodrigues(R)[0].ravel()


POSES = [  # name, roll, pitch, yaw, tvec (in)
    ("head-on 24in", 0, 0, 0, (0, 0, 24)),
    ("off-center 30in", 10, 0, 0, (-3, 2, 30)),
    ("tilted 25deg", 0, 25, 0, (2, -1, 26)),
    ("yawed 30deg", 0, 0, 30, (1, 1, 28)),
    ("upside down", 180, 15, -10, (0, 0, 32)),
    ("rotated 70", 70, 0, 0, (0, 0, 34)),
]


def expected(right, up, fwd):
    """Expected (range, bearing, height, elevation) for a level-frame position."""
    return (math.hypot(right, fwd), math.degrees(math.atan2(-right, fwd)),
            up, math.degrees(math.atan2(up, fwd)))


def position(py):
    """Level-frame (right, up, forward) rebuilt from range/bearing/height."""
    b = math.radians(py[3])
    return (-py[2] * math.sin(b), py[4], py[2] * math.cos(b))


def to_camera(v, cam_pitch):
    """Level-frame vector (right, up, forward) -> OpenCV camera frame
    (right, down, forward) for a camera pitched up by cam_pitch degrees.
    This is the inverse of the script's rotation."""
    a = math.radians(cam_pitch)
    right, up, fwd = v
    up_c = up * math.cos(a) - fwd * math.sin(a)
    fwd_c = up * math.sin(a) + fwd * math.cos(a)
    return np.array([right, -up_c, fwd_c])


def sticker_pitch_truth(rvec, cam_pitch=0.0):
    R, _ = cv2.Rodrigues(np.asarray(rvec, float))
    n = -R[:, 2]  # direction the printed face points, camera frame
    a = math.radians(cam_pitch)
    up, fwd = -n[1], n[2]
    n_up = up * math.cos(a) + fwd * math.sin(a)
    n_fwd = fwd * math.cos(a) - up * math.sin(a)
    return math.degrees(math.atan2(-n_fwd, -n_up))


def main():
    failures, total, times, worst_in = 0, 0, [], 0.0
    for name, r, p, y, t in POSES:
        rvec, tvec = euler(r, p, y), np.array(t, float)
        truth = (tvec[0], -tvec[1], tvec[2])  # camera level: level frame = camera frame
        exp_rng, exp_bearing, exp_height, exp_elev = expected(*truth)
        exp_st_pitch = sticker_pitch_truth(rvec)
        for n in (1, 2, 3, 4):
            for vis in itertools.combinations(range(4), n):
                img = render(38, set(vis), rvec, tvec)
                t0 = time.perf_counter()
                _, _, py = ac.runPipeline(img, [0])  # default = blue alliance
                times.append(time.perf_counter() - t0)
                in_err = math.dist(position(py), truth)
                worst_in = max(worst_in, in_err)
                ang_err = max(abs(py[3] - exp_bearing), abs(py[5] - exp_elev))
                pitch_err = abs(py[6] - exp_st_pitch)
                roll_err = abs((py[7] + r + 180) % 360 - 180)
                passed = (len(py) == 8 and py[0] == 38 and py[1] == n
                          and in_err < (0.03 if n == 1 else 0.02) * tvec[2]
                          and abs(py[2] - exp_rng) < 0.03 * tvec[2]
                          and abs(py[4] - exp_height) < 0.03 * tvec[2]
                          and ang_err < (0.5 if n == 1 else 0.25)
                          and pitch_err < (5 if n == 1 else 2) and roll_err < 5)
                total += 1
                if not passed:
                    failures += 1
                    print("FAIL %-16s tags %s  dist_err %.2fin  ang_err %.2f  pitch_err %.1f  roll_err %.1f"
                          % (name, vis, in_err, ang_err, pitch_err, roll_err))

    blank = np.full((H, W, 3), 128, np.uint8)
    assert ac.runPipeline(blank, [0])[2] == ac.NO_TARGET
    blue = render(38, {0, 1}, euler(0, 0, 0), np.array([0, 0, 24.0]))
    assert ac.runPipeline(blue, [1])[2] == ac.NO_TARGET  # red ignores blue clusters
    assert ac.runPipeline(blue, [0])[2][0] == 38
    red = render(34, {2, 3}, euler(0, 0, 0), np.array([0, 0, 24.0]))
    assert ac.runPipeline(red, [1])[2][0] == 34
    assert ac.runPipeline(red, [0])[2] == ac.NO_TARGET  # blue ignores red clusters

    # Camera pitch: place the cluster in the level frame (right, up, forward),
    # express it in the pitched camera's frame, render, and check the script
    # recovers range/bearing/height/elevation.
    for cam_pitch, level in [(20, (2, 10, 25)), (45, (-3, 20, 20)), (90, (1, 30, 0.0)),
                             (90, (2, 25, -6))]:
        tvec = to_camera(level, cam_pitch)
        py = ac.runPipeline(render(38, {0, 1, 2, 3}, euler(0, 0, 0), tvec), [0, cam_pitch])[2]
        err = math.dist(position(py), level)
        _, exp_bearing, _, exp_elev = expected(*level)
        ang = max(abs(py[3] - exp_bearing), abs(py[5] - exp_elev))
        if math.hypot(level[0], level[2]) < 5:
            ang = 0  # directly overhead: bearing/elevation are ill-defined
        assert py[0] == 38 and err < 0.02 * math.hypot(*level) and ang < 0.5, (cam_pitch, py)

    # Sticker pitch on a real-looking setup: camera pitched up 30 deg, cluster
    # on the underside of a hive 20 in up and 30 in ahead, hive tilted toward
    # (+) or away from (-) the camera.
    cam_pitch, center = 30, (0, 20, 30)
    for tilt in (0, 20, -15, 45):
        b = math.radians(tilt)
        # (right, up, forward) is left-handed, so build the sticker axes in the
        # camera frame and take y = z x x there to get a proper rotation.
        x_cam = to_camera((1, 0, 0), cam_pitch)                       # along the sticker
        z_cam = to_camera((0, math.cos(b), math.sin(b)), cam_pitch)   # into the sticker
        R = np.column_stack([x_cam, np.cross(z_cam, x_cam), z_cam])
        rvec = cv2.Rodrigues(R)[0].ravel()
        py = ac.runPipeline(render(38, {0, 1, 2, 3}, rvec, to_camera(center, cam_pitch)),
                            [0, cam_pitch])[2]
        assert py[0] == 38 and abs(py[6] - tilt) < 2, (tilt, py)
        assert abs(py[2] - 30) < 0.6 and abs(py[4] - 20) < 0.6, (tilt, py)

    # Weak detections are ignored: same image, detector reports a low decision
    # margin or too many corrected bits.
    img = render(38, {0, 1}, euler(0, 0, 0), np.array([0, 0, 24.0]))
    real_detect = ac.detector.detect
    for bad in ({"margin": ac.MIN_DECISION_MARGIN - 1}, {"hamming": ac.MAX_HAMMING + 1}):
        ac.detector.detect = lambda g, bad=bad: [{**d, **bad} for d in real_detect(g)]
        try:
            assert ac.runPipeline(img, [0])[2] == ac.NO_TARGET, bad
        finally:
            ac.detector.detect = real_detect
    assert ac.runPipeline(img, [0])[2][:2] == [38, 2]

    # Both alliance clusters in view: near one with 1 tag vs far one with 4.
    # The near one (biggest tag) must win.
    near = render(34, {2}, euler(0, 0, 0), np.array([-9, 0, 22.0]))
    far = render(30, {0, 1, 2, 3}, euler(0, 0, 0), np.array([11, 0, 45.0]))
    both = np.where(np.abs(near.astype(int) - 110) > 20, near, far).astype(np.uint8)
    py = ac.runPipeline(both, [1])[2]
    assert py[0] == 34 and py[1] == 1, py

    print("%d/%d cases passed, worst position error %.2f in, median runPipeline %.1f ms"
          % (total - failures, total, worst_in, 1000 * sorted(times)[len(times) // 2]))
    return failures


if __name__ == "__main__":
    raise SystemExit(1 if main() else 0)
