"""
BIOBUZZ AprilTag Cluster center - Limelight 3A Python SnapScript.

Uses Limelight's built-in AprilTag3 detector and finds the center of your
alliance's AprilTag Clusters from any 1-4 visible tags of a cluster.

Cluster layout (Manual TU03 Fig. 9-15), tag centers in inches:
    [n+0] -6.5   [n+1] -2.75   (center)   [n+2] +2.75   [n+3] +6.5
Tags are 3.25 in squares. Base IDs n: 30 red scoring, 34 red audience,
38 blue audience, 42 blue scoring.

Math: each tag's 4 corners have known positions on the sticker. Undistort
the corners with the camera calibration (default.cal) and solve the cluster
pose with planar PnP (IPPE) using all visible corners (one tag is enough),
with the cluster center as the origin. The pose is rotated into a level frame
using the camera pitch; range, bearing, height and elevation come from the
cluster center's position, and sticker pitch and roll from its rotation.

llrobot[0]: alliance. 1 = red (clusters 30, 34), 0 = blue (clusters 38, 42).
Blue is the default, since llrobot is all zeros until the robot sets it.
llrobot[1]: camera pitch in degrees, + = tilted up (0 = level, 90 = straight
up). Outputs are corrected to a level frame, as if the camera were level.
Only that alliance's two clusters are tracked; if both are visible, the closer
one (the one containing the biggest tag in the image) is reported.

llpython:
    [0] cluster base ID (-1 if none)
    [1] number of tags seen (0-4)
    [2] range: horizontal distance along the floor to the cluster center (in)
    [3] bearing (deg, + = left, like the FTC SDK) = atan(-right / forward)
    [4] height of the cluster center above the camera lens (in)
    [5] elevation (deg, + = up) = atan(height / forward)
    [6] sticker pitch (deg): tilt of the sticker from flat. 0 = facing straight
        down, + = tilted to face toward the camera, 90 = facing the camera
    [7] roll (deg, |roll| < 90 = right-side up in the image)
Right and forward can be recovered: right = -range*sin(bearing),
forward = range*cos(bearing).

Provided by Rocket Robotics, FTC 21615
Copyright (c) 2026 Maxim Vanier
SPDX-License-Identifier: MIT
"""

import math

import cv2
import numpy as np
from apriltag import apriltag

# Image shrink factor while searching for tags: higher = faster but shorter
# range, 1.0 = full resolution. 2.0 is a good balance; tune on the robot.
DECIMATE = 2.0

# 1 = draw the readout box on the stream, 0 = skip it (faster, e.g. for matches)
READOUT = 1

TAG_CENTER_X = (-6.5, -2.75, 2.75, 6.5)
HALF = 3.25 / 2
RED_CLUSTERS, BLUE_CLUSTERS = (30, 34), (38, 42)
CLUSTER_NAMES = {30: "RED SCORING", 34: "RED AUDIENCE",
                 38: "BLUE AUDIENCE", 42: "BLUE SCORING"}
NO_TARGET = [-1, 0, 0, 0, 0, 0, 0, 0]

# Readout colors (BGR)
RED_TEXT, BLUE_TEXT = (90, 90, 255), (255, 170, 60)
LABEL_TEXT, VALUE_TEXT = (170, 170, 170), (255, 255, 255)

# Camera calibration from default.cal (measured at 1280x960). Scaled to the
# pipeline resolution at runtime. Distortion order is OpenCV's k1,k2,p1,p2,k3.
CAL_RES_X = 1280.0
CAL_K = np.array([[1221.445, 0.0, 637.226],
                  [0.0, 1223.398, 502.549],
                  [0.0, 0.0, 1.0]])
CAL_DIST = np.array([0.17716814022120847, -0.4573406421907695,
                     0.002752733239126054, 0.00036002841308005,
                     0.17825941966394934])

# Sticker coordinates (inches, +x right, +y down, z = 0) of each tag's
# corners, in the detector's 'lb-rb-rt-lt' order.
TAG_CORNERS = [np.array([[x - HALF, HALF, 0], [x + HALF, HALF, 0],
                         [x + HALF, -HALF, 0], [x - HALF, -HALF, 0]], np.float32)
               for x in TAG_CENTER_X]

# 4 threads = one per core on the Limelight 3A. DECIMATE is set at the top.
detector = apriltag("tag36h11", threads=4, decimate=DECIMATE)

# Detections weaker than this are ignored (false positives, blur, glare).
# decision margin = how clearly the tag's bits decoded; hamming = bits corrected.
MIN_DECISION_MARGIN = 20.0
MAX_HAMMING = 1

_K_cache = {}


def camera_matrix(width):
    """Calibration intrinsics scaled to the current image width."""
    if width not in _K_cache:
        K = CAL_K * (width / CAL_RES_X)
        K[2, 2] = 1.0
        _K_cache[width] = K
    return _K_cache[width]


def draw_readout(image, is_red, cam_pitch, out):
    """Formatted box of the llpython outputs in the bottom-left corner."""
    if not READOUT:
        return
    scale = image.shape[1] / 640.0
    font, fs, th = cv2.FONT_HERSHEY_SIMPLEX, 0.45 * scale, max(1, int(round(scale)))
    line_h, pad = int(20 * scale), int(8 * scale)
    color = RED_TEXT if is_red else BLUE_TEXT
    cluster_id, num_tags, rng, bearing, height, elevation, st_pitch, roll = out

    if cluster_id < 0:
        header = "%s ALLIANCE  -  no cluster   cam pitch %+.1f" % ("RED" if is_red else "BLUE", cam_pitch)
        rows = []
    else:
        header = "%s #%d   %d/4 tags   %s   cam pitch %+.1f" % (
            CLUSTER_NAMES[cluster_id], cluster_id, num_tags,
            "upright" if abs(roll) < 90 else "upside down", cam_pitch)
        # Round before formatting (+ 0.0) so tiny negatives don't print as -0.0
        f = lambda v, d, unit: "%+.*f %s" % (d, round(v, d) + 0.0, unit)
        rows = [("range", f(rng, 1, "in"), "bearing", f(bearing, 2, "deg")),
                ("height", f(height, 1, "in"), "elevation", f(elevation, 2, "deg")),
                ("pitch", f(st_pitch, 1, "deg"), "roll", f(roll, 1, "deg"))]

    # Column layout: label, right-aligned value, label, right-aligned value.
    width = lambda text: cv2.getTextSize(text, font, fs, th)[0][0]
    # label 1 x, value 1 right edge, label 2 x, value 2 right edge
    cols = [0, int(128 * scale), int(142 * scale), int(312 * scale)]
    box_w = max(cols[-1], width(header)) + 2 * pad
    box_h = (len(rows) + 1) * line_h + pad
    x0, y1 = int(6 * scale), image.shape[0] - int(6 * scale)
    y0 = y1 - box_h

    roi = image[y0:y1, x0:x0 + box_w]
    roi[:] = (roi * 0.35).astype(np.uint8)  # darken behind the text
    cv2.rectangle(image, (x0, y0), (x0 + box_w, y1), color, 1)

    base_x, base_y = x0 + pad, y0 + line_h - int(4 * scale)
    cv2.putText(image, header, (base_x, base_y), font, fs, color, th, cv2.LINE_AA)
    for i, (l1, v1, l2, v2) in enumerate(rows):
        yy = base_y + (i + 1) * line_h
        cv2.putText(image, l1, (base_x + cols[0], yy), font, fs, LABEL_TEXT, th, cv2.LINE_AA)
        cv2.putText(image, v1, (base_x + cols[1] - width(v1), yy), font, fs, VALUE_TEXT, th, cv2.LINE_AA)
        cv2.putText(image, l2, (base_x + cols[2], yy), font, fs, LABEL_TEXT, th, cv2.LINE_AA)
        cv2.putText(image, v2, (base_x + cols[3] - width(v2), yy), font, fs, VALUE_TEXT, th, cv2.LINE_AA)


def runPipeline(image, llrobot):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    is_red = len(llrobot) and int(llrobot[0]) == 1
    tracked = RED_CLUSTERS if is_red else BLUE_CLUSTERS
    cam_pitch = float(llrobot[1]) if len(llrobot) > 1 else 0.0

    # Group detected corners by cluster: {base_id: (sticker_pts, image_pts)}
    clusters = {}
    for det in detector.detect(gray):
        tag_id = det['id']
        base = 30 + (tag_id - 30) // 4 * 4
        if base not in tracked:
            continue
        corners = np.asarray(det['lb-rb-rt-lt'], np.float32)
        if det['margin'] < MIN_DECISION_MARGIN or det['hamming'] > MAX_HAMMING:
            # Rejected: thin orange outline, so filtering is visible while tuning
            cv2.polylines(image, [corners.astype(np.int32)], True, (0, 140, 255), 1)
            continue
        obj, img = clusters.setdefault(base, ([], []))
        obj.append(TAG_CORNERS[tag_id - base])
        img.append(corners)
        cv2.polylines(image, [corners.astype(np.int32)], True, (0, 255, 0), 2)

    if not clusters:
        draw_readout(image, is_red, cam_pitch, NO_TARGET)
        return np.array([[]]), image, NO_TARGET

    # Closer cluster = the one containing the biggest tag in the image.
    target = max(clusters, key=lambda b: max(cv2.contourArea(c) for c in clusters[b][1]))
    obj, img = clusters[target]

    # Undistort the corners to normalized camera coords (x/z, y/z), then solve
    # the cluster pose with planar PnP. The camera matrix is the identity here.
    K = camera_matrix(gray.shape[1])
    norm = cv2.undistortPoints(np.vstack(img).reshape(-1, 1, 2), K, CAL_DIST)
    obj_pts = np.vstack(obj)
    ok, rvec, t = cv2.solvePnP(obj_pts, norm, np.eye(3), None,
                               flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        draw_readout(image, is_red, cam_pitch, NO_TARGET)
        return np.array([[]]), image, NO_TARGET
    if len(obj) > 1:
        # With 2+ tags, polish the pose with Levenberg-Marquardt on all corners.
        rvec, t = cv2.solvePnPRefineLM(obj_pts, norm, np.eye(3), None, rvec, t)
    t = t.ravel()  # cluster center from the camera, inches (x right, y down, z forward)

    # Undo the camera pitch: rotate camera-frame vectors (x right, y down,
    # z forward) about the camera's x axis into a level frame (right, up, forward).
    a = math.radians(cam_pitch)

    def level(v):
        up, fwd = -v[1], v[2]
        return (v[0], up * math.cos(a) + fwd * math.sin(a),
                fwd * math.cos(a) - up * math.sin(a))

    right, height, fwd = level(t)
    rng = math.hypot(right, fwd)
    bearing = math.degrees(math.atan2(-right, fwd))
    elevation = math.degrees(math.atan2(height, fwd))

    # Sticker pitch from the direction its printed face points (-z of the
    # sticker frame): angle away from straight down, + when tilted toward the camera.
    R, _ = cv2.Rodrigues(rvec)
    _, n_up, n_fwd = level(-R[:, 2])
    st_pitch = math.degrees(math.atan2(-n_fwd, -n_up))

    # Project the center, a point 1 in along +x (for roll), and the cluster
    # outline into the image.
    pts = np.array([[0, 0, 0], [1, 0, 0],
                    [-8.125, -HALF, 0], [8.125, -HALF, 0],
                    [8.125, HALF, 0], [-8.125, HALF, 0]], np.float32)
    p = cv2.projectPoints(pts, rvec, t, K, CAL_DIST)[0].reshape(-1, 2)
    cx, cy = p[0]
    roll = math.degrees(math.atan2(-(p[1][1] - cy), p[1][0] - cx))

    outline = p[2:].astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(image, [outline], True, (255, 0, 255), 1)
    cv2.drawMarker(image, (int(cx), int(cy)), (255, 0, 255), cv2.MARKER_CROSS, 30, 2)

    out = [target, len(obj), rng, bearing, height, elevation, st_pitch, roll]
    draw_readout(image, is_red, cam_pitch, out)
    return outline, image, out
