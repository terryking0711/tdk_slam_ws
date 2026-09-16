#!/usr/bin/env python3
"""Estimate the front_lidar/rear_lidar yaw mismatch in robot_params.yaml.

Drive the robot into a long straight corridor and hold it still, then run
this script. It listens to /front/scan_filtered and /rear/scan_filtered,
projects both scans into base_footprint using the extrinsics currently in
robot_params.yaml, fits a line to the left-wall and right-wall points seen
by each lidar, and reports how much the two lidars disagree on the wall
angle. rear_lidar is treated as the reference; the suggested fix only
touches front_lidar.yaw (and, as a rough secondary hint, front_lidar.y).

Usage:
    python3 calibrate_lidar_yaw.py --config ../config/robot_params.yaml
"""
import argparse
import math
import sys

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

WARN_STD_M = 0.05    # fit std above this: still usable but flagged as noisy
REJECT_STD_M = 0.15  # fit std above this: not a single flat wall, discard the side


def load_extrinsic(config_path, key):
    with open(config_path, "r") as f:
        params = yaml.safe_load(f)
    s = params["sensors"][key]
    return s["x"], s["y"], s["yaw"]


def scan_to_points(msg: LaserScan, x0, y0, yaw0):
    ranges = np.array(msg.ranges)
    angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
    valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
    ranges = ranges[valid]
    angles = angles[valid]
    lx = ranges * np.cos(angles)
    ly = ranges * np.sin(angles)
    c, s = math.cos(yaw0), math.sin(yaw0)
    x = x0 + c * lx - s * ly
    y = y0 + s * lx + c * ly
    return np.stack([x, y], axis=1)


def fit_line_angle(points, trim_iters=3, trim_k=3.0, min_thresh_m=0.03):
    """Outlier-robust orthogonal (total-least-squares) line fit via PCA.

    A "wall" cluster picked by a simple |y| > wall_gap threshold can still
    contain a doorway, pillar, or other non-flat feature. Those points blow
    up the fit and bias the angle, so iteratively drop points that are far
    (> trim_k stddev, floored at min_thresh_m) from the current fit and
    refit, like a trimmed/IRLS line fit.

    Returns (angle_rad, centroid, residual_std_m, outlier_fraction, n_kept).
    """
    pts = points
    n0 = len(pts)
    for _ in range(trim_iters):
        centroid = pts.mean(axis=0)
        centered = pts - centroid
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
        direction = vt[0]
        normal = np.array([-direction[1], direction[0]])
        residual = centered @ normal
        thresh = max(trim_k * residual.std(), min_thresh_m)
        keep = np.abs(residual) < thresh
        if keep.all() or keep.sum() < 20:
            break
        pts = pts[keep]

    centroid = pts.mean(axis=0)
    centered = pts - centroid
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    direction = vt[0]
    angle = math.atan2(direction[1], direction[0])
    normal = np.array([-direction[1], direction[0]])
    residual = centered @ normal
    outlier_fraction = 1.0 - len(pts) / n0
    return angle, centroid, float(residual.std()), outlier_fraction, len(pts)


class ScanCollector(Node):
    def __init__(self, samples):
        super().__init__("lidar_yaw_calib")
        self.samples = samples
        self.front_msgs = []
        self.rear_msgs = []
        self.create_subscription(LaserScan, "/front/scan_filtered", self._front_cb, 10)
        self.create_subscription(LaserScan, "/rear/scan_filtered", self._rear_cb, 10)

    def _front_cb(self, msg):
        if len(self.front_msgs) < self.samples:
            self.front_msgs.append(msg)

    def _rear_cb(self, msg):
        if len(self.rear_msgs) < self.samples:
            self.rear_msgs.append(msg)

    def ready(self):
        return len(self.front_msgs) >= self.samples and len(self.rear_msgs) >= self.samples


def analyze_side(front_pts, rear_pts, label):
    if len(front_pts) < 20 or len(rear_pts) < 20:
        print(f"  [{label}] not enough points (front={len(front_pts)}, rear={len(rear_pts)}), skipped")
        return None
    f_angle, f_c, f_std, f_out, f_n = fit_line_angle(front_pts)
    r_angle, r_c, r_std, r_out, r_n = fit_line_angle(rear_pts)
    # a line has no direction, so wrap the difference into [-90, 90) deg
    dyaw = (r_angle - f_angle + math.pi / 2) % math.pi - math.pi / 2
    r_dir = np.array([math.cos(r_angle), math.sin(r_angle)])
    r_normal = np.array([-r_dir[1], r_dir[0]])
    lateral = float((f_c - r_c) @ r_normal)
    print(
        f"  [{label}] front={math.degrees(f_angle):+.3f} deg (std {f_std * 1000:.1f} mm, "
        f"{f_out * 100:.0f}% trimmed, n={f_n})  "
        f"rear={math.degrees(r_angle):+.3f} deg (std {r_std * 1000:.1f} mm, "
        f"{r_out * 100:.0f}% trimmed, n={r_n})  "
        f"-> dyaw={math.degrees(dyaw):+.3f} deg, lateral={lateral * 1000:+.1f} mm"
    )
    if f_std > REJECT_STD_M or r_std > REJECT_STD_M:
        print(f"      rejected: {label} std is way too high even after trimming "
              "-> this isn't a single flat wall (corner/intersection/open area in view?), "
              "discarding this side entirely")
        return None
    if f_std > WARN_STD_M or r_std > WARN_STD_M or f_out > 0.15 or r_out > 0.15:
        print(f"      warning: {label} has a noisy/non-flat fit even after trimming "
              "(door, pillar, or clutter in view?) - weight this side's estimate less")
    weight = 1.0 / (f_std + r_std + 1e-3) ** 2
    return dyaw, lateral, weight


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True, help="path to robot_params.yaml")
    ap.add_argument("--wall-gap", type=float, default=0.4, help="min |y| (m) in base_footprint to count as a wall point")
    ap.add_argument("--samples", type=int, default=5, help="number of scan messages to average per lidar")
    args = ap.parse_args()

    fx, fy, fyaw = load_extrinsic(args.config, "front_lidar")
    rx, ry, ryaw = load_extrinsic(args.config, "rear_lidar")
    print(f"current front_lidar: x={fx:.4f} y={fy:.4f} yaw={fyaw:.6f} rad ({math.degrees(fyaw):.3f} deg)")
    print(f"current rear_lidar : x={rx:.4f} y={ry:.4f} yaw={ryaw:.6f} rad ({math.degrees(ryaw):.3f} deg)")

    rclpy.init()
    node = ScanCollector(args.samples)
    print("Waiting for scans on /front/scan_filtered and /rear/scan_filtered "
          "(keep the robot still in the corridor)...")
    while rclpy.ok() and not node.ready():
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()

    front_pts = np.concatenate([scan_to_points(m, fx, fy, fyaw) for m in node.front_msgs], axis=0)
    rear_pts = np.concatenate([scan_to_points(m, rx, ry, ryaw) for m in node.rear_msgs], axis=0)

    # The corridor walls might run alongside base_footprint's x-axis (walls to
    # the robot's left/right, split by y) or its y-axis (walls ahead/behind,
    # split by x) depending on which way the robot is currently facing/parked.
    # Try both and keep whichever produces the cleaner (higher-weight) fit,
    # instead of assuming the robot is always oriented the same way.
    axis_options = [
        ("y", 1, "wall @ +y (left) ", "wall @ -y (right)"),
        ("x", 0, "wall @ +x (front)", "wall @ -x (rear) "),
    ]

    best_axis_name = None
    best_results = []
    best_total_weight = -1.0

    for axis_name, col, label_a, label_b in axis_options:
        a_front = front_pts[front_pts[:, col] > args.wall_gap]
        a_rear = rear_pts[rear_pts[:, col] > args.wall_gap]
        b_front = front_pts[front_pts[:, col] < -args.wall_gap]
        b_rear = rear_pts[rear_pts[:, col] < -args.wall_gap]

        print(f"\nTrying split along {axis_name}-axis ({label_a.strip()} / {label_b.strip()}):")
        axis_results = []
        for res in (
            analyze_side(a_front, a_rear, label_a),
            analyze_side(b_front, b_rear, label_b),
        ):
            if res:
                axis_results.append(res)

        total_weight = sum(r[2] for r in axis_results)
        if total_weight > best_total_weight:
            best_total_weight = total_weight
            best_axis_name = axis_name
            best_results = axis_results

    results = best_results

    if not results:
        print("\nNo usable wall found on either axis (missing points, or std stayed above "
              f"{REJECT_STD_M * 1000:.0f}mm even after trimming, on every split tried). This spot "
              "doesn't look like it has a single straight flat wall on any side - e.g. a "
              "corner/intersection is in view. Move to a longer straight corridor section (no "
              "corners/doorways in the wall-gap band) and try again. No correction suggested.")
        rclpy.shutdown()
        sys.exit(1)

    print(f"\n==> Using the {best_axis_name}-axis split (better fit quality).")
    weights = np.array([r[2] for r in results])
    dyaws = np.array([r[0] for r in results])
    laterals = np.array([r[1] for r in results])
    dyaw_avg = float(np.average(dyaws, weights=weights))
    lateral_avg = float(np.average(laterals, weights=weights))

    print(f"\nWeighted average over {len(results)} wall(s) (noisy sides count less): "
          f"dyaw={math.degrees(dyaw_avg):+.3f} deg, lateral={lateral_avg * 1000:+.1f} mm")
    print(f"(unweighted mean for comparison: dyaw={math.degrees(float(dyaws.mean())):+.3f} deg, "
          f"lateral={float(laterals.mean()) * 1000:+.1f} mm)")
    # the wall's normal points mostly along whichever axis we split on, so
    # that's the extrinsic component the lateral offset actually corrects
    lateral_field, lateral_old = ("y", fy) if best_axis_name == "y" else ("x", fx)
    print("\nSuggested correction (edit front_lidar in robot_params.yaml):")
    print(f"  yaw: {fyaw:.6f} -> {fyaw + dyaw_avg:.6f}  ({math.degrees(fyaw):.3f} -> "
          f"{math.degrees(fyaw + dyaw_avg):.3f} deg)")
    print(f"  {lateral_field}  : {lateral_old:.4f} -> {lateral_old - lateral_avg:.4f}  (rough estimate, "
          "valid only once the wall runs roughly parallel to the split axis; re-check after "
          "applying the yaw fix)")
    print("\nAfter editing the yaml, restart the sensor/merger nodes and re-run this script; "
          "dyaw and lateral should shrink toward 0.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
