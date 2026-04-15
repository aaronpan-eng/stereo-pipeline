#!/usr/bin/env python3

import argparse
from pathlib import Path

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu


def _parse_csv(path: Path):
    """Parse a EuRoC data.csv, skipping comment/empty lines.
    Returns list of (timestamp_ns: int, remaining_fields: list[str]).
    """
    rows = []
    for ln in path.read_text().splitlines():
        ln = ln.strip()
        if not ln or ln.startswith("#"):
            continue
        parts = [p.strip() for p in ln.split(",")]
        if len(parts) < 2:
            continue
        try:
            ts_ns = int(parts[0])
        except ValueError:
            continue
        rows.append((ts_ns, parts[1:]))
    rows.sort(key=lambda x: x[0])
    return rows


def _ns_to_stamp(ts_ns: int) -> Time:
    """Convert nanosecond timestamp to ROS2 Time msg."""
    stamp = Time()
    stamp.sec = int(ts_ns // 1_000_000_000)
    stamp.nanosec = int(ts_ns % 1_000_000_000)
    return stamp


class EurocPublisher(Node):
    def __init__(self, seq_path: str, rate_scale: float, publish_imu: bool):
        super().__init__("euroc_publisher")
        self.bridge = CvBridge()

        seq = Path(seq_path)
        mav0 = seq / "mav0"
        if not mav0.is_dir():
            self.get_logger().error(f"mav0 directory not found in {seq}")
            return

        # --- Load cam0 / cam1 image lists ---
        cam0_csv = mav0 / "cam0" / "data.csv"
        cam1_csv = mav0 / "cam1" / "data.csv"
        cam0_data_dir = mav0 / "cam0" / "data"
        cam1_data_dir = mav0 / "cam1" / "data"

        cam0_rows = _parse_csv(cam0_csv)
        cam1_rows = _parse_csv(cam1_csv)

        # Build lookup for cam1 by timestamp
        cam1_lookup = {ts: fields for ts, fields in cam1_rows}

        # Pair by matching timestamps
        self.stereo_pairs = []
        for ts_ns, fields in cam0_rows:
            if ts_ns in cam1_lookup:
                left_path = cam0_data_dir / fields[0]
                right_path = cam1_data_dir / cam1_lookup[ts_ns][0]
                self.stereo_pairs.append((ts_ns, left_path, right_path))

        if not self.stereo_pairs:
            self.get_logger().error("No matching stereo pairs found!")
            return

        self.get_logger().info(f"Loaded {len(self.stereo_pairs)} stereo pairs")

        # --- Load IMU data ---
        self.imu_data = []
        self.publish_imu = publish_imu
        if publish_imu:
            imu_csv = mav0 / "imu0" / "data.csv"
            if imu_csv.exists():
                for ts_ns, fields in _parse_csv(imu_csv):
                    if len(fields) >= 6:
                        self.imu_data.append((
                            ts_ns,
                            [float(f) for f in fields[:3]],  # gyro
                            [float(f) for f in fields[3:6]],  # accel
                        ))
                self.get_logger().info(f"Loaded {len(self.imu_data)} IMU samples")

        # --- Publishers ---
        self.pub_left = self.create_publisher(Image, "/left/image_raw", 10)
        self.pub_right = self.create_publisher(Image, "/right/image_raw", 10)
        if publish_imu:
            self.pub_imu = self.create_publisher(Imu, "/imu", 50)

        # --- Merge all events into a single timeline ---
        # Each event: (timestamp_ns, type, index)
        self.events = []
        for i, (ts_ns, _, _) in enumerate(self.stereo_pairs):
            self.events.append((ts_ns, "stereo", i))
        if publish_imu:
            for i, (ts_ns, _, _) in enumerate(self.imu_data):
                self.events.append((ts_ns, "imu", i))
        self.events.sort(key=lambda x: x[0])

        self.get_logger().info(f"Total events: {len(self.events)} (rate_scale={rate_scale}x)")

        # --- Schedule playback ---
        self.rate_scale = rate_scale
        self.event_idx = 0
        self.t0_ns = self.events[0][0]  # dataset start time

        # Start with a short timer, then reschedule per-event
        self._timer = self.create_timer(0.001, self._publish_next)

    def _publish_next(self):
        """Publish events, sleeping according to dataset timestamps."""
        if self.event_idx >= len(self.events):
            self.get_logger().info("Finished publishing all data.")
            self._timer.cancel()
            return

        ts_ns, event_type, idx = self.events[self.event_idx]

        if event_type == "stereo":
            self._publish_stereo(idx)
        elif event_type == "imu":
            self._publish_imu(idx)

        self.event_idx += 1

        # Schedule next event
        if self.event_idx < len(self.events):
            next_ts = self.events[self.event_idx][0]
            dt_s = (next_ts - ts_ns) / 1e9 / self.rate_scale
            dt_s = max(dt_s, 0.0001)  # floor to avoid zero-period timers
            self._timer.cancel()
            self._timer = self.create_timer(dt_s, self._publish_next)

    def _publish_stereo(self, idx: int):
        ts_ns, left_path, right_path = self.stereo_pairs[idx]

        left_img = cv2.imread(str(left_path), cv2.IMREAD_UNCHANGED)
        right_img = cv2.imread(str(right_path), cv2.IMREAD_UNCHANGED)
        if left_img is None or right_img is None:
            self.get_logger().warn(f"Failed to read images at index {idx}")
            return

        # EuRoC images are grayscale
        if left_img.ndim == 3:
            left_img = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
            right_img = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

        left_msg = self.bridge.cv2_to_imgmsg(left_img, encoding="mono8")
        right_msg = self.bridge.cv2_to_imgmsg(right_img, encoding="mono8")

        stamp = _ns_to_stamp(ts_ns)
        left_msg.header.stamp = stamp
        right_msg.header.stamp = stamp
        left_msg.header.frame_id = "cam0"
        right_msg.header.frame_id = "cam1"

        self.pub_left.publish(left_msg)
        self.pub_right.publish(right_msg)

        stereo_idx = idx + 1
        total = len(self.stereo_pairs)
        self.get_logger().info(
            f"Stereo {stereo_idx}/{total}", throttle_duration_sec=1.0
        )

    def _publish_imu(self, idx: int):
        ts_ns, gyro, accel = self.imu_data[idx]

        msg = Imu()
        msg.header.stamp = _ns_to_stamp(ts_ns)
        msg.header.frame_id = "imu"
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]
        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]

        self.pub_imu.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="Publish EuRoC MAV dataset to ROS2")
    parser.add_argument(
        "seq_path",
        type=str,
        help="Path to EuRoC sequence directory (containing mav0/)",
    )
    parser.add_argument(
        "--rate-scale",
        type=float,
        default=1.0,
        help="Playback speed multiplier (default: 1.0 = realtime, 2.0 = 2x speed)",
    )
    parser.add_argument(
        "--imu",
        action="store_true",
        help="Also publish IMU data on /imu",
    )

    args = parser.parse_args()

    rclpy.init()
    try:
        node = EurocPublisher(args.seq_path, args.rate_scale, args.imu)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
