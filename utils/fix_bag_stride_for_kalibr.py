#!/usr/bin/env python3
"""
fix_bag_stride.py
SPECIFICALLY FOR FIXING ROS1 BAGS FOR CALIBRATION IN KALIBR:
For ROS2 bags converted to ROS1 bags recorded from AlliedVision cameras in mono8 encoding.
Allied vision adds padding to the image when encoded as mono8 - this is a script to remove that padding.
Once the stride is fixed it can be fed into Kalibr with no issues.

Strips row-padding from mono8 sensor_msgs/Image messages in a ROS1 bag.
Fixes step != width mismatches introduced by Allied Vision driver row alignment.

Usage:
    python3 fix_bag_stride.py input.bag output.bag
    python3 fix_bag_stride.py input.bag output.bag --topics /left/image_raw /right/image_raw
"""

import argparse
import sys
import numpy as np
import rosbag
from sensor_msgs.msg import Image


def fix_image(msg):
    """Strip row padding from a mono8 Image message. Returns corrected message."""
    if msg.encoding != "mono8":
        return msg, False

    if msg.step == msg.width:
        return msg, False  # already clean, nothing to do

    # Reshape using the real stride, then crop to true width
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.step))
    cropped = buf[:, :msg.width].copy()

    new_msg = Image()
    new_msg.header      = msg.header
    new_msg.height      = msg.height
    new_msg.width       = msg.width
    new_msg.encoding    = msg.encoding
    new_msg.is_bigendian = msg.is_bigendian
    new_msg.step        = msg.width       # corrected: step == width for mono8
    new_msg.data        = cropped.tobytes()
    return new_msg, True


def main():
    parser = argparse.ArgumentParser(description="Fix mono8 row-padding in ROS1 bags.")
    parser.add_argument("input",  help="Input bag path")
    parser.add_argument("output", help="Output bag path")
    parser.add_argument(
        "--topics", nargs="*", default=None,
        help="Only process these topics (default: all sensor_msgs/Image topics)"
    )
    args = parser.parse_args()

    print(f"Input:  {args.input}")
    print(f"Output: {args.output}")

    with rosbag.Bag(args.input, "r") as inbag:
        topics_in_bag = inbag.get_type_and_topic_info().topics

        # Determine which image topics to process
        image_topics = {
            t for t, info in topics_in_bag.items()
            if info.msg_type == "sensor_msgs/Image"
        }
        if args.topics:
            image_topics = image_topics & set(args.topics)

        print(f"Image topics to process: {sorted(image_topics)}")

        total   = inbag.get_message_count()
        fixed   = 0
        skipped = 0

        with rosbag.Bag(args.output, "w") as outbag:
            for i, (topic, msg, t) in enumerate(inbag.read_messages()):
                if topic in image_topics:
                    new_msg, was_fixed = fix_image(msg)
                    if was_fixed:
                        fixed += 1
                    else:
                        skipped += 1
                    outbag.write(topic, new_msg, t)
                else:
                    outbag.write(topic, msg, t)

                if (i + 1) % 500 == 0:
                    print(f"  {i + 1}/{total} messages processed...", end="\r")

    print(f"\nDone. {fixed} images fixed, {skipped} images already clean.")
    print(f"Output written to: {args.output}")


if __name__ == "__main__":
    main()