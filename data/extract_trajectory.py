#!/usr/bin/env python3

import sys
import csv
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def extract_trajectory_from_bag(bag_path: str, output_csv: str = 'trajectory.csv'):
    """
    Extract odometry trajectory from a ROS 2 bag file.
    
    Args:
        bag_path: Path to the bag file directory or MCAP file
        output_csv: Output CSV file path
    """
    bag_path = Path(bag_path)
    
    if not bag_path.exists():
        print(f"Error: Bag file not found: {bag_path}")
        return
    
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    except ImportError:
        print("Error: rosbag2_py not available. Please install it:")
        print("  pip install rosbag2_py")
        return
    
    print(f"Reading bag file: {bag_path}")
    
    # Determine if it's a directory (SQLite) or file (MCAP)
    if bag_path.is_dir():
        # SQLite bag format - use directory as URI
        storage_id = 'sqlite3'
        uri = str(bag_path)
    elif bag_path.suffix == '.mcap':
        # MCAP file - use the file itself
        storage_id = 'mcap'
        uri = str(bag_path)
    else:
        # Try as directory first, then as file
        storage_id = 'sqlite3'
        uri = str(bag_path)
    
    # Create reader
    reader = SequentialReader()
    storage_options = StorageOptions(uri=uri, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag file: {e}")
        return
    
    # Get topic metadata
    topic_types = reader.get_all_topics_and_types()
    odom_topic = None
    for topic_metadata in topic_types:
        if topic_metadata.name == '/fast_limo/state':
            odom_topic = topic_metadata
            break
    
    if odom_topic is None:
        print("Error: Topic /cuvslam/odometry not found in bag file")
        print("Available topics:")
        for topic_metadata in topic_types:
            print(f"  - {topic_metadata.name}")
        return
    
    # Get message type
    msg_type = get_message(odom_topic.type)
    
    # Read messages and extract trajectory
    trajectory_data = []
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic == '/fast_limo/state':
            try:
                msg = deserialize_message(data, msg_type)
                
                # Extract timestamp (nanoseconds)
                timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                
                # Extract position
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                z = msg.pose.pose.position.z
                
                # Extract quaternion
                qx = msg.pose.pose.orientation.x
                qy = msg.pose.pose.orientation.y
                qz = msg.pose.pose.orientation.z
                qw = msg.pose.pose.orientation.w
                
                trajectory_data.append([timestamp_ns, x, y, z, qx, qy, qz, qw])
            except Exception as e:
                print(f"Warning: Failed to deserialize message: {e}")
                continue
    
    print(f"Extracted {len(trajectory_data)} odometry messages")
    
    if len(trajectory_data) == 0:
        print("Warning: No odometry messages found")
        return
    
    # Write to CSV
    output_path = Path(output_csv)
    with open(output_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        writer.writerows(trajectory_data)
    
    print(f"✓ Trajectory saved to {output_path}")
    print(f"  Total entries: {len(trajectory_data)}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 extract_trajectory.py <bag_path> [output_csv]")
        print("  bag_path: Path to ROS 2 bag file (MCAP or SQLite)")
        print("  output_csv: Output CSV file (default: trajectory.csv)")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    output_csv = sys.argv[2] if len(sys.argv) > 2 else 'trajectory.csv'
    
    extract_trajectory_from_bag(bag_path, output_csv)


if __name__ == '__main__':
    main()

