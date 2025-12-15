#!/usr/bin/env python3
"""
Restamp KITTI times file with new timestamps starting from now at specified Hz.

Usage:
    python restamp_tum.py -i times.txt -o times_restamped.txt --hz 20
"""

import argparse
import time
from pathlib import Path

def restamp_kitti_times(input_file: Path, output_file: Path, hz: float):
    """
    Read KITTI times file and restamp with new timestamps starting from now at specified Hz.
    
    Args:
        input_file: Path to input KITTI times file
        output_file: Path to output times file
        hz: Frequency in Hz
    """
    # Read all lines from input file
    with open(input_file, 'r') as f:
        lines = f.readlines()
    
    # Get current time as starting timestamp
    start_time = time.time()
    dt = 1.0 / hz  # Time between frames in seconds
    
    # Process each line
    restamped_lines = []
    line_count = 0
    for i, line in enumerate(lines):
        line = line.strip()
        if not line:
            continue
        
        # Calculate new timestamp
        new_timestamp = start_time + (line_count * dt)
        new_line = f"{new_timestamp:.9f}\n"
        restamped_lines.append(new_line)
        line_count += 1
    
    # Write output file
    with open(output_file, 'w') as f:
        f.writelines(restamped_lines)
    
    print(f"Restamped {len(restamped_lines)} timestamps from {input_file}")
    print(f"Start time: {start_time:.9f}")
    print(f"End time:   {start_time + (len(restamped_lines) - 1) * dt:.9f}")
    print(f"Duration:   {(len(restamped_lines) - 1) * dt:.3f} seconds")
    print(f"Frequency:  {hz} Hz")
    print(f"Saved to:   {output_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Restamp KITTI times file with new timestamps starting from now at specified Hz'
    )
    parser.add_argument('--input', '-i', type=Path, required=True, 
                        help='Input KITTI times file')
    parser.add_argument('--output', '-o', type=Path, required=True,
                        help='Output times file')
    parser.add_argument('--hz', type=float, required=True,
                        help='Output frequency in Hz')
    
    args = parser.parse_args()
    
    input_file = args.input
    output_file = args.output
    hz = args.hz
    
    if not input_file.exists():
        print(f"Error: Input file not found: {input_file}")
        return 1
    
    if output_file.exists():
        print(f"Error: Output file already exists: {output_file}")
        return 1
    
    restamp_kitti_times(input_file, output_file, hz)
    return 0


if __name__ == '__main__':
    exit(main())
