import argparse
from pathlib import Path
import os

def normalize_timestamps(input_file: Path, output_dir: Path):
    with open(input_file, 'r') as f:
        lines = [line.strip() for line in f.readlines() if line.strip()]

    if not lines:
        print(f"No data in file: {input_file}")
        return

    # Get the initial timestamp to normalize
    first_valid = None
    for line in lines:
        parts = line.split()
        if len(parts) == 8:
            first_valid = float(parts[0])
            break
    if first_valid is None:
        print(f"No valid TUM lines found in {input_file}")
        return

    normalized_lines = []
    for line in lines:
        parts = line.split()
        if len(parts) != 8:
            print(f"Warning: Skipping malformed line: {line}")
            continue
        timestamp = float(parts[0])
        new_timestamp = timestamp - first_valid
        normalized_line = f"{new_timestamp:.9f} {' '.join(parts[1:])}\n"
        normalized_lines.append(normalized_line)

    output_dir.mkdir(parents=True, exist_ok=True)
    stem = input_file.stem
    suffix = input_file.suffix
    out_path = output_dir / f"{stem}_normalized{suffix}"

    with open(out_path, 'w') as f:
        f.writelines(normalized_lines)

    print(f"Normalized file written to: {out_path}")

def main():
    parser = argparse.ArgumentParser(
        description="Normalize timestamps of a TUM formatted file to start at zero."
    )
    parser.add_argument(
        "-i", "--input-file", type=Path, required=True,
        help="Input TUM .txt file"
    )
    parser.add_argument(
        "-o", "--output-dir", type=Path, required=True,
        help="Output directory to save normalized file"
    )
    args = parser.parse_args()

    normalize_timestamps(args.input_file, args.output_dir)

if __name__ == "__main__":
    main()

