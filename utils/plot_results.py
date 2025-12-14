import sys
import argparse
import datetime
import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt


def load_data_tum(filenames: list[str]) -> dict[str, pd.DataFrame]:
    """
    Loads multiple TUM trajectory files into pandas DataFrames and returns a dictionary of the data.
    """
    # Compile data into dictionary
    all_data = {}
    for file in filenames:
        # Grab filename without extension (after the last "/" and before ".txt" in this case)
        filename = Path(file).stem

        # Load data into dataframe
        df = pd.read_csv(file, sep=' ', header=None, names=['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])

        # Save to dictionary
        all_data[filename] = df

    return all_data


def plot_results(data: dict[str, pd.DataFrame], output_dir: str, save_file: bool = False):
    """
    Plots the results from a dictionary of TUM trajectory data.
    """
    # Plot results from each file
    output_file_name = ''
    for key, value in data.items():
        plt.plot(value['tx'], value['ty'], value['tz'], label=key)
        output_file_name += f'{key}_'
    
    # Add timestamp to the output file name
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file_name += f'{timestamp}_'
    if save_file:
        plt.savefig(output_dir / f'{output_file_name}traj.png')
        print(f"Plot saved to {output_dir / f'{output_file_name}traj.png'}")

    # Show plot
    plt.legend()
    plt.show()

    

def main():
    # Add argument for num files, filenames, and output directory
    parser = argparse.ArgumentParser(description="Compare and plot multiple trajectory result files.")
    parser.add_argument('--num-traj', '-n', type=int, required=True, help="Number of trajectories to compare")
    parser.add_argument('--input-files', '-i', nargs='+', required=True, help="Input files with trajectory results")
    parser.add_argument('--output-dir', '-o', type=str, required=True, help="Directory to save output plots")
    parser.add_argument('--save-file', '-s', type=bool, required=False, help="Save file to output directory")
    # Parse the arguments
    args = parser.parse_args()

    # Error check the number of files given
    if len(args.input_files) < args.num_traj:
        print(f"Error: Received {len(args.input_files)} files but {args.num_traj} trajectories specified.")
        sys.exit(1)
    if len(args.input_files) > args.num_traj:
        print(f"Warning: Received more input files than specified trajectories, using only the first {args.num_traj}.")
        args.input_files = args.input_files[:args.num_traj]

    # Set variables to parsed arguments
    num_trajectories = args.num_traj
    input_files = args.input_files
    output_dir = args.output_dir
    save_file = args.save_file

    # Load data
    data = load_data_tum(input_files)
    
    # Plot results
    plot_results(data, output_dir, save_file)


if __name__ == "__main__":
    main()
