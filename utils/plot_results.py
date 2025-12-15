import sys
import argparse
import datetime
import numpy as np
import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation


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
    # Create figure and axis
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')

    # PLOT 3D TRAJECTORY
    output_file_name = ''
    for filename, df in data.items():
        ax.plot(df['tx'], df['ty'], df['tz'], label=filename)
        output_file_name += f'{filename}_'
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper right')

    # PLOT FOR TRANSLATION AND YPR
    fig_trans_rot, axes = plt.subplots(3, 2, figsize=(18,5), layout="constrained")
    fig_trans_rot.suptitle('Traj components v Time')

    # Plot
    for filename, df in data.items():
        # Normalize timestamp
        time = df['timestamp'] - df['timestamp'].iloc[0]

        # Convert quaternion to euler angles
        quats = np.column_stack([df['qx'], df['qy'], df['qz'], df['qw']])
        rot = Rotation.from_quat(quats) # this lib takes in scalar last format
        euler_angles = rot.as_euler('xyz', degrees=True)
        roll, pitch, yaw = euler_angles[:, 0], euler_angles[:, 1], euler_angles[:, 2]

        # Plot trans
        axes[0, 0].plot(time, df['tx'], label=filename)
        axes[1, 0].plot(time, df['ty'], label=filename)
        axes[2, 0].plot(time, df['tz'], label=filename)

        # Plot rot
        axes[0, 1].plot(time, yaw, label=filename)
        axes[1, 1].plot(time, pitch, label=filename)
        axes[2, 1].plot(time, roll, label=filename)

    # Set labels for translation subplots
    trans_ylabels = ['X (m)', 'Y (m)', 'Z (m)']
    for i, ylabel in enumerate(trans_ylabels):
        axes[i, 0].set_ylabel(ylabel)
        axes[i, 0].grid(True, alpha=0.3)

    # Set labels for rotation subplots
    rot_ylabels = ['Roll (deg)', 'Pitch (deg)', 'Yaw (deg)']
    for i, ylabel in enumerate(rot_ylabels):
        axes[i, 1].set_ylabel(ylabel)
        axes[i, 1].grid(True, alpha=0.3)

    # Remove x-ticks from top 2 rows
    for i in range(2):
        axes[i, 0].set_xticklabels([])
        axes[i, 1].set_xticklabels([])

    # Set x-axis labels for bottom row
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 1].set_xlabel('Time (s)')

    # Set legend in a column to the right of the plots
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig_trans_rot.legend(handles, labels, loc='outside right upper') 

    # Add timestamp to the output file name
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file_name += f'{timestamp}_'
    if save_file:
        fig.savefig(output_dir + '/' + f'{output_file_name}traj_3d.png')
        fig_trans_rot.savefig(output_dir + '/' + f'{output_file_name}traj_components.png')
        print(f"Plots saved to {output_dir}")

    # Show plot
    plt.show()

    

def main():
    # Add argument for num files, filenames, and output directory
    parser = argparse.ArgumentParser(description="Compare and plot multiple trajectory result files.")
    parser.add_argument('--num-traj', '-n', type=int, required=True, help="Number of trajectories to compare")
    parser.add_argument('--input-files', '-i', nargs='+', required=True, help="Input files with trajectory results")
    parser.add_argument('--output-dir', '-o', type=str, required=True, help="Directory to save output plots")
    parser.add_argument('--save-file', '-s', action='store_true', help="Save file to output directory")
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
