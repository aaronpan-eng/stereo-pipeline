import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('Agg')
from mpl_toolkits.mplot3d import Axes3D

# Read the CSV
df = pd.read_csv('/home/aaron/test_bags/huntington_traj/trajectory.csv')

# Create 3D plot
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot the trajectory
ax.plot(df['x'], df['y'], df['z'], 'b-', linewidth=1, alpha=0.7)

# Mark start and end points
ax.scatter(df['x'].iloc[0], df['y'].iloc[0], df['z'].iloc[0], 
           c='green', marker='o', s=100, label='Start')
ax.scatter(df['x'].iloc[-1], df['y'].iloc[-1], df['z'].iloc[-1], 
           c='red', marker='x', s=100, label='End')

# Labels and formatting
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('3D Trajectory')
ax.legend()

# Equal aspect ratio for better visualization
max_range = max(df['x'].max()-df['x'].min(), 
                df['y'].max()-df['y'].min(), 
                df['z'].max()-df['z'].min()) / 2.0

mid_x = (df['x'].max() + df['x'].min()) / 2.0
mid_y = (df['y'].max() + df['y'].min()) / 2.0
mid_z = (df['z'].max() + df['z'].min()) / 2.0

ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

plt.savefig('trajectory_plot.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved to trajectory_plot.png")