import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('Agg')

# Read the CSV
df = pd.read_csv('/home/aaron/test_bags/columbus_traj_3/trajectory.csv')

# Create 2D plot (X-Z side view)
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111)

# Plot the trajectory
ax.plot(df['x'], df['z'], 'b-', linewidth=1.5, alpha=0.7)

# Mark start and end points
ax.scatter(df['x'].iloc[0], df['z'].iloc[0], 
           c='green', marker='o', s=100, label='Start', zorder=5)
ax.scatter(df['x'].iloc[-1], df['z'].iloc[-1], 
           c='red', marker='x', s=100, label='End', linewidths=3, zorder=5)

# Labels and formatting
ax.set_xlabel('X (m)')
ax.set_ylabel('Z (m)')
ax.set_title('Trajectory (X-Z Side View)')
ax.legend()
ax.grid(True, alpha=0.3)
ax.axis('equal')

plt.savefig('trajectory_plot.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved to trajectory_plot.png")
