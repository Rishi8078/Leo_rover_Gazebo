import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# Resolve project paths
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
data_dir = os.path.join(project_root, 'data')

# Read the CSV file
try:
    df = pd.read_csv(os.path.join(data_dir, 'traction_data.csv'))
except FileNotFoundError:
    print("Error: 'traction_data.csv' not found. Run the simulation first.")
    exit()

# Check if file has data
if len(df) == 0:
    print("Error: CSV file is empty.")
    exit()

# Convert timestamp to relative time (seconds from start)
df['time'] = df['timestamp'] - df['timestamp'].iloc[0]

# Create a figure with multiple subplots
fig, axes = plt.subplots(3, 2, figsize=(16, 12))
fig.suptitle('Dual-Method Slip Ratio Analysis: Position vs. EKF', fontsize=16, fontweight='bold')

# Plot 1: Average Wheel Velocity
ax1 = axes[0, 0]
ax1.plot(df['time'], df['v_wheel_avg'], color='black', label='Avg Wheel Speed')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Velocity (m/s)')
ax1.set_title('Average Wheel Velocity')
ax1.legend()
ax1.grid(True, alpha=0.3)

# Plot 2: Rover Velocity Comparison
ax2 = axes[0, 1]
ax2.plot(df['time'], df['v_rover_pos'], color='green', label='Position-Derived (Manual)', linewidth=1.5)
ax2.plot(df['time'], df['v_rover_ekf'], color='red', linestyle='--', label='EKF Filtered', linewidth=1.5)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Velocity (m/s)')
ax2.set_title('Rover Velocity: Position Derivation vs EKF')
ax2.legend()
ax2.grid(True, alpha=0.3)

# Plot 3: Slip Ratio Comparison
ax3 = axes[1, 0]
ax3.plot(df['time'], df['avg_slip_pos'], color='blue', label='Slip (Position)', alpha=0.8)
ax3.plot(df['time'], df['avg_slip_ekf'], color='orange', label='Slip (EKF)', alpha=0.8)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Slip Ratio')
ax3.set_title('Average Slip Ratio Comparison')
ax3.legend()
ax3.grid(True, alpha=0.3)

# Plot 4: Slip Difference
ax4 = axes[1, 1]
df['slip_diff'] = df['avg_slip_pos'] - df['avg_slip_ekf']
ax4.plot(df['time'], df['slip_diff'], color='purple', linewidth=1)
ax4.axhline(0, color='black', linestyle=':', alpha=0.5)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Difference (Pos - EKF)')
ax4.set_title('Slip Divergence (Position Method - EKF Method)')
ax4.grid(True, alpha=0.3)

# Plot 5: Slip Distribution
ax5 = axes[2, 0]
ax5.hist(df['avg_slip_pos'], bins=40, color='blue', alpha=0.5, label='Position Method')
ax5.hist(df['avg_slip_ekf'], bins=40, color='orange', alpha=0.5, label='EKF Method')
ax5.set_xlabel('Slip Ratio')
ax5.set_ylabel('Frequency')
ax5.set_title('Distribution of Slip Ratios')
ax5.legend()
ax5.grid(True, alpha=0.3, axis='y')

# Plot 6: Integrated Trajectory (Approximate)
# Reconstructing path roughly to see if valid motion occurred
ax6 = axes[2, 1]
# Simple integration assuming straight line for visualization
dist_pos = np.cumsum(df['v_rover_pos']) * (df['time'].diff().fillna(0))
dist_ekf = np.cumsum(df['v_rover_ekf']) * (df['time'].diff().fillna(0))
ax6.plot(df['time'], dist_pos, label='Dist (Pos)', color='green')
ax6.plot(df['time'], dist_ekf, label='Dist (EKF)', color='red', linestyle='--')
ax6.set_xlabel('Time (s)')
ax6.set_ylabel('Distance Traveled (m)')
ax6.set_title('Cumulative Distance (Integrated Velocity)')
ax6.legend()
ax6.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(data_dir, 'traction_data_plot.png'), dpi=300, bbox_inches='tight')
print("Plot saved as 'traction_data_plot.png'")

# Print summary statistics
print("\n=== Dual-Method Data Summary ===")
print(f"Total data points: {len(df)}")
print(f"Time range: {df['time'].min():.2f} to {df['time'].max():.2f} seconds")

print(f"\nRover Velocity (Mean):")
print(f"  Position-Based: {df['v_rover_pos'].mean():.4f} m/s")
print(f"  EKF-Based:      {df['v_rover_ekf'].mean():.4f} m/s")

print(f"\nAverage Slip Ratio (Mean):")
print(f"  Position-Based: {df['avg_slip_pos'].mean():.4f}")
print(f"  EKF-Based:      {df['avg_slip_ekf'].mean():.4f}")

print(f"\nMax Slip Observed:")
print(f"  Position-Based: {df['avg_slip_pos'].max():.4f}")
print(f"  EKF-Based:      {df['avg_slip_ekf'].max():.4f}")
