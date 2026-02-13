#!/usr/bin/env python3
"""
Plot Sensor Fusion Results.
Compares wheel encoder, IMU, fused velocity against ground truth.
Helps tune fusion parameters.
"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# Resolve project paths
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
data_dir = os.path.join(project_root, 'data')

# Read the CSV file
df = pd.read_csv(os.path.join(data_dir, 'sensor_fusion_data.csv'))

# Convert timestamp to relative time
df['time'] = df['timestamp'] - df['timestamp'].iloc[0]

# Create figure with subplots
fig, axes = plt.subplots(3, 2, figsize=(14, 10))
fig.suptitle('Sensor Fusion Validation', fontsize=16, fontweight='bold')

# ============== Plot 1: Velocity Comparison ==============
ax1 = axes[0, 0]
ax1.plot(df['time'], df['v_ground_truth'], 'k-', linewidth=2, label='Ground Truth', alpha=0.8)
ax1.plot(df['time'], df['v_wheel'], 'b--', linewidth=1.5, label='Wheel Encoder', alpha=0.7)
ax1.plot(df['time'], df['v_imu'], 'g--', linewidth=1.5, label='IMU (integrated)', alpha=0.7)
ax1.plot(df['time'], df['v_fused'], 'r-', linewidth=2, label='Fused', alpha=0.9)
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Velocity (m/s)')
ax1.set_title('Velocity Estimates vs Ground Truth')
ax1.legend(loc='upper right')
ax1.grid(True, alpha=0.3)

# ============== Plot 2: Error Comparison ==============
ax2 = axes[0, 1]
ax2.plot(df['time'], df['error_wheel'], 'b-', linewidth=1.5, label='Wheel Error', alpha=0.7)
ax2.plot(df['time'], df['error_imu'], 'g-', linewidth=1.5, label='IMU Error', alpha=0.7)
ax2.plot(df['time'], df['error_fused'], 'r-', linewidth=2, label='Fused Error', alpha=0.9)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Absolute Error (m/s)')
ax2.set_title('Estimation Errors')
ax2.legend(loc='upper right')
ax2.grid(True, alpha=0.3)

# ============== Plot 3: Error Distribution ==============
ax3 = axes[1, 0]
bins = 30
ax3.hist(df['error_wheel'], bins=bins, alpha=0.5, label=f'Wheel (μ={df["error_wheel"].mean():.4f})', color='blue')
ax3.hist(df['error_imu'], bins=bins, alpha=0.5, label=f'IMU (μ={df["error_imu"].mean():.4f})', color='green')
ax3.hist(df['error_fused'], bins=bins, alpha=0.7, label=f'Fused (μ={df["error_fused"].mean():.4f})', color='red')
ax3.set_xlabel('Error (m/s)')
ax3.set_ylabel('Frequency')
ax3.set_title('Error Distribution')
ax3.legend()
ax3.grid(True, alpha=0.3, axis='y')

# ============== Plot 4: Cumulative Error ==============
ax4 = axes[1, 1]
ax4.plot(df['time'], df['error_wheel'].cumsum(), 'b-', linewidth=1.5, label='Wheel Cumulative', alpha=0.7)
ax4.plot(df['time'], df['error_imu'].cumsum(), 'g-', linewidth=1.5, label='IMU Cumulative', alpha=0.7)
ax4.plot(df['time'], df['error_fused'].cumsum(), 'r-', linewidth=2, label='Fused Cumulative', alpha=0.9)
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Cumulative Error')
ax4.set_title('Cumulative Error Over Time')
ax4.legend(loc='upper left')
ax4.grid(True, alpha=0.3)

# ============== Plot 5: Kalman Filter State ==============
ax5 = axes[2, 0]
if 'kf_state_vel' in df.columns and df['kf_state_vel'].abs().sum() > 0:
    ax5.plot(df['time'], df['kf_state_vel'], 'r-', linewidth=2, label='KF State', alpha=0.9)
    ax5.plot(df['time'], df['v_ground_truth'], 'k--', linewidth=1.5, label='Ground Truth', alpha=0.7)
    ax5.set_title('Kalman Filter State vs Ground Truth')
else:
    ax5.plot(df['time'], df['v_fused'], 'r-', linewidth=2, label='Fused', alpha=0.9)
    ax5.plot(df['time'], df['v_ground_truth'], 'k--', linewidth=1.5, label='Ground Truth', alpha=0.7)
    ax5.set_title('Fused vs Ground Truth')
ax5.set_xlabel('Time (s)')
ax5.set_ylabel('Velocity (m/s)')
ax5.legend()
ax5.grid(True, alpha=0.3)

# ============== Plot 6: Kalman Filter Covariance ==============
ax6 = axes[2, 1]
if 'kf_covariance' in df.columns and df['kf_covariance'].abs().sum() > 0:
    ax6.plot(df['time'], df['kf_covariance'], 'purple', linewidth=2)
    ax6.set_title('Kalman Filter Covariance (Uncertainty)')
    ax6.set_ylabel('Covariance')
else:
    # Show rolling RMSE instead
    window = 20
    rmse_wheel = df['error_wheel'].rolling(window).apply(lambda x: np.sqrt((x**2).mean()))
    rmse_fused = df['error_fused'].rolling(window).apply(lambda x: np.sqrt((x**2).mean()))
    ax6.plot(df['time'], rmse_wheel, 'b-', label='Wheel RMSE', alpha=0.7)
    ax6.plot(df['time'], rmse_fused, 'r-', label='Fused RMSE', alpha=0.9)
    ax6.set_title(f'Rolling RMSE (window={window})')
    ax6.set_ylabel('RMSE (m/s)')
    ax6.legend()
ax6.set_xlabel('Time (s)')
ax6.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(data_dir, 'sensor_fusion_validation.png'), dpi=300, bbox_inches='tight')
print("Plot saved as 'sensor_fusion_validation.png'")

# ============== PRINT STATISTICS ==============
print("\n" + "="*60)
print("SENSOR FUSION VALIDATION RESULTS")
print("="*60)

print(f"\nData points: {len(df)}")
print(f"Time range: {df['time'].min():.2f} to {df['time'].max():.2f} seconds")

print("\n--- Mean Absolute Error (MAE) ---")
print(f"  Wheel Encoder: {df['error_wheel'].mean():.6f} m/s")
print(f"  IMU:           {df['error_imu'].mean():.6f} m/s")
print(f"  Fused:         {df['error_fused'].mean():.6f} m/s")

print("\n--- Root Mean Square Error (RMSE) ---")
rmse_wheel = np.sqrt((df['error_wheel']**2).mean())
rmse_imu = np.sqrt((df['error_imu']**2).mean())
rmse_fused = np.sqrt((df['error_fused']**2).mean())
print(f"  Wheel Encoder: {rmse_wheel:.6f} m/s")
print(f"  IMU:           {rmse_imu:.6f} m/s")
print(f"  Fused:         {rmse_fused:.6f} m/s")

print("\n--- Max Error ---")
print(f"  Wheel Encoder: {df['error_wheel'].max():.6f} m/s")
print(f"  IMU:           {df['error_imu'].max():.6f} m/s")
print(f"  Fused:         {df['error_fused'].max():.6f} m/s")

print("\n--- Improvement ---")
improvement = (rmse_wheel - rmse_fused) / rmse_wheel * 100
print(f"  Fused vs Wheel: {improvement:+.2f}% RMSE improvement")
improvement_imu = (rmse_imu - rmse_fused) / rmse_imu * 100 if rmse_imu > 0 else 0
print(f"  Fused vs IMU:   {improvement_imu:+.2f}% RMSE improvement")

print("\n" + "="*60)

plt.show()
