#!/usr/bin/env python3
"""
Simulate and visualize the path a differential drive robot would draw.
The marker is at the center of the robot between the two wheels.
Usage: python3 simulate_path.py <test_output.txt>
"""

import sys
import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# Robot parameters (these should match the firmware constants)
WHEEL_RADIUS = 1.0  # placeholder - adjust based on actual hardware
ROBOT_WIDTH = 10.0  # distance between wheels (placeholder)
STEPPER_CTRL_FREQ = 20  # Hz
DT = 1.0 / STEPPER_CTRL_FREQ  # time step

def parse_csv_blocks(content):
    """Parse CSV blocks from test output."""
    pattern = r'=== CSV OUTPUT for (\w+) ===\ntimestamp_ms,left_velocity,right_velocity\n(.*?)\n=== END CSV ==='
    matches = re.findall(pattern, content, re.DOTALL)
    
    results = {}
    for test_name, csv_data in matches:
        lines = csv_data.strip().split('\n')
        left_velocities = []
        right_velocities = []
        
        for line in lines:
            if line.strip():
                ts, lv, rv = map(int, line.split(','))
                left_velocities.append(lv)
                right_velocities.append(rv)
        
        results[test_name] = {
            'left_velocities': left_velocities,
            'right_velocities': right_velocities
        }
    
    return results

def simulate_differential_drive(left_velocities, right_velocities, wheel_radius, robot_width, dt):
    """
    Simulate the path of a differential drive robot.
    
    Parameters:
    - left_velocities: list of left wheel velocities (steps/sec or arbitrary units)
    - right_velocities: list of right wheel velocities
    - wheel_radius: radius of the wheels
    - robot_width: distance between the two wheels (wheelbase)
    - dt: time step for each velocity command
    
    Returns:
    - x_positions, y_positions: arrays of the robot center position over time
    - headings: array of robot heading angles over time
    """
    # Initial state
    x = 0.0
    y = 0.0
    theta = 0.0  # heading angle (radians)
    
    x_positions = [x]
    y_positions = [y]
    headings = [theta]
    
    for v_left, v_right in zip(left_velocities, right_velocities):
        # Convert velocities to linear velocities (assuming velocity units are proportional to distance)
        # For simplicity, treat velocity values as distance per time step
        v_l = v_left * wheel_radius * dt / 255.0  # normalize by max velocity
        v_r = v_right * wheel_radius * dt / 255.0
        
        # Differential drive kinematics
        # Linear velocity of the robot center
        v = (v_r + v_l) / 2.0
        
        # Angular velocity
        omega = (v_r - v_l) / robot_width
        
        # Update position and heading
        if abs(omega) < 1e-6:
            # Moving straight
            x += v * np.cos(theta)
            y += v * np.sin(theta)
        else:
            # Moving in an arc
            # Radius of curvature
            R = v / omega
            
            # ICC (Instantaneous Center of Curvature)
            icc_x = x - R * np.sin(theta)
            icc_y = y + R * np.cos(theta)
            
            # Update position
            x = np.cos(omega) * (x - icc_x) - np.sin(omega) * (y - icc_y) + icc_x
            y = np.sin(omega) * (x - icc_x) + np.cos(omega) * (y - icc_y) + icc_y
            theta += omega
        
        x_positions.append(x)
        y_positions.append(y)
        headings.append(theta)
    
    return np.array(x_positions), np.array(y_positions), np.array(headings)

def plot_paths(results, wheel_radius=WHEEL_RADIUS, robot_width=ROBOT_WIDTH, dt=DT):
    """Create plots showing the simulated drawing path for all tests."""
    num_tests = len(results)
    
    if num_tests == 0:
        print("No test results found!")
        return
    
    fig = plt.figure(figsize=(15, 5 * ((num_tests + 1) // 2)))
    
    # Calculate grid layout
    ncols = 2
    nrows = (num_tests + 1) // 2
    
    for idx, (test_name, data) in enumerate(sorted(results.items())):
        ax = plt.subplot(nrows, ncols, idx + 1)
        
        # Simulate the path
        x_pos, y_pos, headings = simulate_differential_drive(
            data['left_velocities'],
            data['right_velocities'],
            wheel_radius,
            robot_width,
            dt
        )
        
        # Plot the path
        ax.plot(x_pos, y_pos, 'b-', linewidth=2, label='Path')
        ax.plot(x_pos[0], y_pos[0], 'go', markersize=12, label='Start')
        ax.plot(x_pos[-1], y_pos[-1], 'ro', markersize=12, label='End')
        
        # Add arrows to show direction
        arrow_interval = max(1, len(x_pos) // 10)
        for i in range(0, len(x_pos) - 1, arrow_interval):
            dx = x_pos[i+1] - x_pos[i]
            dy = y_pos[i+1] - y_pos[i]
            if dx != 0 or dy != 0:
                ax.arrow(x_pos[i], y_pos[i], dx, dy, 
                        head_width=0.5, head_length=0.3, 
                        fc='blue', ec='blue', alpha=0.3)
        
        # Mark waypoints
        ax.plot(x_pos[1:-1], y_pos[1:-1], 'co', markersize=6, alpha=0.6, label='Waypoints')
        
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_title(f'{test_name.replace("_", " ").title()}')
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # Add statistics
        total_distance = np.sum(np.sqrt(np.diff(x_pos)**2 + np.diff(y_pos)**2))
        ax.text(0.02, 0.98, f'Total distance: {total_distance:.2f}', 
               transform=ax.transAxes, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.savefig('navigation_paths.png', dpi=150, bbox_inches='tight')
    print(f"Path plot saved to: navigation_paths.png")
    plt.show()

def plot_velocity_comparison(results):
    """Create a comparison plot of left vs right wheel velocities."""
    fig, axes = plt.subplots(len(results), 1, figsize=(12, 3 * len(results)))
    
    if len(results) == 1:
        axes = [axes]
    
    for ax, (test_name, data) in zip(axes, sorted(results.items())):
        indices = list(range(len(data['left_velocities'])))
        
        ax.bar([i - 0.2 for i in indices], data['left_velocities'], 
               width=0.4, label='Left Wheel', alpha=0.8)
        ax.bar([i + 0.2 for i in indices], data['right_velocities'], 
               width=0.4, label='Right Wheel', alpha=0.8)
        
        ax.set_xlabel('Command Index')
        ax.set_ylabel('Velocity')
        ax.set_title(f'{test_name.replace("_", " ").title()} - Wheel Velocities')
        ax.legend()
        ax.grid(True, alpha=0.3, axis='y')
        ax.set_xticks(indices)
    
    plt.tight_layout()
    plt.savefig('wheel_velocities.png', dpi=150, bbox_inches='tight')
    print(f"Velocity plot saved to: wheel_velocities.png")
    plt.show()

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 simulate_path.py <test_output.txt>")
        print("\nExample:")
        print("  west build -t run > test_output.txt 2>&1")
        print("  python3 simulate_path.py test_output.txt")
        print("\nOptional parameters:")
        print("  --wheel-radius <value>  : Set wheel radius (default: 1.0)")
        print("  --robot-width <value>   : Set distance between wheels (default: 10.0)")
        sys.exit(1)
    
    # Parse optional parameters
    wheel_radius = WHEEL_RADIUS
    robot_width = ROBOT_WIDTH
    
    for i, arg in enumerate(sys.argv):
        if arg == '--wheel-radius' and i + 1 < len(sys.argv):
            wheel_radius = float(sys.argv[i + 1])
        elif arg == '--robot-width' and i + 1 < len(sys.argv):
            robot_width = float(sys.argv[i + 1])
    
    with open(sys.argv[1], 'r') as f:
        content = f.read()
    
    results = parse_csv_blocks(content)
    
    if not results:
        print("No CSV data found in the file!")
        sys.exit(1)
    
    print(f"Found {len(results)} test results:")
    for test_name, data in sorted(results.items()):
        num_commands = len(data['left_velocities'])
        print(f"  - {test_name}: {num_commands} commands")
    
    print(f"\nRobot parameters:")
    print(f"  Wheel radius: {wheel_radius}")
    print(f"  Robot width: {robot_width}")
    print(f"  Control frequency: {STEPPER_CTRL_FREQ} Hz")
    print()
    
    plot_paths(results, wheel_radius, robot_width, DT)
    plot_velocity_comparison(results)

if __name__ == '__main__':
    main()
