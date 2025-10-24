#!/usr/bin/env python3
"""
Plot step command results from navigation tests.
Usage: python3 plot_results.py <test_output.txt>
"""

import sys
import re
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

def parse_csv_blocks(content):
    """Parse CSV blocks from test output."""
    pattern = r'=== CSV OUTPUT for (\w+) ===\ntimestamp_ms,left_velocity,right_velocity\n(.*?)\n=== END CSV ==='
    matches = re.findall(pattern, content, re.DOTALL)
    
    results = {}
    for test_name, csv_data in matches:
        lines = csv_data.strip().split('\n')
        timestamps = []
        left_velocities = []
        right_velocities = []
        
        for line in lines:
            if line.strip():
                ts, lv, rv = map(int, line.split(','))
                timestamps.append(ts)
                left_velocities.append(lv)
                right_velocities.append(rv)
        
        results[test_name] = {
            'timestamps': timestamps,
            'left_velocities': left_velocities,
            'right_velocities': right_velocities
        }
    
    return results

def plot_results(results):
    """Create plots for all test results."""
    num_tests = len(results)
    
    if num_tests == 0:
        print("No test results found!")
        return
    
    fig = plt.figure(figsize=(15, 3 * num_tests))
    gs = GridSpec(num_tests, 1, figure=fig, hspace=0.4)
    
    for idx, (test_name, data) in enumerate(sorted(results.items())):
        ax = fig.add_subplot(gs[idx])
        
        indices = list(range(len(data['timestamps'])))
        
        ax.plot(indices, data['left_velocities'], 'o-', label='Left Velocity', linewidth=2, markersize=8)
        ax.plot(indices, data['right_velocities'], 's-', label='Right Velocity', linewidth=2, markersize=8)
        
        ax.set_xlabel('Command Index')
        ax.set_ylabel('Velocity')
        ax.set_title(f'{test_name.replace("_", " ").title()}')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_xticks(indices)
        
        # Add value labels on points
        for i, (lv, rv) in enumerate(zip(data['left_velocities'], data['right_velocities'])):
            ax.annotate(f'{lv}', (i, lv), textcoords="offset points", xytext=(0,10), ha='center', fontsize=8)
            ax.annotate(f'{rv}', (i, rv), textcoords="offset points", xytext=(0,-15), ha='center', fontsize=8)
    
    plt.savefig('navigation_test_results.png', dpi=150, bbox_inches='tight')
    print(f"Plot saved to: navigation_test_results.png")
    plt.show()

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_results.py <test_output.txt>")
        print("\nExample:")
        print("  west build -t run > test_output.txt 2>&1")
        print("  python3 plot_results.py test_output.txt")
        sys.exit(1)
    
    with open(sys.argv[1], 'r') as f:
        content = f.read()
    
    results = parse_csv_blocks(content)
    
    if not results:
        print("No CSV data found in the file!")
        sys.exit(1)
    
    print(f"Found {len(results)} test results:")
    for test_name in sorted(results.keys()):
        num_commands = len(results[test_name]['timestamps'])
        print(f"  - {test_name}: {num_commands} commands")
    
    plot_results(results)

if __name__ == '__main__':
    main()
