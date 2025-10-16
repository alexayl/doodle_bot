#!/usr/bin/env python3
"""
Navigation Test Data Analyzer
Extracts CSV data from navigation test output and generates graphs with deviation analysis.
"""

import re
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import sys
from pathlib import Path

def extract_csv_from_output(output_text):
    """Extract CSV data from test output text."""
    csv_sections = {}
    
    # Pattern to match CSV sections
    pattern = r'=== MOVEMENT CSV DATA for (\w+) ===\n(.*?)\n=== END CSV DATA ==='
    
    matches = re.findall(pattern, output_text, re.DOTALL)
    
    for test_name, csv_content in matches:
        lines = csv_content.strip().split('\n')
        if len(lines) > 1:  # Has header + data
            csv_sections[test_name] = '\n'.join(lines)
    
    return csv_sections

def analyze_path_deviation(df, expected_pattern):
    """Analyze deviation from expected path pattern."""
    deviations = []
    
    if expected_pattern == 'square':
        # For a square, we expect 90-degree turns
        expected_moves = [(50, 0), (0, 50), (-50, 0), (0, -50)]
        pen_down_moves = df[df['pen_down'] == 1]
        
        for i, (_, row) in enumerate(pen_down_moves.iterrows()):
            if i < len(expected_moves):
                expected_dx, expected_dy = expected_moves[i]
                actual_dx, actual_dy = row['dx'], row['dy']
                deviation = np.sqrt((actual_dx - expected_dx)**2 + (actual_dy - expected_dy)**2)
                deviations.append(deviation)
    
    elif expected_pattern == 'circle':
        # For a circle, check if points are roughly equidistant from center
        pen_down_moves = df[df['pen_down'] == 1]
        center_x = pen_down_moves['target_x'].mean()
        center_y = pen_down_moves['target_y'].mean()
        
        distances = np.sqrt((pen_down_moves['target_x'] - center_x)**2 + 
                          (pen_down_moves['target_y'] - center_y)**2)
        expected_radius = distances.mean()
        deviations = np.abs(distances - expected_radius).tolist()
    
    return deviations

def plot_movement_data(csv_sections, output_dir):
    """Generate plots for movement data."""
    output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    axes = axes.flatten()
    
    deviation_stats = {}
    
    for i, (test_name, csv_data) in enumerate(csv_sections.items()):
        if i >= len(axes):
            break
            
        # Parse CSV data
        from io import StringIO
        df = pd.read_csv(StringIO(csv_data))
        
        # Save individual CSV file
        csv_file = output_dir / f"navigation_{test_name}.csv"
        df.to_csv(csv_file, index=False)
        print(f"Saved CSV: {csv_file}")
        
        # Plot path
        ax = axes[i]
        
        # Plot all movements
        ax.plot(df['start_x'], df['start_y'], 'b-', alpha=0.3, label='All moves')
        
        # Highlight pen-down movements
        pen_down = df[df['pen_down'] == 1]
        if not pen_down.empty:
            ax.plot(pen_down['target_x'], pen_down['target_y'], 'r-', linewidth=2, label='Pen down')
            ax.scatter(pen_down['target_x'], pen_down['target_y'], c='red', s=20, zorder=5)
        
        # Mark start and end
        ax.scatter(df.iloc[0]['start_x'], df.iloc[0]['start_y'], 
                  c='green', s=100, marker='o', label='Start', zorder=10)
        ax.scatter(df.iloc[-1]['target_x'], df.iloc[-1]['target_y'], 
                  c='blue', s=100, marker='s', label='End', zorder=10)
        
        # Calculate deviations
        deviations = analyze_path_deviation(df, test_name)
        if deviations:
            mean_dev = np.mean(deviations)
            max_dev = np.max(deviations)
            std_dev = np.std(deviations)
            deviation_stats[test_name] = {
                'mean': mean_dev,
                'max': max_dev,
                'std': std_dev,
                'count': len(deviations)
            }
            
            ax.set_title(f'{test_name.title()}\nMean Dev: {mean_dev:.2f}, Max: {max_dev:.2f}')
        else:
            ax.set_title(test_name.title())
        
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.axis('equal')
    
    # Hide unused subplots
    for j in range(len(csv_sections), len(axes)):
        axes[j].set_visible(False)
    
    plt.tight_layout()
    
    # Save plot
    plot_file = output_dir / 'navigation_paths.png'
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"Saved plot: {plot_file}")
    
    # Create deviation analysis plot
    if deviation_stats:
        fig2, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # Mean deviation comparison
        test_names = list(deviation_stats.keys())
        mean_devs = [deviation_stats[name]['mean'] for name in test_names]
        max_devs = [deviation_stats[name]['max'] for name in test_names]
        
        x = np.arange(len(test_names))
        width = 0.35
        
        ax1.bar(x - width/2, mean_devs, width, label='Mean Deviation', alpha=0.8)
        ax1.bar(x + width/2, max_devs, width, label='Max Deviation', alpha=0.8)
        ax1.set_xlabel('Test Pattern')
        ax1.set_ylabel('Deviation')
        ax1.set_title('Path Deviation Analysis')
        ax1.set_xticks(x)
        ax1.set_xticklabels(test_names)
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Standard deviation
        std_devs = [deviation_stats[name]['std'] for name in test_names]
        ax2.bar(test_names, std_devs, alpha=0.8, color='orange')
        ax2.set_xlabel('Test Pattern')
        ax2.set_ylabel('Standard Deviation')
        ax2.set_title('Deviation Consistency')
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        deviation_plot = output_dir / 'deviation_analysis.png'
        plt.savefig(deviation_plot, dpi=300, bbox_inches='tight')
        print(f"Saved deviation plot: {deviation_plot}")
    
    return deviation_stats

def print_deviation_report(deviation_stats):
    """Print a detailed deviation analysis report."""
    print("\n" + "="*60)
    print("NAVIGATION DEVIATION ANALYSIS REPORT")
    print("="*60)
    
    for test_name, stats in deviation_stats.items():
        print(f"\n{test_name.upper()} Pattern:")
        print(f"  Mean Deviation:     {stats['mean']:.3f} units")
        print(f"  Maximum Deviation:  {stats['max']:.3f} units")
        print(f"  Standard Deviation: {stats['std']:.3f} units")
        print(f"  Data Points:        {stats['count']}")
        
        # Accuracy assessment
        if stats['mean'] < 0.1:
            accuracy = "Excellent"
        elif stats['mean'] < 0.5:
            accuracy = "Good"
        elif stats['mean'] < 1.0:
            accuracy = "Fair"
        else:
            accuracy = "Poor"
        
        print(f"  Accuracy Rating:    {accuracy}")
    
    print("\n" + "="*60)

def main():
    parser = argparse.ArgumentParser(description='Analyze navigation test data')
    parser.add_argument('--input', '-i', type=str, help='Input file with test output')
    parser.add_argument('--output', '-o', type=str, default='./navigation_analysis', 
                       help='Output directory for CSV files and plots')
    
    args = parser.parse_args()
    
    # Get test output
    if args.input:
        with open(args.input, 'r') as f:
            output_text = f.read()
    else:
        print("Reading from stdin... (paste test output, then Ctrl+D)")
        output_text = sys.stdin.read()
    
    # Extract CSV data
    csv_sections = extract_csv_from_output(output_text)
    
    if not csv_sections:
        print("No CSV data found in input!")
        return 1
    
    print(f"Found {len(csv_sections)} test patterns: {list(csv_sections.keys())}")
    
    # Generate analysis
    deviation_stats = plot_movement_data(csv_sections, args.output)
    
    # Print report
    print_deviation_report(deviation_stats)
    
    return 0

if __name__ == '__main__':
    sys.exit(main())