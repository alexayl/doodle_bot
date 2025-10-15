import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def simulate(filename, canvas_size=(575, 730)):
    fig, ax = plt.subplots(figsize=(8, 10))
    ax.set_xlim(0, canvas_size[0])
    ax.set_ylim(0, canvas_size[1])
    ax.set_aspect('equal')
    ax.set_title("DoodleBot Simulation")
    
    # Parse all commands first
    commands = []
    current_x, current_y = None, None
    pen_down = False
    absolute_mode = True
    first_move = True
    
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('//'):
                continue
                
            # Remove comments
            if ';' in line:
                line = line.split(';')[0].strip()
            
            if line.startswith('G90'):
                absolute_mode = True
            elif line.startswith('G91'):
                absolute_mode = False
            elif line.startswith('G1'):
                parts = line.split()

                if len(parts) >= 3:
                    try:
                        dx = int(parts[1][1:])
                        dy = int(parts[2][1:])
                        
                        # Set initial position from first G1 command
                        if first_move:
                            current_x, current_y = dx, dy
                            pen_down = False
                            first_move = False
                            continue
                        
                        # Store previous position
                        prev_x, prev_y = current_x, current_y
                        
                        # Update position
                        if absolute_mode:
                            current_x, current_y = dx, dy
                        else:
                            current_x += dx
                            current_y += dy
                        
                        # Store command with current pen state
                        commands.append({
                            'from': (prev_x, prev_y),
                            'to': (current_x, current_y),
                            'pen_down': pen_down
                        })
                        
                        # # Update pen state AFTER the movement
                        # if dz > 0:
                        #     pen_down = False
                        # elif dz < 0:
                        #     pen_down = True
                            
                    except (ValueError, IndexError):
                        continue
            
            elif line.startswith('M280'):
                parts = line.split()
                if len(parts) >= 3:
                    try:
                        dz = int(parts[2][1:])
                        if dz == 90:
                            pen_down = False
                        elif dz == 0:
                            pen_down = True
                    except (ValueError, IndexError):
                        continue
    
    # Animation variables
    lines_x, lines_y = [], []
    moves_x, moves_y = [], []
    line_plot, = ax.plot([], [], 'b-', linewidth=2, label='Drawing')
    move_plot, = ax.plot([], [], 'r--', alpha=0.5, label='Moves (pen up)')
    
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    def animate(frame):
        if frame < len(commands):
            cmd = commands[frame]
            prev_x, prev_y = cmd['from']
            curr_x, curr_y = cmd['to']
            
            if cmd['pen_down']:
                lines_x.extend([prev_x, curr_x, None])
                lines_y.extend([prev_y, curr_y, None])
            else:
                moves_x.extend([prev_x, curr_x, None])
                moves_y.extend([prev_y, curr_y, None])
            
            line_plot.set_data(lines_x, lines_y)
            move_plot.set_data(moves_x, moves_y)
        
        return line_plot, move_plot
    
    # Create animation
    anim = animation.FuncAnimation(fig, animate, frames=len(commands), 
                                 interval=3, blit=True, repeat=False)
    
    plt.show()
    return anim

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Simulate G-code drawing")
    parser.add_argument("img_name", help="image name in pathfinding png/")
    args = parser.parse_args()

    img_name = args.img_name
    canvas_size = (575, 730)
    filepath = "gcode/" + img_name + ".gcode"
    anim = simulate(filepath, canvas_size=canvas_size)
