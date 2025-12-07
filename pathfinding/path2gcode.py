import argparse
import time
from img2graph import img2graph
from graph2path import graph2path

def path2gcode(path, filename="output.gcode"):
    
    # start with absolute positioning
    gcode = []
    # gcode.append("G90 ; use absolute positioning")
    # x, y = path[0][0]
    # gcode.append("M280 P0 S90 ; pen up")
    # gcode.append(f"G1 X{x} Y{y} ; move to start position")
    # gcode.append("G91 ; switch to relative positioning")

    def write_move(dx, dy):
        gcode.append(f"G1 X{dx} Y{dy}")
    
    def write_servo(pen_eraser, up_down):
        if pen_eraser == "pen":
            if up_down == "up":
                gcode.append("M280 P0 S40")
            elif up_down == "down":
                gcode.append("M280 P0 S0")
        elif pen_eraser == "eraser":
            if up_down == "up":
                gcode.append("M280 P1 S40")
            elif up_down == "down":
                gcode.append("M280 P1 S0")

    for s, segment in enumerate(path):
        if s == 0:
            continue
        for p, point in enumerate(segment):
            if p == 0:
                prev_point = path[s-1][-1]
                dz = -1
            elif point == segment[-1]:
                prev_point = segment[p-1]
                dz = 1
            else:
                prev_point = segment[p-1]
                dz = 0
            dx = point[0] - prev_point[0]
            dy = point[1] - prev_point[1]
            write_move(dx, dy)
            if(dz):
                write_servo("pen", "up" if dz == 1 else "down")

    # Write to file
    with open(filename, "w") as f:
        f.write("\n".join(gcode))

    print(f"G-code saved to {filename}")


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Convert an image to a graph")
    parser.add_argument("img_name", help="image name in pathfinding png/")
    parser.add_argument("--debug", action="store_true", help="display debug information")
    parser.add_argument("--save", action="store_true", help="save debug information")
    parser.add_argument("--time", action="store_true", help="time the execution")
    args = parser.parse_args()

    img_name = args.img_name
    canvas_size = (575, 730)

    graph, endpoints = img2graph(img_name, canvas_size=canvas_size, debug=[False, False, False, False], 
                      save=[False, False, False, False])
    path = graph2path(graph, endpoints, canvas_size=canvas_size, debug=False, save=False)
    
    if args.time:
        start_time = time.time()
    gcode = path2gcode(path, filename="gcode/" + img_name + ".gcode")
    if args.time:
        end_time = time.time()
        print(f"graph2path executed in {end_time - start_time:.2f} seconds")