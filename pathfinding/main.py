import argparse
import time
from img2graph import img2graph
from graph2path import graph2path
from path2gcode import path2gcode
from simulate import simulate

def main(img_name, canvas_size=(575, 730), time_execution=False, simumlate=False):

    output_file = "gcode/" + img_name + ".gcode"

    if time_execution:
        start_time = time.time()

    graph, endpoints = img2graph(img_name, canvas_size=canvas_size)

    if time_execution:
        end_time = time.time()
        print(f"Image to graph time: {end_time - start_time:.2f} seconds")
        start_time = time.time()

    path = graph2path(graph, endpoints, canvas_size=canvas_size)

    if time_execution:
        end_time = time.time()
        print(f"Graph to path time: {end_time - start_time:.2f} seconds")
        start_time = time.time()

    path2gcode(path, filename=output_file)

    if time_execution:
        end_time = time.time()
        print(f"Path to G-code time: {end_time - start_time:.2f} seconds")
    
    if simumlate:
        simulate(filename=output_file, canvas_size=canvas_size)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Convert an image to a graph")
    parser.add_argument("img_name", help="image name in pathfinding png/")
    parser.add_argument("--sim", action="store_true", help="simulate")
    parser.add_argument("--time", action="store_true", help="time the execution")
    args = parser.parse_args()

    img_name = args.img_name
    time_execution = args.time
    simumlate = args.sim
    canvas_size = (575, 730)

    main(img_name, canvas_size=canvas_size, time_execution=time_execution, simumlate=simumlate)