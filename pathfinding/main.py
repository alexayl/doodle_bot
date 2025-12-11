import argparse
import time
from img2graph import img2graph
from graph2path import graph2path
from path2gcode import path2gcode
from simulate import simulate

def main(img_name, canvas_size=(575, 730), time_execution=False, simumlate=False, granularity=5):

    output_file = "gcode/" + img_name + ".gcode"

    loops = 1
    if time_execution:
        loops = 100
        img2graph_times = []
        graph2path_times = []
        path_gcode_times = []

    for _ in range(loops):
        
        if time_execution:
            start_time = time.time()

        graph, endpoints = img2graph(img_name, granularity=granularity, canvas_size=canvas_size)
        
        if time_execution:
            end_time = time.time()
            # print(f"Image to graph time: {end_time - start_time:.5f} seconds")
            img2graph_times.append(end_time - start_time)
            start_time = time.time()

        path = graph2path(graph, endpoints, canvas_size=canvas_size)

        if time_execution:
            end_time = time.time()
            # print(f"Graph to path time: {end_time - start_time:.5f} seconds")
            graph2path_times.append(end_time - start_time)
            start_time = time.time()

        path2gcode(path, filename=output_file)

        if time_execution:
            end_time = time.time()
            # print(f"Path to G-code time: {end_time - start_time:.5f} seconds")
            path_gcode_times.append(end_time - start_time)

    if simumlate:
        simulate(filename=output_file, canvas_size=canvas_size)

    if time_execution:
        avg_img2graph = sum(img2graph_times) / loops
        avg_graph2path = sum(graph2path_times) / loops
        avg_path2gcode = sum(path_gcode_times) / loops
        print(f"Average Image to graph time over {loops} runs: {avg_img2graph:.5f} seconds")
        print(f"Average Graph to path time over {loops} runs: {avg_graph2path:.5f} seconds")
        print(f"Average Path to G-code time over {loops} runs: {avg_path2gcode:.5f} seconds")

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Convert an image to a graph")
    parser.add_argument("img_name", help="image name in pathfinding png/")
    parser.add_argument("--sim", action="store_true", help="simulate")
    parser.add_argument("--time", action="store_true", help="time the execution")
    parser.add_argument("--g", type=int, default=5, help="granularity")
    args = parser.parse_args()

    img_name = args.img_name
    time_execution = args.time
    simumlate = args.sim
    granularity = args.g
    canvas_size = (1390, 940)

    main(img_name, canvas_size=canvas_size, time_execution=time_execution, simumlate=simumlate, granularity=granularity)