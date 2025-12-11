import argparse
from img2graph import img2graph, save_image
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import time

def get_distance(node1, node2, graph):
    """
    Return 0 if nodes share the same componentID, else Euclidean distance.
    
    Args: 
        node1 (tuple): Coordinates of the first node.
        node2 (tuple): Coordinates of the second node.
        graph (networkx.Graph): The graph containing the nodes.
    
    Returns:
        float: 0 if same componentID, else Euclidean distance.
    """
    if graph.nodes[node1]['componentID'] == graph.nodes[node2]['componentID']:
        return 0
    return np.linalg.norm(np.array(node1) - np.array(node2))

def graph2path(graph, endpoints, starting_node_inset_p=0.20, canvas_size=(1150, 1460), debug=False, save=False):
    """
    Convert a graph to an optimized path visiting all endpoints.
    
    Args:
        graph (networkx.Graph): The input graph.
        endpoints (list): List of endpoint nodes.
        starting_node_inset_p (float): Inset percentage for starting node position.
        canvas_size (tuple): Size of the canvas (width, height).
        debug (bool): Whether to display debug information.
        save (bool): Whether to save debug information.

    Returns:
        list: A list of segments representing the optimized path.
    """

    # Add a starting node at the top left corner
    starting_node = (int(canvas_size[0] * starting_node_inset_p), \
                     int(canvas_size[1] * (1 - starting_node_inset_p)))
    graph.add_node(starting_node)
    graph.nodes[starting_node]['color'] = 'red'
    graph.nodes[starting_node]['endpoint'] = True   ## maybe remove?
    graph.nodes[starting_node]['componentID'] = 0
    endpoints.insert(0, starting_node)

    # Clean endpoint list
    graph_nodes = set(graph.nodes())
    endpoints = [node for node in endpoints if node in graph_nodes]

    # Build a nearest neighbor route starting and ending at endpoints[0] greedy
    start = endpoints[0]
    unvisited = set(endpoints[1:])
    path = [start]
    current = start
    while unvisited:
        next_node = min(unvisited, key=lambda node: get_distance(current, node, graph))
        path.append(next_node)
        unvisited.remove(next_node)
        current = next_node
    path.append(start)
    endpoints = path
    print("Nearest neighbor path found successfully")

    optimized_path = endpoints

    # Draw optimized path
    path_edges = [(optimized_path[i], optimized_path[i + 1]) for i in range(len(optimized_path) - 1)]
    for edge in path_edges:
        node1, node2 = edge
        if graph.nodes[node1]['componentID'] == graph.nodes[node2]['componentID']:
            continue
        distance = get_distance(node1, node2, graph)
        graph.add_edge(node1, node2, weight=distance, color='red')

    # Visualize graph with optimized path
    if debug or save:
        cols, rows = canvas_size
        aspect_ratio = cols / rows
        fig, ax = plt.subplots(figsize=(5 * aspect_ratio, 5))
        pos = {node: node for node in graph.nodes()}
        nx.draw(graph, pos, 
                node_size=5,
                node_color=[graph.nodes[node].get('color', 'black') for node in graph.nodes()], 
                edge_color=[graph.edges[edge].get('color', 'black') for edge in graph.edges()],
                ax=ax)
        ax.set_aspect('equal')
        ax.set_xlim(0, cols)
        ax.set_ylim(0, rows)
        ax.axis('on')
        if debug:
            plt.show()
        if save:
            save_image(fig, "path_" + img_name, "outputs/path/", extension=".png")
        plt.close(fig)

    # construct path in correct data type
    segments = [[optimized_path[0]]]
    for i in range(1, len(optimized_path) - 1, 2):
        starting_endpoint = optimized_path[i]
        ending_endpoint = optimized_path[i+1]
        segment = [starting_endpoint]
        while(segment[-1] != ending_endpoint):
            neighbors = list(graph.neighbors(segment[-1]))
            if len(neighbors) > 2:
                print("ERROR TOO MANY NEIGHBORS")
                break
            if neighbors[0] in segment:
                next_node = neighbors[1]
            else:
                next_node = neighbors[0]
            segment.append(next_node)
        segments.append(segment)
    segments.append([optimized_path[-1]])

    return segments

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Convert an image to a graph")
    parser.add_argument("img_name", help="image name in pathfinding png/")
    parser.add_argument("--debug", action="store_true", help="display debug information")
    parser.add_argument("--save", action="store_true", help="save debug information")
    parser.add_argument("--time", action="store_true", help="time the execution")
    args = parser.parse_args()

    img_name = args.img_name

    debug_all = args.debug
    debug_array = []

    save_all = args.save
    save_array = []

    canvas_size = (900, 530)
    
    if debug_all == True:
        display_array = [True]

    if save_all == True:
        save_array = [True]

    graph, endpoints = img2graph(img_name, granularity=5, canvas_size=canvas_size, debug=[False, False, False, False], 
                      save=[False, False, False, False])
    
    if args.time:
        start_time = time.time()
    path = graph2path(graph, endpoints, canvas_size=canvas_size, debug=debug_all, save=save_all)
    if args.time:
        end_time = time.time()
        print(f"graph2path executed in {end_time - start_time:.2f} seconds")
