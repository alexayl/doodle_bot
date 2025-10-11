import argparse
from img2graph import img2graph
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np

def get_distance(node1, node2, graph):
    """Return 0 if nodes share the same componentID, else Euclidean distance."""
    if graph.nodes[node1]['componentID'] == graph.nodes[node2]['componentID']:
        return 0
    return np.linalg.norm(np.array(node1) - np.array(node2))

def graph2path(graph, endpoints, starting_node_inset_p=0.05, canvas_size=(1000, 1460)):

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

    # to_remove = [node for node in graph_nodes if node not in endpoints]
    # graph.remove_nodes_from(to_remove)

    # Build a nearest neighbor route starting and ending at endpoints[0]
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
    print("Nearest neighbor path built with length:", len(path))

    # Add edges along the path
    # for i in range(len(endpoints) - 1):
    #     node1, node2 = endpoints[i], endpoints[i + 1]
    #     distance = get_distance(node1, node2, graph)
    #     if distance == 0:
    #         continue
    #     graph.add_edge(node1, node2, weight=distance, color='red')

    print("Edges added for nearest neighbor path. Ready for 2-opt optimization...")

    def calc_total_distance(path, graph):
        total = 0
        for i in range(len(path) - 1):
            total += get_distance(path[i], path[i + 1], graph)
        total += get_distance(path[-1], path[0], graph)
        return total

    def two_opt(path, graph, max_iterations=1000):
        best = path
        best_distance = calc_total_distance(best, graph)
        improved = True
        iteration = 0
        while improved and iteration < max_iterations:
            improved = False
            iteration += 1
            for i in range(1, len(best) - 2):
                for j in range(i + 1, len(best)):
                    if j - i == 1:
                        continue
                    new_path = best[:i] + best[i:j][::-1] + best[j:]
                    new_distance = calc_total_distance(new_path, graph)
                    if new_distance < best_distance:
                        best = new_path
                        best_distance = new_distance
                        improved = True
            print(f"Iteration {iteration}: current best distance = {best_distance:.2f}")
        return best

    print("Running 2-opt optimization...")
    optimized_path = two_opt(endpoints, graph)

    print(optimized_path)

    # Draw optimized path
    path_edges = [(optimized_path[i], optimized_path[i + 1]) for i in range(len(optimized_path) - 1)]
    for edge in path_edges:
        node1, node2 = edge
        if graph.nodes[node1]['componentID'] == graph.nodes[node2]['componentID']:
            continue
        distance = get_distance(node1, node2, graph)
        graph.add_edge(node1, node2, weight=distance, color='red')

    print("Optimized path visualized.")
    
    #     same_component = graph.nodes[node1]['componentID'] == graph.nodes[node2]['componentID']
    #     if same_component:
    #         graph.add_edge(node1, node2, weight=0)
    #         graph.edges[(node1, node2)]['color'] = 'red'
    #         continue
    #     distance = np.linalg.norm(np.array(node1) - np.array(node2))
    #     graph.add_edge(node1, node2, weight=distance)
    #     graph.edges[(node1, node2)]['color'] = 'black'
    # last_node = endpoints[-1]
    # distance = np.linalg.norm(np.array(starting_node) - np.array(last_node))
    # graph.add_edge(starting_node, last_node, weight=distance)
    # graph.edges[(starting_node, last_node)]['color'] = 'black'

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
    plt.show()
    plt.close(fig)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Convert an image to a graph")
    parser.add_argument("img_name", help="image name in pathfinding png/")
    parser.add_argument("--debug", action="store_true", help="display debug information")
    parser.add_argument("--save", action="store_true", help="save debug information")
    args = parser.parse_args()

    debug_all = args.debug
    debug_array = []

    save_all = args.save
    save_array = []

    canvas_size = (500, 730)
    
    if debug_all == True:
        display_array = [True]

    if save_all == True:
        save_array = [True]

    graph, endpoints = img2graph(args.img_name, canvas_size=canvas_size, debug=[False, False, False, False], 
                      save=[False, False, False, False])
    path = graph2path(graph, endpoints, canvas_size=canvas_size)
