import networkx as nx
import PIL.Image as Image
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
from skimage.feature import canny

def save_image(fig, img_name, directory, extension=".png"):
    """
    Save a matplotlib figure to file.
    
    Args:
        fig (matplotlib.figure.Figure): Figure object to save
        img_name (str): Name of the image file without extension
        directory (str): Directory where to save the image
        extension (str): File extension (e.g., '.png', '.jpg', '.pdf')
    """
    # Create directory if it doesn't exist
    os.makedirs(directory, exist_ok=True)
    
    # Construct full file path
    file_path = directory + img_name + extension
    
    try:
        # Save the figure
        fig.savefig(file_path, bbox_inches='tight', facecolor='white', edgecolor='none', dpi=500)
    except Exception as e:
        print(f"Error saving figure: {e}")

def load_image(img_name, img_name_ext, directory="png/", extension=".png", debug=False, save=False):
    """
    Load an image and convert it to grayscale.

    Args:
        image_name (str): Name of the image file without extension.
        img_name_ext (str): Name of the image file with extension.
        directory (str): Directory where the image is located.
        extension (str): Extension of the image file.
        debug (bool): Whether to display the debugging information.
        save (bool): Whether to save the debugging information.

    Returns:
        PIL.Image: Grayscale image object.
    """

    # Load image and convert to grayscale
    try:
        # Load image
        path = directory + img_name + extension
        img = Image.open(path)

        # Convert to RGB if image has alpha channel
        if img.mode in ("RGBA", "LA"):
            white_bg = Image.new("RGBA", img.size, (255, 255, 255, 255))
            white_bg.paste(img, mask=img.getchannel("A"))
            img = white_bg.convert("RGB")

        # Convert to grayscale
        img = img.convert("L")
        print("Image loaded successfully")

    # Handle file not found error 
    except IOError:
        print("Unable to load image")
        return None
    
    # Display and or save image if debugging
    if save or debug:
        plt.title("loaded " + img_name_ext)
        plt.imshow(img, cmap='gray')
        plt.axis('on')
        if debug:
            plt.show()
        if save:
            save_image(plt, "load_image_" + img_name, "outputs/load_image/", extension=".png")
        plt.close()

    return img

def scale_to_canvas(img, img_name_ext, canvas_size=(1000, 1460), padding_percentage=0.05, debug=False, save=False):
    """
    Scale an image to fit within a specified canvas size while maintaining 
    aspect ratio.

    Args:
        img (PIL.Image): Input image to be scaled.
        img_name_ext (str): Name of the image file with extension.
        canvas_size (tuple): Desired canvas size as (width, height).
        padding_percentage (float): Percentage of canvas size to use as padding.
        debug (bool): Whether to display the debugging information.
        save (bool): Whether to save the debugging information.

    Returns:
        PIL.Image: Scaled image centered on a canvas of specified size.
    """

    # Calculate padding in pixels
    padding_x = int(canvas_size[0] * padding_percentage)
    padding_y = int(canvas_size[1] * padding_percentage)

    # Calculate new size after padding
    new_size = (canvas_size[0] - 2 * padding_x, canvas_size[1] - 2 * padding_y)

    # Calculate aspect ratios
    img_ratio = img.width / img.height
    canvas_ratio = new_size[0] / new_size[1]
    
    # Determine scaling to maintain aspect ratio
    if img_ratio > canvas_ratio:

        # Image is wider relative to canvas - fit to width
        scaled_width = new_size[0]
        scaled_height = int(new_size[0] / img_ratio)
    else:

        # Image is taller relative to canvas - fit to height
        scaled_height = new_size[1]
        scaled_width = int(new_size[1] * img_ratio)
    
    # Resize image maintaining aspect ratio
    img_resized = img.resize((scaled_width, scaled_height), Image.Resampling.LANCZOS)
    
    # Get median value of all edge pixels
    img_array = np.array(img)
    edge_pixels = []

    # Top and bottom edges
    edge_pixels.extend(img_array[0, :])  # Top row
    edge_pixels.extend(img_array[-1, :])  # Bottom row

    # Left and right edges (excluding corners already counted)
    edge_pixels.extend(img_array[1:-1, 0])  # Left column
    edge_pixels.extend(img_array[1:-1, -1])  # Right column

    # Use median of all edge pixels as background
    bg_color = int(np.median(edge_pixels))

    # Create new image with canvas size and paste resized image centered
    canvas = Image.new('L', canvas_size, bg_color)
    
    # Calculate position to center the scaled image
    paste_x = (canvas_size[0] - scaled_width) // 2
    paste_y = (canvas_size[1] - scaled_height) // 2

    # Paste resized image onto canvas
    canvas.paste(img_resized, (paste_x, paste_y))

    # Display image if debugging
    if debug or save:
        plt.title("scaled " + img_name_ext)
        plt.imshow(canvas, cmap='gray')
        plt.axis('on')
        if debug:
            plt.show()
        if save:
            save_image(plt, "scale_to_canvas_" + img_name, "outputs/scale_to_canvas/", extension=".png")
        plt.close()

    print("Image scaled successfully")
    
    return canvas

def edge_map(img, img_name_ext, debug=False, save=False):
    """
    Extract edges from an image using Canny edge detection.
    
    Args:
        img (PIL.Image): Input grayscale image.
        img_name_ext (str): Name of the image file with extension.
        debug (bool): Whether to display the debugging information.
        save (bool): Whether to save the debugging information.

    Returns:
        np.ndarray: Binary edge map of the image.
    """

    # Convert img to numpy array
    img_array = np.array(img)

    # Extract edges using Canny edge detection
    edges = canny(img_array, sigma=1.0)
    edge_map = (~edges * 255).astype(np.uint8)

    # Display edge map if debugging
    if debug or save:
        plt.title("canny edge map of " + img_name_ext)
        plt.imshow(edge_map, cmap='gray')
        plt.axis('on')
        if debug:
            plt.show()
        if save:
            save_image(plt, "edge_map_" + img_name, "outputs/edge_map/", extension=".png")
        plt.close()

    print("Edge map created successfully")

    return edge_map

def init_nodes(img: np.ndarray) -> nx.Graph:
    """
    Initialize a graph from the edge map by placing nodes at dark pixels.
    
    Args:
        img (np.ndarray): Binary edge map of the image.
        debug (bool): Whether to display the debugging information.
        
    Returns:
        networkx.Graph: Graph with nodes at dark pixel locations.
    """

    # create graph
    graph = nx.Graph()
    nodes = dict()
    rows = img.shape[0]
    
    # add nodes for dark pixels
    for y in range(img.shape[0]):
        for x in range(img.shape[1]):
            if (img[y,x] == 0):
                nodes[(x, rows - y)] = (x, rows - y)
    graph.add_nodes_from(nodes.keys())

    return graph

def init_edges(graph: nx.Graph) -> nx.Graph:
    """
    Initialize edges in the graph by connecting neighboring nodes.

    Args:
        graph (nx.Graph): Graph with nodes at dark pixel locations.

    Returns:
        nx.Graph: Graph with edges connecting neighboring nodes.
    """

    # connect neighboring nodes
    nodes = graph.nodes()
    for (x, y) in nodes:
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                p = (x + dx, y + dy)
                if p in nodes:
                    graph.add_edge((x, y), p, weight=0)
    
    return graph

def rm_nodes(graph: nx.Graph) -> nx.Graph:
    """
    Remove redundant nodes from the graph.

    Args:
        graph (nx.Graph): Graph with nodes and edges.

    Returns:
        nx.Graph: Graph with specified nodes removed.
    """

    # remove redundant nodes
    nodes = list(graph.nodes())
    for (x, y) in nodes:
        graph_nodes = graph.nodes()
        n = (x, y - 1) in graph_nodes
        w = (x - 1, y) in graph_nodes
        s = (x, y + 1) in graph_nodes
        e = (x + 1, y) in graph_nodes
        if n and w and not (s or e) or \
           n and e and not (s or w) or \
           s and w and not (n or e) or \
           s and e and not (n or w) or \
           n + e + s + w > 2:
            graph.remove_node((x, y))

    return graph

def rm_edges(graph: nx.Graph) -> nx.Graph:
    """
    Remove specified edges from the graph.

    Args:
        graph (nx.Graph): Graph with nodes and edges.

    Returns:
        nx.Graph: Graph with specified edges removed.
    """

    # remove redundant edges
    for (x, y) in graph.nodes():
        while graph.degree((x, y)) >= 3:
            neighbor = list(graph.neighbors((x, y)))[0]
            graph.remove_edge((x, y), neighbor)

    return graph

def rm_disjoint_sections(graph: nx.Graph, min_component_size: int=10, max_node_num: int=10000) -> nx.Graph:
    """
    Remove small disjoint sections from the graph.

    Args:
        graph (nx.Graph): Graph with nodes and edges.
        min_size (int): Minimum size of disjoint sections to keep.

    Returns:
        nx.Graph: Graph with small disjoint sections removed.
    """

    # merge disjoint sections
    component_id = 0
    component_size = min_component_size
    while graph.number_of_nodes() > max_node_num or component_size == min_component_size and \
          not nx.is_connected(graph):
        for component in list(nx.connected_components(graph)):
            if len(component) < component_size:
                graph.remove_nodes_from(component)
            else:
                component_id += 1
                for node in component:
                    graph.nodes[node]['componentID'] = component_id
        component_size += 10

    return graph

def optimize_graph(graph: nx.Graph) -> nx.Graph:
    """
    Optimize the graph by removing nodes, removing redundant edges, and 
    removing small disjoint sections.

    Args:
        graph (nx.Graph): Graph with nodes and edges.
        debug (bool): Whether to display the debugging information.

    Returns:
        nx.Graph: Optimized graph.
    """

    graph = rm_nodes(graph)
    graph = rm_edges(graph)
    graph = rm_disjoint_sections(graph)
    
    return graph

def is_cyclical(graph: nx.Graph, component: set) -> bool:
    """
    Check if a connected component is cyclical.

    Args:
        component (nx.Graph): Connected component of the graph.

    Returns:
        bool: True if the component is cyclical, False otherwise.
    """

    subgraph = graph.subgraph(component)

    all_degree_2 = all(deg == 2 for _, deg in subgraph.degree())
    return all_degree_2 and subgraph.number_of_edges() == subgraph.number_of_nodes()

def get_endpoints(graph: nx.Graph) -> tuple[nx.Graph, list]:
    """
    Identify endpoints in the graph and mark them.

    Args:
        graph (nx.Graph): Graph with nodes and edges.

    Returns:
        tuple: Tuple containing the graph with endpoints marked and a list of endpoint nodes.
    """

    endpoints = []
    for component in list(nx.connected_components(graph)):
        if is_cyclical(graph, component):
            node1 = list(component)[0]
            node2 = list(graph.neighbors(node1))[0]
            graph.remove_edge(node1, node2)
            graph.nodes[node1]['color'] = 'red'
            graph.nodes[node2]['color'] = 'red'
            graph.nodes[node1]['endpoint'] = True          ## maybe remove?
            graph.nodes[node2]['endpoint'] = True          ## maybe remove?
            endpoints.append(node1)
            endpoints.append(node2)
        else:
            for node in component:
                if graph.degree(node) == 1:
                    graph.nodes[node]['color'] = 'red'
                    graph.nodes[node]['endpoint'] = True   ## maybe remove?
                    endpoints.append(node)

    return graph, endpoints

def node_map(img, img_name_ext, debug=False, save=False):
    """
    Create a graph from the edge map by placing nodes at dark pixels
    and connecting neighboring nodes. Optimize the graph by removing
    staircase nodes and merging disjoint sections.
    
    Args:
        img (np.ndarray): Binary edge map of the image.
        img_name_ext (str): Name of the image file with extension.
        debug (bool): Whether to display the debugging information.
        save (bool): Whether to save the debugging information.

    Returns:
        networkx.Graph: Graph representation of the image edges.
    """

    # Initialize graph with nodes and edges
    graph = init_nodes(img)
    graph = init_edges(graph)

    # Optimize graph by roming redundant nodes, edges, and negligable disjoint sections
    graph = optimize_graph(graph)

    # Identify and mark endpoints
    graph, endpoints = get_endpoints(graph)
    
    # debug output
    if debug:
        print("nodes (optimized): ", graph.number_of_nodes())  
        # reduction_percentage = (len(nodes) / graph.number_of_nodes() - 1) * 100
        # print(f"  reduced by {reduction_percentage:.1f}%")
        print("edges:             ", graph.number_of_edges())

    # visualize node map
    if debug or save:
        rows, cols = img.shape
        aspect_ratio = cols / rows
        fig, ax = plt.subplots(figsize=(5 * aspect_ratio, 5))
        ax.set_title("node map of " + img_name_ext)
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
            save_image(fig, "node_map_" + img_name, "outputs/node_map/", extension=".png")
        plt.close(fig)
        
    return graph

def img2graph(img_name, debug=[False, False, False, False],
              save=[False, False, False, False]):
    """
        Convert an image to a graph by processing it through several steps:
        loading, scaling, edge detection, and node mapping.
        
        Args:
            img_name (str): Name of the image file without extension.
            debug (list): List of booleans for debugging each step.
            save (list): List of booleans for saving each step.
        
        Returns:
            networkx.Graph: Graph representation of the image edges.
    """
    
    img_name_ext = img_name + ".png"
    img = load_image(img_name, img_name_ext, debug=debug[0], save=save[0])
    img_scaled = scale_to_canvas(img, img_name_ext, debug=debug[1], save=save[1])
    img_edge = edge_map(img_scaled, img_name_ext, debug=debug[2], save=save[2])
    img_graph = node_map(img_edge, img_name_ext, debug=debug[3], save=save[3])
    return img_graph

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Convert an image to a graph")
    parser.add_argument("img_name", help="image name in pathfinding png/")
    parser.add_argument("--debug", action="store_true", help="display debug information")
    parser.add_argument("--save", action="store_true", help="save debug information")
    args = parser.parse_args()

    img_name = args.img_name

    debug_all = args.debug
    display_array = [False,   # load_image
                     False,   # scale_to_canvas
                     False,   # edge_map
                     True     # node_map
    ]

    save_all = args.save
    save_array = [False,   # load_image
                  False,   # scale_to_canvas
                  False,   # edge_map
                  False    # node_map
    ]
    
    if debug_all == True:
        display_array = [True, True, True, True]

    if save_all == True:
        save_array = [True, True, True, True]

    if args.img_name == 'all':
        for file in os.listdir("png/"):
            if file.endswith(".png"):
                img_name = file[:-4]
                img2graph(img_name, display_array, save_array)
    else:
        img2graph(img_name, display_array, save_array)