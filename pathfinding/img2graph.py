import networkx as nx
import PIL.Image as Image
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
from scipy.ndimage import convolve
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

def load_image(img_name, directory="png/", extension=".png", debug=False, save=False):
    """
        Load an image and convert it to grayscale.

        Args:
            image_name (str): Name of the image file without extension.
            directory (str): Directory where the image is located.
            extension (str): Extension of the image file.
            debug (bool): Whether to display the debugging information.

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

def scale_to_canvas(img, canvas_size=(1000, 1460), padding_percentage=0.1, debug=False, save=False):
    """
        Scale an image to fit within a specified canvas size while maintaining 
        aspect ratio.

        Args:
            img (PIL.Image): Input image to be scaled.
            canvas_size (tuple): Desired canvas size as (width, height).
            padding_percentage (float): Percentage of canvas size to use as padding.
            debug (bool): Whether to display the debugging information.

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

def edge_map(img, debug=False, save=False):

    img_array = np.array(img)

    edges = canny(img_array, sigma=1.0)
    edge_map = (~edges * 255).astype(np.uint8)

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
    
def node_map(img, debug=False, save=False):

    graph = nx.Graph()
    nodes = dict()
    rows, cols = img.shape
    
    # add nodes for dark pixels
    for y in range(img.shape[0]):
        for x in range(img.shape[1]):
            if (img[y,x] == 0):
                nodes[(x, rows - y)] = (x, rows - y)
    graph.add_nodes_from(nodes.keys())

    # debug output
    if debug:
        print("nodes:             ", graph.number_of_nodes())

    # connect neighboring nodes
    for (x, y) in nodes:
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                p = (x + dx, y + dy)
                if p in nodes:
                    graph.add_edge((x, y), p, weight=0)

    # remove staircase nodes
    for (x, y) in nodes:
        graph_nodes = graph.nodes
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
    for (x, y) in graph.nodes:
        while graph.degree((x, y)) >= 3:
            neighbor = list(graph.neighbors((x, y)))[0]
            graph.remove_edge((x, y), neighbor)

    # merge disjoint sections
    while len(list(nx.connected_components(graph))) > 1:
        component_id = 0
        endpoints = []
        min_size = 5
        for disjoint in list(nx.connected_components(graph)):
            if len(disjoint) < min_size:
                graph.remove_nodes_from(disjoint)
            else:
                component_id += 1
                end_not_fount = True
                for node in disjoint:
                    if graph.degree(node) == 1:
                        end_not_fount = False
                        graph.nodes[node]['color'] = 'red'
                        graph.nodes[node]['endpoint'] = True
                        graph.nodes[node]['componentID'] = component_id
                        endpoints.append(node)
                    # elif ('color' in graph.nodes[node]):
                    #     graph.nodes[node]['color'] = 'black'
                    #     graph.nodes[node]['endpoint'] = False
                    #     graph.nodes[node]['componentID'] = component_id
                if end_not_fount:

                    # Add a few possible endpoints
                    # for i in range(0, len(disjoint), min_size):
                    #     node = list(disjoint)[i]
                    #     graph.nodes[node]['color'] = 'red'
                    #     graph.nodes[node]['endpoint'] = True
                    #     graph.nodes[node]['componentID'] = component_id
                    #     endpoints.append(node)

                    # Add one endpoint
                    node = list(disjoint)[0]
                    graph.nodes[node]['color'] = 'red'
                    graph.nodes[node]['endpoint'] = True
                    graph.nodes[node]['componentID'] = component_id
                    endpoints.append(node)

        # Collect all potential edges with distances
        smallest_num_edges = 1
        for node1 in endpoints:
            node1_edges = []
            for node2 in endpoints:
                if node1 != node2 and graph.nodes[node1]['componentID'] != graph.nodes[node2]['componentID']:
                    distance = np.hypot(node1[0] - node2[0], node1[1] - node2[1])
                    node1_edges.append((node1, node2, distance))
            
            # Sort by distance and take smallest_num_edges smallest for this endpoint
            node1_edges.sort(key=lambda x: x[2])
            smallest_for_node1 = node1_edges[:smallest_num_edges]
            
            # Add smallest_num_edges for this endpoint
            for node1, node2, distance in smallest_for_node1:
                graph.add_edge(node1, node2, weight=distance)
                graph[node1][node2]['color'] = 'red'
    
    # debug output
    if debug:
        print("nodes (optimized): ", graph.number_of_nodes())  
        reduction_percentage = (len(nodes) / graph.number_of_nodes() - 1) * 100
        print(f"  reduced by {reduction_percentage:.1f}%")
        print("edges:             ", graph.number_of_edges())

    # visualize node map
    if debug or save:
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
        
    return

def img2graph(img_name, debug, save):

    img = load_image(img_name, debug=debug[0], save=save[0])
    img_scaled = scale_to_canvas(img, debug=debug[1], save=save[1])
    img_edge = edge_map(img_scaled, debug=debug[2], save=save[2])
    img_graph = node_map(img_edge, debug=debug[4], save=save[4])
    return img_graph

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Convert an image to a graph")
    parser.add_argument("img_name", help="image name in pathfinding png/")
    parser.add_argument("--debug", action="store_true", help="display debug information")
    parser.add_argument("--save", action="store_true", help="save debug information")
    args = parser.parse_args()

    img_name = args.img_name
    img_name_ext = img_name + ".png"

    debug_all = args.debug
    display_array = [False,   # load_image
                     False,   # scale_to_canvas
                     False,   # edge_map
                     False,   # threshold selection
                     True     # node_map
    ]

    save_all = args.save
    save_array = [False,   # load_image
                  False,   # scale_to_canvas
                  False,   # edge_map
                  False,   # threshold selection
                  False    # node_map
    ]
    
    if debug_all == True:
        display_array = [True, True, True, True, True]

    if save_all == True:
        save_array = [True, True, True, True, True]

    if args.img_name == 'all':
        for file in os.listdir("png/"):
            if file.endswith(".png"):
                img_name = file[:-4]
                img2graph(img_name, display_array, save_array)
    else:
        img2graph(args.img_name, display_array, save_array)