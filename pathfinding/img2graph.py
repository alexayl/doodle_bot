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
        dpi (int): Resolution in dots per inch
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

def scale_to_canvas(img, canvas_size=(1000//3, 1460//3), padding_percentage=0.1, debug=False, save=False):
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
    
    # Detect background color from image corners
    corners = [
        img.getpixel((0, 0)),
        img.getpixel((img.width-1, 0)),
        img.getpixel((0, img.height-1)),
        img.getpixel((img.width-1, img.height-1))
    ]

    # Use the most common corner color as background
    bg_color = max(set(corners), key=corners.count)

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

def majority_filter(img):
        
    kernel = np.array([[1, 0.5, 1],
                       [0.5, 3, 0.5],
                       [1, 0.5, 1]])
    
    neighbor_count = convolve(img.astype(int), kernel, mode='nearest')
    filtered_img = (neighbor_count >= 4).astype(int)
    
    return filtered_img

# def threshold_selection(img, debug=False, smoothing=0, save=False):
    
#     img_array = np.array(img)
#     # thresholds = [round(num, 2) for num in list(np.logspace(-2, -0.3, 9))]
#     thresholds = list(np.linspace(0.1, 0.9, 9))
#     thresholded_imgs = []
#     selected_index = [None]

#     fig, axes = plt.subplots(3, 3, figsize=(12, 8))
#     axes = axes.flatten()
    
#     def on_click(event):
#         if event.inaxes in axes:
#             index = list(axes).index(event.inaxes)
#             selected_index[0] = index
#             plt.close(fig)
    
#     for i, threshold in enumerate(thresholds):
#         binary_img = 1 - (img_array > threshold * 255).astype(int)
#         for _ in range(smoothing):
#             binary_img = majority_filter(binary_img)
#         thresholded_imgs.append(binary_img)
#         axes[i].imshow(binary_img, cmap='gray')
#         axes[i].set_title(f"{i + 1}: threshold = {threshold}" if debug else f"{i + 1}")
#         axes[i].axis('off')
    
#     fig.canvas.mpl_connect('button_press_event', on_click)
#     plt.suptitle("Click on desired threshold image")
#     plt.tight_layout()
#     plt.show()
    
#     if selected_index[0] is not None:
#         selected_img = thresholded_imgs[selected_index[0]]
#         print("Threshold selected successfully")
#         plt.close('all')
#         plt.title("chosen mappping")
#         plt.imshow(selected_img, cmap='gray')
#         plt.axis('off')
#         plt.show()
#         if save:
#             save_image(plt, "threshold_selection_" + img_name, "outputs/threshold_selection/", extension=".png")
#         plt.close()
#         return selected_img
#     else:
#         print("Threshold selected unsuccessfully")
#         plt.close('all')
#         plt.title("chosen mappping")
#         plt.imshow(selected_img, cmap='gray')
#         plt.axis('off')
#         plt.show()
#         if save:
#             save_image(plt, "threshold_selection_" + img_name, "outputs/threshold_selection/", extension=".png")
#         plt.close()
#         return thresholded_imgs[5]
    
def node_map(img, debug=False, save=False):

    graph = nx.Graph()
    nodes = dict()
    rows, cols = img.shape
    
    # add nodes for dark pixels
    for y in range(img.shape[0]):
        for x in range(img.shape[1]):
            if (img[y,x] == 0):
                nodes[(x, cols - y)] = (x, cols - y)
    graph.add_nodes_from(nodes.keys())

    if debug:
        print("nodes: ", len(nodes))

    # connect neighboring nodes
    for (x, y) in nodes:
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                p = (x + dx, y + dy)
                if p in nodes:
                    dist = np.sqrt(dx ** 2 + dy ** 2)
                    graph.add_edge((x, y), p)

    if debug:
        print("edges: ", len(graph.edges))

    # Set figure size to match image aspect ratio
    aspect_ratio = cols / rows
    plt.figure(figsize=(5 * aspect_ratio, 5))
    nx.draw(graph, nodes, node_size=3)
    plt.gca().set_aspect('equal')
    plt.xlim(0, cols)
    plt.ylim(0, rows)
    plt.axis('on')
    plt.show()
    plt.close()

    return

def img2graph(img_name, debug, save):

    img = load_image(img_name, debug=debug[0], save=save[0])
    img_scaled = scale_to_canvas(img, debug=debug[1], save=save[1])
    img_edge = edge_map(img_scaled, debug=debug[2], save=save[2])
    # img_pix_map = threshold_selection(img_edge, debug=debug[3], save=save[3])
    img_graph = node_map(img_edge, debug=debug[4], save=save[4])

if __name__ == "__main__":

    debug_all = False
    display_array = [False,   # load_image
                     False,   # scale_to_canvas
                     False,   # edge_map
                     False,   # threshold selection
                     True     # node_map
    ]

    save_all = False
    save_array = [False,   # load_image
                  False,   # scale_to_canvas
                  False,   # edge_map
                  True,   # threshold selection
                  True    # node_map
    ]
    
    if debug_all == True:
        display_array = [True, True, True, True, True]

    if save_all == True:
        save_array = [True, True, True, True, True]

    parser = argparse.ArgumentParser(description="Convert an image to a graph")
    parser.add_argument("img_name", help="image name in pathfinding png/")
    args = parser.parse_args()

    img_name = args.img_name
    img_name_ext = img_name + ".png"

    img2graph(args.img_name, display_array, save_array)