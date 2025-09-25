import networkx as nx
import PIL.Image as Image
import numpy as np
import matplotlib.pyplot as plt
import argparse

def plot_9x9_grid(maps):
    pass

def combine_maps(edge_map, value_map):
    pass

def edge_map(img):
    pass

def value_map(img):
    pass

def scale_to_canvas(img, canvas_size=(1000, 1460)):
    pass

def img2graph(img):
    pass

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description="Convert an image to a graph")
    parser.add_argument("image_path", help="Path to the image file")
    args = parser.parse_args()
    img2graph(args.image_path)