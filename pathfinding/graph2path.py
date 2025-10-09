# import argparse
# from img2graph import img2graph
# import networkx as nx
# import matplotlib.pyplot as plt
# import numpy as np

# def graph2path(graph):

#     path = nx.approximation.traveling_salesman_problem(graph)

# if __name__ == "__main__":

#     parser = argparse.ArgumentParser(description="Convert an image to a graph")
#     parser.add_argument("img_name", help="image name in pathfinding png/")
#     parser.add_argument("--debug", action="store_true", help="display debug information")
#     parser.add_argument("--save", action="store_true", help="save debug information")
#     args = parser.parse_args()

#     debug_all = args.debug
#     debug_array = []

#     save_all = args.save
#     save_array = []
    
#     if debug_all == True:
#         display_array = [True]

#     if save_all == True:
#         save_array = [True]

#     graph = img2graph(args.img_name, debug=[False, False, False, True], 
#                       save=[False, False, False, False])
#     path = graph2path(graph)
