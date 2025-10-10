# Doodlebot Pathfinding
 
## Overview

<img width="1882" height="548" alt="image2graph" src="https://github.com/user-attachments/assets/4e07b96d-a831-4cc3-812b-d5695d8b2061" />

The Pathfinding Algorithm subsystem serves as the core logic of the DoodleBot’s drawing operation, transforming an input image into a sequence of executable motion commands in the form of G-code. Implemented entirely in Python, the algorithm follows a structured multi-stage pipeline designed to extract drawable paths, determine an efficient traversal order, and generate precise motion instructions for the robot.

The process begins when the CV application subsystem supplies an image source and canvas parameters. The algorithm first applies Canny edge detection to the image to isolate prominent boundaries, producing an edge map that captures the essential drawing strokes. From this edge map, a graph is constructed by converting edge pixels into nodes and connecting neighboring pixels. The graph is then optimized through pruning operations that remove redundant nodes and edges, enforce disjoint line segments, and simplify straight paths via collinearity pruning. Smaller, negligible components are discarded to reduce computational load and maintain fidelity to the original image.

Once the image is reduced to a set of line segments, the Graph-to-Path Algorithm determines an efficient traversal order by formulating the problem as a Traveling Salesman Problem (TSP). Using the nearest neighbor heuristic to generate an initial path, and refining it with the 2-opt algorithm to remove crossings and minimize path length, the subsystem produces a compact and efficient tour that visits all segments exactly once.

Finally, the optimized path is converted into G-code using relative positioning commands. For each pair of consecutive points, positional deltas (dx, dy) are computed, and corresponding G1 X{dx} Y{dy} commands are generated under incremental mode (G91). Marker control is handled through the Z axis, lifting the marker between disconnected segments to prevent unintended strokes. The resulting G-code is saved for execution by the CV subsystem, enabling DoodleBot to accurately reproduce the input image.

In erasing mode, the pathfinding process is simplified. Instead of analyzing an image, the algorithm generates a full-coverage zig-zag pattern across the canvas, ensuring complete erasure. This path is similarly converted into G-code for robot execution.
