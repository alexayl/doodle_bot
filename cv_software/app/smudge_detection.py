"""
Smudge detection and targeted erasing for whiteboard cleanup.

Uses CV to detect dark regions (smudges) on the board and generates
G-code paths to erase them efficiently.
"""
from __future__ import annotations
import cv2
import numpy as np
from typing import List, Tuple, Optional
import math


class SmudgeDetector:
    """Detect smudges (dark regions) on calibrated whiteboard."""
    
    def __init__(
        self,
        min_smudge_area_mm2: float = 25.0,  # Ignore tiny smudges < 5mm x 5mm
        darkness_threshold: int = 200,       # Pixels darker than this are "smudged"
        erosion_kernel_size: int = 3,        # Clean up noise
        dilation_kernel_size: int = 5,       # Connect nearby smudges
    ):
        self.min_smudge_area_mm2 = min_smudge_area_mm2
        self.darkness_threshold = darkness_threshold
        self.erosion_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (erosion_kernel_size, erosion_kernel_size)
        )
        self.dilation_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (dilation_kernel_size, dilation_kernel_size)
        )
    
    def detect_smudges(
        self,
        frame_bgr: np.ndarray,
        H_img2board: np.ndarray,
        board_width_mm: float,
        board_height_mm: float,
    ) -> List[np.ndarray]:
        """
        Detect smudge regions on the board.
        
        Args:
            frame_bgr: Camera frame (BGR)
            H_img2board: Homography matrix (image -> board coords in mm)
            board_width_mm: Board width in mm
            board_height_mm: Board height in mm
        
        Returns:
            List of smudge contours in board coordinates (mm)
        """
        # Convert to grayscale
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        
        # Threshold: dark regions = smudges
        # Invert so smudges are white (255) and clean board is black (0)
        _, binary = cv2.threshold(gray, self.darkness_threshold, 255, cv2.THRESH_BINARY_INV)
        
        # Morphological operations to clean up noise and connect nearby smudges
        binary = cv2.erode(binary, self.erosion_kernel, iterations=1)
        binary = cv2.dilate(binary, self.dilation_kernel, iterations=2)
        
        # Find contours (smudge regions)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Transform contours to board coordinates and filter by size
        smudge_contours_board = []
        
        for contour in contours:
            # Transform contour points to board coordinates
            contour_pts = contour.reshape(-1, 2).astype(np.float32)
            contour_board = cv2.perspectiveTransform(
                contour_pts[None, :, :], H_img2board
            )[0]
            
            # Calculate area in mm^2
            area_mm2 = cv2.contourArea(contour_board)
            
            # Filter out tiny smudges
            if area_mm2 >= self.min_smudge_area_mm2:
                # Ensure contour is within board bounds
                contour_board = self._clip_to_board(
                    contour_board, board_width_mm, board_height_mm
                )
                smudge_contours_board.append(contour_board)
        
        return smudge_contours_board
    
    def _clip_to_board(
        self,
        contour: np.ndarray,
        width_mm: float,
        height_mm: float,
    ) -> np.ndarray:
        """Clip contour points to board bounds."""
        contour[:, 0] = np.clip(contour[:, 0], 0, width_mm)
        contour[:, 1] = np.clip(contour[:, 1], 0, height_mm)
        return contour


class ErasePathGenerator:
    """Generate efficient erase paths for smudge regions."""
    
    def __init__(
        self,
        line_spacing_mm: float = 5.0,  # Spacing between erase lines
        margin_mm: float = 2.0,         # Extend past smudge edges
    ):
        self.line_spacing_mm = line_spacing_mm
        self.margin_mm = margin_mm
    
    def generate_erase_path(
        self,
        smudge_contours: List[np.ndarray],
        current_position: Tuple[float, float] = (0, 0),
    ) -> List[str]:
        """
        Generate G-code to erase detected smudges.
        
        Args:
            smudge_contours: List of smudge contours in board coordinates (mm)
            current_position: Current bot position (x_mm, y_mm)
        
        Returns:
            List of G-code commands to erase all smudges
        """
        if not smudge_contours:
            return []
        
        gcode = []
        current_x, current_y = current_position
        
        # Sort smudges by distance from current position (nearest first)
        smudges_sorted = self._sort_by_distance(smudge_contours, current_x, current_y)
        
        for smudge_contour in smudges_sorted:
            # Get bounding box of smudge (with margin)
            x_coords = smudge_contour[:, 0]
            y_coords = smudge_contour[:, 1]
            
            x_min = float(np.min(x_coords)) - self.margin_mm
            x_max = float(np.max(x_coords)) + self.margin_mm
            y_min = float(np.min(y_coords)) - self.margin_mm
            y_max = float(np.max(y_coords)) + self.margin_mm
            
            # Generate horizontal raster pattern over smudge
            smudge_gcode = self._generate_raster_pattern(
                x_min, x_max, y_min, y_max, current_x, current_y
            )
            
            gcode.extend(smudge_gcode)
            
            # Update current position (end of last line)
            if smudge_gcode:
                # Parse last position from last G1 command
                last_cmd = smudge_gcode[-1]
                current_x, current_y = self._parse_position_from_gcode(last_cmd, current_x, current_y)
        
        return gcode
    
    def _generate_raster_pattern(
        self,
        x_min: float,
        x_max: float,
        y_min: float,
        y_max: float,
        start_x: float,
        start_y: float,
    ) -> List[str]:
        """Generate horizontal raster pattern to cover rectangular region."""
        gcode = []
        
        # Move to start of first line (eraser up)
        gcode.append("M280 P1 S90")  # Eraser up
        gcode.append(f"G1 X{int(x_min)} Y{int(y_min)}")
        
        # Lower eraser
        gcode.append("M280 P1 S0")  # Eraser down
        
        # Generate horizontal lines with spacing
        y = y_min
        direction = 1  # 1 = left-to-right, -1 = right-to-left
        
        while y <= y_max:
            if direction == 1:
                # Move right
                gcode.append(f"G1 X{int(x_max)} Y{int(y)}")
            else:
                # Move left
                gcode.append(f"G1 X{int(x_min)} Y{int(y)}")
            
            # Move to next line
            y += self.line_spacing_mm
            if y <= y_max:
                gcode.append(f"G1 Y{int(y)}")
            
            # Reverse direction for next line
            direction *= -1
        
        # Raise eraser
        gcode.append("M280 P0 S90")  # Eraser up
        
        return gcode
    
    def _sort_by_distance(
        self,
        contours: List[np.ndarray],
        x: float,
        y: float,
    ) -> List[np.ndarray]:
        """Sort contours by distance from (x, y) - nearest first."""
        def distance_to_contour(contour: np.ndarray) -> float:
            # Use centroid of contour
            M = cv2.moments(contour)
            if M["m00"] > 0:
                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"]
            else:
                cx = float(np.mean(contour[:, 0]))
                cy = float(np.mean(contour[:, 1]))
            return math.hypot(cx - x, cy - y)
        
        return sorted(contours, key=distance_to_contour)
    
    def _parse_position_from_gcode(
        self,
        cmd: str,
        default_x: float,
        default_y: float,
    ) -> Tuple[float, float]:
        """Parse X/Y position from G1 command."""
        x = default_x
        y = default_y
        
        if "X" in cmd:
            try:
                x_str = cmd.split("X")[1].split()[0]
                x = float(x_str)
            except (IndexError, ValueError):
                pass
        
        if "Y" in cmd:
            try:
                y_str = cmd.split("Y")[1].split()[0]
                y = float(y_str)
            except (IndexError, ValueError):
                pass
        
        return x, y


def generate_smudge_erase_gcode(
    frame_bgr: np.ndarray,
    H_img2board: np.ndarray,
    board_width_mm: float,
    board_height_mm: float,
    current_position: Tuple[float, float] = (0, 0),
    detector_params: Optional[dict] = None,
    path_params: Optional[dict] = None,
) -> Tuple[List[str], List[np.ndarray]]:
    """
    Detect smudges and generate G-code to erase them.
    
    Args:
        frame_bgr: Camera frame
        H_img2board: Homography matrix
        board_width_mm: Board width in mm
        board_height_mm: Board height in mm
        current_position: Current bot position (x_mm, y_mm)
        detector_params: Optional parameters for SmudgeDetector
        path_params: Optional parameters for ErasePathGenerator
    
    Returns:
        (gcode_commands, smudge_contours) tuple
    """
    # Create detector and path generator
    detector_params = detector_params or {}
    path_params = path_params or {}
    
    detector = SmudgeDetector(**detector_params)
    path_gen = ErasePathGenerator(**path_params)
    
    # Detect smudges
    smudge_contours = detector.detect_smudges(
        frame_bgr, H_img2board, board_width_mm, board_height_mm
    )
    
    # Generate erase path
    gcode = path_gen.generate_erase_path(smudge_contours, current_position)
    
    return gcode, smudge_contours
