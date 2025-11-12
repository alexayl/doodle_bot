# app/path_corrections.py
"""
Learning-based path correction system.
Records deviations during execution, builds a correction model,
and applies corrections to G-code before transmission.
"""

from __future__ import annotations
import os
import json
import math
from typing import Dict, List, Tuple, Optional
from pathlib import Path
from dataclasses import dataclass, asdict
import numpy as np

# Directory for storing correction history
CORRECTIONS_DIR = os.getenv("CORRECTIONS_DIR", "outputs/corrections")
os.makedirs(CORRECTIONS_DIR, exist_ok=True)

@dataclass
class WaypointDeviation:
    """Single observed deviation at a waypoint."""
    waypoint_index: int
    target_x_mm: float
    target_y_mm: float
    actual_x_mm: float
    actual_y_mm: float
    deviation_x_mm: float
    deviation_y_mm: float
    timestamp: float
    confidence: float

@dataclass
class CorrectionModel:
    """Statistical model of corrections per waypoint."""
    path_name: str
    waypoint_corrections: Dict[int, Tuple[float, float]]  # index -> (dx_mm, dy_mm)
    observation_counts: Dict[int, int]  # how many times each waypoint was observed
    last_updated: float

class PathCorrectionLearner:
    """
    Records deviations during path execution and builds correction models.
    """
    
    def __init__(self):
        self.current_path_name: Optional[str] = None
        self.current_deviations: List[WaypointDeviation] = []
        self.recording = False
        
    def start_recording(self, path_name: str):
        """Start recording deviations for a specific path."""
        self.current_path_name = path_name
        self.current_deviations = []
        self.recording = True
        print(f"📊 Started recording deviations for path: {path_name}")
        
    def record_deviation(
        self, 
        waypoint_index: int,
        target_mm: Tuple[float, float],
        actual_mm: Tuple[float, float],
        confidence: float,
        timestamp: float
    ):
        """Record a single waypoint deviation."""
        if not self.recording:
            return
            
        tx, ty = target_mm
        ax, ay = actual_mm
        dx = ax - tx
        dy = ay - ty
        
        deviation = WaypointDeviation(
            waypoint_index=waypoint_index,
            target_x_mm=tx,
            target_y_mm=ty,
            actual_x_mm=ax,
            actual_y_mm=ay,
            deviation_x_mm=dx,
            deviation_y_mm=dy,
            timestamp=timestamp,
            confidence=confidence
        )
        
        self.current_deviations.append(deviation)
        
    def stop_recording_and_save(self):
        """Stop recording and save deviations to disk."""
        if not self.recording or not self.current_path_name:
            return
            
        self.recording = False
        
        if not self.current_deviations:
            print(f"⚠️  No deviations recorded for {self.current_path_name}")
            return
            
        # Save raw deviations
        filepath = Path(CORRECTIONS_DIR) / f"{self.current_path_name}_deviations.json"
        data = {
            "path_name": self.current_path_name,
            "deviations": [asdict(d) for d in self.current_deviations]
        }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
            
        print(f"💾 Saved {len(self.current_deviations)} deviations to {filepath}")
        
        # Update the correction model
        self._update_correction_model()
        
        self.current_deviations = []
        self.current_path_name = None
        
    def _update_correction_model(self):
        """Build/update statistical correction model from historical data."""
        if not self.current_path_name:
            return
            
        # Load all historical deviation files for this path
        pattern = f"{self.current_path_name}_deviations.json"
        all_deviations: List[WaypointDeviation] = []
        
        for fpath in Path(CORRECTIONS_DIR).glob(pattern):
            try:
                with open(fpath, 'r') as f:
                    data = json.load(f)
                    for d in data.get("deviations", []):
                        all_deviations.append(WaypointDeviation(**d))
            except Exception as e:
                print(f"⚠️  Error loading {fpath}: {e}")
                
        if not all_deviations:
            return
            
        # Group by waypoint index and compute mean correction
        waypoint_groups: Dict[int, List[Tuple[float, float]]] = {}
        
        for dev in all_deviations:
            idx = dev.waypoint_index
            if idx not in waypoint_groups:
                waypoint_groups[idx] = []
            # Negate deviation to get correction (we want to move opposite direction)
            waypoint_groups[idx].append((-dev.deviation_x_mm, -dev.deviation_y_mm))
            
        # Compute mean correction per waypoint
        corrections = {}
        counts = {}
        
        for idx, corrections_list in waypoint_groups.items():
            if len(corrections_list) >= 2:  # Need at least 2 observations
                mean_dx = np.mean([c[0] for c in corrections_list])
                mean_dy = np.mean([c[1] for c in corrections_list])
                corrections[idx] = (float(mean_dx), float(mean_dy))
                counts[idx] = len(corrections_list)
                
        if not corrections:
            print(f"⚠️  Not enough data to build correction model for {self.current_path_name}")
            return
            
        # Save correction model
        import time
        model = CorrectionModel(
            path_name=self.current_path_name,
            waypoint_corrections=corrections,
            observation_counts=counts,
            last_updated=time.time()
        )
        
        model_path = Path(CORRECTIONS_DIR) / f"{self.current_path_name}_model.json"
        with open(model_path, 'w') as f:
            json.dump({
                "path_name": model.path_name,
                "waypoint_corrections": {str(k): v for k, v in model.waypoint_corrections.items()},
                "observation_counts": {str(k): v for k, v in model.observation_counts.items()},
                "last_updated": model.last_updated
            }, f, indent=2)
            
        print(f"✅ Updated correction model for {self.current_path_name}")
        print(f"   Corrections for {len(corrections)} waypoints (from {sum(counts.values())} observations)")
        for idx, (dx, dy) in sorted(corrections.items()):
            mag = math.sqrt(dx**2 + dy**2)
            print(f"   WP {idx}: ({dx:+.1f}, {dy:+.1f}) mm [{counts[idx]} obs], mag={mag:.1f}mm")


class PathCorrectionApplier:
    """
    Loads correction models and applies them to waypoints before G-code generation.
    """
    
    @staticmethod
    def load_model(path_name: str) -> Optional[CorrectionModel]:
        """Load correction model for a path."""
        model_path = Path(CORRECTIONS_DIR) / f"{path_name}_model.json"
        
        if not model_path.exists():
            return None
            
        try:
            with open(model_path, 'r') as f:
                data = json.load(f)
                # Convert string keys back to int
                corrections = {int(k): tuple(v) for k, v in data["waypoint_corrections"].items()}
                counts = {int(k): v for k, v in data["observation_counts"].items()}
                
                return CorrectionModel(
                    path_name=data["path_name"],
                    waypoint_corrections=corrections,
                    observation_counts=counts,
                    last_updated=data["last_updated"]
                )
        except Exception as e:
            print(f"⚠️  Error loading correction model for {path_name}: {e}")
            return None
            
    @staticmethod
    def apply_corrections(
        waypoints: List[Tuple[float, float]], 
        path_name: str,
        max_correction_mm: float = 10.0
    ) -> List[Tuple[float, float]]:
        """
        Apply learned corrections to waypoints.
        Returns corrected waypoints.
        """
        model = PathCorrectionApplier.load_model(path_name)
        
        if not model:
            print(f"ℹ️  No correction model found for {path_name}, using original waypoints")
            return waypoints
            
        corrected = []
        corrections_applied = 0
        
        for idx, (x, y) in enumerate(waypoints):
            if idx in model.waypoint_corrections:
                dx, dy = model.waypoint_corrections[idx]
                
                # Clamp correction magnitude
                mag = math.sqrt(dx**2 + dy**2)
                if mag > max_correction_mm:
                    scale = max_correction_mm / mag
                    dx *= scale
                    dy *= scale
                    
                corrected.append((x + dx, y + dy))
                corrections_applied += 1
            else:
                corrected.append((x, y))
                
        if corrections_applied > 0:
            print(f"✨ Applied {corrections_applied}/{len(waypoints)} learned corrections to {path_name}")
        else:
            print(f"ℹ️  No corrections applied (model exists but no matching waypoints)")
            
        return corrected
        
    @staticmethod
    def get_model_stats(path_name: str) -> Optional[Dict]:
        """Get statistics about a correction model."""
        model = PathCorrectionApplier.load_model(path_name)
        
        if not model:
            return None
            
        corrections = list(model.waypoint_corrections.values())
        magnitudes = [math.sqrt(dx**2 + dy**2) for dx, dy in corrections]
        
        return {
            "path_name": model.path_name,
            "num_waypoints_with_corrections": len(corrections),
            "total_observations": sum(model.observation_counts.values()),
            "mean_correction_magnitude_mm": float(np.mean(magnitudes)) if magnitudes else 0.0,
            "max_correction_magnitude_mm": float(np.max(magnitudes)) if magnitudes else 0.0,
            "last_updated": model.last_updated
        }
