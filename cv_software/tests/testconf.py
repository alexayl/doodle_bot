import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]   # repo root
APP  = ROOT / "app"

# allow `import app.cv_core`
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# allow `import cv_core` to resolve to app/cv_core.py
if str(APP) not in sys.path:
    sys.path.insert(0, str(APP))