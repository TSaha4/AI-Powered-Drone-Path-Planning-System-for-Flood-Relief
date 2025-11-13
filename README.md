# Drone Flood-Relief Mission Planner

A computer-vision and path-planning pipeline that ingests annotated flood maps, infers safe supply drop locations, and exports an autonomous drone mission in QGroundControl waypoint format.


## Why It Matters

Floods often render roads unusable, making aerial delivery one of the fastest ways to reach stranded communities. This project automates the tedious work of plotting mission plans by turning a single annotated map into a complete flight path, including safe drop zones, takeoff/landing instructions, and servo triggers for payload release.


## Core Capabilities

- **Adaptive flood-area segmentation** driven by operator-selected pixels.
- **Contour clustering** with centroid and convex hull extraction for each flood zone.
- **Safe drop-point selection** along cluster boundaries closest to the planned route.
- **Nearest-neighbor TSP routing** to minimize total flight distance while returning home.
- **Mission export** as `QGC WPL 110` waypoints compatible with QGroundControl.
- **Map visualization** with contours, safe points, home base, and flight path overlays.


## Project Structure

- `main.py` – CLI entry point that orchestrates image processing, path planning, and mission export.
- `image_processing.py` – Interactive HSV sampling, flood masking, contour filtering.
- `clustering.py` – Computes convex hulls and centroids for detected flood regions.
- `safe_dropzone.py` – Chooses drop points using point-to-path distance minimization.
- `pathfinding.py` – Greedy nearest-neighbor TSP implementation.
- `mission_output.py` – Generates waypoint files and draws the final map overlay.
- `*.png` – Sample flood maps (e.g., `varanasi.png`, `kanpur.png`, `assam.png`).


## Algorithms & Techniques

- **Dynamic HSV thresholding** – Samples operator clicks, expands hue/saturation/value margins, accounts for red wrap-around.
- **Morphological filtering** – `cv2.morphologyEx` with close/open operations to denoise masks.
- **Contour analysis** – `cv2.findContours`, area filtering, convex hull construction, image moment centroids.
- **Point-to-segment geometry** – Fast projection math to keep drop points near the flight path.
- **Nearest Neighbor TSP** – Greedy O(n²) heuristic that is fast, simple, and returns to the starting point.
- **Mission sequencing** – Generates takeoff, transit, loiter, descent, servo trigger, climb, RTL, and land commands.


## Requirements

| Component | Version/Notes |
|-----------|---------------|
| Python    | 3.8+ recommended |
| OpenCV    | `pip install opencv-python` |
| NumPy     | `pip install numpy` |
| Matplotlib| `pip install matplotlib` (for visualization) |

> Optional: Install `opencv-python-headless` on servers without display support and run with `--display-width` to skip GUI scaling.


## Setup

```bash
python -m venv .venv
. .venv/Scripts/activate   # Windows PowerShell: .\.venv\Scripts\Activate.ps1
pip install opencv-python numpy matplotlib
```

If you already have these dependencies system-wide, you can skip the virtual environment step.


## Running the Pipeline

```bash
python main.py path/to/flood_map.png \
    --display-width 750 \
    --min-area 200
```

### CLI Arguments

- `image_path` *(optional)* – Path to the annotated flood map (default: `varanasi.png`).
- `--display-width` – Resize width for the interactive window (default: 750 px).
- `--min-area` – Minimum contour area in pixels to keep (default: 200).

### Interactive Sampling Workflow

1. An OpenCV window opens: `Select flood points (click) and press 'c'`.
2. Click multiple pixels inside the flood (red) regions to collect HSV samples.
3. Press `c` to finalize sampling and start automatic processing.

The terminal logs the computed HSV thresholds, contour counts, and path order. Any display windows can be dismissed by pressing a key once viewing is complete.


## Outputs

- `enriched_drone_mission.waypoints` – QGC WPL 110 mission file ready for import into QGroundControl or ArduPilot-based GCS.
- Visualization windows illustrating:
  - Flood mask (binary view of detected flood pixels).
  - Final mission map with contours (green), safe drop points (blue), home location (black), and flight path (red).

To deploy, import the waypoint file into QGroundControl, review altitude/servo parameters, and sync to the drone.


## How It Works (Detailed)

1. **Load Map & Resize** – Reads the provided image, scales to the requested width while preserving aspect ratio.
2. **HSV Sampling** – Operator clicks capture flood colors; dynamic thresholds are computed with configurable margins.
3. **Mask Generation** – Flood mask is built via HSV thresholding and cleaned with morphological operations.
4. **Contour Extraction** – External contours are detected, filtered by area, and converted to convex hulls.
5. **Cluster Centroids** – Image moments yield per-cluster centroids; hull vertices define candidate drop edges.
6. **Home Selection** – The safe point closest to the image center becomes takeoff/landing home.
7. **Route Planning** – Nearest-neighbor TSP orders safe points, prepending/appending home for a closed loop.
8. **Safe Point Refinement** – For each contour, the edge point closest to consecutive path segments is chosen to minimize travel deviation.
9. **Mission Encoding** – Waypoint file adds takeoff, travel, loiter, descend, servo trigger, climb, return, and land commands.
10. **Visualization & Export** – Displays interactive plots and writes the mission plan to disk.


## Extending the Project

- **Alternate color schemes** – Adjust margins or add presets for non-red flood annotations.
- **Advanced TSP solvers** – Integrate heuristics (2-opt, simulated annealing) for larger point sets.
- **Terrain awareness** – Incorporate elevation maps or no-fly zones for safer routing.
- **Real GPS mapping** – Replace pixel-to-GPS heuristic with georeferenced map projections.
- **Batch processing** – Automate mission generation for multiple maps or live drone feeds.
- **Real-time data ingestion** – Stream satellite or aerial imagery (e.g., Sentinel-2, PlanetScope, drone video) into the pipeline, auto-refreshing detections as new frames arrive.
- **Simulation integration** – Feed the generated waypoint files directly into SITL or Gazebo-based drone simulators for mission rehearsal and validation before flight.


## Troubleshooting

- **No contours detected** – Ensure sample clicks cover representative flood pixels; adjust `--min-area` downwards.
- **Image display errors** – On headless servers, install `opencv-python-headless` or disable visualization.
- **Unwanted drop points** – Increase morphological kernel size or tweak HSV margins to better isolate flood areas.
