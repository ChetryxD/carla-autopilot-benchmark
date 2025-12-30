<img width="1732" height="1017" alt="Screenshot 2025-12-30 174543" src="https://github.com/user-attachments/assets/3bfd4d0b-03fd-424b-b6e4-275511146c2e" />




CARLA TM Autopilot Benchmark Sweep (CARLA 0.9.16)
Third-Person Video + JSON Metrics
📋 Project Overview
This repository benchmarks CARLA's Traffic Manager (TM) autopilot in a controlled and repeatable way using CARLA 0.9.16 on Town10HD_Opt. It runs the ego vehicle across a grid of traffic density × target speed configurations, records driving metrics (distance, speed, collisions, etc.), and optionally saves a high-quality Third-Person Perspective (TPP) video for each run. Each run is stored in its own timestamped folder with a clean output structure for easy review and presentation.

✨ Features
✅ 1) TM Autopilot Benchmark Sweep (Traffic × Speed)
The benchmark runs a sweep over:

Traffic counts: 0, 10, 30
Target speeds: 20, 30, 40 km/h

Total runs: 9 configurations
✅ 2) Saves summary.json Per Run
Each run generates a summary.json with:

Duration and distance traveled
Average speed (overall + moving-only if enabled)
Target speed & traffic count
Collision count
Stuck handling events (lane change / autopilot reset if enabled)
Video settings used

This makes benchmarking comparable and plot-ready.
✅ 3) Saves TPP Video (Optional)
If opencv-python + numpy are installed, the script records TPP video per run:

Output: video.avi (MJPG codec by default)

TPP view shows:

Surrounding map context (signals, lanes, vehicles)
Ego vehicle behavior (braking, waiting, lane changes)
Traffic density visualization


🚀 Setup
Requirements

CARLA version: 0.9.16
Python environment with CARLA PythonAPI available
CARLA server must be running before starting the script

Option A: Conda Environment (Recommended)
From repo root:
bashconda env create -f environment.yml
conda activate carla-ai
Option B: pip Install
bashpip install -r requirements.txt
For video recording, ensure:
bashpip install opencv-python numpy
CARLA PythonAPI Path (Important)
Your script must be able to import carla.
If CARLA PythonAPI is not installed into your environment, ensure your CARLA PythonAPI folder is on the Python path.
Typical CARLA path example:
C:\Users\<you>\Downloads\CARLA_0.9.16\PythonAPI\carla

🎮 Usage
1. Run CARLA Server First
Windows:
bashCarlaUE4.exe
Default server: 127.0.0.1:2000
Optional check:
bashnetstat -ano | findstr :2000
2. Run the Benchmark Sweep
From the repo root:
bashpython scripts/40_benchmark_sweep_tm_autopilot_star_v3.py
This runs the full 9-run grid:

Traffic: 0, 10, 30
Speed: 20, 30, 40 km/h


📂 Outputs
Each run creates a timestamped folder like:
runs/
  20251230_145147_tm_t20_cars0/
    config.json
    summary.json
    video.avi   (optional)
config.json
Stores run configuration:

Mode
Duration
Target speed
Traffic count
Video resolution / FPS
TM port

summary.json
Stores final metrics:

Distance traveled
Average speed
Collision count
Stuck handling stats (if enabled)

video.avi (Optional)
Third-person camera recording for review and presentation.

📊 Results
✅ Best Run Summary (Example)
Replace this with your best run summary.json:
json{
  "mode": "tm_autopilot",
  "map": "Carla/Maps/Town10HD_Opt",
  "duration_s": 300,
  "wall_time_s": 300.03,
  "distance_m": 767.64,
  "avg_speed_mps": 2.559,
  "avg_speed_kmh": 9.21,
  "target_speed_kmh": 25,
  "traffic_count": 20,
  "collisions": 1,
  "route_refreshes": 0,
  "creep_assists": 1,
  "offroad_percent": 0.0,
  "ignore_traffic_lights": false,
  "ignore_stop_signs": false,
  "video_view": "TPP",
  "video_res": [1920, 1080],
  "video_fps": 20,
  "video_file": "video.mp4",
  "tm_port": 8000
}
🎥 Demo Screenshot / Clip
Recommended structure:
docs/
  media/
    demo.png
Then embed it:
markdown![TPP Demo](docs/media/demo.png)
(For a short clip, use docs/media/demo.mp4 and mention it in the README)

⚠️ Known Limitations
1) TM Hesitation / Intersection Uncertainty
Traffic Manager can:

Stop behind vehicles even when a lane change might be possible
Hesitate at intersections
Sometimes get confused on turns in dense urban areas

This is expected behavior in some Town10HD_Opt scenes.
2) GPU Crash: DXGI_ERROR_DEVICE_REMOVED
If CARLA crashes with DXGI_ERROR_DEVICE_REMOVED, this is usually GPU/VRAM pressure.
✅ Fixes:

Lower resolution (e.g., 1280×720)
Lower FPS (e.g., 15)
Reduce traffic count
Close other GPU-heavy applications
Update GPU drivers


📁 Suggested Repository Structure
carla-ai-projects/
  scripts/
    40_benchmark_sweep_tm_autopilot_star_v3.py
  docs/
    media/
      demo.png
      demo.mp4
  runs/                 # generated outputs (don't push all runs)
  environment.yml
  requirements.txt
  README.md
  .gitignore

📄 License
No license added yet. If you make this repo public, consider adding an MIT License.

🤝 Contributing
Contributions are welcome! Please feel free to submit a Pull Request.
