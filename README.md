<img width="1732" height="1017" alt="Screenshot 2025-12-30 174543" src="https://github.com/user-attachments/assets/3bfd4d0b-03fd-424b-b6e4-275511146c2e" />

<img width="1335" height="862" alt="image" src="https://github.com/user-attachments/assets/d180e820-b73c-448c-857d-e45dd3810c94" />


# 🚗 CARLA TM Autopilot Benchmark Sweep  
### (CARLA 0.9.16) — Third-Person Video + JSON Metrics

---

## 📋 Project Overview

This repository benchmarks **CARLA's Traffic Manager (TM) autopilot** in a controlled and repeatable way using **CARLA 0.9.16** on `Town10HD_Opt`.

It runs the ego vehicle across a grid of:

- **Traffic density**
- **Target speed configurations**

For each run, it:

- Records driving metrics (distance, speed, collisions, etc.)
- Saves structured `summary.json`
- Optionally records high-quality **Third-Person Perspective (TPP) video**

Each run is stored in its own timestamped folder for clean benchmarking and presentation.

---

## ✨ Features

### ✅ 1) TM Autopilot Benchmark Sweep (Traffic × Speed)

The benchmark runs a full sweep:

- **Traffic counts:** `0, 10, 30`
- **Target speeds:** `20, 30, 40 km/h`

**Total configurations:** 9 runs

---

### ✅ 2) Saves `summary.json` Per Run

Each run generates a `summary.json` containing:

- Duration
- Distance traveled
- Average speed (overall + moving-only if enabled)
- Target speed
- Traffic count
- Collision count
- Stuck handling events
- Video settings

This makes benchmarking structured and plot-ready.

---

### ✅ 3) Saves TPP Video (Optional)

If `opencv-python` + `numpy` are installed, the script records video:

- Output: `video.avi` (MJPG codec)
- View: Third-Person Perspective (TPP)

TPP view shows:

- Surrounding traffic
- Lane structure
- Signals
- Ego vehicle behavior
- Traffic density

---

## 🚀 Setup Requirements

- CARLA version: **0.9.16**
- Python environment with CARLA PythonAPI
- CARLA server must be running before execution

---

## 🧪 Environment Setup

### Option A — Conda (Recommended)

```bash
conda env create -f environment.yml
conda activate carla-ai

Option B — pip

pip install -r requirements.txt

For video recording:

pip install opencv-python numpy
🔧 CARLA PythonAPI Path (Important)

Ensure carla can be imported.

Typical Windows path:

C:\Users\<you>\Downloads\CARLA_0.9.16\PythonAPI\carla

Add to Python path if necessary.

🎮 Usage
1️⃣ Start CARLA Server

Windows:

CarlaUE4.exe

Optional check:

netstat -ano | findstr :2000
2️⃣ Run Benchmark Sweep

From repository root:

python scripts/40_benchmark_sweep_tm_autopilot_star_v3.py

This executes:

Traffic: 0, 10, 30

Speed: 20, 30, 40 km/h

Total: 9 benchmark runs

📂 Outputs

Each run creates:

runs/
  20251230_145147_tm_t20_cars0/
    config.json
    summary.json
    video.avi   (optional)
config.json

Stores:

Mode

Duration

Target speed

Traffic count

Video resolution

FPS

TM port

summary.json (Example Best Run)
{
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
    demo.mp4

Embed screenshot:

![TPP Demo](docs/media/demo.png)
⚠️ Known Limitations
1) TM Hesitation / Intersection Uncertainty

Traffic Manager may:

Stop behind vehicles instead of changing lanes

Hesitate at intersections

Struggle in dense Town10HD_Opt scenes

This is expected TM behavior.

2) GPU Crash: DXGI_ERROR_DEVICE_REMOVED

If CARLA crashes:

Cause: GPU / VRAM pressure

Fixes:

Lower resolution (1280×720)

Reduce FPS (15–20)

Reduce traffic count

Close GPU-heavy apps

Update GPU drivers

📁 Suggested Repository Structure
carla-ai-projects/
│
├── scripts/
│   └── 40_benchmark_sweep_tm_autopilot_star_v3.py
│
├── docs/
│   └── media/
│       ├── demo.png
│       └── demo.mp4
│
├── runs/               # generated outputs (do not commit all runs)
├── environment.yml
├── requirements.txt
├── README.md
└── .gitignore


🤝 Contributing

Contributions are welcome.
Feel free to submit a Pull Request.****
