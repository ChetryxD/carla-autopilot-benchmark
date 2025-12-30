# scripts/40_benchmark_sweep_tm_autopilot_star_v3.py
# CARLA 0.9.16 friendly.
# Fixes:
# 1) ego spawn uses bp_lib.find() + fallbacks (no filter bug)
# 2) always cleans up, even when spawn fails
# 3) optional NUKE_EXISTING_ACTORS so spawn never gets blocked by leftovers
# 4) still records TPP HQ video (AVI MJPG) if opencv+numpy exist

import os, json, time, math, random
from datetime import datetime

import carla

try:
    import numpy as np
    import cv2
except Exception:
    np = None
    cv2 = None

# ------------------------
# USER CONFIG
# ------------------------
HOST, PORT = "127.0.0.1", 2000
MAP_NAME = "Town10HD_Opt"
TM_PORT = 8000

DURATION_S = 600
FIXED_DELTA_SECONDS = 0.05   # 20 Hz
REALTIME_PACING = True

TRAFFIC_GRID = [0, 10, 30]
SPEED_GRID_KMH = [20, 30, 40]

# If you want single run test first:
# TRAFFIC_GRID = [0]
# SPEED_GRID_KMH = [20]

# Strongly recommended while iterating:
NUKE_EXISTING_ACTORS = True  # destroys all vehicles + sensors in the world before each run

# Video
SAVE_VIDEO = True
VIDEO_W, VIDEO_H = 1920, 1080
VIDEO_FPS = 20
VIDEO_EXT = "avi"
VIDEO_FOURCC = "MJPG"

# ---- CAMERA (UPDATED: wide drone-like TPP) ----
# Choose: "tpp_close", "tpp_mid", "drone_wide", "drone_ultrawide"
CAM_VIEW = "drone_wide"

CAM_FOV = 105  # wider FOV so you see surroundings/signals better (was 90)

# (kept for reference, not used directly anymore)
TPP_BACK = 7.0
TPP_UP = 2.8
TPP_PITCH_DEG = -12.0

# Spawn
SPAWN_TRIES = 200
PREFER_NON_JUNCTION = True
MIN_CLEAR_RADIUS_M = 4.0   # require no other vehicle within this radius

# Logging
LOG_EVERY_N_TICKS = 200

# ------------------------
# Supervisor (TM deadlock resolver)
# ------------------------
SPEED_STUCK_EPS_MPS = 0.15
STUCK_SECONDS = 6.0

LEAD_DIST_TRIGGER_M = 10.0
OBST_SENSOR_DISTANCE_M = 25.0
OBST_SENSOR_HIT_RADIUS_M = 0.8

FORCE_LC_COOLDOWN_S = 8.0
FORCE_LC_HOLD_S = 2.0

AP_RESET_COOLDOWN_S = 10.0
NUDGE_TICKS = 12           # ~0.6s at 20Hz
NUDGE_THROTTLE = 0.30


# ------------------------
# Utils
# ------------------------
def ensure_dir(p):
    os.makedirs(p, exist_ok=True)
    return p

def now_tag():
    return datetime.now().strftime("%Y%m%d_%H%M%S")

def kmh_from_mps(v):
    return float(v) * 3.6

def get_speed_mps(vehicle):
    v = vehicle.get_velocity()
    return math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)

def set_sync(world, enable=True, fixed_delta=0.05):
    s = world.get_settings()
    s.synchronous_mode = bool(enable)
    s.fixed_delta_seconds = float(fixed_delta) if enable else None
    world.apply_settings(s)

def tm_set_if_exists(tm, name, *args):
    fn = getattr(tm, name, None)
    if callable(fn):
        try:
            fn(*args)
            return True
        except Exception:
            return False
    return False

def destroy_actors(actors):
    for a in actors:
        try:
            if a is not None:
                a.destroy()
        except Exception:
            pass

def nuke_world(world):
    # Destroys all vehicles + all sensors in the world (good for clean benchmarking)
    try:
        vehicles = world.get_actors().filter("vehicle.*")
        sensors = world.get_actors().filter("sensor.*")
        for a in list(sensors):
            try: a.destroy()
            except Exception: pass
        for a in list(vehicles):
            try: a.destroy()
            except Exception: pass
    except Exception:
        pass

def get_light_state(vehicle):
    try:
        return vehicle.get_traffic_light_state()
    except Exception:
        pass
    try:
        tl = vehicle.get_traffic_light()
        return None if tl is None else tl.state
    except Exception:
        return None

class CollisionCounter:
    def __init__(self):
        self.count = 0
    def callback(self, _event):
        self.count += 1

class ObstacleProbe:
    def __init__(self):
        self.distance = None
    def callback(self, event):
        try:
            self.distance = float(event.distance)
        except Exception:
            self.distance = None

class VideoRecorder:
    def __init__(self, out_path, w, h, fps, fourcc="MJPG"):
        if cv2 is None or np is None:
            raise RuntimeError("Install numpy + opencv-python to save video.")
        self.out_path = out_path
        self.w, self.h, self.fps = int(w), int(h), int(fps)
        four = cv2.VideoWriter_fourcc(*fourcc)
        self.writer = cv2.VideoWriter(out_path, four, self.fps, (self.w, self.h))
        if not self.writer.isOpened():
            raise RuntimeError(f"VideoWriter failed: {out_path}")

    def push_carla_image(self, image):
        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
        bgr = arr[:, :, :3]
        if (image.width != self.w) or (image.height != self.h):
            bgr = cv2.resize(bgr, (self.w, self.h), interpolation=cv2.INTER_AREA)
        self.writer.write(bgr)

    def close(self):
        try:
            self.writer.release()
        except Exception:
            pass

# ------------------------
# CAMERA PRESETS (NEW)
# ------------------------
def camera_rel_transform(view_name: str) -> carla.Transform:
    """
    Returns a relative camera transform to attach to ego.
    All presets are chase/drone style to see roads, signals, surroundings.
    """
    presets = {
        # closer chase
        "tpp_close": carla.Transform(
            carla.Location(x=-7.0, y=0.0, z=2.8),
            carla.Rotation(pitch=-12.0, yaw=0.0, roll=0.0)
        ),
        # medium chase
        "tpp_mid": carla.Transform(
            carla.Location(x=-10.0, y=0.0, z=4.5),
            carla.Rotation(pitch=-18.0, yaw=0.0, roll=0.0)
        ),
        # wide “drone” view (recommended)
        "drone_wide": carla.Transform(
            carla.Location(x=-14.0, y=0.0, z=10.5),
            carla.Rotation(pitch=-32.0, yaw=0.0, roll=0.0)
        ),
        # even wider / higher for intersections
        "drone_ultrawide": carla.Transform(
            carla.Location(x=-18.0, y=0.0, z=14.0),
            carla.Rotation(pitch=-38.0, yaw=0.0, roll=0.0)
        ),
    }
    return presets.get(view_name, presets["drone_wide"])


def pick_spawn_points(world_map):
    spawns = world_map.get_spawn_points()
    preferred, fallback = [], []
    for t in spawns:
        wp = None
        try:
            wp = world_map.get_waypoint(t.location, project_to_road=True, lane_type=carla.LaneType.Driving)
        except Exception:
            try:
                wp = world_map.get_waypoint(t.location)
            except Exception:
                wp = None
        if wp is None:
            continue
        fallback.append(t)
        if PREFER_NON_JUNCTION and getattr(wp, "is_junction", False):
            continue
        preferred.append(t)
    random.shuffle(preferred)
    random.shuffle(fallback)
    print(f"[spawn] candidates preferred(non-junction)={len(preferred)}  fallback(all)={len(fallback)}")
    return preferred, fallback

def is_spawn_clear(world, loc, radius_m=4.0):
    try:
        vehicles = world.get_actors().filter("vehicle.*")
        for v in vehicles:
            if v.get_location().distance(loc) < radius_m:
                return False
        return True
    except Exception:
        return True

def get_blueprint_candidates(bp_lib):
    # Use find() (exact) and fallback options; never rely on filter("vehicle.tesla.model3")
    names = [
        "vehicle.tesla.model3",
        "vehicle.lincoln.mkz_2017",
        "vehicle.audi.tt",
        "vehicle.toyota.prius",
        "vehicle.volkswagen.t2",
    ]
    bps = []
    for n in names:
        try:
            bps.append(bp_lib.find(n))
        except Exception:
            pass
    # If none found, last-resort: any vehicle.*
    if not bps:
        try:
            bps = list(bp_lib.filter("vehicle.*"))
        except Exception:
            bps = []
    return bps

def try_spawn_ego(world, world_map, bp_lib):
    preferred, fallback = pick_spawn_points(world_map)
    spawn_list = preferred + fallback
    bps = get_blueprint_candidates(bp_lib)

    if not spawn_list or not bps:
        return None

    for i in range(min(SPAWN_TRIES, len(spawn_list))):
        t = spawn_list[i]
        # require a clear area
        if not is_spawn_clear(world, t.location, MIN_CLEAR_RADIUS_M):
            continue

        bp = random.choice(bps)
        try:
            if bp.has_attribute("role_name"):
                bp.set_attribute("role_name", "ego")
        except Exception:
            pass

        # slight lift to avoid ground contact edge cases
        t2 = carla.Transform(
            carla.Location(x=t.location.x, y=t.location.y, z=t.location.z + 0.3),
            t.rotation
        )

        ego = world.try_spawn_actor(bp, t2)
        if ego is None:
            continue

        # verify it is on drivable lane
        try:
            wp = world_map.get_waypoint(ego.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
        except Exception:
            try:
                wp = world_map.get_waypoint(ego.get_location())
            except Exception:
                wp = None

        if wp is None:
            try: ego.destroy()
            except Exception: pass
            continue

        return ego

    return None


def run_one(client, traffic_count, target_speed_kmh):
    run_id = f"{now_tag()}_tm_t{int(target_speed_kmh)}_cars{int(traffic_count)}"
    run_dir = ensure_dir(os.path.join("runs", run_id))
    print(f"Run folder: {run_dir}")

    cfg = {
        "mode": "tm_autopilot_star_v3",
        "map": f"Carla/Maps/{MAP_NAME}",
        "duration_s": DURATION_S,
        "fixed_delta_seconds": FIXED_DELTA_SECONDS,
        "target_speed_kmh": float(target_speed_kmh),
        "traffic_count": int(traffic_count),
        "tm_port": int(TM_PORT),
        "video_view": CAM_VIEW,                  # UPDATED
        "video_res": [VIDEO_W, VIDEO_H],
        "video_fps": int(VIDEO_FPS),
        "video_fov": int(CAM_FOV),               # UPDATED
        "video_file": f"video.{VIDEO_EXT}",
    }
    with open(os.path.join(run_dir, "config.json"), "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)

    world = client.get_world()
    world_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    original_settings = world.get_settings()

    actors = []
    sensors = []
    recorder = None
    video_file = None

    # TM
    tm = client.get_trafficmanager(TM_PORT)
    tm_set_if_exists(tm, "set_synchronous_mode", True)

    try:
        if NUKE_EXISTING_ACTORS:
            nuke_world(world)
            time.sleep(0.2)

        set_sync(world, True, FIXED_DELTA_SECONDS)

        # advance a few ticks so server is stable
        for _ in range(5):
            world.tick()

        # ego
        ego = try_spawn_ego(world, world_map, bp_lib)
        if ego is None:
            raise RuntimeError("Failed to spawn ego vehicle.")
        actors.append(ego)

        # collision sensor
        col_counter = CollisionCounter()
        col_bp = bp_lib.find("sensor.other.collision")
        col_sensor = world.spawn_actor(col_bp, carla.Transform(), attach_to=ego)
        col_sensor.listen(lambda e: col_counter.callback(e))
        sensors.append(col_sensor)

        # obstacle sensor
        obst_probe = ObstacleProbe()
        obst_bp = bp_lib.find("sensor.other.obstacle")
        if obst_bp.has_attribute("distance"):
            obst_bp.set_attribute("distance", str(OBST_SENSOR_DISTANCE_M))
        if obst_bp.has_attribute("hit_radius"):
            obst_bp.set_attribute("hit_radius", str(OBST_SENSOR_HIT_RADIUS_M))
        if obst_bp.has_attribute("only_dynamics"):
            obst_bp.set_attribute("only_dynamics", "true")
        obst_tf = carla.Transform(carla.Location(x=2.5, z=1.1))
        obst_sensor = world.spawn_actor(obst_bp, obst_tf, attach_to=ego)
        obst_sensor.listen(lambda e: obst_probe.callback(e))
        sensors.append(obst_sensor)

        # video (UPDATED: uses CAM_VIEW preset)
        if SAVE_VIDEO and cv2 is not None and np is not None:
            cam_bp = bp_lib.find("sensor.camera.rgb")
            cam_bp.set_attribute("image_size_x", str(VIDEO_W))
            cam_bp.set_attribute("image_size_y", str(VIDEO_H))
            cam_bp.set_attribute("fov", str(CAM_FOV))
            cam_tf = camera_rel_transform(CAM_VIEW)  # UPDATED
            cam = world.spawn_actor(
                cam_bp, cam_tf, attach_to=ego,
                attachment_type=carla.AttachmentType.SpringArmGhost
            )
            sensors.append(cam)
            video_file = os.path.join(run_dir, f"video.{VIDEO_EXT}")
            recorder = VideoRecorder(video_file, VIDEO_W, VIDEO_H, VIDEO_FPS, fourcc=VIDEO_FOURCC)
            cam.listen(lambda img: recorder.push_carla_image(img))

        # spawn traffic
        traffic_vehicles = []
        if traffic_count > 0:
            spawn_points = world_map.get_spawn_points()
            random.shuffle(spawn_points)
            for t in spawn_points:
                if len(traffic_vehicles) >= traffic_count:
                    break
                if not is_spawn_clear(world, t.location, 2.0):
                    continue
                # skip too-close to ego
                if t.location.distance(ego.get_location()) < 15.0:
                    continue
                v_bp = random.choice(list(bp_lib.filter("vehicle.*")))
                try:
                    if v_bp.has_attribute("role_name"):
                        v_bp.set_attribute("role_name", "traffic")
                except Exception:
                    pass
                v = world.try_spawn_actor(v_bp, t)
                if v:
                    traffic_vehicles.append(v)

            for v in traffic_vehicles:
                v.set_autopilot(True, tm.get_port())

            actors.extend(traffic_vehicles)

        # ego autopilot
        ego.set_autopilot(True, tm.get_port())
        tm_set_if_exists(tm, "auto_lane_change", ego, True)

        # apply target speed via speed limit
        def apply_target_speed():
            try:
                limit = float(ego.get_speed_limit())
            except Exception:
                limit = 50.0
            if limit < 5.0:
                limit = 50.0
            perc_diff = (1.0 - (float(target_speed_kmh) / limit)) * 100.0
            perc_diff = max(-80.0, min(80.0, perc_diff))
            tm_set_if_exists(tm, "vehicle_percentage_speed_difference", ego, float(perc_diff))
            return limit, perc_diff

        limit_kmh, perc_diff = apply_target_speed()

        # settle
        for _ in range(10):
            world.tick()

        print(
            f"Driving ✅ (TM autopilot STAR v3) target={target_speed_kmh} km/h  traffic={traffic_count} "
            f"(speed_limit≈{limit_kmh:.0f}, diff={perc_diff:+.1f}%)  Ctrl+C to stop"
        )

        # loop
        start_wall = time.time()
        ticks = 0
        sim_seconds = 0.0
        prev_loc = ego.get_location()
        dist_m = 0.0

        moving_kmh_sum = 0.0
        moving_n = 0

        stuck_start = None
        next_force_lc_time = 0.0
        next_ap_reset_time = 0.0
        force_lc_events = 0
        ap_resets = 0

        while sim_seconds < DURATION_S:
            t0 = time.time()
            world.tick()
            ticks += 1
            sim_seconds = ticks * FIXED_DELTA_SECONDS

            loc = ego.get_location()
            dist_m += loc.distance(prev_loc)
            prev_loc = loc

            spd_mps = get_speed_mps(ego)
            spd_kmh = kmh_from_mps(spd_mps)

            if spd_mps > 0.5:
                moving_kmh_sum += spd_kmh
                moving_n += 1

            # stuck detection
            now = time.time()
            if spd_mps < SPEED_STUCK_EPS_MPS:
                if stuck_start is None:
                    stuck_start = now
            else:
                stuck_start = None

            lead = obst_probe.distance
            light_state = get_light_state(ego)
            is_greenish = (light_state is None) or (light_state == carla.TrafficLightState.Green)

            if stuck_start is not None and (now - stuck_start) > STUCK_SECONDS and is_greenish:
                # A) stuck behind car -> force lane change
                if (lead is not None) and (lead < LEAD_DIST_TRIGGER_M) and (now >= next_force_lc_time):
                    force_lc_events += 1
                    next_force_lc_time = now + FORCE_LC_COOLDOWN_S
                    ok_left = tm_set_if_exists(tm, "force_lane_change", ego, True)
                    ok_right = False if ok_left else tm_set_if_exists(tm, "force_lane_change", ego, False)
                    print(f"🟦 stuck behind lead({lead:.1f}m)+green -> force lane change #{force_lc_events} "
                          f"(left={ok_left}, right={ok_right})")
                    stuck_start = None

                # B) stuck but no close lead -> autopilot reset + nudge
                if ((lead is None) or (lead >= LEAD_DIST_TRIGGER_M)) and (now >= next_ap_reset_time):
                    ap_resets += 1
                    next_ap_reset_time = now + AP_RESET_COOLDOWN_S
                    print(f"🟨 stuck+no close lead (lead={lead})+green -> autopilot reset #{ap_resets}")
                    ego.set_autopilot(False)
                    for _ in range(NUDGE_TICKS):
                        ego.apply_control(carla.VehicleControl(throttle=NUDGE_THROTTLE, steer=0.0, brake=0.0))
                        world.tick()
                        ticks += 1
                        sim_seconds = ticks * FIXED_DELTA_SECONDS
                    ego.set_autopilot(True, tm.get_port())
                    apply_target_speed()
                    tm_set_if_exists(tm, "auto_lane_change", ego, True)
                    stuck_start = None

            if (ticks % LOG_EVERY_N_TICKS) == 0:
                print(f"step={ticks:5d} spd={spd_kmh:5.1f}km/h lead={lead if lead is None else round(lead,2)} "
                      f"col={col_counter.count:3d} dodge={force_lc_events} reset={ap_resets}")

            if REALTIME_PACING:
                dt = time.time() - t0
                sleep_s = FIXED_DELTA_SECONDS - dt
                if sleep_s > 0:
                    time.sleep(sleep_s)

        wall_time = time.time() - start_wall
        avg_mps = dist_m / max(1e-6, sim_seconds)
        avg_kmh = kmh_from_mps(avg_mps)
        avg_kmh_moving = (moving_kmh_sum / moving_n) if moving_n > 0 else 0.0

        summary = {
            "mode": "tm_autopilot_star_v3",
            "map": f"Carla/Maps/{MAP_NAME}",
            "duration_s": float(sim_seconds),
            "wall_time_s": float(wall_time),
            "distance_m": float(dist_m),
            "avg_speed_kmh": float(avg_kmh),
            "avg_speed_kmh_moving_only": float(avg_kmh_moving),
            "target_speed_kmh": float(target_speed_kmh),
            "traffic_count": int(traffic_count),
            "collisions": int(col_counter.count),
            "force_lane_change_events": int(force_lc_events),
            "autopilot_resets": int(ap_resets),
            "video_view": CAM_VIEW,
            "video_fov": int(CAM_FOV),
            "video_res": [VIDEO_W, VIDEO_H],
            "video_fps": int(VIDEO_FPS),
            "video_file": (os.path.basename(video_file) if video_file else None),
            "tm_port": int(TM_PORT),
        }

        with open(os.path.join(run_dir, "summary.json"), "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)

        print("\n✅ Run summary:", summary)
        print("Saved:", os.path.join(run_dir, "summary.json"))
        return summary

    except KeyboardInterrupt:
        print("\nStopped by user.")
        raise
    finally:
        # ALWAYS cleanup + restore, even if ego spawn failed
        try:
            if recorder is not None:
                recorder.close()
        except Exception:
            pass

        for s in sensors:
            try: s.stop()
            except Exception: pass

        destroy_actors(sensors)
        destroy_actors(actors)

        try:
            world.apply_settings(original_settings)
        except Exception:
            pass


def main():
    print("✅ script started")
    random.seed(7)

    ensure_dir("runs")

    client = carla.Client(HOST, PORT)
    client.set_timeout(20.0)

    print(f"Connecting to CARLA {HOST}:{PORT} ...")
    world = client.get_world()
    print(f"✅ Connected. Current map: {world.get_map().name}")

    all_summaries = []
    total_runs = len(TRAFFIC_GRID) * len(SPEED_GRID_KMH)
    sweep_out = os.path.join("runs", f"sweep_{now_tag()}.json")

    run_idx = 0
    for traffic in TRAFFIC_GRID:
        for spd in SPEED_GRID_KMH:
            run_idx += 1
            print(f"\n=== RUN {run_idx}/{total_runs}: traffic={traffic} speed={spd} km/h mode=tm_autopilot ===")
            try:
                all_summaries.append(run_one(client, traffic, spd))
            except KeyboardInterrupt:
                partial = os.path.join("runs", f"sweep_{now_tag()}_partial.json")
                with open(partial, "w", encoding="utf-8") as f:
                    json.dump(all_summaries, f, indent=2)
                print("Saved partial sweep:", partial)
                return
            except Exception as e:
                print("❌ crashed:", repr(e))
                crash = {"traffic": traffic, "target_speed_kmh": spd, "error": repr(e), "time": now_tag()}
                crash_file = os.path.join("runs", f"crash_{now_tag()}_t{spd}_cars{traffic}.json")
                with open(crash_file, "w", encoding="utf-8") as f:
                    json.dump(crash, f, indent=2)
                print("Saved crash log:", crash_file)
                time.sleep(1.0)

    with open(sweep_out, "w", encoding="utf-8") as f:
        json.dump(all_summaries, f, indent=2)

    print("\n✅ Sweep done. Saved:", sweep_out)


if __name__ == "__main__":
    main()
