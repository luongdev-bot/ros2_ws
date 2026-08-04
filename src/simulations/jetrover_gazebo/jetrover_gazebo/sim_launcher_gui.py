#!/usr/bin/env python3
"""GUI launcher that keeps a SLAM map tied to the world it was mapped in.

    ros2 run jetrover_gazebo sim_launcher_gui
    # or scripts/run_sim_gui.sh, which sources the workspace and checks the build

The problem this solves: a saved map is only meaningful in the world it came
from, but nothing in a .yaml/.pgm pair records that. Load the hospital map into
the warehouse and AMCL will happily localise onto garbage. So every map saved
here gets a sidecar MAP_META ("slam_map.json") naming its world, and the Nav2
tab launches the world named there rather than asking the user to remember.

Layout of the map store ($JETROVER_MAP_DIR, default ~/ros2_ws/maps):

    maps/
      hospital_01/
        map.yaml          <- nav2 map_server input
        map.pgm
        slam_map.json     <- {world_id, world_file, world_sha256, spawn, ...}
"""
import datetime
import hashlib
import json
import os
import pathlib
import queue
import shlex
import shutil
import signal
import subprocess
import threading
import time
import tkinter as tk
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from tkinter import messagebox, ttk

WS = pathlib.Path(os.environ.get("JETROVER_WS", pathlib.Path.home() / "ros2_ws"))
MAP_DIR = pathlib.Path(os.environ.get("JETROVER_MAP_DIR", WS / "maps"))
MAP_META = "slam_map.json"
# Records which world the running SLAM session was started in, so "save map"
# knows the answer. Cleared by stop_all(): a session that has been torn down
# must not go on labelling maps, or a later run in another world inherits it.
#
# It deliberately SURVIVES closing the GUI, so you can restart the launcher
# while SLAM keeps running in its own terminal and still save the map. The cost
# is that the world binding is an operator convention, not a guarantee: if you
# start SLAM in another world from a terminal instead of this GUI, or leave a
# Nav2 map_server publishing /map, "save map" will still label whatever it finds
# with this marker's world. Start SLAM from tab 1 and the binding holds.
SESSION_FILE = MAP_DIR / ".active_session.json"


def sh(value) -> str:
    """Quote a value for safe interpolation into a `bash -c` string.

    Every dynamic token below goes through this. Map names are user input and
    reach a shell; without quoting, a name like `x'; rm -rf ~` would run.
    """
    return shlex.quote(str(value))


# Sourcing happens inside each spawned shell, never here: ROS setup files
# reference unset variables and would trip `set -u` in the parent. Same rule the
# other launcher scripts follow.
ENV_SETUP = (
    "source /opt/ros/humble/setup.bash; "
    f"source {sh(WS / 'install/setup.bash')}; "
    "export MACHINE_TYPE=JetRover_Mecanum; export LIDAR_TYPE=A1"
)
WAIT_SIM = ("echo 'Waiting for the simulation (/scan)...'; "
            "until ros2 topic list 2>/dev/null | grep -qx /scan; do sleep 1; done; sleep 2")
WAIT_NAV = ("echo 'Waiting for Nav2 (/bt_navigator)...'; "
            "until ros2 node list 2>/dev/null | grep -qx /bt_navigator; "
            "do sleep 2; done; sleep 1")
WAIT_CTRL = ("echo 'Waiting for the controller_manager...'; "
             "until ros2 node list 2>/dev/null | grep -qx /controller_manager; "
             "do sleep 2; done; echo 'Waiting for arm_controller to become active...'; "
             "until timeout 5 ros2 control list_controllers 2>/dev/null | "
             "grep -q 'arm_controller.*active'; do sleep 2; done; sleep 1")
WAIT_VOICE_DEPS = ("echo 'Waiting for Nav2 (/bt_navigator)...'; "
                   "until ros2 node list 2>/dev/null | grep -qx /bt_navigator; "
                   "do sleep 2; done; echo 'Waiting for grasp_executor...'; "
                   "until ros2 node list 2>/dev/null | grep -qx /grasp_executor; "
                   "do sleep 2; done; sleep 1")
WAIT_GRASP_EXECUTOR = ("echo 'Waiting for grasp_executor...'; "
                       "until ros2 node list 2>/dev/null | grep -qx /grasp_executor; "
                       "do sleep 2; done; sleep 1")

VOICE_SCENARIOS = (
    ("Function calling (color_sort, không cần map)", "function_calling"),
    ("Navigation transport", "navigation_transport"),
    ("Transport & delivery", "transport_delivery"),
    ("Road network", "road_network"),
)


def share_dir() -> pathlib.Path:
    """Locate the installed jetrover_gazebo share directory."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return pathlib.Path(get_package_share_directory("jetrover_gazebo"))
    except Exception:
        # ament_index needs the workspace sourced. Fall back to the install tree
        # so the GUI still reports something useful instead of an import trace.
        return WS / "install/jetrover_gazebo/share/jetrover_gazebo"


def voice_scenarios_share_dir() -> pathlib.Path:
    """Locate the installed voice_llm_scenarios share directory."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return pathlib.Path(get_package_share_directory("voice_llm_scenarios"))
    except Exception:
        return WS / "install/voice_llm_scenarios/share/voice_llm_scenarios"


def arm_pose_file(slam_mode: str) -> pathlib.Path:
    """Which taught arm pose to boot the robot in.

    2D SLAM and Nav2 only use the lidar, so the arm just has to stay out of the
    scan plane - the folded "home" pose does that (lowest link 0.193 m vs the
    0.157 m lidar plane) and is the compact shape you want while driving.

    3D SLAM maps with the depth camera, and "home" aims it 30.7 deg down - at
    the floor a metre ahead rather than at the room. The "horizontal" pose sits
    at 4.6 deg, which is why it stays the 3D default. Both files document their
    forward kinematics in full.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        config = pathlib.Path(get_package_share_directory("jetrover_moveit_config")) / "config"
    except Exception:
        config = WS / "install/jetrover_moveit_config/share/jetrover_moveit_config/config"
    return config / ("slam_initial_positions.yaml" if slam_mode == "3d"
                     else "home_initial_positions.yaml")


def world_digest(path: pathlib.Path) -> str:
    """SHA-256 of a world file, or "" if it cannot be read.

    Stored alongside each map because worlds/*.sdf are regenerated by
    tools/generate_worlds.py: the same filename can come to mean different
    geometry, which would silently invalidate every map made against it.
    """
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return ""


def world_name_from_sdf(path: pathlib.Path) -> str:
    """Return the Gazebo world name declared inside an SDF file.

    The filename is not authoritative: ``color_sort_world.sdf`` deliberately
    contains a world named ``color_blocks_world``.  Gazebo's ``/world/<name>``
    services use the XML name, while the launch argument ``world`` is a file
    path, so keep both values distinct.
    """
    try:
        root = ET.parse(path).getroot()
        world = root.find("world")
        name = world.get("name") if world is not None else None
        if name:
            return name
    except (OSError, ET.ParseError):
        pass
    return path.stem


def load_catalog(worlds_dir: pathlib.Path) -> list:
    """Every worlds/*.sdf, annotated from config/world_catalog.yaml."""
    entries = {}
    catalog_path = share_dir() / "config/world_catalog.yaml"
    if catalog_path.is_file():
        try:
            import yaml
            data = yaml.safe_load(catalog_path.read_text()) or {}
            for item in data.get("worlds", []):
                entries[item["file"]] = item
        except Exception as exc:                                   # noqa: BLE001
            print(f"warning: could not read {catalog_path}: {exc}")

    worlds = []
    for sdf in sorted(worlds_dir.glob("*.sdf")):
        meta = entries.get(sdf.name, {})
        spawn = meta.get("spawn") or {}
        worlds.append({
            "id": meta.get("id", sdf.stem),
            "file": sdf.name,
            "path": sdf,
            "world_name": world_name_from_sdf(sdf),
            # A world with no catalogue entry is still usable - it just shows up
            # under its filename with no blurb.
            "label": meta.get("label", sdf.stem),
            "description": meta.get("description", "(chưa có mô tả trong world_catalog.yaml)"),
            "spawn": {k: float(spawn.get(k, 0.0)) for k in ("x", "y", "yaw")},
            "slam_friendly": bool(meta.get("slam_friendly", True)),
        })
    return worlds


def load_maps() -> list:
    """Saved maps, newest first. Anything malformed is skipped, not fatal."""
    maps = []
    if not MAP_DIR.is_dir():
        return maps
    for entry in sorted(MAP_DIR.iterdir()):
        # Dot-directories are ours: the .active_session.json marker and the
        # .<name>.saving / .<name>.previous scratch dirs used while committing a
        # save. A staging dir has a map.yaml and a sidecar, so without this it
        # would show up in the list as a real map mid-save.
        if not entry.is_dir() or entry.name.startswith("."):
            continue
        meta_path = entry / MAP_META
        yaml_path = entry / "map.yaml"
        if not (meta_path.is_file() and yaml_path.is_file()):
            continue
        try:
            meta = json.loads(meta_path.read_text())
            if not isinstance(meta, dict):
                raise ValueError("sidecar is not a JSON object")
            world_file = meta.get("world_file")
            # The world file reaches a shell command later, so require a bare
            # filename - never an absolute path or one containing "..".
            if (not isinstance(world_file, str) or ".." in pathlib.PurePath(world_file).parts
                    or world_file != pathlib.Path(world_file).name):
                raise ValueError(f"bad world_file {world_file!r}")
            # Inside the try: a sidecar with spawn.x = "abc" would otherwise
            # raise straight out of here, and this function promises to skip
            # malformed maps rather than die on them.
            # Absent or null spawn means "use the origin"; anything else that is
            # not a dict is a malformed sidecar, not an empty one. `or {}` alone
            # would quietly accept 0, "", [] and false as "no spawn".
            spawn = meta.get("spawn")
            if spawn is None:
                spawn = {}
            if not isinstance(spawn, dict):
                raise ValueError(f"bad spawn {spawn!r}")
            meta["spawn"] = {k: float(spawn.get(k, 0.0)) for k in ("x", "y", "yaw")}
        except (OSError, json.JSONDecodeError, ValueError, TypeError) as exc:
            print(f"warning: skipping map {entry.name}: {exc}")
            continue
        # The directory name is authoritative: someone may have renamed it.
        meta["map_name"] = entry.name
        meta["dir"] = entry
        meta["yaml"] = yaml_path
        maps.append(meta)
    maps.sort(key=lambda m: m.get("created", ""), reverse=True)
    return maps


def open_term(title: str, command: str) -> None:
    """Run a command in its own terminal window, kept open after it exits."""
    subprocess.Popen([
        "gnome-terminal", "--title", title, "--",
        "bash", "-c", f"{command}; echo; echo '--- process exited (press Enter to close) ---'; read _",
    ])


class App(ttk.Frame):
    def __init__(self, master):
        super().__init__(master, padding=10)
        self.grid(sticky="nsew")
        master.columnconfigure(0, weight=1)
        master.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        self.worlds = load_catalog(share_dir() / "worlds")
        self.status = tk.StringVar(value="Sẵn sàng.")
        # Worker threads must not touch Tk at all. They post results here and the
        # main thread drains the queue from an after() tick.
        self.results = queue.Queue()

        notebook = ttk.Notebook(self)
        notebook.grid(row=0, column=0, sticky="nsew")
        self._build_slam_tab(notebook)
        self._build_nav_tab(notebook)
        self._build_voice_tab(notebook)

        bar = ttk.Frame(self)
        bar.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        bar.columnconfigure(0, weight=1)
        ttk.Label(bar, textvariable=self.status, foreground="#555").grid(row=0, column=0, sticky="w")
        ttk.Button(bar, text="Dừng tất cả", command=self.stop_all).grid(row=0, column=1)

        self.after(150, self._drain_results)

    def _drain_results(self):
        try:
            while True:
                callback, args = self.results.get_nowait()
                callback(*args)
        except queue.Empty:
            pass
        self.after(150, self._drain_results)

    # ------------------------------------------------------------------ SLAM
    def _build_slam_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=10)
        notebook.add(tab, text="  1. Tạo map (SLAM)  ")
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(1, weight=1)

        ttk.Label(tab, text="Chọn world để chạy SLAM:",
                  font=("", 10, "bold")).grid(row=0, column=0, sticky="w")

        self.world_tree = ttk.Treeview(tab, columns=("label", "description"),
                                       show="headings", height=9)
        self.world_tree.heading("label", text="World")
        self.world_tree.heading("description", text="Mô tả")
        self.world_tree.column("label", width=240, stretch=False)
        self.world_tree.column("description", width=520)
        self.world_tree.grid(row=1, column=0, sticky="nsew", pady=(4, 8))

        for world in self.worlds:
            # Worlds with no walls map to an empty grid, so flag them rather
            # than letting someone waste a run finding out.
            suffix = "" if world["slam_friendly"] else "   [không hợp SLAM]"
            self.world_tree.insert("", "end", iid=world["file"],
                                   values=(world["label"] + suffix, world["description"]))
        if self.worlds:
            first = next((w["file"] for w in self.worlds if w["slam_friendly"]),
                         self.worlds[0]["file"])
            self.world_tree.selection_set(first)

        row = ttk.Frame(tab)
        row.grid(row=2, column=0, sticky="ew")
        ttk.Label(row, text="Chế độ:").pack(side="left")
        self.slam_mode = tk.StringVar(value="2d")
        ttk.Radiobutton(row, text="2D (slam_toolbox)", value="2d",
                        variable=self.slam_mode).pack(side="left", padx=(6, 12))
        ttk.Radiobutton(row, text="3D (RTAB-Map)", value="3d",
                        variable=self.slam_mode).pack(side="left")
        ttk.Button(row, text="Khởi động SLAM", command=self.start_slam).pack(side="right")

        ttk.Separator(tab, orient="horizontal").grid(row=3, column=0, sticky="ew", pady=12)

        ttk.Label(tab, text="Sau khi chạy quanh bản đồ xong, lưu lại:",
                  font=("", 10, "bold")).grid(row=4, column=0, sticky="w")
        save = ttk.Frame(tab)
        save.grid(row=5, column=0, sticky="ew", pady=(4, 0))
        ttk.Label(save, text="Tên map:").pack(side="left")
        self.map_name = tk.StringVar()
        ttk.Entry(save, textvariable=self.map_name, width=28).pack(side="left", padx=6)
        self.save_button = ttk.Button(save, text="Lưu map", command=self.save_map)
        self.save_button.pack(side="left")
        ttk.Label(save, text="(map sẽ được gắn với world đang chạy)",
                  foreground="#777").pack(side="left", padx=8)

    def _selected_world(self):
        selection = self.world_tree.selection()
        if not selection:
            return None
        return next((w for w in self.worlds if w["file"] == selection[0]), None)

    def start_slam(self):
        world = self._selected_world()
        if world is None:
            messagebox.showwarning("Chưa chọn world", "Hãy chọn một world trong danh sách.")
            return
        if not world["slam_friendly"] and not messagebox.askyesno(
                "World không hợp SLAM",
                f"{world['label']} không có tường bao, SLAM sẽ ra map trống.\n\nVẫn chạy?"):
            return

        mode = self.slam_mode.get()
        spawn = world["spawn"]

        if mode == "2d":
            slam = "ros2 launch slam slam.launch.py"
            title = "SLAM 2D (slam_toolbox)"
            prep = ""
        else:
            slam = "ros2 launch slam rtabmap_slam.launch.py use_gpu:=true"
            title = "SLAM 3D (RTAB-Map, CUDA)"
            prep = f"bash {sh(WS / 'scripts/pose_arm_camera.sh')} &&"

        # Both modes use gazebo_arm.launch.py: gazebo.launch.py has no
        # gz_ros2_control, so the arm joints are unactuated and the arm collapses
        # under gravity. 2D SLAM ignores the camera but still wants a robot that
        # looks like the robot. Same reasoning as scripts/run_slam.sh.
        sim = (f"ros2 launch jetrover_gazebo gazebo_arm.launch.py "
               f"world:={sh(share_dir() / 'worlds' / world['file'])} "
               f"world_name:={sh(world['world_name'])} "
               f"spawn_x:={sh(spawn['x'])} spawn_y:={sh(spawn['y'])} "
               f"spawn_yaw:={sh(spawn['yaw'])} "
               f"initial_positions_file:={sh(arm_pose_file(mode))}")

        # A leftover Gazebo publishes a second /clock and /tf, which makes TF
        # jump backwards and breaks both RTAB-Map and RViz. This also clears any
        # previous session marker.
        if not self.stop_all(quiet=True):
            messagebox.showerror(
                "Chưa dừng được phiên cũ",
                "Vẫn còn tiến trình SLAM/Gazebo/RViz đang chạy.\n\n"
                "Đã huỷ khởi động để tránh nhiều node cùng publish /map.\n"
                "Hãy đóng các terminal cũ rồi bấm lại.")
            return

        try:
            open_term("Gazebo", f"{ENV_SETUP}; {sim}")
            open_term(title, f"{ENV_SETUP}; {WAIT_SIM}; {prep} {slam}")
            open_term("Teleop (w/a/s/d)",
                      f"{ENV_SETUP}; {WAIT_SIM}; ros2 run peripherals teleop_key_control")
        except OSError as exc:
            messagebox.showerror("Không mở được terminal", str(exc))
            self.status.set("Khởi động SLAM thất bại.")
            return

        # Written only after the terminals actually started: a session marker for
        # a launch that never happened would mislabel the next map saved.
        self._write_session(world, mode)
        self.status.set(f"Đã mở 3 terminal cho '{world['label']}' ({mode.upper()}). "
                        "Lái robot bằng cửa sổ Teleop.")

    def _write_session(self, world, mode):
        MAP_DIR.mkdir(parents=True, exist_ok=True)
        SESSION_FILE.write_text(json.dumps({
            "world_id": world["id"],
            "world_file": world["file"],
            "world_label": world["label"],
            "world_sha256": world_digest(world["path"]),
            "spawn": world["spawn"],
            "slam_mode": mode,
            "started": datetime.datetime.now().isoformat(timespec="seconds"),
        }, indent=2, ensure_ascii=False))

    def _read_session(self):
        try:
            session = json.loads(SESSION_FILE.read_text())
            if not isinstance(session, dict) or not isinstance(session.get("world_file"), str):
                raise ValueError("session marker missing world_file")
            return session
        except (OSError, json.JSONDecodeError, ValueError):
            return None

    def save_map(self):
        name = self.map_name.get().strip()
        if not name:
            messagebox.showwarning("Thiếu tên", "Hãy nhập tên cho map.")
            return
        # The name becomes a directory: keep it a safe single path component.
        if name != pathlib.Path(name).name or name.startswith(".") or "/" in name:
            messagebox.showwarning("Tên không hợp lệ",
                                   "Tên map không được chứa '/' hoặc bắt đầu bằng '.'.")
            return

        session = self._read_session()
        if session is None:
            messagebox.showerror(
                "Chưa có phiên SLAM",
                "Không biết map này thuộc world nào.\n\n"
                "Hãy bấm 'Khởi động SLAM' từ cửa sổ này trước khi lưu map.\n"
                "(Nếu bạn vừa bấm 'Dừng tất cả', phiên đã bị xoá - hãy chạy lại.)")
            return

        target = MAP_DIR / name
        if target.exists() and not messagebox.askyesno(
                "Ghi đè?", f"Map '{name}' đã tồn tại. Ghi đè?"):
            return

        self.save_button.state(["disabled"])
        self.status.set(f"Đang lưu map '{name}'...")
        # map_saver_cli blocks until it has a map; run it off the Tk thread so
        # the window keeps repainting instead of looking hung.
        threading.Thread(target=self._save_worker, args=(name, target, session),
                         daemon=True).start()

    def _save_worker(self, name, target, session):
        # Save into a scratch directory and swap it in only on success. Writing
        # straight into `target` means a failed overwrite destroys the map that
        # was already there.
        staging = MAP_DIR / f".{name}.saving"
        backup = MAP_DIR / f".{name}.previous"
        ok, detail = False, ""
        # Assume the backup must be preserved until the swap has demonstrably
        # succeeded; the commit path clears this once the old map is redundant.
        kept_backup = True
        try:
            shutil.rmtree(staging, ignore_errors=True)
            staging.mkdir(parents=True, exist_ok=True)
            # use_sim_time matters: without it map_saver stamps against wall time
            # and can decide the incoming /map is stale, then wait forever.
            cmd = (f"{ENV_SETUP}; ros2 run nav2_map_server map_saver_cli "
                   f"-f {sh(staging / 'map')} --ros-args -p use_sim_time:=true")
            # Own process group: on timeout, killing only the `bash -c` wrapper
            # leaves map_saver_cli running and still writing into staging while
            # we delete it. Kill the whole group instead.
            proc = subprocess.Popen(["bash", "-c", cmd], stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, text=True,
                                    start_new_session=True)
            try:
                out, err = proc.communicate(timeout=180)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.communicate()
                raise
            ok = proc.returncode == 0 and (staging / "map.yaml").is_file()
            detail = (err or out or "").strip()[-800:]

            if ok:
                (staging / MAP_META).write_text(json.dumps({
                    "map_name": name,
                    "world_id": session.get("world_id", ""),
                    "world_file": session["world_file"],
                    "world_label": session.get("world_label", session["world_file"]),
                    "world_sha256": session.get("world_sha256", ""),
                    "spawn": session.get("spawn", {"x": 0.0, "y": 0.0, "yaw": 0.0}),
                    "slam_mode": session.get("slam_mode", "2d"),
                    "created": datetime.datetime.now().isoformat(timespec="seconds"),
                }, indent=2, ensure_ascii=False))
                # Move the old map aside rather than deleting it outright: if
                # the swap below fails we can still put it back, instead of
                # having destroyed a good map for a save that never landed.
                # A leftover .previous means an earlier save died mid-commit, so
                # it is the newest copy of the old map - never clear it blindly.
                if backup.exists() and not target.exists():
                    backup.replace(target)
                shutil.rmtree(backup, ignore_errors=True)

                had_previous = target.exists()
                if had_previous:
                    target.replace(backup)
                try:
                    staging.replace(target)
                except OSError:
                    if had_previous:
                        backup.replace(target)
                    raise
                # Only now is the backup redundant. `finally` must NOT delete it
                # unconditionally: if the restore above had itself failed, the
                # backup would be the sole surviving copy of the old map.
                shutil.rmtree(backup, ignore_errors=True)
                kept_backup = False
        except subprocess.TimeoutExpired:
            detail = "map_saver_cli quá 180 s không xong - SLAM đã publish /map chưa?"
        except Exception as exc:                                   # noqa: BLE001
            # Any failure must still re-enable the button, so catch broadly and
            # report rather than dying silently inside a daemon thread.
            ok, detail = False, f"{type(exc).__name__}: {exc}"
        finally:
            shutil.rmtree(staging, ignore_errors=True)
            if kept_backup and backup.exists():
                # A backup survives here either because the restore after a
                # failed swap also failed, or because an earlier save crashed
                # mid-commit and left one behind. Either way it may be the last
                # copy of the old map, so leave it on disk and say where it is.
                detail = (f"{detail}\n\nMap cũ được giữ tại: {backup}").strip()
            self.results.put((self._save_done, (name, ok, detail, session)))

    def _save_done(self, name, ok, detail, session):
        self.save_button.state(["!disabled"])
        if ok:
            self.status.set(f"Đã lưu map '{name}' (world: {session.get('world_label')}).")
            self.map_name.set("")
            self.refresh_maps()
            messagebox.showinfo(
                "Đã lưu",
                f"Map '{name}' đã lưu và gắn với world '{session.get('world_label')}'.\n\n"
                "Sang tab '2. Chạy Nav2' để chạy điều hướng trên map này.")
        else:
            self.status.set(f"Lưu map '{name}' thất bại.")
            messagebox.showerror("Lưu thất bại", detail or "map_saver_cli lỗi không rõ.")

    # ------------------------------------------------------------------ Nav2
    def _build_nav_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=10)
        notebook.add(tab, text="  2. Chạy Nav2  ")
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(1, weight=1)

        ttk.Label(tab, text="Chọn map đã lưu - Nav2 sẽ tự mở đúng world của map đó:",
                  font=("", 10, "bold")).grid(row=0, column=0, sticky="w")

        self.map_tree = ttk.Treeview(tab, columns=("name", "world", "mode", "created"),
                                     show="headings", height=11)
        for key, text, width in (("name", "Tên map", 200), ("world", "World", 260),
                                 ("mode", "Chế độ", 80), ("created", "Ngày tạo", 170)):
            self.map_tree.heading(key, text=text)
            self.map_tree.column(key, width=width, stretch=(key == "world"))
        self.map_tree.grid(row=1, column=0, sticky="nsew", pady=(4, 8))

        row = ttk.Frame(tab)
        row.grid(row=2, column=0, sticky="ew")
        ttk.Button(row, text="Làm mới", command=self.refresh_maps).pack(side="left")
        ttk.Button(row, text="Xoá map", command=self.delete_map).pack(side="left", padx=6)
        ttk.Button(row, text="Chạy Nav2 với map này",
                   command=self.start_nav).pack(side="right")

        self.maps = []
        self.refresh_maps()

    def refresh_maps(self):
        self.maps = load_maps()
        self.map_tree.delete(*self.map_tree.get_children())
        for entry in self.maps:
            self.map_tree.insert("", "end", iid=entry["map_name"], values=(
                entry["map_name"],
                entry.get("world_label", entry.get("world_file", "?")),
                str(entry.get("slam_mode", "2d")).upper(),
                entry.get("created", ""),
            ))
        if self.maps:
            self.map_tree.selection_set(self.maps[0]["map_name"])
        if hasattr(self, "voice_map_tree"):
            self._refresh_voice_maps()

    def _selected_map(self):
        selection = self.map_tree.selection()
        if not selection:
            return None
        return next((m for m in self.maps if m["map_name"] == selection[0]), None)

    def start_nav(self):
        entry = self._selected_map()
        if entry is None:
            messagebox.showwarning("Chưa chọn map", "Hãy chọn một map trong danh sách.")
            return

        # load_maps() has already checked world_file is a bare filename; require
        # it to name a world we actually ship before handing it to a shell.
        known = {w["file"]: w for w in self.worlds}
        if entry["world_file"] not in known:
            messagebox.showerror(
                "Thiếu world",
                f"Map này được tạo trong '{entry['world_file']}', nhưng world đó không còn "
                "trong package.\n\nChạy lại: colcon build --packages-select jetrover_gazebo")
            return
        world_path = known[entry["world_file"]]["path"]

        # The generated worlds are rebuilt by tools/generate_worlds.py, so the
        # same filename can come to mean different geometry. Warn rather than
        # block: a small edit far from the mapped area is usually still fine.
        saved_hash = entry.get("world_sha256", "")
        if saved_hash and saved_hash != world_digest(world_path):
            if not messagebox.askyesno(
                    "World đã thay đổi",
                    f"'{entry['world_file']}' đã bị sửa kể từ khi map "
                    f"'{entry['map_name']}' được tạo.\n\n"
                    "Nếu tường/vật cản đã đổi, AMCL có thể định vị sai.\n\nVẫn chạy?"):
                return

        # Respawn at the SAME point the SLAM run started from. This is load
        # bearing, not tidiness: slam_toolbox puts the map frame's origin
        # wherever the robot stood when mapping began, so re-spawning there puts
        # the robot at (0, 0, 0) in map coordinates - which is exactly what
        # nav2_params.yaml's `amcl.initial_pose` already assumes. Spawn anywhere
        # else and AMCL starts convinced it is somewhere it is not.
        spawn = entry["spawn"]
        # Nav2 drives on the lidar, so boot in the folded "home" pose regardless
        # of which SLAM mode produced the map.
        sim = (f"ros2 launch jetrover_gazebo gazebo_arm.launch.py "
               f"world:={sh(world_path)} "
               f"world_name:={sh(known[entry['world_file']]['world_name'])} "
               f"spawn_x:={sh(spawn['x'])} spawn_y:={sh(spawn['y'])} "
               f"spawn_yaw:={sh(spawn['yaw'])} "
               f"initial_positions_file:={sh(arm_pose_file('2d'))}")
        # localization:=true starts map_server + AMCL on the saved map, instead
        # of relying on live SLAM for map->odom (see navigation.launch.py).
        nav = (f"ros2 launch navigation navigation.launch.py use_sim_time:=true "
               f"localization:=true map:={sh(entry['yaml'])} use_rviz:=true")

        if not self.stop_all(quiet=True):
            messagebox.showerror(
                "Chưa dừng được phiên cũ",
                "Vẫn còn tiến trình SLAM/Gazebo/RViz đang chạy.\n\n"
                "Đã huỷ khởi động để tránh xung đột /map giữa SLAM và Nav2.\n"
                "Hãy đóng các terminal cũ rồi bấm lại.")
            return
        try:
            open_term("Gazebo", f"{ENV_SETUP}; {sim}")
            open_term("Nav2 + RViz", f"{ENV_SETUP}; {WAIT_SIM}; {nav}")
        except OSError as exc:
            messagebox.showerror("Không mở được terminal", str(exc))
            return
        self.status.set(
            f"Nav2 đang chạy map '{entry['map_name']}' trong world "
            f"'{entry.get('world_label')}'. Đặt đích bằng nút 'Nav2 Goal' trong RViz.")

    def delete_map(self):
        entry = self._selected_map()
        if entry is None:
            return
        if not messagebox.askyesno("Xoá map", f"Xoá hẳn map '{entry['map_name']}'?"):
            return
        shutil.rmtree(entry["dir"], ignore_errors=True)
        self.refresh_maps()
        self.status.set(f"Đã xoá map '{entry['map_name']}'.")

    # ----------------------------------------------------------- Voice Agent
    def _build_voice_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=10)
        notebook.add(tab, text="  3. Voice Agent  ")
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(4, weight=1)

        ttk.Label(tab, text="Chọn kịch bản LLM giọng nói tiếng Việt:",
                  font=("", 10, "bold")).grid(row=0, column=0, sticky="w")

        self.voice_scenario = tk.StringVar(value=VOICE_SCENARIOS[0][0])
        scenario_box = ttk.Combobox(
            tab, textvariable=self.voice_scenario,
            values=[label for label, _key in VOICE_SCENARIOS],
            state="readonly", width=52,
        )
        scenario_box.grid(row=1, column=0, sticky="w", pady=(4, 12))
        scenario_box.bind("<<ComboboxSelected>>", self._voice_scenario_changed)

        input_mode_row = ttk.Frame(tab)
        self.voice_input_mode = tk.StringVar(value="voice")
        ttk.Label(input_mode_row, text="Cách ra lệnh:",
                  font=("", 10, "bold")).pack(side="left", padx=(0, 12))
        ttk.Radiobutton(
            input_mode_row, text="Giọng nói (mic)", variable=self.voice_input_mode,
            value="voice",
        ).pack(side="left", padx=(0, 12))
        ttk.Radiobutton(
            input_mode_row, text="Gõ chữ (chat)", variable=self.voice_input_mode,
            value="text_chat",
        ).pack(side="left")
        input_mode_row.grid(row=2, column=0, sticky="w", pady=(0, 12))

        ttk.Label(tab, text="Chọn map đã lưu (không dùng cho Function calling):",
                  font=("", 10, "bold")).grid(row=3, column=0, sticky="w")

        style = ttk.Style()
        style.map("VoiceMap.Treeview", foreground=[("disabled", "#999")])
        self.voice_map_tree = ttk.Treeview(
            tab, columns=("name", "world", "mode", "created"),
            show="headings", height=8, style="VoiceMap.Treeview",
        )
        for key, text, width in (("name", "Tên map", 200), ("world", "World", 260),
                                 ("mode", "Chế độ", 80), ("created", "Ngày tạo", 170)):
            self.voice_map_tree.heading(key, text=text)
            self.voice_map_tree.column(key, width=width, stretch=(key == "world"))
        self.voice_map_tree.grid(row=4, column=0, sticky="nsew", pady=(4, 8))

        row = ttk.Frame(tab)
        row.grid(row=5, column=0, sticky="ew")
        ttk.Button(row, text="Làm mới", command=self.refresh_maps).pack(side="left")
        ttk.Button(row, text="Khởi động Voice Agent",
                   command=self.start_voice_agent).pack(side="right")

        self._refresh_voice_maps()
        self._voice_scenario_changed()

    def _voice_scenario_key(self):
        selected = self.voice_scenario.get()
        return next((key for label, key in VOICE_SCENARIOS if label == selected), None)

    def _voice_scenario_changed(self, _event=None):
        needs_map = self._voice_scenario_key() != "function_calling"
        if needs_map:
            self.voice_map_tree.state(["!disabled"])
            if not self.voice_map_tree.selection() and self.maps:
                self.voice_map_tree.selection_set(self.maps[0]["map_name"])
        else:
            self.voice_map_tree.selection_remove(*self.voice_map_tree.selection())
            self.voice_map_tree.state(["disabled"])

    def _refresh_voice_maps(self):
        selected = self.voice_map_tree.selection()
        selected_name = selected[0] if selected else None
        self.voice_map_tree.delete(*self.voice_map_tree.get_children())
        for entry in self.maps:
            self.voice_map_tree.insert("", "end", iid=entry["map_name"], values=(
                entry["map_name"],
                entry.get("world_label", entry.get("world_file", "?")),
                str(entry.get("slam_mode", "2d")).upper(),
                entry.get("created", ""),
            ))
        if self._voice_scenario_key() != "function_calling" and self.maps:
            names = {entry["map_name"] for entry in self.maps}
            self.voice_map_tree.selection_set(
                selected_name if selected_name in names else self.maps[0]["map_name"])

    def _selected_voice_map(self):
        selection = self.voice_map_tree.selection()
        if not selection:
            return None
        return next((m for m in self.maps if m["map_name"] == selection[0]), None)

    @staticmethod
    def _ollama_responding():
        try:
            with urllib.request.urlopen(
                    "http://localhost:11434/api/version", timeout=2) as response:
                response.read(1)
            return True
        except (OSError, TimeoutError, urllib.error.URLError):
            return False

    def _ensure_ollama(self):
        if self._ollama_responding():
            return True

        ollama_bin = pathlib.Path.home() / ".local/bin/ollama"
        if not ollama_bin.is_file() or not os.access(ollama_bin, os.X_OK):
            messagebox.showerror(
                "Thiếu Ollama",
                f"Ollama executable is missing or not executable:\n{ollama_bin}")
            self.status.set("Không thể khởi động Voice Agent: thiếu Ollama.")
            return False

        self.status.set("Ollama chưa phản hồi; đang thử khởi động Ollama serve...")
        self.update_idletasks()
        log_path = pathlib.Path("/tmp/ollama_serve_voice_demo.log")
        try:
            with log_path.open("a") as log_file:
                subprocess.Popen(
                    [str(ollama_bin), "serve"], stdout=log_file,
                    stderr=subprocess.STDOUT, start_new_session=True,
                )
        except OSError as exc:
            messagebox.showerror("Không khởi động được Ollama", str(exc))
            self.status.set("Không thể khởi động Voice Agent: Ollama lỗi.")
            return False

        for _attempt in range(15):
            time.sleep(1)
            if self._ollama_responding():
                return True

        messagebox.showerror(
            "Không khởi động được Ollama",
            "Không khởi động được Ollama. Kiểm tra ~/.local/bin/ollama và log "
            "/tmp/ollama_serve_voice_demo.log")
        self.status.set("Không thể khởi động Voice Agent: Ollama không phản hồi.")
        return False

    @staticmethod
    def _world_name_from_sdf(world_path):
        root = ET.parse(world_path).getroot()
        world = root if root.tag == "world" else root.find("world")
        world_name = world.get("name", "").strip() if world is not None else ""
        if not world_name:
            raise ValueError("không tìm thấy thẻ <world name=...>")
        return world_name

    def start_voice_agent(self):
        scenario = self._voice_scenario_key()
        scenario_label = self.voice_scenario.get()
        input_mode = self.voice_input_mode.get()
        if scenario is None:
            messagebox.showerror("Kịch bản không hợp lệ", "Hãy chọn lại một kịch bản Voice Agent.")
            return

        voice_python = pathlib.Path.home() / "voice_llm_env/bin/python3"
        piper_voice = (pathlib.Path.home()
                       / ".local/share/piper-voices/vi_VN-vais1000-medium.onnx")
        if not voice_python.is_file() or not os.access(voice_python, os.X_OK):
            messagebox.showerror(
                "Thiếu Python cho Voice Agent",
                f"Voice-agent Python executable is missing or not executable:\n{voice_python}")
            return
        if not piper_voice.is_file():
            messagebox.showerror(
                "Thiếu giọng Piper",
                f"Piper Vietnamese voice model is missing:\n{piper_voice}")
            return
        if shutil.which("gnome-terminal") is None:
            messagebox.showerror("Thiếu gnome-terminal", "gnome-terminal is not installed.")
            return
        if not self._ensure_ollama():
            return

        if scenario == "function_calling":
            self.stop_all(quiet=True)
            agent_title = ("Voice agent (tiếng Việt)" if input_mode == "voice"
                           else "Text chat agent (tiếng Việt)")
            agent_launch = ("voice_agent.launch.py" if input_mode == "voice"
                            else "text_chat_agent.launch.py")
            try:
                open_term(
                    "Colour sort (Gazebo + grasp)",
                    f"{ENV_SETUP}; ros2 launch jetrover_grasp grasp_demo.launch.py "
                    "auto_grasp:=false",
                )
                open_term(
                    agent_title,
                    f"{ENV_SETUP}; {WAIT_GRASP_EXECUTOR}; "
                    f"ros2 launch voice_llm_agent {agent_launch}",
                )
            except OSError as exc:
                messagebox.showerror("Không mở được terminal", str(exc))
                self.status.set("Khởi động Voice Agent thất bại.")
                return
            self.status.set(f"Đã mở 2 terminal cho Voice Agent '{scenario_label}'.")
            return

        entry = self._selected_voice_map()
        if entry is None:
            messagebox.showwarning("Chưa chọn map", "Hãy chọn một map trong danh sách.")
            return

        known = {world["file"]: world for world in self.worlds}
        if entry["world_file"] not in known:
            messagebox.showerror(
                "Thiếu world",
                f"Map này được tạo trong '{entry['world_file']}', nhưng world đó không còn "
                "trong package.\n\nChạy lại: colcon build --packages-select jetrover_gazebo")
            return

        world_path = known[entry["world_file"]]["path"]
        map_yaml = pathlib.Path(entry["yaml"])
        pose_file = arm_pose_file("2d")
        if not world_path.is_file():
            messagebox.showerror(
                "Thiếu world", f"World của map '{entry['map_name']}' không tồn tại:\n{world_path}")
            return
        if not map_yaml.is_file():
            messagebox.showerror(
                "Thiếu map.yaml", f"map.yaml của map '{entry['map_name']}' không tồn tại:\n{map_yaml}")
            return
        if not pose_file.is_file():
            messagebox.showerror(
                "Thiếu tư thế tay máy", f"File tư thế tay máy 2D không tồn tại:\n{pose_file}")
            return

        try:
            world_name = self._world_name_from_sdf(world_path)
        except (OSError, ET.ParseError, ValueError) as exc:
            messagebox.showerror(
                "Không đọc được world_name",
                f"Không trích xuất được world_name từ SDF:\n{world_path}\n\n{exc}")
            return

        if entry["map_name"] == "warehouse":
            locations = voice_scenarios_share_dir() / "config/locations_warehouse.yaml"
            # Match the shell launchers: a symlink/source checkout may contain a
            # newer config before the package's data files have been rebuilt.
            if not locations.is_file():
                source_locations = (WS / "src/voice_llm_scenarios/config"
                                    / "locations_warehouse.yaml")
                if source_locations.is_file():
                    locations = source_locations
            if not locations.is_file():
                messagebox.showerror(
                    "Thiếu bảng địa điểm",
                    f"Warehouse locations file is missing:\n{locations}")
                return
        else:
            warning = (
                f"CẢNH BÁO: Map '{entry['map_name']}' chưa có bảng địa điểm riêng — "
                "tool di chuyển theo tên địa điểm (move_to_location) sẽ không tìm thấy "
                "địa điểm nào, các tool khác vẫn hoạt động bình thường."
            )
            if scenario == "road_network":
                warning += (
                    f"\n\nCẢNH BÁO: Đồ thị đường (ROAD_GRAPH) dùng tên địa điểm của map "
                    f"warehouse nên không áp dụng được cho map '{entry['map_name']}'; "
                    "chương trình vẫn tiếp tục chạy."
                )
            messagebox.showwarning("Map chưa có bảng địa điểm", warning)
            locations = ""

        spawn = entry["spawn"]
        sim = (f"ros2 launch jetrover_gazebo gazebo_arm.launch.py "
               f"world:={sh(world_path)} world_name:={sh(world_name)} "
               f"spawn_x:={sh(spawn['x'])} spawn_y:={sh(spawn['y'])} "
               f"spawn_yaw:={sh(spawn['yaw'])} "
               f"initial_positions_file:={sh(pose_file)}")
        nav = ("ros2 launch navigation navigation.launch.py use_sim_time:=true "
               f"localization:=true map:={sh(map_yaml)} use_rviz:=true")

        if scenario == "navigation_transport":
            agent_title = ("Voice agent (tiếng Việt)" if input_mode == "voice"
                           else "Text chat agent (tiếng Việt)")
            agent_launch = ("voice_agent.launch.py" if input_mode == "voice"
                            else "text_chat_agent.launch.py")
            terminals = [
                (f"Gazebo ({entry['map_name']})", f"{ENV_SETUP}; {sim}"),
                ("Nav2 + RViz", f"{ENV_SETUP}; {WAIT_SIM}; {nav}"),
                (agent_title,
                 f"{ENV_SETUP}; {WAIT_NAV}; ros2 launch voice_llm_agent "
                 f"{agent_launch} locations_yaml_path:={sh(locations)}"),
            ]
        elif scenario == "transport_delivery":
            agent_title = ("Voice agent (tiếng Việt)" if input_mode == "voice"
                           else "Text chat agent (tiếng Việt)")
            agent_launch = ("voice_agent.launch.py" if input_mode == "voice"
                            else "text_chat_agent.launch.py")
            grasp = (
                "ros2 run arm_perception color_pick --ros-args -p use_sim_time:=true "
                "-p start_enabled:=false & color_pick_pid=$!; "
                "ros2 run jetrover_grasp grasp_executor --ros-args "
                "-p use_sim_time:=true -p auto_grasp:=false -p mobile_enabled:=true "
                "-p 'bin_red:=[-0.135, -0.32, 0.02]' "
                "-p 'bin_green:=[-0.055, -0.32, 0.02]' "
                "-p 'bin_blue:=[0.025, -0.32, 0.02]' "
                "-p 'bin_yellow:=[0.105, -0.32, 0.02]' & grasp_pid=$!; "
                "wait $color_pick_pid $grasp_pid"
            )
            terminals = [
                (f"Gazebo ({entry['map_name']} + grasp)", f"{ENV_SETUP}; {sim}"),
                ("Nhận diện màu + tay gắp", f"{ENV_SETUP}; {WAIT_CTRL}; {grasp}"),
                ("Nav2 + RViz", f"{ENV_SETUP}; {WAIT_SIM}; {nav}"),
                (agent_title,
                 f"{ENV_SETUP}; {WAIT_VOICE_DEPS}; ros2 launch voice_llm_agent "
                 f"{agent_launch} locations_yaml_path:={sh(locations)} "
                 "grasp_executor_node_name:=grasp_executor"),
            ]
        else:
            if input_mode == "voice":
                input_process = (
                    f"LD_LIBRARY_PATH={sh(pathlib.Path.home() / '.local/lib')} "
                    f"{sh(voice_python)} -m voice_llm_agent.infrastructure.ros.voice_loop_node "
                    "--ros-args "
                    "-p user_utterance_topic:=/road_network_tool_executor/user_utterance "
                    "-p agent_reply_topic:=/road_network_tool_executor/agent_reply "
                    "-p whisper_model_size:=small -p whisper_language:=vi "
                    "-p 'whisper_initial_prompt:=Đây là một đoạn hội thoại tiếng Việt với robot.' "
                    "-p whisper_device:=cpu -p whisper_compute_type:=int8 "
                    "-p record_seconds:=5.0 -p silence_rms_threshold:=0.05 "
                    f"-p piper_voice_path:={sh(piper_voice)} "
                )
            else:
                input_process = (
                    f"LD_LIBRARY_PATH={sh(pathlib.Path.home() / '.local/lib')} "
                    f"{sh(voice_python)} -m voice_llm_agent.infrastructure.ros.text_chat_node "
                    "--ros-args "
                    "-p user_utterance_topic:=/road_network_tool_executor/user_utterance "
                    "-p agent_reply_topic:=/road_network_tool_executor/agent_reply "
                    f"-p piper_voice_path:={sh(piper_voice)} "
                )
            road_agent = (
                "ros2 run voice_llm_scenarios road_network_tool_executor --ros-args "
                "-r __node:=road_network_tool_executor "
                "-p camera_topic:=/depth_cam/image -p cmd_vel_topic:=/cmd_vel "
                f"-p locations_yaml_path:={sh(locations)} "
                "-p ollama_base_url:=http://localhost:11434 "
                "-p ollama_model:=qwen2.5vl:3b -p nav_timeout_s:=120.0 "
                "& road_executor_pid=$!; "
                f"{input_process}"
                "& voice_loop_pid=$!; wait $road_executor_pid $voice_loop_pid"
            )
            agent_title = ("Voice agent (mạng lưới đường)" if input_mode == "voice"
                           else "Text chat agent (mạng lưới đường)")
            terminals = [
                (f"Gazebo ({entry['map_name']})", f"{ENV_SETUP}; {sim}"),
                ("Nav2 + RViz", f"{ENV_SETUP}; {WAIT_SIM}; {nav}"),
                (agent_title, f"{ENV_SETUP}; {WAIT_NAV}; {road_agent}"),
            ]

        self.stop_all(quiet=True)
        try:
            for title, command in terminals:
                open_term(title, command)
        except OSError as exc:
            messagebox.showerror("Không mở được terminal", str(exc))
            self.status.set("Khởi động Voice Agent thất bại.")
            return

        self.status.set(
            f"Đã mở {len(terminals)} terminal cho Voice Agent '{scenario_label}' "
            f"với map '{entry['map_name']}'.")

    # ----------------------------------------------------------------- shared
    def stop_all(self, quiet=False):
        # stop_sim.sh reads WS from the environment (defaulting to ~/ros2_ws);
        # pass ours so a custom JETROVER_WS cleans up its own workspace rather
        # than the default one.
        result = subprocess.run(
            ["bash", str(WS / "scripts/stop_sim.sh")],
            env={**os.environ, "WS": str(WS)},
            capture_output=True, text=True, check=False)
        ok = result.returncode == 0
        # The SLAM run this marker described is gone; keeping it would let a
        # later "Lưu map" tag a map with a world that is no longer running.
        SESSION_FILE.unlink(missing_ok=True)
        if not ok:
            detail = (result.stderr or result.stdout or "Không rõ lỗi").strip()
            self.status.set("Không dừng hết được simulation.")
            if not quiet:
                messagebox.showerror("Dừng simulation thất bại", detail)
        elif not quiet:
            self.status.set("Đã dừng simulation.")
        return ok


def main():
    root = tk.Tk()
    root.title("JetRover - SLAM & Nav2 launcher")
    root.geometry("880x560")
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
