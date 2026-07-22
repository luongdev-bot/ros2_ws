# jetrover_gazebo — worlds and the SLAM → Nav2 launcher

## One-time setup

The worlds pull their furniture from Gazebo Fuel. Cache it once:

```bash
bash ~/ros2_ws/scripts/install_gazebo_worlds.sh
colcon build          # the GUI shells out to slam / navigation / peripherals too
```

Build the whole workspace, not just this package: the launcher opens terminals
running `slam`, `navigation` and `peripherals` launch files, so building only
`jetrover_gazebo` gives you a window whose every button fails with "package not
found". Those three are declared as `exec_depend` in `package.xml`.

Without this the first launch still works, but Gazebo downloads meshes while
the simulation is already running: the world looks empty for minutes and SLAM
maps nothing.

The script is idempotent — re-run it any time. If it fails (no network, Fuel
down), the same assets can be fetched by hand:

```bash
ign fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/industrial-warehouse" -j 4
ign fuel download -u "https://fuel.gazebosim.org/1.0/MovAi/worlds/tugbot_depot" -j 4
ign fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot" -j 4
# ...and each model listed in scripts/install_gazebo_worlds.sh
```

Everything lands in `~/.ignition/fuel`, not in the repo. Note the URLs use
`fuel.gazebosim.org`: the Fuel cache is keyed by hostname, so the older
`fuel.ignitionrobotics.org` form downloads a second copy that the bundled
worlds will not find.

## The launcher

```bash
ros2 run jetrover_gazebo sim_launcher_gui      # from a sourced shell
bash ~/ros2_ws/scripts/run_sim_gui.sh          # or the wrapper (sources + checks first)
```

The wrapper is what the desktop shortcut points at: it sources the workspace,
checks the build and `python3-tk`, and reports failures through
`zenity`/`notify-send` instead of dying silently with no console.

To put it on the Desktop and in the app menu, alongside the existing SLAM 2D /
Nav2 2D icons:

```bash
bash ~/ros2_ws/scripts/install_sim_gui_shortcut.sh
```

Like the other launchers here it sets no `Icon=`, so it uses the system default
icon. If GNOME shows an "untrusted" prompt on the Desktop copy, right-click it
and choose *Allow Launching*.

The older `SLAM 2D` / `Nav2 2D` icons still work and are unchanged — they run a
fixed world (`jetrover_world.sdf`) through `run_slam.sh` / `run_nav.sh`. This
new icon is the one that lets you pick the world and keeps the map tied to it.

**Tab 1 — Tạo map (SLAM).** Pick a world, pick 2D or 3D, press *Khởi động
SLAM*. Three terminals open (Gazebo, SLAM, teleop). Drive with `w/a/s/d` in the
teleop window until the map looks complete, then type a name and press *Lưu
map*.

**Tab 2 — Chạy Nav2.** Pick a saved map and press *Chạy Nav2 với map này*. It
reopens **the world that map was made in**, spawns the robot at the same point,
and starts Nav2 with `map_server` + AMCL on the saved map. Set goals with the
*Nav2 Goal* tool in RViz.

### Why the map remembers its world

A `.yaml`/`.pgm` pair records nothing about where it came from, so loading the
hospital map into the warehouse silently gives AMCL garbage to localise
against. Every map saved through the GUI therefore gets a sidecar:

```text
maps/<name>/
  map.yaml         # nav2 map_server input
  map.pgm
  slam_map.json    # {world_id, world_file, spawn, slam_mode, created}
```

The spawn point in that sidecar is load-bearing. `slam_toolbox` puts the map
frame's origin wherever the robot stood when mapping began, so Nav2 must
re-spawn the robot at that same point for `amcl.initial_pose` (0, 0, 0 in
`nav2_params.yaml`) to be correct.

The binding is an **operator convention, not a guarantee**. The GUI records the
world when *it* starts SLAM, and saving reads that record. If you start SLAM in
some other world straight from a terminal, or leave a Nav2 `map_server`
publishing `/map`, *Lưu map* will still label whatever map it finds with the
last world the GUI launched. Start SLAM from tab 1 and it holds.

`maps/` is git-ignored — the worlds that produce the maps are tracked, so any
map can be re-made.

## The worlds

| File | What it is | Source |
| ---- | ---------- | ------ |
| `jetrover_world.sdf` | empty plane, robot testing | authored |
| `color_blocks_world.sdf` | colour-pick demo | authored |
| `hospital.sdf` | 18×12 m ward corridor, 8 rooms | generated |
| `factory.sdf` | 22×14 m production hall, racking | generated |
| `office.sdf` | 14×10 m open plan + meeting rooms | generated |
| `apartment.sdf` | 10×8 m flat, narrow doorways | generated |
| `warehouse.sdf` | AWS RoboMaker industrial warehouse | Fuel (Open Robotics) |
| `depot.sdf` | ~30×15 m depot | Fuel (MovAi) |

### What each world looks like to the lidar

Measured by spawning the robot and capturing one `/scan` (RPLidar A1, 360
beams, 12 m range). "Returns" is the share of beams that hit something — a low
figure means open space beyond lidar range, not a broken world.

| World | Returns | Nearest / farthest | Notes |
| ----- | ------- | ------------------ | ----- |
| `hospital` | 99% | 1.41 / 9.06 m | 3 m corridor, sight line to the far end |
| `factory` | 91% | 3.42 / 11.92 m | open hall, racking at mid range |
| `office` | 99% | 1.76 / 7.14 m | short sight lines, scattered furniture |
| `apartment` | 99% | 1.12 / 4.96 m | tightest; hardest to map |
| `warehouse` | 99% | 1.51 / 10.62 m | central aisle between shelf rows |
| `depot` | 73% | 3.70 / 11.67 m | ~30 m long, so some directions exceed lidar range |

No world spawns the robot inside geometry (no beam under 0.30 m in any of
them). If you add a world and SLAM produces a blank map, capture a scan the
same way before blaming SLAM — a spawn point outside the building looks
identical to a broken sensor.

The first two are hand-written and edited directly. The middle four are
**generated** — edit the layout in `tools/generate_worlds.py` and re-run it,
never the `.sdf`. The last two are **vendored** from Fuel by
`tools/vendor_fuel_worlds.py`, which re-points model URIs at
`fuel.gazebosim.org` (the Fuel cache is keyed by hostname), injects the
Ignition system plugins the JetRover's sensors need, and strips the Tugbot
robot that shipped inside the depot world.

`config/world_catalog.yaml` supplies each world's display name, blurb and
**spawn point**. A world with no entry there still appears in the GUI, under
its filename and spawning at the origin.

Adding a world: drop the `.sdf` in `worlds/`, add a catalogue entry, rebuild,
then run the asset check below.

### Check Fuel assets before trusting a world

```bash
python3 src/simulations/jetrover_gazebo/tools/check_world_assets.py
```

Ignition Fortress reads COLLADA, OBJ, STL and FBX meshes. It does **not** read
glTF (`.glb`/`.gltf`) — that arrived in Gazebo Garden. A Fuel model whose visual
is glTF downloads fine, loads without complaint, and then renders as nothing
while spamming `[Err] MeshManager` once per frame.

Launching the world headless does not catch this: `ign gazebo -s` never
exercises the render path, so the mesh is never parsed. That is how
`OpenRobotics/Pallet_Rack_Section` and `/Pallet_Jack` originally got into
`factory.sdf` — their collision geometry is boxes, so SLAM and Nav2 worked
correctly the whole time and only the visuals were missing. They have been
replaced with the `.DAE` `aws_robomaker_warehouse_*` shelves.

## Arm pose while driving

`gazebo_arm.launch.py` takes `initial_positions_file`. Three are available in
`jetrover_moveit_config/config/`, and each documents its own forward
kinematics:

| File | Pose | Use |
| ---- | ---- | --- |
| `home_initial_positions.yaml` | folded, camera 30.7° down | 2D SLAM, Nav2 — the GUI default |
| `slam_initial_positions.yaml` | horizontal, camera 4.6° down | 3D SLAM (RTAB-Map) |
| `initial_positions.yaml` | pick_init | `arm_perception` colour picking |

All three keep the lowest arm link at 0.193 m, above the 0.157 m lidar plane,
so the arm never shows up in `/scan`. The split between the first two is about
the **camera**: 2D and Nav2 ignore it, but 3D SLAM does not, and `home` aims it
at the floor a metre ahead instead of at the room.

`scripts/run_slam.sh` and `run_nav.sh` predate this GUI and still boot 2D runs
in the `horizontal` pose. Both work; they just differ from what the GUI does.
Change the `initial_positions_file` in those scripts if you want them aligned.

## Interaction with stop_sim.sh

The GUI calls `scripts/stop_sim.sh` before every launch, because a leftover
Gazebo publishes a second `/clock` and `/tf` and makes TF jump backwards.

That script matches processes by command line, and two of its patterns
(`"$WS/install"` and `"jetrover_gazebo"`) match the GUI itself and the
`ros2 run` that supervises it. It therefore skips its own ancestor chain — see
`ancestor_pids()` there. If you ever launch the GUI by some other route, keep
that in mind: without the guard, cleanup kills the launcher.

`stop_sim.sh` does **not** match a bare `ign gazebo server` process, which is
what survives if a launch is SIGKILLed rather than closed. Those have to be
cleared by hand:

```bash
pkill -9 -f "ign gazebo"
```
