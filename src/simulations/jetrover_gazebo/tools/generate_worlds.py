#!/usr/bin/env python3
"""Generate the hand-authored SLAM worlds (hospital / factory / office / apartment).

Run it after editing a LAYOUT below, then rebuild the package:

    python3 src/simulations/jetrover_gazebo/tools/generate_worlds.py
    colcon build --packages-select jetrover_gazebo

The generated .sdf files are committed, so this script is NOT part of the build -
it exists because hand-editing several hundred lines of wall boxes per world is
how you end up with a wall a metre out of place and a SLAM map nobody trusts.

Layout conventions, all chosen for the JetRover + RPLidar A1 (360 deg, 12 m):
  * Rooms stay under ~12 m across so the lidar reaches the far wall; beyond that
    SLAM sees open space and loop closure gets unreliable.
  * Doorways are >= 1.2 m so a 0.4 m robot can drive through without clipping.
  * (0, 0) is always clear: gazebo.launch.py spawns the robot there.
  * Walls are 2.5 m tall - well above any plausible lidar mount - and 0.15 m
    thick so they register as solid rather than as a thin double return.
"""
import math
import pathlib

OUT = pathlib.Path(__file__).resolve().parent.parent / "worlds"

WALL_HEIGHT = 2.5
WALL_THICKNESS = 0.15
FUEL = "https://fuel.gazebosim.org/1.0/OpenRobotics/models"

# --------------------------------------------------------------------------
# Layouts.  walls: (x1, y1, x2, y2) segments.  props: (fuel_model, x, y, yaw).
# A doorway is simply the gap between two consecutive wall segments.
# --------------------------------------------------------------------------


def _room_row(y, x_min, x_max, doors, door_w=1.4):
    """Horizontal wall at `y` from x_min..x_max, with a gap at each door x."""
    gaps = sorted((d - door_w / 2, d + door_w / 2) for d in doors)
    segs, cursor = [], x_min
    for lo, hi in gaps:
        if lo > cursor:
            segs.append((cursor, y, lo, y))
        cursor = hi
    if cursor < x_max:
        segs.append((cursor, y, x_max, y))
    return segs


def _rect(x_min, y_min, x_max, y_max):
    """The four outer walls of a rectangular building."""
    return [
        (x_min, y_min, x_max, y_min),
        (x_max, y_min, x_max, y_max),
        (x_max, y_max, x_min, y_max),
        (x_min, y_max, x_min, y_min),
    ]


# --- hospital: central corridor, four wards each side ----------------------
HOSPITAL_DOORS_N = [-6.75, -2.25, 2.25, 6.75]
HOSPITAL = {
    "name": "hospital",
    "description": "Ward corridor with eight rooms - long straight runs and "
                   "repeated doorways, the classic loop-closure stress test.",
    "walls": (
        _rect(-9, -6, 9, 6)
        + _room_row(1.5, -9, 9, HOSPITAL_DOORS_N)
        + _room_row(-1.5, -9, 9, HOSPITAL_DOORS_N)
        + [(-4.5, 1.5, -4.5, 6), (0, 1.5, 0, 6), (4.5, 1.5, 4.5, 6)]
        + [(-4.5, -1.5, -4.5, -6), (0, -1.5, 0, -6), (4.5, -1.5, 4.5, -6)]
    ),
    "props": [
        ("BedsideTable", -6.5, 4.5, 0.0), ("IVStand", -7.6, 3.2, 0.0),
        ("BedsideTable", -2.0, 4.5, 0.0), ("IVStand", -3.1, 3.2, 0.0),
        ("BedsideTable", 2.5, 4.5, 0.0), ("IVStand", 1.4, 3.2, 0.0),
        ("BedsideTable", 7.0, 4.5, 0.0), ("IVStand", 5.9, 3.2, 0.0),
        ("BedsideTable", -6.5, -4.5, math.pi), ("IVStand", -7.6, -3.2, 0.0),
        ("BedsideTable", -2.0, -4.5, math.pi), ("IVStand", -3.1, -3.2, 0.0),
        ("BedsideTable", 2.5, -4.5, math.pi), ("CGMClassic", 1.4, -3.4, 0.0),
        ("BedsideTable", 7.0, -4.5, math.pi), ("CGMClassic", 5.9, -3.4, 0.0),
        # Corridor clutter - the interesting part for a local costmap.
        ("BMWCart", -5.0, 0.0, 0.0), ("BMWCart", 6.2, 0.6, math.pi / 2),
        ("Chair", -8.0, 0.0, 0.0), ("Chair", 8.2, -0.5, math.pi),
    ],
}

# --- factory: open bays, racking down the middle ---------------------------
FACTORY = {
    "name": "factory",
    "description": "Open production hall with racking aisles and machine bays - "
                   "wide free space broken by regular tall obstacles.",
    "walls": (
        _rect(-11, -7, 11, 7)
        # Machine bays along the north wall, open to the south.
        + [(-8, 7, -8, 3.5), (-4, 7, -4, 3.5), (0, 7, 0, 3.5), (4, 7, 4, 3.5)]
        + [(-11, 3.5, -8, 3.5), (-8, 3.5, -4, 3.5), (-4, 3.5, 0, 3.5)]
        # Enclosed tool room in the south-east corner, one door facing west.
        + [(6, -7, 6, -2.5), (6, -2.5, 11, -2.5)]
    ),
    # NOT OpenRobotics/Pallet_Rack_Section or /Pallet_Jack, which are the
    # obvious picks for a factory: their visuals are .glb, and Ignition Fortress
    # cannot load glTF (support arrived in Gazebo Garden). Their collision
    # geometry is boxes, so the lidar still sees them and SLAM works - but they
    # render as nothing and spam [Err] MeshManager for every frame. The AWS
    # warehouse models below are .DAE and are already proven by warehouse.sdf.
    "props": [
        ("aws_robomaker_warehouse_ShelfE_01", -6.0, -1.5, 0.0),
        ("aws_robomaker_warehouse_ShelfE_01", -6.0, -3.5, 0.0),
        ("aws_robomaker_warehouse_ShelfD_01", -1.0, -1.5, 0.0),
        ("aws_robomaker_warehouse_ShelfD_01", -1.0, -3.5, 0.0),
        ("aws_robomaker_warehouse_ShelfF_01", 3.0, -1.5, 0.0),
        ("SquareShelf", -9.5, 1.0, 0.0), ("SquareShelf", -9.5, -1.0, 0.0),
        ("Euro pallet", -7.5, 5.0, 0.0), ("Euro pallet", -3.0, 5.0, 0.3),
        ("Euro pallet", 1.5, 5.2, 0.0), ("Euro pallet", 8.0, 2.0, 0.0),
        ("aws_robomaker_warehouse_PalletJackB_01", 8.5, -5.0, math.pi / 2),
        ("aws_robomaker_warehouse_PalletJackB_01", -9.0, -5.5, 0.0),
    ],
}

# --- office: open plan west, meeting rooms east ----------------------------
OFFICE = {
    "name": "office",
    "description": "Open-plan desks plus three glazed meeting rooms - lots of "
                   "small scattered obstacles rather than long walls.",
    "walls": (
        _rect(-7, -5, 7, 5)
        # Spine wall splitting open plan (west) from meeting rooms (east),
        # with a single 2.4 m opening on the centre line.
        + [(1.5, -5, 1.5, -1.2), (1.5, 1.2, 1.5, 5)]
        # Meeting-room dividers off the spine.
        + [(1.5, 2.2, 7, 2.2), (1.5, -2.2, 7, -2.2)]
    ),
    "props": [
        ("Desk", -5.5, 3.0, 0.0), ("Chair", -5.5, 2.0, 0.0),
        ("Desk", -2.5, 3.0, 0.0), ("Chair", -2.5, 2.0, 0.0),
        ("Desk", -5.5, -1.0, 0.0), ("Chair", -5.5, -2.0, 0.0),
        ("Desk", -2.5, -1.0, 0.0), ("Chair", -2.5, -2.0, 0.0),
        ("Bookshelf", -6.6, 0.5, math.pi / 2),
        ("Bookshelf", -6.6, -4.0, math.pi / 2),
        ("Table", 4.5, 3.6, 0.0), ("Chair", 3.6, 3.6, 0.0), ("Chair", 5.4, 3.6, math.pi),
        ("Table", 4.5, -3.6, 0.0), ("Chair", 3.6, -3.6, 0.0), ("Chair", 5.4, -3.6, math.pi),
        ("Table", 4.5, 0.0, 0.0), ("Chair", 3.6, 0.0, 0.0),
    ],
}

# --- apartment: tight domestic rooms ---------------------------------------
APARTMENT = {
    "name": "apartment",
    "description": "Small flat - narrow doorways and short sight lines, the "
                   "hardest of the six for a 360 deg scanner.",
    "walls": (
        _rect(-5, -4, 5, 4)
        # Hall runs east-west along y = 0; rooms north and south of it.
        + _room_row(1.2, -5, 5, [-3.0, 1.0])
        + _room_row(-1.2, -5, 5, [-3.0, 2.0])
        + [(-1.5, 1.2, -1.5, 4), (2.5, 1.2, 2.5, 4)]
        + [(0.0, -1.2, 0.0, -4)]
    ),
    "props": [
        ("Sofa", -3.2, 3.0, math.pi), ("Table", -3.2, 2.0, 0.0),
        ("Bookshelf", -0.2, 3.4, math.pi),
        ("Desk", 3.8, 3.0, math.pi / 2), ("Chair", 3.0, 3.0, 0.0),
        ("Table", -2.5, -2.8, 0.0), ("Chair", -3.4, -2.8, 0.0),
        ("Chair", -1.6, -2.8, math.pi),
        ("SquareShelf", 2.0, -3.4, 0.0),
    ],
}

LAYOUTS = [HOSPITAL, FACTORY, OFFICE, APARTMENT]

# --------------------------------------------------------------------------

PREAMBLE = """<?xml version="1.0" ?>
<!--
  GENERATED FILE - do not edit by hand.
  Source of truth: tools/generate_worlds.py (edit the layout there, re-run it).

  {description}
-->
<sdf version="1.9">
  <world name="{name}">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.75 0.8 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.4 0.4 0.4 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>60 60</size></plane>
          </geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>60 60</size></plane>
          </geometry>
          <material>
            <ambient>0.75 0.75 0.72 1</ambient>
            <diffuse>0.75 0.75 0.72 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

WALL_LINK = """      <link name="wall_{i}">
        <pose>{cx:.4f} {cy:.4f} {hz:.4f} 0 0 {yaw:.6f}</pose>
        <collision name="collision">
          <geometry><box><size>{length:.4f} {thickness} {height}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{length:.4f} {thickness} {height}</size></box></geometry>
          <material>
            <ambient>0.85 0.85 0.83 1</ambient>
            <diffuse>0.85 0.85 0.83 1</diffuse>
          </material>
        </visual>
      </link>
"""

PROP = """    <include>
      <uri>{fuel}/{model}</uri>
      <name>{name}</name>
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.6f}</pose>
    </include>
"""


def build(layout):
    parts = [PREAMBLE.format(name=layout["name"], description=layout["description"])]

    # All walls live in ONE static model. Gazebo pays a per-model cost at load
    # and per-entity cost each step, and a world with 40 separate wall models
    # measurably drops the real-time factor on an integrated GPU.
    parts.append('\n    <model name="walls">\n      <static>true</static>\n')
    for i, (x1, y1, x2, y2) in enumerate(layout["walls"]):
        length = math.hypot(x2 - x1, y2 - y1)
        if length < 1e-6:
            raise ValueError(f"{layout['name']}: zero-length wall at index {i}")
        parts.append(WALL_LINK.format(
            i=i, cx=(x1 + x2) / 2, cy=(y1 + y2) / 2, hz=WALL_HEIGHT / 2,
            yaw=math.atan2(y2 - y1, x2 - x1),
            # Extend by one thickness so corner joins do not leave a slit the
            # lidar can see through - a slit becomes a phantom opening on the map.
            length=length + WALL_THICKNESS,
            thickness=WALL_THICKNESS, height=WALL_HEIGHT))
    parts.append("    </model>\n\n")

    # Fuel props are separate <include>s: they are non-static so the robot can
    # nudge them, which is what makes the local costmap worth testing.
    seen = {}
    for model, x, y, yaw in layout["props"]:
        seen[model] = seen.get(model, 0) + 1
        parts.append(PROP.format(
            fuel=FUEL, model=model.replace(" ", "%20"),
            name=f"{model.replace(' ', '_').lower()}_{seen[model]}",
            x=x, y=y, yaw=yaw))

    parts.append("\n  </world>\n</sdf>\n")
    return "".join(parts)


def main():
    for layout in LAYOUTS:
        # Guard the spawn point: gazebo.launch.py drops the robot at (0, 0) and
        # a wall through it wedges the robot inside geometry on every launch.
        for x1, y1, x2, y2 in layout["walls"]:
            if _point_near_segment(0.0, 0.0, x1, y1, x2, y2) < 0.6:
                raise ValueError(
                    f"{layout['name']}: wall ({x1},{y1})-({x2},{y2}) is inside "
                    "the robot spawn clearance at the origin")
        path = OUT / f"{layout['name']}.sdf"
        path.write_text(build(layout))
        print(f"wrote {path.relative_to(OUT.parent.parent.parent.parent)} "
              f"({len(layout['walls'])} walls, {len(layout['props'])} props)")


def _point_near_segment(px, py, x1, y1, x2, y2):
    """Shortest distance from (px, py) to the segment (x1,y1)-(x2,y2)."""
    dx, dy = x2 - x1, y2 - y1
    seg_sq = dx * dx + dy * dy
    if seg_sq == 0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / seg_sq))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


if __name__ == "__main__":
    main()
