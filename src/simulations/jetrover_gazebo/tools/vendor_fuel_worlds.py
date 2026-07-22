"""Vendor the two Fuel worlds into jetrover_gazebo/worlds/.

Two transformations are applied:
  1. fuel.ignitionrobotics.org -> fuel.gazebosim.org, so the model URIs resolve
     against the cache that scripts/install_gazebo_worlds.sh populates. The Fuel
     cache is keyed by hostname, so the old domain would force a second download
     into a second cache dir at runtime.
  2. Inject the Ignition system plugins the JetRover needs (sensors, physics,
     scene broadcaster, imu, user commands). industrial-warehouse ships without
     any plugin block, so lidar and camera publish nothing without this.
"""
import pathlib
import re

CACHE = pathlib.Path.home() / ".ignition/fuel/fuel.gazebosim.org"
OUT = pathlib.Path(__file__).resolve().parent.parent / "worlds"

PLUGINS = """
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>

    <!-- The warehouse floor model is visual-only, so without this collision
         plane the robot falls through the world on spawn. -->
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
      </link>
    </model>
"""

HEADER = """<!--
  Vendored from Gazebo Fuel: {src}
  Licence: Apache-2.0 (Open Robotics / {owner}).

  Local changes vs upstream:
    * model URIs re-pointed at fuel.gazebosim.org (the Fuel cache is keyed by
      hostname; the upstream fuel.ignitionrobotics.org URIs miss the cache that
      scripts/install_gazebo_worlds.sh fills and re-download at runtime).
{extra}  Run scripts/install_gazebo_worlds.sh once before using this world.
-->
"""


def vendor(src: pathlib.Path, dest_name: str, world_name: str, owner: str,
           add_plugins: bool, extra_note: str, drop_models=()) -> None:
    text = src.read_text()
    text = text.replace("fuel.ignitionrobotics.org", "fuel.gazebosim.org")

    for model in drop_models:
        # Strip an <include> block by the model its uri points at.
        pattern = (r"\n\s*<include>(?:(?!</include>).)*?"
                   + re.escape(model) + r"(?:(?!</include>).)*?</include>")
        text, n = re.subn(pattern, "", text, flags=re.S)
        assert n == 1, f"expected 1 include of {model}, removed {n}"

    # Give the world a stable, descriptive name. tugbot_depot ships as
    # "world_demo", which is meaningless in a catalogue of six worlds.
    text = re.sub(r"<world name=['\"][^'\"]*['\"]>",
                  f'<world name="{world_name}">', text, count=1)

    if add_plugins:
        # Anchor on </physics>: every Fuel world has exactly one, and plugins
        # must be direct children of <world>.
        assert text.count("</physics>") == 1, "expected exactly one </physics>"
        text = text.replace("</physics>", "</physics>\n" + PLUGINS, 1)

    header = HEADER.format(src=src.name, owner=owner, extra=extra_note)
    # Insert the provenance comment after the <?xml ...?> declaration.
    text = re.sub(r"(<\?xml[^>]*\?>\n)", r"\1" + header, text, count=1)

    dest = OUT / dest_name
    dest.write_text(text)
    print(f"wrote {dest} ({len(text)} bytes)")


vendor(
    CACHE / "openrobotics/worlds/industrial-warehouse/4/industrial-warehouse.sdf",
    "warehouse.sdf", "warehouse", "Open Robotics",
    add_plugins=True,
    extra_note=("    * added the Ignition system plugins (physics, sensors, scene\n"
                "      broadcaster, imu, user commands) - upstream ships none, so the\n"
                "      JetRover's lidar and depth camera would publish nothing.\n"
                "    * added a collision ground plane (upstream's floor is visual-only).\n"),
)

vendor(
    CACHE / "movai/worlds/tugbot_depot/1/tugbot_depot.sdf",
    "depot.sdf", "depot", "MovAi",
    add_plugins=False,
    extra_note=("    * renamed the world from 'world_demo' to 'depot'.\n"
                "    * removed the bundled MovAi Tugbot robot. It is a full robot\n"
                "      model, not scenery: it ships its own sensors and drive plugin,\n"
                "      which publish onto the same /scan and /cmd_vel the JetRover\n"
                "      uses, and it stood 1.2 m from our spawn point.\n"
                "    Upstream already declares the system plugins we need.\n"),
    drop_models=("MovAi/models/Tugbot",),
)
