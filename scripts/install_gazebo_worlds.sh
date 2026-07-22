#!/usr/bin/env bash
# Pre-download every Fuel model the bundled worlds reference, into the local
# Fuel cache (~/.ignition/fuel).  Run this ONCE after cloning the workspace.
#
#   Usage:  install_gazebo_worlds.sh
#
# Why this exists: the world SDFs in jetrover_gazebo/worlds reference models by
# their https://fuel.gazebosim.org/... URI. Gazebo *can* fetch those lazily on
# first run, but it does so while the simulation is already starting - the world
# appears empty for minutes, sensors publish into a void, and SLAM maps nothing.
# Downloading up front turns the first run into a normal run.
#
# The models land in the Fuel cache, NOT in the repo: they are ~40 MB of meshes
# and textures that would bloat git for no benefit. That means this script needs
# network access, but only the first time - the cache persists.
set -u

# Every model referenced by worlds/*.sdf, grouped by the world that needs it.
# Keep in sync when adding a world; a missing model shows up as an invisible
# object at runtime, which is much harder to diagnose than a failure here.
WAREHOUSE_MODELS=(
  aws_robomaker_warehouse_Bucket_01
  aws_robomaker_warehouse_ClutteringA_01
  aws_robomaker_warehouse_ClutteringC_01
  aws_robomaker_warehouse_ClutteringD_01
  aws_robomaker_warehouse_GroundB_01
  aws_robomaker_warehouse_Lamp_01
  aws_robomaker_warehouse_PalletJackB_01
  aws_robomaker_warehouse_ShelfD_01
  aws_robomaker_warehouse_ShelfE_01
  aws_robomaker_warehouse_ShelfF_01
  aws_robomaker_warehouse_TrashCanC_01
  aws_robomaker_warehouse_WallB_01
)

DEPOT_MODELS=(
  Depot
)

# Props for the hand-written worlds (hospital / factory / office / apartment).
# "Euro%20pallet" is URL-encoded on purpose: the model's Fuel name contains a
# space, and ign fuel passes the URI through unescaped.
PROP_MODELS=(
  CGMClassic
  BedsideTable
  IVStand
  BMWCart
  Chair
  Desk
  Table
  SquareShelf
  Bookshelf
  Sofa
  Euro%20pallet
)
# NOT OpenRobotics/Pallet_Rack_Section or /Pallet_Jack: their visuals are .glb
# and Ignition Fortress cannot load glTF, so they download fine but render as
# nothing while spamming [Err] MeshManager. factory.sdf uses the .DAE
# aws_robomaker_warehouse_* shelves above instead.

FUEL_BASE="https://fuel.gazebosim.org/1.0/OpenRobotics"

command -v ign >/dev/null || {
  echo "ERROR: the 'ign' CLI is not on PATH. Install ignition-tools:" >&2
  echo "  sudo apt install ignition-fortress" >&2
  exit 1
}

failed=()

# `-j 4` parallelises the mesh/texture fetches inside one resource. `-t` is NOT
# passed: it only filters collections and is ignored for single resources.
fetch() {
  local label="$1" url="$2"
  # `ign fuel download` is a no-op when the item is already cached, so this
  # script is safe to re-run.
  if ign fuel download -u "$url" -j 4 >/dev/null 2>&1; then
    echo "  ok    $label"
  else
    echo "  FAIL  $label" >&2
    failed+=("$label")
  fi
}

download() {
  local name="$1"
  fetch "$name" "$FUEL_BASE/models/$name"
}

echo "Downloading worlds..."
fetch industrial-warehouse "$FUEL_BASE/worlds/industrial-warehouse"
# tugbot_depot lives under a different owner, so it cannot use FUEL_BASE.
fetch tugbot_depot "https://fuel.gazebosim.org/1.0/MovAi/worlds/tugbot_depot"

echo "Downloading warehouse models..."
for m in "${WAREHOUSE_MODELS[@]}"; do download "$m"; done

echo "Downloading depot models..."
for m in "${DEPOT_MODELS[@]}"; do download "$m"; done

echo "Downloading props for the hand-written worlds..."
for m in "${PROP_MODELS[@]}"; do download "$m"; done

echo
if [ ${#failed[@]} -eq 0 ]; then
  echo "All assets cached in ~/.ignition/fuel - the bundled worlds will load offline now."
else
  echo "${#failed[@]} item(s) failed to download:" >&2
  printf '  %s\n' "${failed[@]}" >&2
  echo >&2
  echo "The worlds still load, but the failed models render as empty space." >&2
  echo "Re-run this script once the network is back." >&2
  exit 1
fi
