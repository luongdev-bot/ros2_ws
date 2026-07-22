#!/usr/bin/env python3
"""Check every Fuel model the worlds reference for assets Fortress cannot load.

    python3 src/simulations/jetrover_gazebo/tools/check_world_assets.py

Why this is a separate check and not just "does the world load": launching a
world headless (`ign gazebo -s`) does not exercise the render path, so a model
whose visual mesh is in an unsupported format loads silently and reports no
error. It only shows up later as an invisible object plus a wall of
[Err] MeshManager lines - which is exactly how Pallet_Rack_Section got into
factory.sdf in the first place.

Ignition Fortress (gz-common4 / gz-rendering6) reads COLLADA, OBJ, STL and FBX.
It does NOT read glTF (.glb/.gltf); that arrived in Gazebo Garden.

The check reads the mesh URIs each model's SDF actually references, per
<visual>/<collision>. An earlier version just globbed the model directory for
file extensions, which passed any model that happened to ship one supported
file - including the dangerous case of a .glb visual beside a .dae collision.
"""
import pathlib
import re
import sys

WORLDS = pathlib.Path(__file__).resolve().parent.parent / "worlds"
CACHE = pathlib.Path.home() / ".ignition/fuel"

SUPPORTED = {".dae", ".obj", ".stl", ".fbx"}


def cache_dir_for(uri: str):
    """Map a Fuel model URI to its directory in the local cache."""
    m = re.match(r"https?://([^/]+)/\d+\.\d+/([^/]+)/models/(.+)$", uri.strip())
    if not m:
        return None
    host, owner, name = m.groups()
    # The cache lower-cases owner and model name but keeps URL-encoding as-is.
    base = CACHE / host / owner.lower() / "models" / name.lower()
    if not base.is_dir():
        return None
    versions = sorted((d for d in base.iterdir() if d.is_dir()),
                      key=lambda d: int(d.name) if d.name.isdigit() else -1)
    return versions[-1] if versions else None


MAX_SDF_VERSION = (1, 9)  # highest SDF spec Fortress understands


def _version_tuple(text: str):
    """'1.10' -> (1, 10). float() would make that 1.1 and call it 1.1 < 1.9."""
    try:
        return tuple(int(part) for part in text.strip().split("."))
    except ValueError:
        return None


def sdf_files_for(model_dir: pathlib.Path):
    """The SDF file(s) Fortress would actually load, filtered to ones that exist.

    model.config is authoritative: Fuel models ship several revisions
    (model-1_2.sdf, model-1_4.sdf, ...) and Fortress picks the newest whose
    declared version it supports. Reading every *.sdf instead would fail a model
    over an old compatibility revision that never gets loaded.

    Anything declared but absent is dropped here rather than skipped later, so
    the caller sees an empty list and reports it instead of silently checking
    nothing and passing.
    """
    config = model_dir / "model.config"
    if config.is_file():
        # Tolerant of whitespace around '=' and of extra attributes on the tag.
        entries = re.findall(
            r"<sdf\b[^>]*?\bversion\s*=\s*[\"']([\d.]+)[\"'][^>]*>\s*([^<]+?)\s*</sdf>",
            config.read_text(errors="replace"))
        if entries:
            usable = []
            for version, filename in entries:
                parsed = _version_tuple(version)
                path = model_dir / filename.strip()
                if parsed is not None and parsed <= MAX_SDF_VERSION and path.is_file():
                    usable.append((parsed, path))
            # Once model.config declares SDFs it is the only authority. If none
            # of them is usable, return nothing so the caller reports it - do
            # NOT fall back to some other *.sdf lying in the directory, which
            # would check an unrelated file and pass a model whose declared SDF
            # is missing or too new for Fortress.
            return [max(usable, key=lambda p: p[0])[1]] if usable else []
    # No model.config, or it declares no SDF at all - inspect what is present.
    return sorted(p for p in model_dir.glob("*.sdf") if p.is_file())


def referenced_meshes(model_dir: pathlib.Path):
    """[(element, mesh_uri)] for every mesh this model's SDF points at.

    Known limitation: <include>d submodels and actor <skin> files are not
    followed, so a mesh reached only that way is not checked.
    """
    found = []
    for sdf in sdf_files_for(model_dir):
        try:
            text = sdf.read_text(errors="replace")
        except OSError as exc:
            # Reported rather than swallowed: an unreadable SDF means this model
            # was never actually checked.
            found.append(("unreadable SDF", f"{sdf.name}: {exc}"))
            continue
        for element in ("visual", "collision"):
            for block in re.findall(rf"<{element}\b[^>]*>(.*?)</{element}>", text, re.S):
                # Only <uri> inside a <mesh>. A <visual> also carries
                # <material><script><uri>...gazebo.material</uri>, which is an
                # Ogre script, not geometry - matching those flagged every
                # primitive-shaped model as broken.
                for mesh in re.findall(r"<mesh>(.*?)</mesh>", block, re.S):
                    for uri in re.findall(r"<uri>\s*([^<]+?)\s*</uri>", mesh):
                        found.append((element, uri))
    return found


def main() -> int:
    problems = []
    missing = []

    for world in sorted(WORLDS.glob("*.sdf")):
        uris = sorted(set(re.findall(r"<uri>\s*(https?://[^<]+?)\s*</uri>",
                                     world.read_text())))
        bad_here = []
        for uri in uris:
            model_dir = cache_dir_for(uri)
            if model_dir is None:
                missing.append((world.name, uri))
                continue
            if not sdf_files_for(model_dir):
                # Nothing to inspect means nothing was verified; saying "ok"
                # here is the false reassurance this script exists to prevent.
                bad_here.append((uri.rsplit("/", 1)[-1], "model", "no SDF found"))
                continue

            for element, mesh_uri in referenced_meshes(model_dir):
                ext = pathlib.PurePosixPath(mesh_uri).suffix.lower()
                # Anything not explicitly supported is a problem, including
                # formats this script has never heard of, and URIs with no
                # extension at all - silence there is how the glTF case slipped
                # through the first time.
                if ext not in SUPPORTED:
                    bad_here.append((uri.rsplit("/", 1)[-1], element,
                                     ext or "(no extension)"))

        status = "FAIL" if bad_here else "ok"
        print(f"  {status:4s} {world.name:24s} {len(uris)} Fuel model(s)")
        for name, element, ext in sorted(set(bad_here)):
            print(f"         !! {name}: {element} mesh is {ext} - "
                  "Fortress cannot load it")
            problems.append((world.name, name))

    if missing:
        print(f"\n{len(missing)} referenced model(s) not in the cache - run "
              "scripts/install_gazebo_worlds.sh:")
        for world, uri in missing:
            print(f"  {world}: {uri.rsplit('/', 1)[-1]}")

    # A model that is not cached cannot be checked, so it is a failure too:
    # reporting "all good" while some models were never examined is exactly the
    # false reassurance this script exists to prevent.
    if problems or missing:
        if problems:
            print(f"\n{len(problems)} unsupported-asset problem(s). "
                  "Swap the model for a COLLADA/OBJ one.")
        return 1

    print("\nAll referenced models use formats Fortress can render.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
