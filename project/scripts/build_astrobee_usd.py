#!/usr/bin/env python3
"""Build `assets/space_asset/astrobee/astrobee.usd` from the official NASA Astrobee meshes.

Source: https://github.com/nasa/astrobee_media (`astrobee_freeflyer/`, Apache License 2.0,
see `assets/space_asset/astrobee/source/package.xml`). The meshes are the ones the
official robot description (`nasa/astrobee: description/description/urdf/model.urdf.xacro`)
uses for the `body` link: body, port PMC (pmc + bumper + skin) and the starboard PMC
(the same three meshes turned by rpy (0, pi, pi) = a half turn about X).

Must run with the Isaac Sim Python (uses `omni.kit.asset_converter` for COLLADA):

    ~/isaac-sim/python.sh project/scripts/build_astrobee_usd.py

Frames / units: metres, Z up in the USD file. The Astrobee body frame is kept as authored
by NASA (+X forward, +Y starboard/right, +Z down, origin at the body centre). The result
is visual only: no rigid body, no collider, no mass.
"""

import argparse
import asyncio
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ASTROBEE_DIR = ROOT.joinpath("assets", "space_asset", "astrobee")
MESHES = ("body", "pmc", "pmc_bumper", "pmc_skin_")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out_dir", type=str, default=ASTROBEE_DIR.as_posix())
    return parser.parse_args()


def main():
    args = parse_args()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("omni.kit.asset_converter")
    import omni.kit.asset_converter as converter
    from pxr import Gf, Sdf, Usd, UsdGeom

    out_dir = Path(args.out_dir)
    src_dir = ASTROBEE_DIR.joinpath("source", "meshes")
    parts_dir = out_dir.joinpath("parts")
    parts_dir.mkdir(parents=True, exist_ok=True)

    async def convert(src: Path, dst: Path) -> bool:
        ctx = converter.AssetConverterContext()
        ctx.ignore_animations = True
        ctx.ignore_camera = True
        ctx.ignore_light = True
        ctx.use_meter_as_world_unit = True
        ctx.embed_textures = True
        task = converter.get_instance().create_converter_task(src.as_posix(), dst.as_posix(), None, ctx)
        ok = await task.wait_until_finished()
        if not ok:
            print(f"[ASTROBEE] conversion failed: {src.name}: {task.get_error_message()}", flush=True)
        return bool(ok)

    for name in MESHES:
        future = asyncio.ensure_future(convert(src_dir / f"{name}.dae", parts_dir / f"{name}.usd"))
        while not future.done():
            app.update()
        if not future.result():
            app.close()
            raise SystemExit(1)
        print(f"[ASTROBEE] converted {name}.dae -> parts/{name}.usd", flush=True)

    ## Assembly: the `body` link of the NASA description (visual meshes only)
    path = out_dir / "astrobee.usd"
    stage = Usd.Stage.CreateNew(path.as_posix())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Astrobee")
    stage.SetDefaultPrim(root.GetPrim())

    def add(prim_path: str, mesh: str):
        prim = UsdGeom.Xform.Define(stage, prim_path).GetPrim()
        prim.GetReferences().AddReference(f"./parts/{mesh}.usd")
        # The converter writes each part Y-up: its root prim carries xformOp:orient =
        # Rx(-90 deg) on top of the Z-up COLLADA data. This stage is Z-up like the DAE
        # files, so that rotation is overridden to identity (NASA body frame kept as is)
        prim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatf).Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    add("/Astrobee/body", "body")
    for side in ("port", "stbd"):
        xf = UsdGeom.Xform.Define(stage, f"/Astrobee/pmc_{side}")
        if side == "stbd":
            # URDF rpy (0, pi, pi): R = Rz(pi) Ry(pi) = Rx(pi)
            xf.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))
        for mesh in ("pmc", "pmc_bumper", "pmc_skin_"):
            add(f"/Astrobee/pmc_{side}/{mesh.rstrip('_')}", mesh)
    stage.GetRootLayer().Save()

    ## Check: the assembled robot is the ~0.32 m Astrobee cube in metres
    stage = Usd.Stage.Open(path.as_posix())
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    for p in ("/Astrobee", "/Astrobee/body", "/Astrobee/pmc_port", "/Astrobee/pmc_stbd"):
        box = cache.ComputeWorldBound(stage.GetPrimAtPath(p)).ComputeAlignedRange()
        mn, mx = box.GetMin(), box.GetMax()
        print(f"[ASTROBEE] bbox {p}: min {[round(v, 4) for v in mn]} max {[round(v, 4) for v in mx]} "
              f"size {[round(mx[i] - mn[i], 4) for i in range(3)]} m", flush=True)
    size = cache.ComputeWorldBound(stage.GetPrimAtPath("/Astrobee")).ComputeAlignedRange().GetSize()
    if not all(0.2 < s < 0.5 for s in size) or math.isnan(size[0]):
        print(f"[ASTROBEE] WARNING: unexpected size {list(size)} (expected ~0.32 m per axis)", flush=True)
    print(f"[ASTROBEE] wrote {path}", flush=True)
    app.close()


if __name__ == "__main__":
    main()
