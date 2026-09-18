"""P0 MEP composition: one rigid body, handle-centred coordinates, original USD intact."""
from isaaclab.sim import clone
from pxr import Usd, UsdGeom, UsdPhysics
from simforge.integrations.isaaclab.spawner.from_files.impl import spawn_from_usd


@clone
def spawn_mep(prim_path, cfg, translation=None, orientation=None, **kwargs):
    prim = spawn_from_usd(prim_path, cfg, translation, orientation, **kwargs)
    stage = prim.GetStage()
    content = stage.GetPrimAtPath(prim.GetPath().AppendPath("content"))
    handle = stage.GetPrimAtPath(prim.GetPath().AppendPath("content/geometry/gripper_fixture/Cylinder_01"))
    if not handle.IsValid():
        raise ValueError(f"MEP grasp handle missing below {prim_path}; check space_asset/mep.usd references")
    # Source: authored model coordinates; target: MEP rigid-body frame. Metres.
    # Move the content, never the rigid root, so init_state.pos denotes the handle.
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    centre = bounds.ComputeRelativeBound(handle, prim).ComputeAlignedRange().GetMidpoint()
    UsdGeom.Xformable(content).AddTranslateOp().Set(-centre)
    colliders = 0
    for child in Usd.PrimRange(content):
        if child.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError(f"MEP must have a single rigid body; nested body: {child.GetPath()}")
        if child.IsA(UsdGeom.Gprim):
            UsdPhysics.CollisionAPI.Apply(child).CreateCollisionEnabledAttr(True)
            if child.IsA(UsdGeom.Mesh):
                UsdPhysics.MeshCollisionAPI.Apply(child).CreateApproximationAttr("convexDecomposition")
            colliders += 1
        if child.GetTypeName().endswith("Light"):
            child.SetActive(False)
    if not colliders:
        raise ValueError(f"MEP has no collision geometry: {prim_path}")
    print(f"SATELLITE_MISSION_ASSET handle_offset={tuple(centre)} colliders={colliders}", flush=True)
    return prim
