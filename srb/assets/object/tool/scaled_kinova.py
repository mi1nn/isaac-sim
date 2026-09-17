"""Spawn-time physical scaling for the enlarged Kinova (source USD stays intact)."""

from isaaclab.sim import clone
from pxr import Gf, Usd, UsdPhysics
from simforge.integrations.isaaclab.spawner.from_files.impl import spawn_from_usd

KINOVA_SCALE = 4.2
KINOVA_TCP_DISTANCE = 0.16 * KINOVA_SCALE


@clone
def spawn_scaled_kinova(prim_path, cfg, translation=None, orientation=None, **kwargs):
    prim = spawn_from_usd(prim_path, cfg, translation, orientation, **kwargs)
    scale = cfg.scale or (1.0, 1.0, 1.0)
    if any(abs(s - KINOVA_SCALE) > 1e-6 for s in scale):
        raise ValueError("Kinova300Large TCP and dynamics require uniform scale=4.2")
    # Read unmodified source masses: do not compound scaling if spawn is repeated.
    source = Usd.Stage.Open(cfg.usd_path)
    source_root = source.GetDefaultPrim()
    for body in Usd.PrimRange(prim):
        if not body.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        relative = body.GetPath().MakeRelativePath(prim.GetPath())
        original = source.GetPrimAtPath(source_root.GetPath().AppendPath(relative))
        original_mass = UsdPhysics.MassAPI(original).GetMassAttr().Get()
        mass = UsdPhysics.MassAPI.Apply(body)
        mass.CreateMassAttr(max(float(original_mass) * KINOVA_SCALE**3, 0.01))
        # Zero asks PhysX to calculate inertia from the *scaled* collision shapes
        # and explicit new mass, avoiding double application of USD scale.
        if body.GetName() == "kinova300_end_effector":
            # Virtual TCP link has no collision geometry. Give its tiny mass a
            # finite inertia instead of the original 1e-7 kg / zero inertia.
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-5))
        else:
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(0.0))
    return prim
