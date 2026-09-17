from dataclasses import MISSING
from typing import Callable, Tuple

from pxr import Usd

from srb.core.sim import SpawnerCfg
from srb.utils.cfg import configclass

from .impl import spawn_particles_grid, spawn_particles_pyramid


@configclass
class ParticlesSpawnerCfg(SpawnerCfg):
    # Distribution
    particle_size: float = 0.01
    particle_spacing: float = 0.0
    ratio: float = 1.0

    ## Particle system
    particle_system_enabled: bool = True
    particle_contact_offset: float | None = None
    max_velocity: float | None = None
    enable_ccd: bool = False
    wind: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Advanced
    contact_offset: float | None = None
    rest_offset: float | None = None
    solid_rest_offset: float | None = None
    fluid_rest_offset: float | None = None
    solver_position_iterations: float = 16
    max_depenetration_velocity: float | None = None
    max_neighborhood: float = 96
    neighborhood_scale: float = 1.01
    global_self_collision_enabled: bool | None = None
    non_particle_collision_enabled: bool | None = None

    ## Particle set
    particle_enabled: bool = True
    self_collision: bool = True
    fluid: bool = True
    # Advanced
    particle_group: int = 0

    ## PBD Material
    density: float = 0.0
    friction: float = 0.2
    damping: float = 0.0
    viscosity: float = 0.0
    cohesion: float = 0.0
    surface_tension: float = 0.0
    drag: float = 0.0
    lift: float = 0.0
    # Advanced
    adhesion: float = 0.0
    particle_adhesion_scale: float = 1.0
    adhesion_offset_scale: float = 0.0
    particle_friction_scale: float = 1.0
    vorticity_confinement: float = 0.0
    cfl_coefficient: float = 1.0
    gravity_scale: float = 1.0

    ## Visuals
    cast_shadows: bool = False

    ## Particle anisotropy (fluid only)
    particle_anisotropy_enabled: bool = True
    anisotropy_scale: float = 1.0
    anisotropy_min: float = 0.2
    anisotropy_max: float = 2.0

    ## Particle smoothing (fluid only)
    particle_smoothing_enabled: bool = True
    smoothing_strength: float = 0.8

    ## Particle isosurface (fluid only)
    isosurface_enabled: bool = False
    isosurface_max_vertices: int = 2**20
    isosurface_max_triangles: int = 2**21
    isosurface_max_subgrids: int = 2**11
    isosurface_grid_spacing: float | None = None
    isosurface_surface_distance: float | None = None
    isosurface_mesh_smoothing_passes: int = 4
    isosurface_mesh_normal_smoothing_passes: int = 4
    isosurface_grid_filtering_passes: str = "GSRS"
    isosurface_grid_smoothing_radius: float | None = None


@configclass
class GridParticlesSpawnerCfg(ParticlesSpawnerCfg):
    func: Callable[..., Usd.Prim] = spawn_particles_grid

    # Distribution
    dim_x: int = MISSING  # type: ignore
    dim_y: int = MISSING  # type: ignore
    dim_z: int = MISSING  # type: ignore

    # Initial velocity
    velocity: Tuple[
        float | Tuple[float, float],
        float | Tuple[float, float],
        float | Tuple[float, float],
    ] = (0.0, 0.0, 0.0)


@configclass
class PyramidParticlesSpawnerCfg(ParticlesSpawnerCfg):
    func: Callable[..., Usd.Prim] = spawn_particles_pyramid

    # Distribution
    dim_x: int = MISSING  # type: ignore
    dim_y: int = MISSING  # type: ignore
    dim_z: int = MISSING  # type: ignore

    # Initial velocity
    velocity: Tuple[
        float | Tuple[float, float],
        float | Tuple[float, float],
        float | Tuple[float, float],
    ] = (0.0, 0.0, 0.0)
