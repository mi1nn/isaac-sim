# Graph Report - srb  (2026-09-18)

## Corpus Check
- 362 files · ~106,729 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 1 file(s) not represented in the graph (top: .typed 1)

## Summary
- 2452 nodes · 4866 edges · 155 communities (112 shown, 11 thin omitted)
- Extraction: 93% EXTRACTED · 7% INFERRED · 0% AMBIGUOUS · INFERRED: 324 edges (avg confidence: 0.89)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- Mobile Robot Assets
- Core Action Group
- Mobile Aerial Core
- Tasks Manipulation Capture
- Action Mobile Core
- Robot Mobile Manipulation
- Integrations Dreamer Wrapper
- Manipulation Robot Assets
- Manipulation Mobile Tasks
- Sim Core Spawners
- Action Core Term
- Sim Ros Traj
- Waypoint Tasks Mobile
- Collection Tasks Sample
- Utils Hydra Real
- Sim Interfaces Real
- Tasks Mobile Tracking
- Object Assets Rock
- Core Domain
- Sim Core Hardware
- Tool Object Assets
- Sim Waypoint Navigation
- Core Mdp Events
- Tasks Mobile Manipulation
- Ground Mobile Manipulation
- Sb3 Wrapper Integrations
- Asset Core Object
- Interfaces Manual Math
- Sim Ros Hardware
- Tasks Manipulation Mobile
- Peg Tasks Manipulation
- Core Visuals Post
- Sim Core Interfaces
- Core Asset
- Asset Core Robot
- Sim Moveit Servo
- Direct Core Env
- Sim Image Ros
- Asset Core Scenery
- Core Env Extension
- Utils Main Integrations
- Orbital Tasks Mobile
- Core Env
- Panel Tasks Manipulation
- Interfaces Haptic Teleop
- Tasks Excavation Manipulation
- Utils Cache Core
- Interfaces Interface Ros
- Main
- Tasks Manipulation
- Tasks Screwdriving Manipulation
- Ground Tasks Mobile
- Tasks Mobile Landing
- Orbital Tasks Mobile
- Rendezvous Tasks Mobile
- Wrappers Smoothing Action
- Core Asset
- Sim Setup Core
- Mobile Ground Core
- Sim Hardware Interfaces
- Sim Template Hardware
- Interfaces Teleop Keyboard
- Math Utils Interfaces
- Sim Ros Hardware
- Peg Tasks Manipulation
- Utils
- Core Env Event
- Interfaces Combined Teleop
- Interfaces Ros Teleop
- Main
- Interfaces Interface Gui
- Skrl Wrapper Integrations
- Interfaces Enums
- Main Utils Ros
- Sim Moveit Hardware
- Interfaces Spacemouse Teleop
- Tasks Mobile Manipulation
- Scenery Assets Facility
- Core Env Manipulation
- Orbital Mobile Core
- Sim Vel Ros
- Main
- Utils Ros Isaacsim
- Process Utils Main
- Core Asset Scenery
- Direct Core Env
- Managed Core Env
- Sim Ros Hardware
- Main Isaacsim Utils
- Utils Core Sim
- Core Asset Variant
- Direct Core Marl
- Manager Exp Sb3
- Sbx Main Integrations
- Interfaces Interface Ros
- Sim Ros Display
- Pedestal Object Assets
- Asset Core Frame
- Interfaces Interface Gui
- Core Manager Action
- Core Asset Variant
- Tasks Excavation Manipulation
- Tasks Manipulation
- Peg Tasks Manipulation
- Peg Tasks Manipulation
- Collection Tasks Sample
- Tasks Screwdriving Manipulation
- Tasks Mobile Aerial
- Ground Tasks Mobile
- Ground Tasks Mobile
- Tasks Mobile Manipulation
- Orbital Tasks Mobile
- Rendezvous Tasks Mobile
- Waypoint Tasks Mobile
- Waypoint Tasks Mobile
- Waypoint Tasks Orbital
- Importer Utils
- Utils Ros
- Utils Ros
- Payload Assets Object
- Core Asset
- Nucleus Utils
- Tracing Utils

## God Nodes (most connected - your core abstractions)
1. `StepReturn` - 54 edges
2. `HardwareInterface` - 50 edges
3. `Object` - 38 edges
4. `Domain` - 34 edges
5. `VisualExt` - 34 edges
6. `Asset` - 32 edges
7. `BaseEnvCfg` - 28 edges
8. `ActionGroup` - 27 edges
9. `rotmat_to_rot6d()` - 27 edges
10. `DirectEnv` - 26 edges

## Surprising Connections (you probably didn't know these)
- `hydra_main()` --calls--> `RealEnvGenerator`  [INFERRED]
  __main__.py → interfaces/sim_to_real/core/generator.py
- `parse_cli_args()` --calls--> `with_rich()`  [INFERRED]
  __main__.py → utils/tracing.py
- `run_agent_with_env()` --calls--> `update_offline_srb_cache()`  [INFERRED]
  __main__.py → utils/cache.py
- `run_agent_with_env()` --calls--> `last_logdir()`  [INFERRED]
  __main__.py → utils/cfg.py
- `run_agent_with_env()` --calls--> `new_logdir()`  [INFERRED]
  __main__.py → utils/cfg.py

## Import Cycles
- 3-file cycle: `core/sim/spawners/shapes/extras/__init__.py -> core/sim/spawners/shapes/extras/cfg.py -> core/sim/spawners/shapes/extras/impl.py -> core/sim/spawners/shapes/extras/__init__.py`

## Communities (155 total, 11 thin omitted)

### Community 0 - "Mobile Robot Assets"
Cohesion: 0.06
Nodes (41): AnymalC, AnymalD, RandomAnymalQuadruped, Cassie, Crazyflie, Ingenuity, ApolloLander, PeregrineLander (+33 more)

### Community 1 - "Core Action Group"
Cohesion: 0.05
Nodes (36): ActionGroup, ActionGroupRegistry, configclass, Tensor, BodyAccelerationActionGroup, BodyAccelerationRelativeActionGroup, configclass, Tensor (+28 more)

### Community 2 - "Mobile Aerial Core"
Cohesion: 0.07
Nodes (35): AerialEnv, AerialEnvCfg, AerialEventCfg, AerialSceneCfg, configclass, AerialEnvVisualExtCfg, configclass, MobileEnv (+27 more)

### Community 3 - "Tasks Manipulation Capture"
Cohesion: 0.06
Nodes (30): IntEnum, Pose, Stage, capture_cylinder_pose(), CaptureCfg, CaptureManager, CaptureState, configclass (+22 more)

### Community 4 - "Action Mobile Core"
Cohesion: 0.05
Nodes (24): MulticopterBodyAccelerationAction, MulticopterBodyAccelerationActionCfg, ActionTerm, ActionTermCfg, configclass, Tensor, ActionTerm, ActionTermCfg (+16 more)

### Community 5 - "Robot Mobile Manipulation"
Cohesion: 0.07
Nodes (20): GenericAerialManipulator, GenericGroundManipulator, GenericOrbitalManipulator, Humanoid21, Humanoid28, UnitreeG1, UnitreeH1, AerialManipulator (+12 more)

### Community 6 - "Integrations Dreamer Wrapper"
Cohesion: 0.05
Nodes (18): Driver, DriverParallelEnv, eval_only(), SimulationApp, make_replay(), Path, SimulationApp, run() (+10 more)

### Community 7 - "Manipulation Robot Assets"
Cohesion: 0.08
Nodes (26): Canadarm3, Franka, KinovaGen3n7, KinovaJ2n6s, KinovaJ2n7s, SOArm100D5, SOArm100D7, UnitreeZ1 (+18 more)

### Community 8 - "Manipulation Mobile Tasks"
Cohesion: 0.08
Nodes (27): OrbitalManipulationEnv, OrbitalManipulationEnvCfg, OrbitalManipulationEventCfg, OrbitalManipulationSceneCfg, configclass, OrbitalManipulationEnvVisualExtCfg, configclass, InitialStateCfg (+19 more)

### Community 9 - "Sim Core Spawners"
Cohesion: 0.08
Nodes (34): GridParticlesSpawnerCfg, ParticlesSpawnerCfg, configclass, SpawnerCfg, PyramidParticlesSpawnerCfg, _create_particles_grid(), clone, Prim (+26 more)

### Community 10 - "Action Core Term"
Cohesion: 0.06
Nodes (20): BodyAccelerationAction, BodyAccelerationActionCfg, ActionTerm, ActionTermCfg, configclass, Tensor, DummyAction, DummyActionCfg (+12 more)

### Community 11 - "Sim Ros Traj"
Cohesion: 0.12
Nodes (26): BasePatternCfg, CapsulePatternCfg, CapsulePatternState, CirclePatternCfg, CirclePatternState, Config, _get_transform_msg(), LemniscatePatternCfg (+18 more)

### Community 12 - "Waypoint Tasks Mobile"
Cohesion: 0.13
Nodes (22): EventCfg, LocomotionEventCfg, LocomotionSceneCfg, LocomotionTask, LocomotionTaskCfg, configclass, EventCfg, configclass (+14 more)

### Community 13 - "Collection Tasks Sample"
Cohesion: 0.13
Nodes (21): BaseModel, InitialStateCfg, SceneEntityCfg, TexResConfig, SampleCfg, select_sample(), EventCfg, MultiEventCfg (+13 more)

### Community 14 - "Utils Hydra Real"
Cohesion: 0.09
Nodes (31): load_cfg_from_registry(), Any, Any, Replace string representations of slices with slice objects in a dictionary.…, Replace slice objects with their string representations in a dictionary. Args:…, replace_slices_with_strings(), replace_strings_with_slices(), Any (+23 more)

### Community 15 - "Sim Interfaces Real"
Cohesion: 0.10
Nodes (16): dtype, EnvSpec, EnvInfo, Any, BaseModel, ndarray, Path, Space (+8 more)

### Community 16 - "Tasks Mobile Tracking"
Cohesion: 0.14
Nodes (18): _compute_step_return(), EventCfg, LocomotionEventCfg, LocomotionSceneCfg, LocomotionTask, LocomotionTaskCfg, configclass, configclass (+10 more)

### Community 17 - "Object Assets Rock"
Cohesion: 0.13
Nodes (21): Bag, BeneficiationUnit, BoltM8, NutM8, JugglingBall, Hole, Peg, ProfileHole (+13 more)

### Community 18 - "Core Domain"
Cohesion: 0.06
Nodes (17): Domain, Enum, Self, str, Range of Solar light intensity in W/m² calculated as the intensity ±…, Angular diameter of the Solar light source in degrees. - Earth | Mars: Taken at…, Variation of the angular diameter of the Solar light source in degrees., Range of the angular diameter of the Solar light source in degrees calculated… (+9 more)

### Community 19 - "Sim Core Hardware"
Cohesion: 0.07
Nodes (3): HardwareInterface, Any, ndarray

### Community 20 - "Tool Object Assets"
Cohesion: 0.14
Nodes (22): AllegroHand, FrankaHand, Kinova300, Kinova300Large, Kinova 3-finger gripper scaled up 4.2x so that it can wrap the MEP handle.…, RobotiqHandE, RandomScoop, Scoop (+14 more)

### Community 21 - "Sim Waypoint Navigation"
Cohesion: 0.10
Nodes (26): _generate_summary_plots_and_table(), _load_and_group_experiments(), main(), _plot_trajectories_with_confidence(), _process_file(), Namespace, Node, Path (+18 more)

### Community 22 - "Core Mdp Events"
Cohesion: 0.15
Nodes (27): follow_xform_orientation_linear_trajectory(), offset_pos_natural(), offset_pose_natural(), SceneEntityCfg, Tensor, randomize_command(), randomize_gravity_uniform(), randomize_pos() (+19 more)

### Community 23 - "Tasks Mobile Manipulation"
Cohesion: 0.08
Nodes (11): Tensor, Tensor, Tensor, Tensor, Tensor, Tensor, Tensor, Tensor (+3 more)

### Community 24 - "Ground Mobile Manipulation"
Cohesion: 0.16
Nodes (15): GroundManipulationEnv, GroundManipulationEnvCfg, GroundManipulationEventCfg, GroundManipulationSceneCfg, configclass, GroundManipulationEnvVisualExtCfg, configclass, EventCfg (+7 more)

### Community 25 - "Sb3 Wrapper Integrations"
Cohesion: 0.10
Nodes (7): Any, ndarray, Tensor, VecEnv, Sb3EnvWrapper, VecEnvObs, VecEnvStepReturn

### Community 26 - "Asset Core Object"
Cohesion: 0.11
Nodes (7): Light, ObjectRegistry, ObjectType, Enum, Self, str, Payload

### Community 27 - "Interfaces Manual Math"
Cohesion: 0.11
Nodes (9): ManualJointControlInterface, Tensor, Read open/close joint commands off the gripper's binary action term., Keyboard-driven joint-level control of a manipulator and its gripper. Commands…, Write the current targets to the articulations. Call every physics step., Adopt the measured joint positions as targets (stops any commanded motion)., Adopt the configured initial joint positions as targets., Restrict control to the arm's own joints. The gripper is attached with a fixed… (+1 more)

### Community 28 - "Sim Ros Hardware"
Cohesion: 0.13
Nodes (8): PositionRepresentation, Any, Enum, ndarray, Self, RosTfInterface, RosTfInterfaceCfg, RotationRepresentation

### Community 29 - "Tasks Manipulation Mobile"
Cohesion: 0.10
Nodes (17): NamedTuple, _compute_step_return(), script, Tensor, _compute_step_return(), script, Tensor, _compute_step_return() (+9 more)

### Community 30 - "Peg Tasks Manipulation"
Cohesion: 0.20
Nodes (15): EventCfg, MultiEventCfg, MultiSceneCfg, MultiTask, MultiTaskCfg, configclass, configclass, SceneCfg (+7 more)

### Community 31 - "Core Visuals Post"
Cohesion: 0.14
Nodes (20): AutoExposureCfg, ChromaticAberrationCfg, DepthOfFieldCfg, FogCfg, LensFlareCfg, MotionBlurCfg, configclass, ReshadeCfg (+12 more)

### Community 32 - "Sim Core Interfaces"
Cohesion: 0.16
Nodes (7): device, Any, ndarray, Space, Tensor, RealEnv, SupportsFloat

### Community 33 - "Core Asset"
Cohesion: 0.13
Nodes (7): ArticulationCfg, Asset, Any, BaseModel, SpawnerCfg, This method allows for additional scene setup that is specific to the asset. It…, PositiveFloat

### Community 34 - "Asset Core Robot"
Cohesion: 0.16
Nodes (6): Robot, RobotRegistry, Enum, Self, str, RobotType

### Community 35 - "Sim Moveit Servo"
Cohesion: 0.13
Nodes (6): MoveitServo, MoveitServoCfg, Enum, ndarray, Self, RotationRepresentation

### Community 36 - "Direct Core Env"
Cohesion: 0.18
Nodes (7): DirectEnv, _flatten_observations(), script, Tensor, Put every attached body exactly on its mount frame. An assembly is a plain…, _sum_rewards(), __DirectRLEnv

### Community 37 - "Sim Image Ros"
Cohesion: 0.12
Nodes (13): ndarray, Converts a sensor_msgs/Image to a numpy array without cv_bridge., Constructs and returns the combined image observation. Images are ordered by…, Defines the observation space for the combined image., Configuration for the ROS image interface., A hardware interface for receiving and processing image data from ROS topics.…, Initializes the ROS image interface., Starts the ROS image interface, creating subscribers for each topic. (+5 more)

### Community 38 - "Asset Core Scenery"
Cohesion: 0.18
Nodes (5): ExtravehicularScenery, IntravehicularScenery, Scenery, SceneryRegistry, Subterrane

### Community 39 - "Core Env Extension"
Cohesion: 0.22
Nodes (14): configclass, VisualExtCfg, construct_observation(), process_depth_f32(), process_depth_u8(), process_img_f32_as_f32(), process_img_f32_as_u8(), process_img_u8_as_f32() (+6 more)

### Community 40 - "Utils Main Integrations"
Cohesion: 0.14
Nodes (17): Env, Path, SimulationApp, run(), Path, SimulationApp, run(), _identify_config() (+9 more)

### Community 41 - "Orbital Tasks Mobile"
Cohesion: 0.19
Nodes (12): InitialStateCfg, RigidObjectCfg, TexResConfig, select_obstacle(), EventCfg, configclass, SceneCfg, Task (+4 more)

### Community 42 - "Core Env"
Cohesion: 0.19
Nodes (7): AssetBaseCfg, BaseEnvCfg, _recursive_impl(), _recursive_asset_impl(), _recursive_marker_impl(), _recursive_impl(), configclass

### Community 44 - "Panel Tasks Manipulation"
Cohesion: 0.20
Nodes (12): PanelCfg, BaseModel, InitialStateCfg, select_solar_panel(), EventCfg, configclass, SceneCfg, Task (+4 more)

### Community 45 - "Interfaces Haptic Teleop"
Cohesion: 0.11
Nodes (5): HapticROSTeleopInterface, DeviceBase, ndarray, Node, Tensor

### Community 46 - "Tasks Excavation Manipulation"
Cohesion: 0.20
Nodes (9): EventCfg, configclass, SceneCfg, Task, TaskCfg, configclass, Tensor, VisualTask (+1 more)

### Community 47 - "Utils Cache Core"
Cohesion: 0.21
Nodes (14): list_registered(), read_offline_srb_env_cache(), read_offline_srb_hardware_interface_cache(), read_offline_srb_object_cache(), read_offline_srb_robot_cache(), read_offline_srb_scenery_cache(), update_offline_srb_cache(), update_offline_srb_env_cache() (+6 more)

### Community 48 - "Interfaces Interface Ros"
Cohesion: 0.16
Nodes (3): ActionTerm, Node, RosInterface

### Community 49 - "Main"
Cohesion: 0.20
Nodes (14): AutoNamespaceTaskAction, AutoRealNamespaceTaskAction, enter_repl(), ExplicitAction, launch_gui(), main(), impl(), parse_cli_args() (+6 more)

### Community 50 - "Tasks Manipulation"
Cohesion: 0.22
Nodes (9): EventCfg, configclass, SceneCfg, Task, TaskCfg, configclass, Tensor, VisualTask (+1 more)

### Community 51 - "Tasks Screwdriving Manipulation"
Cohesion: 0.22
Nodes (9): EventCfg, configclass, SceneCfg, Task, TaskCfg, configclass, Tensor, VisualTask (+1 more)

### Community 52 - "Ground Tasks Mobile"
Cohesion: 0.22
Nodes (9): EventCfg, configclass, SceneCfg, Task, TaskCfg, configclass, Tensor, VisualTask (+1 more)

### Community 53 - "Tasks Mobile Landing"
Cohesion: 0.22
Nodes (9): EventCfg, configclass, SceneCfg, Task, TaskCfg, configclass, Tensor, VisualTask (+1 more)

### Community 54 - "Orbital Tasks Mobile"
Cohesion: 0.22
Nodes (9): EventCfg, configclass, SceneCfg, Task, TaskCfg, configclass, Tensor, VisualTask (+1 more)

### Community 55 - "Rendezvous Tasks Mobile"
Cohesion: 0.22
Nodes (9): EventCfg, configclass, SceneCfg, Task, TaskCfg, configclass, Tensor, VisualTask (+1 more)

### Community 56 - "Wrappers Smoothing Action"
Cohesion: 0.24
Nodes (9): ActionWrapper, ActionSmoothingWrapper, FillMethod, maybe_wrap_action_smoothing(), Any, Enum, ndarray, Tensor (+1 more)

### Community 57 - "Core Asset"
Cohesion: 0.17
Nodes (5): AssetRegistry, AssetType, Enum, Self, str

### Community 58 - "Sim Setup Core"
Cohesion: 0.29
Nodes (7): AssembledBodies, AssembledRobot, BaseModel, ndarray, RobotAssembler, RobotAssemblerCfg, FixedJoint

### Community 59 - "Mobile Ground Core"
Cohesion: 0.25
Nodes (7): GroundEnv, GroundEnvCfg, GroundEventCfg, GroundSceneCfg, configclass, GroundEnvVisualExtCfg, configclass

### Community 60 - "Sim Hardware Interfaces"
Cohesion: 0.18
Nodes (3): DummyInterface, DummyInterfaceCfg, ndarray

### Community 61 - "Sim Template Hardware"
Cohesion: 0.18
Nodes (4): Any, ndarray, TemplateInterface, TemplateInterfaceCfg

### Community 62 - "Interfaces Teleop Keyboard"
Cohesion: 0.17
Nodes (4): EventOmniKeyboardTeleopInterface, OmniKeyboardTeleopInterface, ndarray, __Se3Keyboard

### Community 63 - "Math Utils Interfaces"
Cohesion: 0.24
Nodes (12): Print the physical evidence that distinguishes a grasp from mere overlap., script, Tensor, quat_to_rot6d(), rad_to_deg(), Returns wxyz quaternion from roll-pitch-yaw angles. Accepts either separate…, rpy_to_quat(), slerp() (+4 more)

### Community 65 - "Peg Tasks Manipulation"
Cohesion: 0.20
Nodes (11): HoleCfg, PegCfg, PegInHoleCfg, Any, BaseModel, InitialStateCfg, TexResConfig, select_peg_in_hole_assembly() (+3 more)

### Community 66 - "Utils"
Cohesion: 0.20
Nodes (9): TextIO, Any, StderrSuppress, StdoutSuppress, StreamRedirect, suppress_stderr(), wrapper(), suppress_stdout() (+1 more)

### Community 67 - "Core Env Event"
Cohesion: 0.24
Nodes (6): # NOTE: This used to fall back to "cpu" for every other scene. Because, BaseEventCfg, configclass, BaseSceneCfg, configclass, InteractiveSceneCfg

### Community 68 - "Interfaces Combined Teleop"
Cohesion: 0.18
Nodes (4): CombinedTeleopInterface, DeviceBase, ndarray, Tensor

### Community 69 - "Interfaces Ros Teleop"
Cohesion: 0.15
Nodes (4): DeviceBase, ndarray, Node, ROSTeleopInterface

### Community 70 - "Main"
Cohesion: 0.22
Nodes (13): eval_agent(), manual_agent(), cb_reset(), random_agent(), Drive the manipulator joint-by-joint from the keyboard. Unlike the other…, ros_agent(), agent_impl(), agent_impl() (+5 more)

### Community 71 - "Interfaces Interface Gui"
Cohesion: 0.17
Nodes (4): Bool, Float64, GuiInterface, Node

### Community 72 - "Skrl Wrapper Integrations"
Cohesion: 0.27
Nodes (5): Any, Space, Tensor, SkrlEnvWrapper, IsaacLabWrapper

### Community 73 - "Interfaces Enums"
Cohesion: 0.23
Nodes (5): InterfaceType, Enum, Self, str, TeleopDeviceType

### Community 74 - "Main Utils Ros"
Cohesion: 0.22
Nodes (11): hydra_main(), __init__(), step(), __init__(), step(), __wrap_env_in_performance_test(), __init__(), __perf_report() (+3 more)

### Community 75 - "Sim Moveit Hardware"
Cohesion: 0.26
Nodes (3): MoveitGripper, MoveitGripperCfg, ndarray

### Community 76 - "Interfaces Spacemouse Teleop"
Cohesion: 0.18
Nodes (4): DeviceBase, ndarray, SpacemouseTeleopInterface, SpaceNavigator

### Community 77 - "Tasks Mobile Manipulation"
Cohesion: 0.15
Nodes (10): _compute_step_return(), script, Tensor, _compute_step_return(), script, Tensor, _compute_step_return(), script (+2 more)

### Community 78 - "Scenery Assets Facility"
Cohesion: 0.30
Nodes (6): Lunalab, Oberpfaffenhofen, GroundPlane, MarsSurface, MoonSurface, Terrain

### Community 79 - "Core Env Manipulation"
Cohesion: 0.42
Nodes (7): ManipulationEnv, ManipulationEnvCfg, ManipulationEventCfg, ManipulationSceneCfg, configclass, ManipulationEnvVisualExtCfg, configclass

### Community 80 - "Orbital Mobile Core"
Cohesion: 0.36
Nodes (7): OrbitalEnv, OrbitalEnvCfg, OrbitalEventCfg, OrbitalSceneCfg, configclass, OrbitalEnvVisualExtCfg, configclass

### Community 81 - "Sim Vel Ros"
Cohesion: 0.24
Nodes (3): ndarray, RosCmdVelInterface, RosCmdVelInterfaceCfg

### Community 82 - "Main"
Cohesion: 0.23
Nodes (6): EntityToList, Lang, Enum, Self, str, SupportedAlgo

### Community 83 - "Utils Ros Isaacsim"
Cohesion: 0.29
Nodes (11): get_isaacsim_path(), _append_ld_library_path(), enable_ros2_bridge(), _get_ros2_bridge_dir(), get_transform_broadcasters(), _prepend_sys_path(), Path, Make ROS 2 importable from inside Isaac Sim, whichever way it is available.… (+3 more)

### Community 84 - "Process Utils Main"
Cohesion: 0.25
Nodes (10): agent_impl(), run_agent_with_env(), _find_stale_sessions(), install_session_signal_handlers(), _is_python_sh(), _on_sigtstp(), Signal handling that keeps an Isaac Sim session from outliving its terminal.…, Route SIGTERM/SIGHUP to the Ctrl+C shutdown path and explain Ctrl+Z. Must be… (+2 more)

### Community 85 - "Core Asset Scenery"
Cohesion: 0.24
Nodes (4): Enum, Self, str, SceneryType

### Community 86 - "Direct Core Env"
Cohesion: 0.27
Nodes (5): DirectEnvCfg, configclass, __PostInitCaller, __DirectRLEnvCfg, type

### Community 87 - "Managed Core Env"
Cohesion: 0.36
Nodes (5): ManagedEnvCfg, configclass, ManagedEnv, __ManagerBasedRLEnv, __ManagerBasedRLEnvCfg

### Community 88 - "Sim Ros Hardware"
Cohesion: 0.31
Nodes (3): ndarray, RosImu, RosImuCfg

### Community 89 - "Main Isaacsim Utils"
Cohesion: 0.22
Nodes (8): generate_real_agent(), hydra_main(), _generate_real_agent_subprocess(), real_agent_impl(), run_real_agent_with_env(), hydra_main(), get_isaacsim_python(), hide_isaacsim_ui()

### Community 90 - "Utils Core Sim"
Cohesion: 0.33
Nodes (4): convert_to_snake_case(), resolve_env_prim_path(), sanitize_action_term_name(), sanitize_cam_name()

### Community 91 - "Core Asset Variant"
Cohesion: 0.29
Nodes (3): AssetVariant, Enum, str

### Community 92 - "Direct Core Marl"
Cohesion: 0.57
Nodes (3): DirectMarlEnvCfg, configclass, DirectMarlEnv

### Community 93 - "Manager Exp Sb3"
Cohesion: 0.36
Nodes (3): ExperimentManager, Any, VecEnv

### Community 94 - "Sbx Main Integrations"
Cohesion: 0.32
Nodes (5): MergeDictObsWrapper, run(), ObservationWrapper, ObsType, WrapperObsType

### Community 95 - "Interfaces Interface Ros"
Cohesion: 0.32
Nodes (3): Any, Tensor, Time

### Community 96 - "Sim Ros Display"
Cohesion: 0.32
Nodes (5): main(), Node, A ROS 2 node that subscribes to the transform (TF) of a specific frame and…, Main logic of the node, executed at a fixed rate by the timer. It looks up the…, TfToPathNode

### Community 97 - "Pedestal Object Assets"
Cohesion: 0.52
Nodes (4): IndustrialPedestal100, IndustrialPedestal25, IndustrialPedestal50, Pedestal

### Community 98 - "Asset Core Frame"
Cohesion: 0.43
Nodes (4): Frame, BaseModel, BaseModel, Transform

### Community 105 - "Tasks Excavation Manipulation"
Cohesion: 0.50
Nodes (3): _compute_step_return(), script, Tensor

### Community 106 - "Tasks Manipulation"
Cohesion: 0.50
Nodes (3): _compute_step_return(), script, Tensor

### Community 107 - "Peg Tasks Manipulation"
Cohesion: 0.50
Nodes (3): _compute_step_return(), script, Tensor

### Community 108 - "Peg Tasks Manipulation"
Cohesion: 0.50
Nodes (3): _compute_step_return(), script, Tensor

### Community 109 - "Collection Tasks Sample"
Cohesion: 0.50
Nodes (3): _compute_step_return(), script, Tensor

### Community 110 - "Tasks Screwdriving Manipulation"
Cohesion: 0.50
Nodes (3): _compute_step_return(), script, Tensor

### Community 111 - "Tasks Mobile Aerial"
Cohesion: 0.50
Nodes (3): _compute_step_return(), script, Tensor

### Community 112 - "Ground Tasks Mobile"
Cohesion: 0.50
Nodes (3): _compute_step_return(), script, Tensor

### Community 113 - "Ground Tasks Mobile"
Cohesion: 0.50
Nodes (3): _compute_step_return(), script, Tensor

### Community 114 - "Tasks Mobile Manipulation"
Cohesion: 0.50
Nodes (3): _compute_step_return(), script, Tensor

### Community 115 - "Orbital Tasks Mobile"
Cohesion: 0.50
Nodes (3): _compute_step_return(), script, Tensor

### Community 116 - "Rendezvous Tasks Mobile"
Cohesion: 0.50
Nodes (3): _compute_step_return(), script, Tensor

### Community 117 - "Waypoint Tasks Mobile"
Cohesion: 0.50
Nodes (3): _compute_step_return(), script, Tensor

### Community 118 - "Waypoint Tasks Mobile"
Cohesion: 0.50
Nodes (3): _compute_step_return(), script, Tensor

### Community 119 - "Waypoint Tasks Orbital"
Cohesion: 0.50
Nodes (3): _compute_step_return(), script, Tensor

## Knowledge Gaps
- **1 isolated node(s):** `Config`
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 593 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **11 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Asset` connect `Core Asset` to `Asset Core Robot`, `Asset Core Scenery`, `Core Asset Variant`, `Core Env`, `Object Assets Rock`, `Core Domain`, `Core Asset`, `Core Asset Variant`, `Core Asset`?**
  _High betweenness centrality (0.149) - this node is a cross-community bridge._
- **Why does `HardwareInterface` connect `Sim Core Hardware` to `Sim Core Interfaces`, `Sim Ros Hardware`, `Sim Moveit Servo`, `Sim Image Ros`, `Main Utils Ros`, `Sim Moveit Hardware`, `Sim Ros Hardware`, `Sim Interfaces Real`, `Sim Vel Ros`, `Sim Ros Hardware`, `Utils Core Sim`, `Sim Hardware Interfaces`, `Sim Template Hardware`?**
  _High betweenness centrality (0.121) - this node is a cross-community bridge._
- **Why does `BaseEnvCfg` connect `Core Env` to `Core Asset`, `Core Env Event`, `Core Env Extension`, `Core Env Manipulation`, `Core Domain`, `Direct Core Env`, `Managed Core Env`, `Direct Core Marl`?**
  _High betweenness centrality (0.118) - this node is a cross-community bridge._
- **Are the 3 inferred relationships involving `Object` (e.g. with `AssetRegistry` and `AssetType`) actually correct?**
  _`Object` has 3 INFERRED edges - model-reasoned connections that need verification._
- **Are the 16 inferred relationships involving `Domain` (e.g. with `MarsSurface` and `MoonSurface`) actually correct?**
  _`Domain` has 16 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Config` to the rest of the system?**
  _1 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Mobile Robot Assets` be split into smaller, more focused modules?**
  _Cohesion score 0.05661005661005661 - nodes in this community are weakly interconnected._