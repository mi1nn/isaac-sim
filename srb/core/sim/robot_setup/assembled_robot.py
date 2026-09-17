from isaacsim.robot_setup.assembler import AssembledRobot as __AssembledRobot

from .assembled_bodies import AssembledBodies


class AssembledRobot(__AssembledRobot):
    def __init__(self, assembled_robots: AssembledBodies):
        self.assembled_robots = assembled_robots

    def set_attach_path_root_joints_enabled(self, enabled: bool):
        self.assembled_robots.set_attach_path_root_joints_enabled(enabled)
