from isaacsim.robot_setup.assembler import AssembledBodies as __AssembledBodies


class AssembledBodies(__AssembledBodies):
    def set_attach_path_root_joints_enabled(self, enabled: bool):
        for root_joint in self.root_joints:
            root_joint.GetProperty("physics:jointEnabled").Set(enabled)
