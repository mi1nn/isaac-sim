"""MOCK ONLY: scripted display signals, not physics or success criteria.

No poses/quaternions are emitted in STEP 2. All numeric ramps are synthetic.
Events depend only on scenario time; no capture/docking thresholds exist here.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Scenario:
    capture_seconds: float = 20.0
    attached_hold_seconds: float = 3.0
    docking_seconds: float = 20.0
    event_seconds: float = 1.0

    def __post_init__(self):
        values = (self.capture_seconds, self.attached_hold_seconds,
                  self.docking_seconds, self.event_seconds)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Mock durations must be finite and positive')
        if self.event_seconds > min(self.attached_hold_seconds, self.docking_seconds):
            raise ValueError('event_seconds must fit within hold and docking durations')

    def sample(self, elapsed):
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError('elapsed must be finite and nonnegative')
        attached = self.capture_seconds
        switch = attached + self.attached_hold_seconds
        docked = switch + self.docking_seconds
        phase = 1 if elapsed < switch else 2
        if elapsed < attached:
            progress = elapsed / self.capture_seconds
            state = ('MEP_SEARCH', 'MEP_TRACKING', 'MEP_APPROACH', 'MEP_ALIGN')[
                min(3, int(progress * 4))]
            state_code = ('MEP_SEARCH', 'MEP_TRACKING', 'MEP_APPROACH', 'MEP_ALIGN').index(state)
            residual = (1.0 - progress) ** 2
            distance, angle, velocity = 0.8 * residual, 35.0 * residual, 0.08 * residual
        elif elapsed < switch:
            state, state_code = 'MEP_ATTACHED', 4
            distance = angle = velocity = 0.0
        elif elapsed < docked:
            progress = (elapsed - switch) / self.docking_seconds
            state = ('DOCK_SEARCH', 'DOCK_PRE_APPROACH', 'DOCK_ALIGN')[min(2, int(progress * 3))]
            state_code = ('DOCK_SEARCH', 'DOCK_PRE_APPROACH', 'DOCK_ALIGN').index(state) + 5
            residual = (1.0 - progress) ** 2
            distance, angle, velocity = 1.2 * residual, 25.0 * residual, 0.06 * residual
        else:
            state, state_code = 'DOCKED', 8
            distance = angle = velocity = 0.0
        return {
            'scenario_elapsed': elapsed,
            'active_position_error': distance,
            # Synthetic components use the unit vector (0.6, 0.0, 0.8).
            'error_x': 0.6 * distance, 'error_y': 0.0, 'error_z': 0.8 * distance,
            'orientation_error': angle, 'relative_velocity': velocity,
            'mission_phase': 'MEP_CAPTURE' if phase == 1 else 'SATELLITE_DOCKING',
            'mission_phase_code': phase, 'mission_state': state,
            'mission_state_code': state_code,
            'active_target_type': 'MEP_ATTACH_POINT' if phase == 1 else 'SATELLITE_DOCK_POINT',
            'tracking_valid': True,
            'capture_event': int(attached <= elapsed < attached + self.event_seconds),
            'docking_start_event': int(switch <= elapsed < switch + self.event_seconds),
            'docking_event': int(docked <= elapsed < docked + self.event_seconds),
            'capture_success': elapsed >= attached, 'docking_success': elapsed >= docked,
            'mock_only': True,
        }
