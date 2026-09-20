import math
import pytest
from mep_monitor.mock_scenario import Scenario


def test_two_phases_and_event_boundaries():
    scenario = Scenario(2.0, 1.0, 2.0, 0.5)
    samples = [scenario.sample(t) for t in (0, 1, 2, 2.5, 3, 4, 5, 5.5, 100)]
    assert samples[0]['active_position_error'] > samples[1]['active_position_error'] > 0
    assert samples[2]['mission_state'] == 'MEP_ATTACHED'
    assert samples[2]['capture_event'] == 1
    assert samples[3]['capture_event'] == 0
    assert samples[3]['mission_phase'] == 'MEP_CAPTURE'
    assert samples[4]['mission_phase'] == 'SATELLITE_DOCKING'
    assert samples[4]['docking_start_event'] == 1
    assert samples[4]['active_position_error'] > samples[5]['active_position_error'] > 0
    assert samples[6]['docking_event'] == 1
    assert samples[7]['docking_event'] == 0
    assert samples[-1]['mission_state'] == 'DOCKED'
    assert samples[-1]['capture_success'] and samples[-1]['docking_success']
    for sample in samples:
        assert math.hypot(sample['error_x'], sample['error_y'], sample['error_z']) == pytest.approx(sample['active_position_error'])
        assert sample['mock_only']


@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf')])
def test_invalid_duration(value):
    with pytest.raises(ValueError):
        Scenario(capture_seconds=value)


def test_event_must_fit():
    with pytest.raises(ValueError):
        Scenario(attached_hold_seconds=0.5, event_seconds=1.0)
