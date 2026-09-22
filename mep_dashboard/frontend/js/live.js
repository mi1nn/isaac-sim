'use strict';
// Mock values below mirror the real mission, so the labels match what firebase_bridge.py
// / vision_capture_demo.py (branch feature/integration) will actually publish once the
// live Firestore connection is wired in:
//   - `state` is a real State enum value (vision_capture_demo.py State), not a made-up label.
//   - `phase` is the friendly name for firebase_bridge.py's phase classification
//     (CAPTURE / DOCKING / MISSION -- MISSION_STATES covers MRV approach + moving-client
//     release/chase/rendezvous/separation).
//   - The 7-step Mission Progress groups the full state machine into mission-level stages:
//     1 MRV APPROACH (MRV_MOVE_STEP_1/2, ARM_DEPLOY) -> 2 SEARCH & TRACK (SEARCH..PREDICTING)
//     -> 3 MEP CAPTURE (APPROACHING..RETREAT, shown here) -> 4 CLIENT RELEASE & CHASE
//     (CLIENT_RELEASE, CLIENT_CRUISE, CHASE, VELOCITY_MATCHING, RENDEZVOUS)
//     -> 5 DOCKING APPROACH (DOCK_TARGET_ACQUIRE..FINAL_INSERTION) -> 6 DOCKED & STABILIZE
//     (DOCKED, STABILIZING, STOPPING) -> 7 RELEASE & SEPARATION (ROBOT_RELEASE, ARM_RETREAT,
//     MRV_SEPARATION, SUCCESS).
window.initLive = function () {
  const initial = { progress: 3, total_steps: 7, state: 'SLOW_APPROACH', phase: 'MEP CAPTURE PHASE',
    position_error: 0.018, total_velocity: 0.021, total_angular_velocity: 0.35, remaining_distance: 0.118 };
  // This is a browser-only preview. Controls never send a network command.
  let running = true;
  let elapsed = 0;
  let sample = 0;
  const initialSeries = () => Array.from({ length: 61 }, (_, index) => ({
    x: index - 60,
    y: index === 60 ? initial.position_error : 0.7 * Math.exp(-index / 16.1) * (1 + .055 * Math.sin(index * 2.3)) + .001
  }));
  const chart = Dashboard.chart('position-chart', 'line', {
    datasets: [{ label: 'Position Error (m)', data: initialSeries(), borderColor: '#47acff',
      borderWidth: 2, pointRadius: 0, tension: .18, fill: false }]
  }, { scales: {
    x: Dashboard.axis('Time [s]', { type: 'linear', min: -60, max: 0 }),
    y: Dashboard.axis('Position Error [m]', { type: 'logarithmic', min: .001, max: 1,
      ticks: { color: '#92afc4', callback: value => [1, .1, .01, .001].includes(value) ? value : '' } })
  } });
  const play = document.getElementById('play');
  const stop = document.getElementById('stop');
  function controls(label) {
    play.disabled = running;
    stop.disabled = !running;
    document.getElementById('playback-status').textContent = label;
  }
  function showTime() {
    document.getElementById('mission-time').textContent = [Math.floor(elapsed / 3600), Math.floor(elapsed / 60) % 60, elapsed % 60]
      .map(value => String(value).padStart(2, '0')).join(':');
  }
  play.addEventListener('click', () => { running = true; controls('MOCK RUNNING'); });
  stop.addEventListener('click', () => { running = false; controls('MOCK STOPPED'); });
  document.getElementById('reset').addEventListener('click', () => {
    running = false; elapsed = 0; sample = 0;
    document.getElementById('position-error').textContent = initial.position_error.toFixed(3);
    showTime();
    if (chart) {
      chart.data.datasets[0].data = initialSeries();
      chart.options.scales.x.min = -60; chart.options.scales.x.max = 0;
      chart.update('none');
    }
    controls('MOCK RESET · READY');
  });
  controls('MOCK PREVIEW');
  setInterval(() => {
    if (!running) return;
    elapsed += 1; sample += 1;
    const error = .018 + .0025 * Math.sin(sample * .39) + .0008 * Math.sin(sample * 1.73);
    document.getElementById('position-error').textContent = error.toFixed(3);
    showTime();
    if (chart) {
      const data = chart.data.datasets[0].data;
      data.push({ x: sample, y: error });
      if (data.length > 121) data.shift();
      chart.options.scales.x.min = data[0].x;
      chart.options.scales.x.max = sample;
      chart.update('none');
    }
  }, 1000);
};
