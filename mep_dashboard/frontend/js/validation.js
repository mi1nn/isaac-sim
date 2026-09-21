'use strict';
window.initValidation = function () {
  const tolerance = .05;
  const captureFailures = new Set([5, 18, 33]);
  const dockingFailures = new Set([5, 12, 27, 39, 45]);
  // Deterministic fixtures: all summaries, plots and selected details share these records.
  const runs = Array.from({ length: 48 }, (_, index) => {
    const id = index + 1;
    const captureError = .071 * Math.exp(-index / 11) + .007 + .005 * Math.abs(Math.sin(id * 1.71));
    const dockingError = .064 * Math.exp(-index / 9) + .006 + .004 * Math.abs(Math.cos(id * 1.39));
    const ca = id * 2.39996, da = id * 2.12;
    return { id, captureError, dockingError,
      captureXY: { x: captureError * Math.cos(ca), y: captureError * Math.sin(ca) },
      dockingXY: { x: dockingError * Math.cos(da), y: dockingError * Math.sin(da) },
      captureSuccess: !captureFailures.has(id), dockingSuccess: !dockingFailures.has(id),
      duration: 480 + ((id * 17) % 89), config: id % 9 === 0 ? 'Safe-Mode' : 'Nominal',
      orientation: .21 + .09 * Math.abs(Math.sin(id)), depth: .108 + .01 * Math.abs(Math.cos(id)),
      velocity: .006 + .003 * Math.abs(Math.sin(id * .4)),
      date: new Date(Date.UTC(2026, 8, 21, 4, 0) - (48 - id) * 43 * 60000).toISOString().slice(0, 19).replace('T', ' ')
    };
  });
  const count = predicate => runs.filter(predicate).length;
  const pct = value => (100 * value / runs.length).toFixed(1) + '%';
  const mean = values => values.reduce((a, b) => a + b, 0) / values.length;
  const overall = count(run => run.captureSuccess && run.dockingSuccess);
  const captureCount = count(run => run.captureSuccess), dockingCount = count(run => run.dockingSuccess);
  const avgTime = mean(runs.map(run => run.duration));
  document.getElementById('overall-rate').textContent = pct(overall);
  document.getElementById('overall-count').textContent = `${overall} / 48 runs`;
  document.getElementById('avg-time').textContent = `${Math.round(avgTime)} s`;
  document.getElementById('avg-minutes').textContent = `(${(avgTime / 60).toFixed(1)} min)`;
  document.getElementById('time-change').textContent = `▼ ${Math.round((1 - avgTime / 770) * 100)}%`;
  for (const [name, successes] of [['capture', captureCount], ['docking', dockingCount]]) {
    document.getElementById(`${name}-rate`).textContent = pct(successes);
    document.getElementById(`${name}-count`).textContent = `${successes} / 48 runs`;
  }
  function metrics(name) {
    const errors = runs.map(run => run[`${name}Error`]);
    const average = mean(errors);
    const std = Math.sqrt(mean(errors.map(error => (error - average) ** 2)));
    const rows = name === 'capture' ? [['Total Runs', '48']] : [];
    rows.push(['Mean Error', `${average.toFixed(3)} m`], ['Std Dev', `${std.toFixed(3)} m`]);
    if (name === 'docking') rows.push(['Orientation Error', `${mean(runs.map(run => run.orientation)).toFixed(2)} deg`],
      ['Insertion Depth', `${mean(runs.map(run => run.depth)).toFixed(3)} m`]);
    rows.push(['Within Tolerance', pct(errors.filter(error => error <= tolerance).length)]);
    document.getElementById(`${name}-metrics`).innerHTML = rows.map(([key, value]) =>
      `<div><dt>${key}</dt><dd class="${key === 'Within Tolerance' ? 'good' : ''}">${value}</dd></div>`).join('');
  }
  metrics('capture'); metrics('docking');
  const circle = Array.from({ length: 101 }, (_, index) => ({
    x: tolerance * Math.cos(index / 100 * 2 * Math.PI), y: tolerance * Math.sin(index / 100 * 2 * Math.PI)
  }));
  function scatter(name, color, center) {
    Dashboard.chart(`${name}-scatter`, 'scatter', { datasets: [
      { label: 'Tolerance R = 0.05 m', data: circle, showLine: true, pointRadius: 0, borderColor: '#cebda3', borderWidth: 1.4, borderDash: [5, 4] },
      { label: 'Run error (m)', data: runs.map(run => run[`${name}XY`]), pointRadius: 3, pointHoverRadius: 5, backgroundColor: color },
      { label: center, data: [{ x: 0, y: 0 }], pointStyle: 'cross', pointRadius: 10, borderWidth: 2, borderColor: '#b0ffe9', backgroundColor: '#b0ffe9' }
    ] }, { scales: {
      x: Dashboard.axis('X Error [m]', { type: 'linear', min: -.1, max: .1, ticks: { color: '#92afc4', stepSize: .05, callback: value => value.toFixed(2), font: { size: 10 } } }),
      y: Dashboard.axis('Y Error [m]', { min: -.1, max: .1, ticks: { color: '#92afc4', stepSize: .05, callback: value => value.toFixed(2), font: { size: 10 } } })
    } });
  }
  scatter('capture', '#48bcff', 'Target (0, 0)'); scatter('docking', '#26e7b2', 'Dock Center (0, 0)');
  for (const [name, color] of [['capture', '#46d0ee'], ['docking', '#27e5b0']]) {
    Dashboard.chart(`${name}-trend`, 'line', { datasets: [{
      label: `${name} error (m)`, data: runs.map(run => ({ x: run.id, y: run[`${name}Error`] })),
      borderColor: color, backgroundColor: color, borderWidth: 1.4, pointRadius: 2, tension: .1
    }] }, { scales: {
      x: Dashboard.axis('Run Number', { type: 'linear', min: 1, max: 48 }),
      y: Dashboard.axis('Error [m]', { type: 'logarithmic', min: .001, max: 1,
        ticks: { color: '#92afc4', callback: value => [1, .1, .01, .001].includes(value) ? value : '', font: { size: 10 } } })
    } });
  }
  const runID = id => `RUN-${String(id).padStart(4, '0')}`;
  const mark = success => `<span class="${success ? 'good' : 'bad'}" aria-label="${success ? 'Success' : 'Failed'}">${success ? '✓' : '✕'}</span>`;
  document.getElementById('run-history').innerHTML = [...runs].reverse().map(run => `<tr data-run="${run.id}"><td><button aria-label="Select ${runID(run.id)}" aria-pressed="false">${runID(run.id)}</button></td><td>${run.config}</td><td>${mark(run.captureSuccess)}</td><td>${mark(run.dockingSuccess)}</td><td>${run.duration}</td><td>${run.date}</td></tr>`).join('');
  const detailList = rows => `<dl>${rows.map(([key, value]) => `<div><dt>${key}</dt><dd>${value}</dd></div>`).join('')}</dl>`;
  function selectRun(id) {
    const run = runs.find(item => item.id === id);
    const success = run.captureSuccess && run.dockingSuccess;
    document.querySelectorAll('[data-run]').forEach(row => {
      const selected = Number(row.dataset.run) === id;
      row.classList.toggle('selected', selected);
      row.querySelector('button').setAttribute('aria-pressed', String(selected));
    });
    const result = document.getElementById('run-result');
    result.textContent = success ? '✓ SUCCESS' : '✕ FAILED';
    result.className = `result ${success ? 'good' : 'bad'}`;
    document.getElementById('run-details').innerHTML = `<div>${detailList([
      ['Run ID', runID(id)], ['Config', run.config], ['Date (UTC)', run.date],
      ['Result', `<span class="${success ? 'good' : 'bad'}">${success ? 'Success' : 'Failed'}</span>`]
    ])}<p class="detail-notes">${success ? 'Mock mission completed. Capture and docking succeeded.' : `Mock failure: ${[!run.captureSuccess && 'capture', !run.dockingSuccess && 'docking'].filter(Boolean).join(' / ')}.`}</p></div>${detailList([
      ['Capture Error', `${run.captureError.toFixed(3)} m`], ['Docking Error', `${run.dockingError.toFixed(3)} m`],
      ['Orientation Error', `${run.orientation.toFixed(2)} deg`], ['Insertion Depth', `${run.depth.toFixed(3)} m`],
      ['Rel. Velocity', `${run.velocity.toFixed(3)} m/s`], ['Mission Time', `${run.duration} s`]
    ])}<div class="run-placeholder"><b>⌖</b><span>NO RUN IMAGE</span><small>${runID(id)} · MOCK</small></div>`;
  }
  document.getElementById('run-history').addEventListener('click', event => {
    const row = event.target.closest('[data-run]');
    if (row) selectRun(Number(row.dataset.run));
  });
  selectRun(48);
};
