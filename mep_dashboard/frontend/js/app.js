'use strict';
window.Dashboard = {
  charts: [],
  chart(id, type, data, options = {}) {
    if (!window.Chart) return null;
    const chart = new Chart(document.getElementById(id), {
      type, data,
      options: {
        responsive: true, maintainAspectRatio: false, animation: false,
        plugins: { legend: { display: false } },
        scales: {
          x: { grid: { color: '#153344' }, border: { color: '#4a6d82' }, ticks: { color: '#92afc4', maxTicksLimit: 7 } },
          y: { grid: { color: '#153344' }, border: { color: '#4a6d82' }, ticks: { color: '#92afc4', maxTicksLimit: 5 } }
        }, ...options
      }
    });
    this.charts.push(chart);
    return chart;
  },
  axis(title, extra = {}) {
    return { title: { display: true, text: title, color: '#9ab5c9', font: { size: 11 } },
      grid: { color: '#153344' }, border: { color: '#567486' },
      ticks: { color: '#92afc4', maxTicksLimit: 6, font: { size: 10 } }, ...extra };
  }
};
if (window.Chart) {
  Chart.defaults.color = '#93b4cd';
  Chart.defaults.font.family = "'Segoe UI', Arial, sans-serif";
  Chart.defaults.font.size = 11;
} else document.getElementById('chart-warning').hidden = false;
const tabs = [...document.querySelectorAll('[role="tab"]')];
let validationInitialized = false;
function selectTab(tab) {
  tabs.forEach(item => {
    const selected = item === tab;
    item.setAttribute('aria-selected', String(selected));
    item.tabIndex = selected ? 0 : -1;
    document.getElementById(item.getAttribute('aria-controls')).hidden = !selected;
  });
  if (tab.id === 'validation-tab' && !validationInitialized) {
    window.initValidation(); validationInitialized = true;
  }
  requestAnimationFrame(() => Dashboard.charts.forEach(chart => {
    if (chart.canvas.offsetParent !== null) chart.resize();
  }));
}
tabs.forEach((tab, index) => {
  tab.addEventListener('click', () => selectTab(tab));
  tab.addEventListener('keydown', event => {
    let next;
    if (event.key === 'ArrowRight') next = tabs[(index + 1) % tabs.length];
    if (event.key === 'ArrowLeft') next = tabs[(index + tabs.length - 1) % tabs.length];
    if (event.key === 'Home') next = tabs[0];
    if (event.key === 'End') next = tabs[tabs.length - 1];
    if (next) { event.preventDefault(); next.focus(); selectTab(next); }
  });
});
function updateClock() {
  const now = new Date();
  document.getElementById('clock').textContent = now.toISOString().slice(0, 19).replace('T', ' ') + ' (UTC)';
  document.getElementById('clock').dateTime = now.toISOString();
}
updateClock(); setInterval(updateClock, 1000);
window.initLive();
