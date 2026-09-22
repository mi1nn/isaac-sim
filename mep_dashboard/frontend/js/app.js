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
          x: { grid: { color: 'rgba(255,255,255,.06)' }, border: { color: 'rgba(255,255,255,.14)' }, ticks: { color: '#8d97a8', maxTicksLimit: 7 } },
          y: { grid: { color: 'rgba(255,255,255,.06)' }, border: { color: 'rgba(255,255,255,.14)' }, ticks: { color: '#8d97a8', maxTicksLimit: 5 } }
        }, ...options
      }
    });
    this.charts.push(chart);
    return chart;
  },
  axis(title, extra = {}) {
    return { title: { display: true, text: title, color: '#a4adbd', font: { size: 11 } },
      grid: { color: 'rgba(255,255,255,.06)' }, border: { color: 'rgba(255,255,255,.14)' },
      ticks: { color: '#8d97a8', maxTicksLimit: 6, font: { size: 10 } }, ...extra };
  }
};
if (window.Chart) {
  Chart.defaults.color = '#a4adbd';
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
  const parts = new Intl.DateTimeFormat("en-CA", {
    timeZone: "Asia/Seoul",
    year: "numeric", month: "2-digit", day: "2-digit",
    hour: "2-digit", minute: "2-digit", second: "2-digit",
    hour12: false
  }).formatToParts(now);
  const value = Object.fromEntries(parts.map(part => [part.type, part.value]));
  document.getElementById("clock").textContent =
    value.year + "-" + value.month + "-" + value.day + " " +
    value.hour + ":" + value.minute + ":" + value.second + " (KST)";
  document.getElementById("clock").dateTime = now.toISOString();
}
updateClock(); setInterval(updateClock, 1000);
window.initLive();
