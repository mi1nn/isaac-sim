'use strict';
// Browser reader for the server-side Firebase Admin API. No credential is shipped to the browser.
window.ValidationFirestore = {
  async request(path) {
    const response = await fetch(path, { headers: { Accept: 'application/json' } });
    let body = {};
    try { body = await response.json(); } catch (_) { /* handled below */ }
    if (!response.ok) throw new Error(body.detail || 'Firebase Connection Error');
    return body;
  },
  loadRuns() { return this.request('/api/validation/runs'); },
  loadTelemetry(sessionId) { return this.request(`/api/validation/runs/${encodeURIComponent(sessionId)}/telemetry`); },
  loadVideoMetadata(sessionId) { return this.request(`/api/validation/runs/${encodeURIComponent(sessionId)}/video-metadata`); }
};
