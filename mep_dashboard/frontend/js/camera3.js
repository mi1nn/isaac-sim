'use strict';
// CAMERA 3: Astrobee observation feed. Polls the newest JPEG the backend received over
// ROS 2 (/astrobee/camera/image_raw). Image only: no Astrobee state is fetched or shown.
(function () {
  const img = document.getElementById('camera-3-feed');
  const placeholder = document.getElementById('camera-3-placeholder');
  const link = document.getElementById('camera-3-link');
  const PERIOD_MS = 200;
  let url = null;
  let live = null;
  function show(isLive) {
    if (isLive === live) return;
    live = isLive;
    img.hidden = !isLive;
    placeholder.hidden = isLive;
    link.textContent = isLive ? '● LIVE' : 'OFFLINE';
    link.classList.toggle('live', isLive);
  }
  async function poll() {
    try {
      const response = await fetch('/api/camera/astrobee.jpg', { cache: 'no-store' });
      if (response.status === 200) {
        const next = URL.createObjectURL(await response.blob());
        img.src = next;
        if (url) URL.revokeObjectURL(url);
        url = next;
        show(true);
      } else show(false);
    } catch (error) { show(false); }
    setTimeout(poll, PERIOD_MS);
  }
  show(false);
  poll();
})();
