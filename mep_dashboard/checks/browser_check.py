"""Browser smoke check against the running local server."""
import json
from pathlib import Path
from playwright.sync_api import sync_playwright

OUT = Path(__file__).resolve().parent
with sync_playwright() as p:
    browser = p.chromium.launch(headless=True)
    page = browser.new_page(viewport={'width': 1920, 'height': 1080}, device_scale_factor=1)
    errors, failed, commands = [], [], []
    page.on('pageerror', lambda error: errors.append(str(error)))
    page.on('console', lambda message: errors.append(message.text) if message.type == 'error' else None)
    page.on('requestfailed', lambda request: failed.append(request.url))
    page.on('request', lambda request: commands.append(request.url) if request.method != 'GET' else None)
    assert page.goto('http://127.0.0.1:8000').status == 200
    page.wait_for_function('window.Dashboard && Dashboard.charts.length === 1')
    assert page.locator('#position-error').inner_text() == '0.018'
    page.wait_for_timeout(2200)
    assert page.locator('#mission-time').inner_text() != '00:00:00'
    page.locator('#stop').click()
    stopped = page.locator('#mission-time').inner_text()
    value = page.locator('#position-error').inner_text()
    page.wait_for_timeout(1300)
    assert page.locator('#mission-time').inner_text() == stopped
    assert page.locator('#position-error').inner_text() == value
    page.locator('#reset').click()
    assert page.locator('#mission-time').inner_text() == '00:00:00'
    assert page.locator('#position-error').inner_text() == '0.018'
    page.screenshot(path=str(OUT / 'live-1920.png'))
    page.locator('#play').click()
    page.wait_for_timeout(1200)
    assert page.locator('#mission-time').inner_text() != '00:00:00'
    page.locator('#stop').click()
    layout = {}
    def dimensions():
        return page.evaluate('({width:innerWidth,height:innerHeight,scrollWidth:document.documentElement.scrollWidth,scrollHeight:document.documentElement.scrollHeight})')
    layout['live'] = dimensions()
    page.locator('#validation-tab').click()
    page.wait_for_function('Dashboard.charts.length === 5')
    assert page.locator('#run-history tr').count() == 48
    assert page.locator('#overall-rate').inner_text() == '85.4%'
    page.locator('[data-run="45"] button').click()
    assert page.locator('#run-result').inner_text() == '✕ FAILED'
    assert 'RUN-0045' in page.locator('#run-details').inner_text()
    page.locator('[data-run="48"] button').click()
    page.screenshot(path=str(OUT / 'validation-1920.png'))
    layout['validation'] = dimensions()
    for _ in range(3):
        page.locator('#live-tab').click()
        page.locator('#validation-tab').click()
    assert page.evaluate('Dashboard.charts.length') == 5
    assert page.evaluate('Dashboard.charts.every(c => c.width > 0 && c.height > 0)')
    for info in layout.values():
        assert info['scrollHeight'] == info['height'], info
        assert info['scrollWidth'] == info['width'], info
    page.locator('#validation-tab').focus()
    page.keyboard.press('ArrowLeft')
    assert page.locator('#live-tab').get_attribute('aria-selected') == 'true'
    page.set_viewport_size({'width': 390, 'height': 844})
    assert dimensions()['scrollWidth'] == 390
    page.locator('#validation-tab').click()
    assert dimensions()['scrollWidth'] == 390
    assert not errors, errors
    assert not failed, failed
    assert not commands, commands
    report = {'result': 'PASS', 'javascript_errors': errors, 'failed_requests': failed,
              'command_requests': commands, 'desktop_layout': layout,
              'checks': ['live mock updates', 'stop freezes telemetry', 'reset restores initial data',
                         'play resumes', 'tabs and arrow navigation', 'all five Chart.js charts',
                         '48 selectable runs', 'failed run details', 'no duplicate charts',
                         '1920x1080 no page overflow', '390px no horizontal overflow']}
    (OUT / 'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    browser.close()
