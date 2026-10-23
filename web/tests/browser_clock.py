"""Advance the real rendered game at a fixed simulation rate, independent of CI GPU speed."""
from datetime import datetime, timedelta, timezone

STEP_MS = 40


def control_clock(page):
    epoch = datetime(2026, 1, 1, tzinfo=timezone.utc)
    page.clock.install(time=epoch)
    page.clock.pause_at(epoch + timedelta(seconds=1))
    page.add_init_script("""
        window.__smokeFrames = [];
        window.requestAnimationFrame = callback => window.__smokeFrames.push(callback);
        window.__smokeTick = () => {
            const callbacks = window.__smokeFrames.splice(0);
            for (const callback of callbacks) callback(performance.now());
        };
    """)


def advance(page, duration_ms):
    remaining = duration_ms
    while remaining > 0:
        step = min(STEP_MS, remaining)
        page.clock.run_for(step)
        page.evaluate('window.__smokeTick()')
        remaining -= step


def wait_state(page, predicate, timeout):
    for _ in range((timeout + STEP_MS - 1) // STEP_MS):
        if page.evaluate(predicate):
            return
        advance(page, STEP_MS)
    snapshot = page.evaluate('window.rustDriving?.snapshot()')
    raise AssertionError(f'Condition did not pass within {timeout} ms of simulated driving: {predicate}; {snapshot}')
