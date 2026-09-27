"""printf from a worker-hosted chip reaches chip_log line by line
(finding chip-printf-fully-buffered, 2026-09-27).

wasi-libc decides stdout's buffering on the first write by asking
fd_fdstat_get whether fd 1 is a character device without seek/tell rights
(its isatty). The shim answered success without writing the struct, so a
chip's first printf line came out and the rest sat in libc's 1 KB buffer.
Same fixture as the browser runtime's test
(frontend/src/__tests__/chip-printf-line-buffered.test.ts).
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent / 'backend'))
pytest.importorskip('wasmtime', reason='chip runtime needs wasmtime')

from app.services.wasm_chip_runtime import WasmChipRuntime  # noqa: E402

FIXTURE = (Path(__file__).parent.parent.parent.parent / 'frontend' / 'src' / '__tests__'
           / 'board-buses' / 'fixtures' / 'chips-other-chips' / 'printf-probe.wasm')


def test_every_printf_line_reaches_chip_log(monkeypatch):
    now = [0]
    monkeypatch.setattr(WasmChipRuntime, 'sim_now_nanos', lambda _self: now[0])
    logs: list[str] = []

    def emit(payload: dict) -> None:
        if payload.get('type') == 'chip_log':
            logs.append(str(payload.get('text', '')).rstrip('\n'))

    rt = WasmChipRuntime(FIXTURE.read_bytes(), {}, emit)
    rt.run_chip_setup()
    assert logs == ['setup']
    for k in (1, 2, 3):
        deadline = rt.next_timer_deadline()
        assert deadline is not None
        now[0] = deadline
        rt.fire_due_timers()
        # Before the fix: still ['setup'], the tick lines were buffered.
        assert logs[-1] == f'tick {k}'
