"""Unit test for the stream parser (run: python -m pytest tools/)."""
from serial_monitor import WINDOW_SIZE, parse_stream


def fake_stream(n_windows: int = 2) -> list[str]:
    lines = []
    for w in range(1, n_windows + 1):
        lines.append("")
        lines.append(f"WINDOW {w} | samples=1024 | missed=0 | dropped=0")
        lines += [f"{i},{i},{-i},16384" for i in range(WINDOW_SIZE)]
        lines.append(f"AI_RESULT window={w} class=1.3 inference_ms=4210")
    lines.append("WINDOW 3 | samples=1024 | missed=2 | dropped=0")
    lines.append("AI_ERROR window=3 inference_ms=10")
    return lines


def test_parse_stream():
    windows = list(parse_stream(fake_stream()))
    assert len(windows) == 3
    assert windows[0].complete and windows[0].predicted == "1.3"
    assert windows[0].samples[5] == (5, -5, 16384)
    assert windows[2].predicted is None and windows[2].missed == 2
