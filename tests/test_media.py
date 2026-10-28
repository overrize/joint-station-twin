"""录屏素材（M4.1）：视频不存在时跳过；存在时检查是有效 mp4、字幕非空、有预览帧和孔位近景帧。"""
import re
from pathlib import Path

import pytest

BUILD = Path(__file__).resolve().parents[1] / "build"
VIDEOS = [
    ("twin/media/dfmfix_demo_e3retryok/TWIN-e3retryok", "复拧"),
    ("twin/media/dfmfix_demo_d2slip/TWIN-d2slip", "整件停止"),
    ("twin/media/calibration/calibration", "标定完成"),
    ("twin_station_hub/media/baseline/HUB-0001", "整件合格"),
    ("twin/media/dfmfix_line/TWIN-L_line", "第 2 台"),
]


@pytest.mark.parametrize("stem,must_say", VIDEOS)
def test_video(stem, must_say):
    mp4 = BUILD / f"{stem}.mp4"
    if not mp4.exists():
        pytest.skip(f"还没有 {mp4.name}：见 docs/03-plan.md M4.1")
    head = mp4.read_bytes()[:12]
    assert head[4:8] == b"ftyp", "不是 mp4"
    assert mp4.stat().st_size > 1_000_000
    srt = (BUILD / f"{stem}.srt").read_text(encoding="utf-8")
    stamps = re.findall(r"(\d\d):(\d\d):(\d\d),(\d{3}) --> ", srt)
    assert len(stamps) >= 3
    secs = [int(h) * 3600 + int(m) * 60 + int(s) + int(ms) / 1000 for h, m, s, ms in stamps]
    assert secs == sorted(secs)
    assert must_say in srt
    for k in range(1, 5):
        assert (mp4.parent / f"{mp4.stem}_preview{k}.png").exists()
    assert (mp4.parent / f"{mp4.stem}_closeup.png").exists(), "没有切到过孔位近景"
