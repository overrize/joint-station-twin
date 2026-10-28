"""现场自标定：治具实际位置偏了，触碰三个基准点后能把坐标系找回来。"""
from sim.kinematic import simulate_calibration


def test_exact_without_noise(modules):
    r = simulate_calibration(modules["FX-JOINT-V1"], {"xyz_mm": [0.8, -0.5, 0.2], "rpy_deg": [0.1, -0.05, 0.3]})
    assert r["before"]["translation_mm"] > 0.5
    assert r["after"]["translation_mm"] < 1e-6 and r["after"]["rotation_deg"] < 1e-6


def test_with_probe_noise(modules):
    r = simulate_calibration(modules["FX-JOINT-V1"], {"xyz_mm": [0.8, -0.5, 0.2], "rpy_deg": [0.1, -0.05, 0.3]}, noise_mm=0.02)
    assert r["after"]["translation_mm"] < 0.1
