"""擦除：模型没见过的组合必须返回未知，不编数。"""
from jsdemo.procmodel import recommend


def test_unknown_material(lib):
    assert recommend("PEEK", "M2x6-tapping", 1.6, lib)["status"] == "unknown"


def test_hole_out_of_range(lib):
    assert recommend("PA66", "M2x6-tapping", 2.2, lib)["status"] == "unknown"


def test_library_exact(lib):
    r = recommend("AL6061", "M3x8-machine", None, lib)
    assert r["status"] == "library" and r["window_nm"] == [0.90, 1.10]


def test_model_interpolation(lib):
    r = recommend("PA66", "M2x6-tapping", 1.6, lib)
    assert r["status"] == "model" and r["confidence"] == "medium"
    assert r["window_nm"] == [0.15, 0.21]
