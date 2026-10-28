import pytest

from jsdemo.io import load_library, load_modules, load_product, load_rules
from jsdemo.pipeline import Runner


@pytest.fixture(scope="session")
def lib():
    return load_library()


@pytest.fixture(scope="session")
def product():
    return load_product()


@pytest.fixture(scope="session")
def modules():
    return load_modules()


@pytest.fixture(scope="session")
def rules():
    return load_rules()


@pytest.fixture(scope="session")
def run(tmp_path_factory):
    """完整跑一遍流水线（自动签字），后续测试检查它的产出。"""
    out = tmp_path_factory.mktemp("station")
    r = Runner(out, yes=True, quiet=True)
    rep = r.run(ecn_path="ecn/ECN-001.yaml")
    return {"out": out, "report": rep, "runner": r}
