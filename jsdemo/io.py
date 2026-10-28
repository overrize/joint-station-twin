"""读写仓库里的知识库、模块库和工艺库。仓库就是唯一真源。"""
from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parent.parent


def load_yaml(path: Path | str) -> Any:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def dump_yaml(data: Any, path: Path | str) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)


def write_text(text: str, path: Path | str) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def load_product(name: str = "joint-v1", root: Path = ROOT) -> dict:
    base = root / "kb" / "product" / name
    bom = load_yaml(base / "bom.yaml")
    fp = load_yaml(base / "fastening_points.yaml")
    ctq = load_yaml(base / "ctq.yaml")
    return {"name": name, "bom": bom, "fastening": fp, "ctq": ctq["ctq"]}


def load_modules(root: Path = ROOT) -> dict[str, dict]:
    mods = {}
    for p in sorted((root / "kb" / "modules").glob("*.yaml")):
        m = load_yaml(p)
        mods[m["id"]] = m
    return mods


def load_rules(root: Path = ROOT) -> dict:
    return {
        "dfm": load_yaml(root / "kb" / "rules" / "dfm.yaml")["rules"],
        "golden": load_yaml(root / "kb" / "rules" / "golden.yaml"),
    }


def load_library(root: Path = ROOT) -> dict:
    return load_yaml(root / "process" / "library" / "fastening.yaml")


def groups_by_id(product: dict) -> dict[str, dict]:
    return {g["id"]: g for g in product["fastening"]["groups"]}


def points(product: dict) -> list[dict]:
    """按装配顺序排好的紧固点。"""
    order = product["bom"]["assembly_order"]
    pts = list(product["fastening"]["points"])
    return sorted(pts, key=lambda p: (order.index(p["group"]), p["id"]))


def clone(x: Any) -> Any:
    return copy.deepcopy(x)
