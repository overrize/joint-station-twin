import argparse
import sys

from .pipeline import Runner


def main(argv=None):
    ap = argparse.ArgumentParser(prog="jsdemo", description="关节紧固工站 demo 流水线")
    sp = ap.add_subparsers(dest="cmd", required=True)
    r = sp.add_parser("run")
    r.add_argument("--out", default="build/station")
    r.add_argument("--yes", action="store_true", help="自动通过全部签字关口")
    r.add_argument("--stop-after", choices=["intake", "dfm", "process", "compose", "generate", "review", "verify", "fai"])
    r.add_argument("--ecn")
    r.add_argument("--sop", help="工站 SOP（上游 agent 输出，标准见 docs/06-sop-contract.md）；不给时用工站设计引用的 SOP，都没有时用 kb/product/joint-v1")
    r.add_argument("--station", help="工站设计（上游 agent 输出，jsdemo.station/1，标准见 docs/07-station-contract.md）：布局、治具、分工都按它；"
                                     "不给时布局取示例 kb/station/joint-v1.station.yaml")
    a = ap.parse_args(argv)
    rep = Runner(a.out, yes=a.yes, sop=a.sop, station_spec=a.station).run(stop_after=a.stop_after, ecn_path=a.ecn)
    print(f"\n状态：{rep['status']}  报告：{a.out}/report.json")
    return 0 if rep["status"] in ("accepted", "incomplete") else 1


if __name__ == "__main__":
    sys.exit(main())
