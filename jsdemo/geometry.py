"""坐标系与自标定。所有紧固点都在治具坐标系下定义，现场只标定治具坐标系。"""
from __future__ import annotations

import math

Vec = list  # [x, y, z]，单位 mm


def sub(a, b): return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
def add(a, b): return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
def dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def norm(a): return math.sqrt(dot(a, a))
def scale(a, s): return [a[0] * s, a[1] * s, a[2] * s]
def cross(a, b): return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
def unit(a):
    n = norm(a)
    if n < 1e-12:
        raise ValueError("零向量无法归一化")
    return scale(a, 1.0 / n)


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(A, v):
    return [A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
            A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
            A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2]]


def transpose(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


class Pose:
    """刚体位姿：p_parent = R * p_child + t。"""

    def __init__(self, R=None, t=None):
        self.R = R or [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
        self.t = t or [0.0, 0.0, 0.0]

    @staticmethod
    def from_xyz_rpy(xyz, rpy_deg=(0.0, 0.0, 0.0)):
        r, p, y = (math.radians(v) for v in rpy_deg)
        Rx = [[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]]
        Ry = [[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]]
        Rz = [[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]]
        return Pose(matmul(Rz, matmul(Ry, Rx)), list(map(float, xyz)))

    def apply(self, p):
        return add(matvec(self.R, p), self.t)

    def compose(self, other: "Pose") -> "Pose":
        return Pose(matmul(self.R, other.R), add(matvec(self.R, other.t), self.t))

    def inverse(self) -> "Pose":
        Rt = transpose(self.R)
        return Pose(Rt, scale(matvec(Rt, self.t), -1.0))

    def yaw_deg(self) -> float:
        return math.degrees(math.atan2(self.R[1][0], self.R[0][0]))


def frame_from_three_points(p1, p2, p3) -> Pose:
    """原点在 p1，x 轴指向 p2，xy 平面过 p3。"""
    x = unit(sub(p2, p1))
    z = unit(cross(x, sub(p3, p1)))
    y = cross(z, x)
    R = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
    return Pose(R, list(p1))


def calibrate_fixture(datums_in_fixture: list, measured_in_base: list) -> Pose:
    """由三个基准点的名义坐标（治具系）和实测坐标（基座系）求治具在基座系中的位姿。"""
    F = frame_from_three_points(*datums_in_fixture)   # 基准三角形在治具系
    M = frame_from_three_points(*measured_in_base)    # 基准三角形在基座系
    return M.compose(F.inverse())


def pose_error(a: Pose, b: Pose) -> dict:
    dt = norm(sub(a.t, b.t))
    Rrel = matmul(transpose(a.R), b.R)
    c = max(-1.0, min(1.0, (Rrel[0][0] + Rrel[1][1] + Rrel[2][2] - 1.0) / 2.0))
    return {"translation_mm": dt, "rotation_deg": math.degrees(math.acos(c))}
