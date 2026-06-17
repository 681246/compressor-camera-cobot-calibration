"""Compressor camera-cobot calibration demo.

Portfolio-safe version of an industrial calibration program.
It demonstrates how gantry XYZ, cobot 6D pose, and compressor offset can be
combined through homogeneous transformation matrices.

Units:
- gantry translation: mm
- cobot translation: mm
- cobot rotation: axis-angle radians
- compressor translation: mm
- compressor rotation input in Calculate2: degrees
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, degrees, radians, sin, sqrt
from typing import Iterable, List

import numpy as np


Pose3 = List[float]
Pose6 = List[float]


def _as_list(values: Iterable[float]) -> List[float]:
    return [float(v) for v in values]


class Formation:
    """Matrix helpers for gantry and cobot calibration."""

    @staticmethod
    def axis_angle_to_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
        """Convert axis-angle rotation vector into a 3x3 rotation matrix."""
        theta = sqrt(rx * rx + ry * ry + rz * rz)
        if theta == 0:
            return np.eye(3)

        kx, ky, kz = rx / theta, ry / theta, rz / theta
        cth = cos(theta)
        sth = sin(theta)
        vth = 1.0 - cth

        return np.array(
            [
                [kx * kx * vth + cth, kx * ky * vth - kz * sth, kx * kz * vth + ky * sth],
                [kx * ky * vth + kz * sth, ky * ky * vth + cth, ky * kz * vth - kx * sth],
                [kx * kz * vth - ky * sth, ky * kz * vth + kx * sth, kz * kz * vth + cth],
            ],
            dtype=float,
        )

    @staticmethod
    def matrix_to_axis_angle(rotation: np.ndarray) -> Pose3:
        """Convert a 3x3 rotation matrix into an axis-angle vector."""
        trace_value = np.trace(rotation)
        value = max(min((trace_value - 1.0) / 2.0, 1.0), -1.0)
        theta = np.arccos(value)
        if abs(theta) < 1e-9:
            return [0.0, 0.0, 0.0]

        scale = theta / (2.0 * sin(theta))
        rx = scale * (rotation[2, 1] - rotation[1, 2])
        ry = scale * (rotation[0, 2] - rotation[2, 0])
        rz = scale * (rotation[1, 0] - rotation[0, 1])
        return [float(rx), float(ry), float(rz)]

    def pose6_to_matrix(self, pose: Pose6, reflect_x: bool = False) -> np.ndarray:
        """Convert [x, y, z, rx, ry, rz] into a 4x4 transform matrix."""
        if len(pose) != 6:
            raise ValueError("pose must contain 6 values")

        x, y, z, rx, ry, rz = _as_list(pose)
        if reflect_x:
            x *= -1.0

        mat = np.eye(4)
        mat[:3, :3] = self.axis_angle_to_matrix(rx, ry, rz)
        mat[:3, 3] = [x, y, z]
        return mat

    def matrix_to_pose6(self, matrix: np.ndarray) -> Pose6:
        """Convert a 4x4 transform matrix into [x, y, z, rx, ry, rz]."""
        if matrix.shape != (4, 4):
            raise ValueError("matrix must be 4x4")

        x, y, z = matrix[:3, 3]
        rx, ry, rz = self.matrix_to_axis_angle(matrix[:3, :3])
        return [float(x), float(y), float(z), float(rx), float(ry), float(rz)]

    def gantry_mat_formation(self, gantry_pose: Pose3) -> np.ndarray:
        """Form 4x4 gantry matrix from XYZ position in mm."""
        if len(gantry_pose) != 3:
            raise ValueError("gantry_pose must contain 3 values")

        x, y, z = _as_list(gantry_pose)
        mat = np.eye(4)
        mat[:3, 3] = [x, y, z]
        return mat

    def cobot_mat_formation(self, cobot_pose: Pose6) -> np.ndarray:
        """Form 4x4 cobot matrix from UR-style 6D pose.

        X reflection is applied to match the gantry/cobot frame relationship used
        in the original industrial cell.
        """
        return self.pose6_to_matrix(cobot_pose, reflect_x=True)

    def offset_mat_formation(self, offset_pose: Pose6) -> np.ndarray:
        """Form 4x4 offset matrix from [x, y, z, rx, ry, rz]."""
        return self.pose6_to_matrix(offset_pose, reflect_x=False)

    def nor_matmul(self, gantry_pose: Pose3, cobot_pose: Pose6) -> np.ndarray:
        """Combine gantry and cobot into a single world transform."""
        return self.gantry_mat_formation(gantry_pose) @ self.cobot_mat_formation(cobot_pose)

    @staticmethod
    def revertg(values: Iterable[float]) -> Pose3:
        """Round gantry XYZ output to 4 decimals."""
        return [round(float(v), 4) for v in list(values)[:3]]


@dataclass
class CalibrationResult:
    gantry_pose: Pose3
    cobot_pose: Pose6


class Calculate:
    """Full calibration and transformation pipeline."""

    def __init__(self, common_gantry_pose: Pose3, common_cobot_pose: Pose6):
        self.mform = Formation()
        self.common_gantry_pose = common_gantry_pose.copy()
        self.common_cobot_pose = common_cobot_pose.copy()
        self.common_matrix = self.mform.nor_matmul(self.common_gantry_pose, self.common_cobot_pose)
        self.future_origin: list[Pose6] = []

    def targetpointform(self, target_gantry_pose: Pose3, target_cobot_pose: Pose6) -> None:
        self.target_gantry_pose = target_gantry_pose.copy()
        self.target_cobot_pose = target_cobot_pose.copy()
        self.target_matrix = self.mform.nor_matmul(self.target_gantry_pose, self.target_cobot_pose)

    def set_offset_matrix(self, compressor_offset: Pose6) -> None:
        self.compressor_offset = compressor_offset.copy()
        self.compressor_offset_matrix = self.mform.offset_mat_formation(compressor_offset.copy())

    def step1(self) -> np.ndarray:
        """Calculate pose difference between common reference and target."""
        self.pose_diff = np.linalg.inv(self.common_matrix) @ self.target_matrix
        return self.pose_diff

    def step2(self) -> np.ndarray:
        """Reconstruct new common reference matrix."""
        self.new_common_matrix = self.common_matrix @ self.pose_diff
        return self.new_common_matrix

    def step3(self) -> Pose6:
        """Apply compressor offset and calculate corrected reference."""
        result = self.target_matrix @ self.compressor_offset_matrix
        relative_offset = np.linalg.inv(self.target_matrix) @ result
        final_matrix = self.new_common_matrix @ relative_offset

        final_pose = self.mform.matrix_to_pose6(final_matrix)
        diff_pose = self.mform.matrix_to_pose6(self.pose_diff)
        self.final_new_common = (np.array(final_pose) - np.array(diff_pose)).tolist()
        return self.final_new_common

    def step4(self) -> CalibrationResult:
        """Separate gantry and cobot components from corrected reference."""
        diff_pose = self.mform.matrix_to_pose6(self.pose_diff)
        new_side_ref = (np.array(self.final_new_common) + np.array(diff_pose)).tolist()
        self.future_origin = [new_side_ref.copy(), new_side_ref.copy()]

        for i in range(3, 6):
            self.future_origin[0][i] *= -1.0

        tcp_ref = [-self.target_cobot_pose[0], self.target_cobot_pose[1], self.target_cobot_pose[2], 0.0, 0.0, 0.0]
        delta = np.array(new_side_ref) - np.array(tcp_ref)
        gantry_delta = self.mform.revertg(delta[:3])
        cobot_pose = [
            self.target_cobot_pose[0],
            self.target_cobot_pose[1],
            self.target_cobot_pose[2],
            float(delta[3]),
            float(delta[4]),
            float(delta[5]),
        ]
        return CalibrationResult(gantry_delta, cobot_pose)

    def rv2rpy(self, rx: float, ry: float, rz: float) -> Pose3:
        """Convert rotation vector to roll-pitch-yaw in degrees."""
        rotation = self.mform.axis_angle_to_matrix(rx, ry, rz)
        r11, r21, r31 = rotation[0, 0], rotation[1, 0], rotation[2, 0]
        r32, r33 = rotation[2, 1], rotation[2, 2]
        beta = atan2(-r31, sqrt(r11 * r11 + r21 * r21))
        alpha = atan2(r21, r11)
        gamma = atan2(r32, r33)
        return [degrees(gamma), degrees(beta), degrees(alpha)]


class Calculate2:
    """Simplified compressor offset application helper."""

    def __init__(self, offset: Pose6):
        self.mform = Formation()
        self.chg_offset(offset)

    def chg_offset(self, offset: Pose6) -> None:
        if len(offset) != 6:
            raise ValueError("offset must contain 6 values")
        self.offset = offset.copy()
        self.g_offset = self.offset[:3]
        self.c_offset = [0.0, 0.0, 0.0, radians(self.offset[3]), radians(self.offset[4]), radians(self.offset[5])]

    def get_g_offset_applied(self, gantry_pose: Pose3) -> Pose3:
        if len(gantry_pose) != 3:
            raise ValueError("gantry_pose must contain 3 values")
        return [float(gantry_pose[i]) + float(self.g_offset[i]) for i in range(3)]

    def get_cobot_trans(self, ref_pose: Pose6) -> Pose6:
        ref_matrix = self.mform.pose6_to_matrix(ref_pose)
        offset_matrix = self.mform.pose6_to_matrix(self.c_offset)
        corrected_matrix = ref_matrix @ offset_matrix
        return self.mform.matrix_to_pose6(corrected_matrix)

    def pose_diff(self, init_pose: Pose6, final_pose: Pose6) -> Pose6:
        init_matrix = self.mform.pose6_to_matrix(init_pose)
        final_matrix = self.mform.pose6_to_matrix(final_pose)
        diff_matrix = np.linalg.inv(init_matrix) @ final_matrix
        return self.mform.matrix_to_pose6(diff_matrix)


if __name__ == "__main__":
    common_gantry = [1000.0, 2000.0, 500.0]
    common_cobot = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
    target_gantry = [1200.0, 2100.0, 550.0]
    target_cobot = [110.0, 220.0, 320.0, 0.1, 0.2, 0.3]
    compressor_offset = [2.0, 1.0, 0.0, 0.0, 0.0, 1.5]

    calc = Calculate(common_gantry, common_cobot)
    calc.targetpointform(target_gantry, target_cobot)
    calc.set_offset_matrix(compressor_offset)
    calc.step1()
    calc.step2()
    calc.step3()
    result = calc.step4()

    print("Corrected gantry:", result.gantry_pose)
    print("Corrected cobot:", result.cobot_pose)

    simple = Calculate2(compressor_offset)
    print("Direct gantry offset:", simple.get_g_offset_applied(target_gantry))
    print("Direct cobot transform:", simple.get_cobot_trans(target_cobot))
