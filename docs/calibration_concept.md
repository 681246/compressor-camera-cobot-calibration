# Calibration Concept

## Problem

In a compressor inspection cell, the trained reference position and the actual production position may not perfectly match. This can happen due to:

- compressor trolley placement variation
- mechanical drift in gantry movement
- cobot/camera mounting variation
- daily system offset change
- difference between trained reference and current reference

Even small deviations can affect camera view and inspection repeatability.

## Concept

The calibration logic compares a known common reference with a target/checkpoint reference. It builds transformation matrices for both gantry and cobot poses, calculates the pose difference, applies the compressor offset, and generates corrected movement values.

## Main Entities

| Entity | Meaning |
|---|---|
| Common reference | Trained reference position used as the base |
| Target point | Inspection or calibration point to be corrected |
| Compressor offset | Current compressor displacement from trained position |
| Gantry correction | XYZ correction applied to gantry movement |
| Cobot correction | RX/RY/RZ correction applied to cobot pose |

## Simplified Formula View

```text
common_reference_matrix = gantry_matrix * cobot_matrix

target_point_matrix = target_gantry_matrix * target_cobot_matrix

pose_difference = inverse(common_reference_matrix) * target_point_matrix

corrected_reference = target_point_matrix * compressor_offset_matrix
```

The exact implementation is available in `src/compressor_calib.py`.
