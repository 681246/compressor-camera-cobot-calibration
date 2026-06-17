# Offset Correction Logic

## Full Pipeline: `Calculate`

The `Calculate` class handles the full matrix-based calibration pipeline.

### Step 1

Calculates the pose difference:

```text
pose_difference = inverse(common_reference_matrix) * target_point_matrix
```

### Step 2

Reconstructs the new common reference:

```text
new_common_matrix = common_reference_matrix * pose_difference
```

### Step 3

Applies the compressor offset matrix and generates a corrected common reference.

### Step 4

Separates the correction into:

- gantry XYZ correction
- cobot pose correction

### Step 6

Calculates adjusted future origins for side-based inspection logic.

## Simplified Pipeline: `Calculate2`

The `Calculate2` class applies compressor offset directly:

- gantry offset is applied to gantry XYZ
- cobot offset is converted into a transform and multiplied with the reference pose

This is useful for easier testing and faster validation of offset behavior.

## Validation Direction

Recommended test cases:

| Scenario | Expected Behaviour |
|---|---|
| Zero offset | Corrected pose should remain unchanged |
| Positive X offset | Gantry X should increase by offset |
| Negative X offset | Gantry X should reduce by offset |
| Rotation offset | Cobot RX/RY/RZ should change according to transform |
