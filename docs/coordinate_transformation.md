# Coordinate Transformation

## Gantry Frame

The gantry pose is represented as XYZ position in millimeters:

```text
gantry_pose = [x, y, z]
```

The program converts this into a 4x4 homogeneous transformation matrix.

## Cobot Frame

The UR cobot pose is represented as:

```text
cobot_pose = [x, y, z, rx, ry, rz]
```

where:

- `x, y, z` are translation values
- `rx, ry, rz` are axis-angle rotation values

The program uses RoboDK `robomath.UR_2_Pose()` and `robomath.Pose_2_UR()` helpers to convert between UR pose list and matrix representation.

## Gantry-Cobot Relation

The program combines gantry and cobot matrices:

```text
world_transform = gantry_matrix * cobot_matrix
```

This allows the system to reason about the inspection point in a combined world reference.

## X-Axis Reflection

The cobot matrix formation applies an X-axis reflection to align the cobot frame direction with the gantry frame direction:

```python
cp_local[0] *= -1
```

This is a machine-frame alignment rule and should be validated against the actual mechanical layout.
